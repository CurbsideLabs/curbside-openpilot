#!/usr/bin/env python3
"""
Curbside demo daemon.

Replaces plannerd when the ``DemoScriptMode`` param is set (see system/manager/process_config.py).
Runs a distance-based state machine that publishes:

- ``longitudinalPlan``: scripted speed/accel (drive forward N feet, then stop). ``shouldStop=False``
  while moving makes controlsd emit ``cruiseControl.resume`` automatically, so a remote "go" from a
  standstill works with stock controls once the car is engaged.
- ``longitudinalPlanSP``: empty-but-valid keep-alive. selfdrived requires this service alive+valid
  (it is not in its ignore list), and plannerd is its only other publisher — without this, demo
  mode is a permanent commIssue and the car can never engage.
- ``demoControl``: scripted desired curvature, active only inside a maneuver's S-curve window.
  Outside the window (plain forward runs, the travel segment of a pull-over) lateral stays on the
  driving model. controlsd substitutes our curvature for the model's while active, keeping the
  closed-loop torque controller, curvature rate limits, and panda safety fully intact.

Commands arrive over the ``DemoCommand`` param (written by tools/demo/web.py) with a monotonically
increasing ``seq``; demod tracks the last seq it executed and never deletes the param, so a command
written mid-cycle cannot be lost to a read/remove race. Live state is published back on the
``DemoStatus`` param for the web UI.

Odometry is dead reckoning by integrating ``carState.vEgo`` — accurate to well under a foot at
these distances. All motion is bounded by hard caps (max speed, max distance, max scripted
curvature) and aborts on a radar lead, driver override, disengage, or a stale client heartbeat.
"""
import json
import math
import time
from dataclasses import dataclass
from enum import Enum

import numpy as np

from cereal import messaging, car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

FT_TO_M = 0.3048
M_TO_FT = 1.0 / FT_TO_M

# Hard safety caps (never exceeded regardless of the command).
MAX_SPEED_MS = 15.0 * CV.MPH_TO_MS      # ~6.7 m/s
MAX_DISTANCE_M = 300.0 * FT_TO_M        # per-command travel cap
# Peak scripted curvature cap, with margin under controlsd's MAX_CURVATURE=0.2 clamp
# (drive_helpers.py) so the torque controller can actually track the scripted path and the
# S-curve integrates back to zero heading. Commands whose geometry needs more are rejected.
MAX_SCRIPT_CURVATURE = 0.15             # 1/m (~6.7 m min radius)

# Comfort / tuning.
DEFAULT_CRUISE_MS = 5.0 * CV.MPH_TO_MS  # lot-speed default
ACCEL_MAX = 1.0                         # m/s^2 while speeding up
DECEL_MAX = 1.5                         # m/s^2 while slowing for the stop point
T_TRACK = 0.5                           # s, speed-tracking lookahead for accel command
STOP_DIST_M = 0.3                       # within this of the target we command a stop

# Pull-over / pull-out geometry (feet -> meters at command time).
# 50 ft run-out at the default 9 ft offset gives peak curvature 2*pi*2.74/15.24^2 ~= 0.074 1/m
# (~12 deg road-wheel angle) — comfortably trackable at lot speeds.
DEFAULT_RUNOUT_FT = 50.0                # longitudinal length of the S-curve
DEFAULT_OFFSET_FT = 9.0                 # lateral offset to reach the curb (~2.7 m)
DEFAULT_PULLOUT_FORWARD_FT = 50.0       # straight distance after pulling out

# Calibration constant mapping positive script curvature to "toward the right-hand curb" on this
# car. A negative offset_ft in a command flips the side (left-hand curb) — flip only this constant
# if the car steers the wrong way with positive offsets.
CURB_CURVATURE_SIGN = 1.0

# Guardrails.
LEAD_ABORT_DIST_M = 6.0                 # abort if a radar lead is closer than this
WATCHDOG_TIMEOUT_S = 3.0                # abort if the client heartbeat goes stale mid-maneuver


class State(Enum):
  IDLE = "IDLE"
  ARMED = "ARMED"
  EXECUTING = "EXECUTING"
  DONE = "DONE"
  ABORT = "ABORT"


@dataclass
class Maneuver:
  kind: str          # "forward" | "pullover" | "pullout"
  target_m: float    # total forward distance for this command
  lat_start_m: float  # odometer position where the S-curve begins
  lat_len_m: float   # longitudinal length of the S-curve (0 => no scripted lateral)
  offset_m: float    # lateral offset achieved over the S-curve (signed)
  cruise_ms: float   # target cruise speed for this command

  def in_lat_window(self, odo: float) -> bool:
    """Scripted lateral applies only inside the S-curve; everywhere else the model steers."""
    if self.lat_len_m <= 0.0 or self.offset_m == 0.0:
      return False
    return self.lat_start_m <= odo <= self.lat_start_m + self.lat_len_m

  def curvature(self, odo: float) -> float:
    """Scripted desired curvature at the current odometer reading.

    Uses a single sinusoidal curvature over the run-out: kappa(s) = A * sin(2*pi*s/L).
    This produces net-zero heading change and a pure lateral offset of A*L^2/(2*pi),
    i.e. two smooth opposing arcs (an S).
    """
    if not self.in_lat_window(odo):
      return 0.0
    s = odo - self.lat_start_m
    amp = 2.0 * np.pi * self.offset_m / (self.lat_len_m ** 2)
    return float(amp * np.sin(2.0 * np.pi * s / self.lat_len_m))


def build_maneuver(cmd: dict) -> tuple[Maneuver | None, str]:
  """Translate a command dict into a Maneuver, or (None, reason) if invalid/out of bounds."""

  def num(key, default):
    v = cmd.get(key, default)
    try:
      f = float(v)
    except (TypeError, ValueError):
      return None
    # reject NaN/inf: json.loads accepts NaN and np.clip propagates it into actuator commands
    return f if math.isfinite(f) else None

  kind = cmd.get("type")
  cruise = num("cruise_mph", DEFAULT_CRUISE_MS * CV.MS_TO_MPH)
  if cruise is None or not 0.0 < cruise * CV.MPH_TO_MS <= MAX_SPEED_MS:
    return None, "bad cruise speed"
  cruise *= CV.MPH_TO_MS

  def s_curve_ok(offset_m: float, runout_m: float) -> bool:
    return abs(2.0 * np.pi * offset_m) / (runout_m ** 2) <= MAX_SCRIPT_CURVATURE

  if kind == "forward":
    dist = num("distance_ft", None)
    if dist is None or not 0.0 < dist * FT_TO_M <= MAX_DISTANCE_M:
      return None, "bad distance"
    return Maneuver("forward", dist * FT_TO_M, 0.0, 0.0, 0.0, cruise), ""

  if kind in ("pullover", "pullout"):
    runout = num("runout_ft", DEFAULT_RUNOUT_FT)
    offset = num("offset_ft", DEFAULT_OFFSET_FT)
    straight = num("travel_ft" if kind == "pullover" else "forward_ft",
                   50.0 if kind == "pullover" else DEFAULT_PULLOUT_FORWARD_FT)
    if runout is None or offset is None or straight is None or runout <= 0.0 or straight < 0.0 or offset == 0.0:
      return None, "bad geometry"
    runout, offset, straight = runout * FT_TO_M, offset * FT_TO_M, straight * FT_TO_M
    if not s_curve_ok(offset, runout):
      return None, f"offset too sharp: need runout >= {math.sqrt(2.0 * np.pi * abs(offset) / MAX_SCRIPT_CURVATURE) * M_TO_FT:.0f} ft"
    target = straight + runout
    if not 0.0 < target <= MAX_DISTANCE_M:
      return None, "bad distance"
    # positive offset_ft steers toward the curb side (CURB_CURVATURE_SIGN); pullout mirrors it
    side = CURB_CURVATURE_SIGN if kind == "pullover" else -CURB_CURVATURE_SIGN
    lat_start = straight if kind == "pullover" else 0.0
    return Maneuver(kind, target, lat_start, runout, offset * side, cruise), ""

  return None, "unknown command"


class DemoController:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.state = State.IDLE
    self.maneuver: Maneuver | None = None
    self.odo = 0.0
    self.fault = ""
    # skip any command that predates this process (e.g. left over from before a restart)
    self.last_seq = self._peek_seq()

  # --- command intake -------------------------------------------------------
  def _peek_seq(self) -> int:
    raw = self.params.get("DemoCommand")
    if raw:
      try:
        return int(json.loads(raw).get("seq", 0))
      except (ValueError, TypeError):
        pass
    return 0

  def _read_command(self) -> dict | None:
    # seq-gated, never removed: a concurrent write from web.py cannot be deleted unread
    raw = self.params.get("DemoCommand")
    if not raw:
      return None
    try:
      cmd = json.loads(raw)
      seq = int(cmd.get("seq", 0))
    except (ValueError, TypeError):
      cloudlog.exception("demod: bad DemoCommand")
      return None
    if seq == self.last_seq:
      return None
    self.last_seq = seq
    return cmd

  def _client_alive(self) -> bool:
    raw = self.params.get("DemoHeartbeat")
    if not raw:
      return False
    try:
      # CLOCK_MONOTONIC is system-wide, so it is comparable with the web server process
      return (time.monotonic() - float(raw)) < WATCHDOG_TIMEOUT_S
    except (ValueError, TypeError):
      return False

  # --- guardrails -----------------------------------------------------------
  def _check_guardrails(self, sm) -> str:
    if not sm['carControl'].longActive:
      return "disengaged"
    cs = sm['carState']
    if cs.gasPressed or cs.brakePressed or cs.steeringPressed:
      return "driver override"
    lead = sm['radarState'].leadOne
    if lead.status and lead.dRel < LEAD_ABORT_DIST_M:
      return "lead detected"
    if not self._client_alive():
      return "client lost"
    return ""

  # --- longitudinal profile -------------------------------------------------
  def _accel_to_stop_at(self, target_m: float, v_ego: float, cruise_ms: float) -> tuple[float, bool]:
    dist_remaining = target_m - self.odo
    # decel-limited approach speed so we arrive stopped at the target
    v_curve = np.sqrt(2.0 * DECEL_MAX * max(dist_remaining, 0.0))
    v_target = float(np.clip(min(cruise_ms, v_curve), 0.0, cruise_ms))
    accel = float(np.clip((v_target - v_ego) / T_TRACK, -DECEL_MAX, ACCEL_MAX))
    should_stop = dist_remaining <= STOP_DIST_M and v_ego < self.CP.vEgoStopping
    return accel, should_stop

  # --- main step ------------------------------------------------------------
  def update(self, sm) -> tuple[float, bool, bool, float]:
    """Advance the state machine. Returns (accel, should_stop, lat_active, curvature)."""
    v_ego = max(sm['carState'].vEgo, 0.0)
    long_active = sm['carControl'].longActive

    cmd = self._read_command()
    if cmd is not None:
      self._handle_command(cmd)

    accel, should_stop, lat_active, curvature = 0.0, True, False, 0.0

    if self.state == State.ARMED:
      # wait until the safety driver has engaged, then start from the current position
      if long_active:
        self.odo = 0.0
        self.state = State.EXECUTING
        cloudlog.info(f"demod: EXECUTING {self.maneuver.kind} target={self.maneuver.target_m * M_TO_FT:.0f}ft")

    if self.state == State.EXECUTING:
      self.odo += v_ego * DT_MDL
      fault = self._check_guardrails(sm)
      if fault:
        self.fault = fault
        self.state = State.ABORT
        cloudlog.warning(f"demod: ABORT ({fault})")
      else:
        accel, should_stop = self._accel_to_stop_at(self.maneuver.target_m, v_ego, self.maneuver.cruise_ms)
        # scripted lateral only inside the S-curve; the model steers everywhere else
        lat_active = self.maneuver.in_lat_window(self.odo)
        curvature = self.maneuver.curvature(self.odo)
        if self.odo >= self.maneuver.target_m and v_ego < self.CP.vEgoStopping:
          self.state = State.DONE
          cloudlog.info("demod: DONE")

    if self.state == State.ABORT:  # not elif: applies on the same frame a guardrail trips
      # controlled stop; lateral is released to the model (or the overriding driver)
      accel, should_stop = self._accel_to_stop_at(self.odo, v_ego, self.maneuver.cruise_ms if self.maneuver else 0.0)
      if v_ego < self.CP.vEgoStopping:
        should_stop = True
        self.state = State.IDLE  # fault stays visible in status until the next command

    return accel, should_stop, lat_active, curvature

  def _handle_command(self, cmd: dict) -> None:
    kind = cmd.get("type")
    if kind == "abort":
      if self.state in (State.ARMED, State.EXECUTING):
        self.fault = "operator abort"
        self.state = State.ABORT
        cloudlog.warning("demod: ABORT (operator)")
      return

    # only accept a new motion when no maneuver is in flight
    if self.state not in (State.IDLE, State.DONE):
      cloudlog.warning(f"demod: ignoring {kind} while {self.state.value}")
      return

    maneuver, err = build_maneuver(cmd)
    if maneuver is None:
      self.fault = f"rejected: {err}"
      cloudlog.warning(f"demod: rejected command {cmd}: {err}")
      return

    self.maneuver = maneuver
    self.odo = 0.0
    self.fault = ""
    self.state = State.ARMED
    cloudlog.info(f"demod: ARMED {maneuver.kind}")

  def status(self, v_ego: float) -> dict:
    return {
      "state": self.state.value,
      "kind": self.maneuver.kind if self.maneuver else "",
      "odo_ft": round(self.odo * M_TO_FT, 1),
      "target_ft": round(self.maneuver.target_m * M_TO_FT, 1) if self.maneuver else 0.0,
      "offset_ft": round(self.maneuver.offset_m * M_TO_FT, 1) if self.maneuver else 0.0,
      "v_mph": round(v_ego * CV.MS_TO_MPH, 1),
      "fault": self.fault,
    }


def main():
  params = Params()
  cloudlog.info("demod is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)

  # poll on carState (card, 100 Hz) rather than modelV2 so a modeld stall can't stop our
  # longitudinalPlan stream (which selfdrived watches for commIssue)
  sm = messaging.SubMaster(['carState', 'carControl', 'controlsState', 'selfdriveState', 'radarState'],
                           poll='carState')
  pm = messaging.PubMaster(['longitudinalPlan', 'longitudinalPlanSP', 'demoControl', 'driverAssistance', 'alertDebug'])

  ctrl = DemoController(CP)
  frame = 0
  last_status = ""

  while True:
    sm.update()
    if not sm.updated['carState']:
      continue
    frame += 1
    if frame % 5 != 0:  # 100 Hz carState -> 20 Hz plan, one step per DT_MDL
      continue

    v_ego = max(sm['carState'].vEgo, 0.0)
    accel, should_stop, lat_active, curvature = ctrl.update(sm)

    # longitudinalPlan — validity over the same service subset plannerd uses, so a transient
    # modelV2/carControl hiccup can't invalidate the plan and soft-disable mid-demo
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])
    lp = plan_send.longitudinalPlan
    lp.aTarget = accel
    lp.shouldStop = should_stop
    lp.allowBrake = True
    lp.allowThrottle = True
    lp.hasLead = True
    lp.speeds = [0.2]  # nominal non-empty trajectory for UI consumers; resume is driven by shouldStop
    pm.send('longitudinalPlan', plan_send)

    # longitudinalPlanSP keep-alive: selfdrived requires this service alive+valid to engage;
    # its consumers (speed-limit assist / ICBM) treat the default contents as "no data"
    sp_send = messaging.new_message('longitudinalPlanSP')
    sp_send.valid = True
    pm.send('longitudinalPlanSP', sp_send)

    # demoControl (scripted lateral)
    demo_send = messaging.new_message('demoControl')
    demo_send.valid = True
    dc = demo_send.demoControl
    dc.active = lat_active
    dc.desiredCurvature = float(curvature)
    dc.state = ctrl.state.value
    dc.odometer = float(ctrl.odo)
    pm.send('demoControl', demo_send)

    # driverAssistance (kept valid so controlsd's hud checks pass, like maneuversd)
    assistance_send = messaging.new_message('driverAssistance')
    assistance_send.valid = True
    pm.send('driverAssistance', assistance_send)

    # on-screen overlay
    alert_send = messaging.new_message('alertDebug')
    alert_send.valid = True
    st = ctrl.status(v_ego)
    alert_send.alertDebug.alertText1 = f"DEMO {st['state']}  {st['odo_ft']:.0f}/{st['target_ft']:.0f} ft"
    alert_send.alertDebug.alertText2 = st['fault'] if st['fault'] else (st['kind'] or 'ready')
    pm.send('alertDebug', alert_send)

    # status for the web UI: at most 4 Hz, and only when something changed (params.put fsyncs)
    if frame % 25 == 0:
      status_json = json.dumps(st)
      if status_json != last_status:
        last_status = status_json
        params.put("DemoStatus", status_json)


if __name__ == "__main__":
  main()
