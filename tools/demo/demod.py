#!/usr/bin/env python3
"""
Curbside demo daemon.

Replaces plannerd when the ``DemoScriptMode`` param is set (see system/manager/process_config.py).
Runs a distance-based state machine that publishes:

- ``longitudinalPlan``: scripted speed/accel (drive forward N feet then stop, or hold a cruise
  speed during model-executed maneuvers). ``shouldStop=False`` while moving makes controlsd emit
  ``cruiseControl.resume`` automatically, so a remote "go" from a standstill works with stock
  controls once the car is engaged.
- ``longitudinalPlanSP``: empty-but-valid keep-alive. selfdrived requires this service alive+valid
  (it is not in its ignore list), and plannerd is its only other publisher — without this, demo
  mode is a permanent commIssue and the car can never engage.
- ``demoControl``: scripted desired curvature (active only inside a maneuver's S-curve window) and
  the commanded model desire for lane changes / turns. modeld ORs the desire in as a *virtual
  blinker* (see desire_helper.apply_demo_desire) — the model executes the maneuver with real
  perception and every existing gate (speed window, blind spot, ALC mode) still applies.

Maneuver kinds:
  scripted (Plan 1): forward, pullover, pullout — deterministic, end stopped (DONE).
  model-executed (Plan 2): lanechange, turn — demod holds a cruise speed and injects the desire;
  the driving model steers. These end in CRUISE (speed held, model lateral) so the demo can chain
  maneuvers; send ``stop`` for a controlled stop or another command to continue.

Commands arrive over the ``DemoCommand`` param (written by tools/demo/web.py) with a monotonically
increasing ``seq``; demod tracks the last seq it executed and never deletes the param, so a command
written mid-cycle cannot be lost to a read/remove race. Live state is published back on the
``DemoStatus`` param for the web UI.

Odometry is dead reckoning by integrating ``carState.vEgo``. All motion is bounded by hard caps
(max speed, max distance, max scripted curvature) and aborts on a close radar lead, driver
override, or disengage. Maneuvers execute to completion independent of the client connection
(they are distance/time-bounded and end stopped); the client heartbeat only tethers the
unbounded CRUISE hold, which gracefully stops if the client disappears.
"""
import json
import math
import time
from dataclasses import dataclass
from enum import Enum

import numpy as np

from cereal import messaging, car, log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

FT_TO_M = 0.3048
M_TO_FT = 1.0 / FT_TO_M

LaneChangeState = log.LaneChangeState

# Hard safety caps (never exceeded regardless of the command).
MAX_SPEED_MS = 15.0 * CV.MPH_TO_MS      # scripted (lot) maneuvers
MAX_DESIRE_SPEED_MS = 30.0 * CV.MPH_TO_MS  # model-executed street maneuvers (lane change needs >=20)
MAX_DISTANCE_M = 300.0 * FT_TO_M        # per-command travel cap for scripted maneuvers
# Peak scripted curvature cap, with margin under controlsd's MAX_CURVATURE=0.2 clamp
# (drive_helpers.py) so the torque controller can actually track the scripted path and the
# S-curve integrates back to zero heading. Commands whose geometry needs more are rejected.
MAX_SCRIPT_CURVATURE = 0.15             # 1/m (~6.7 m min radius)

# Comfort / tuning.
DEFAULT_CRUISE_MS = 5.0 * CV.MPH_TO_MS  # lot-speed default for scripted maneuvers
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

# Model-executed (desire) maneuvers.
LANE_CHANGE_MIN_MS = 21.0 * CV.MPH_TO_MS   # DesireHelper's gate is 20 mph; margin so it triggers
DEFAULT_LANE_CHANGE_CRUISE_MPH = 25.0
LANE_CHANGE_TIMEOUT_S = 20.0               # abort if the model never starts/finishes the change
DEFAULT_TURN_CRUISE_MPH = 8.0
DEFAULT_TURN_HOLD_S = 8.0                  # how long the turn desire is held
MAX_TURN_HOLD_S = 15.0

# Calibration constant mapping positive script curvature to "toward the right-hand curb" on this
# car. A negative offset_ft in a command flips the side (left-hand curb) — flip only this constant
# if the car steers the wrong way with positive offsets.
CURB_CURVATURE_SIGN = 1.0

# Guardrails.
LEAD_ABORT_DIST_M = 6.0                 # abort floor: radar lead closer than this
LEAD_ABORT_HEADWAY_S = 1.0              # scales the abort distance with speed for street maneuvers
WATCHDOG_TIMEOUT_S = 6.0                # abort if the client heartbeat goes stale mid-maneuver
                                        # (client beats at 2 Hz; window absorbs WiFi/eMMC hiccups)


class State(Enum):
  IDLE = "IDLE"
  ARMED = "ARMED"
  EXECUTING = "EXECUTING"
  CRUISE = "CRUISE"       # desire maneuver finished; holding speed on model lateral
  STOPPING = "STOPPING"   # operator-requested controlled stop (no fault)
  DONE = "DONE"
  ABORT = "ABORT"


DESIRE_BY_KIND = {
  ("lanechange", "left"): "laneChangeLeft",
  ("lanechange", "right"): "laneChangeRight",
  ("turn", "left"): "turnLeft",
  ("turn", "right"): "turnRight",
}


@dataclass
class Maneuver:
  kind: str          # "forward" | "pullover" | "pullout" | "lanechange" | "turn"
  target_m: float    # total forward distance (scripted kinds; 0 for desire kinds)
  lat_start_m: float  # odometer position where the S-curve begins
  lat_len_m: float   # longitudinal length of the S-curve (0 => no scripted lateral)
  offset_m: float    # lateral offset achieved over the S-curve (signed)
  cruise_ms: float   # target cruise speed for this command
  direction: str = ""  # desire kinds: "left" | "right"
  hold_s: float = 0.0  # turn: how long to hold the desire

  @property
  def is_desire(self) -> bool:
    return self.kind in ("lanechange", "turn")

  @property
  def desire(self) -> str:
    return DESIRE_BY_KIND.get((self.kind, self.direction), "")

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

  if kind in ("lanechange", "turn"):
    direction = cmd.get("direction")
    if direction not in ("left", "right"):
      return None, "direction must be left or right"
    default_cruise = DEFAULT_LANE_CHANGE_CRUISE_MPH if kind == "lanechange" else DEFAULT_TURN_CRUISE_MPH
    cruise = num("cruise_mph", default_cruise)
    if cruise is None or not 0.0 < cruise * CV.MPH_TO_MS <= MAX_DESIRE_SPEED_MS:
      return None, "bad cruise speed"
    cruise *= CV.MPH_TO_MS
    if kind == "lanechange" and cruise < LANE_CHANGE_MIN_MS:
      return None, f"lane change needs cruise >= {LANE_CHANGE_MIN_MS * CV.MS_TO_MPH:.0f} mph"
    hold_s = num("hold_s", DEFAULT_TURN_HOLD_S)
    if hold_s is None or not 0.0 < hold_s <= MAX_TURN_HOLD_S:
      return None, "bad hold_s"
    return Maneuver(kind, 0.0, 0.0, 0.0, 0.0, cruise, direction=direction, hold_s=hold_s), ""

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
    self.elapsed = 0.0
    self.seen_lane_change = False
    self.fault = ""
    # skip any command that predates this process (e.g. left over from before a restart)
    self.last_seq = self._peek_seq()

  # --- command intake -------------------------------------------------------
  def _peek_seq(self) -> int:
    cmd = self.params.get("DemoCommand")  # JSON param: Params.get returns a parsed dict (or None)
    if isinstance(cmd, dict):
      try:
        return int(cmd.get("seq", 0))
      except (ValueError, TypeError):
        pass
    return 0

  def _read_command(self) -> dict | None:
    # seq-gated, never removed: a concurrent write from web.py cannot be deleted unread
    cmd = self.params.get("DemoCommand")  # JSON param: parsed dict (or None)
    if not isinstance(cmd, dict):
      return None
    try:
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

  def _desire_prereqs_ok(self, kind: str) -> str:
    """Model-executed maneuvers need their sunnypilot features enabled; reject with guidance."""
    if kind == "lanechange":
      try:
        timer = int(self.params.get("AutoLaneChangeTimer", return_default=True) or 0)
      except (ValueError, TypeError):
        timer = 0
      if timer < 1:  # AutoLaneChangeMode.NUDGELESS
        return "enable nudgeless auto lane change (AutoLaneChangeTimer >= 1)"
    elif kind == "turn":
      if not self.params.get_bool("LaneTurnDesire"):
        return "enable LaneTurnDesire param"
    return ""

  # --- guardrails -----------------------------------------------------------
  def _check_guardrails(self, sm) -> str:
    if not sm['carControl'].longActive:
      return "disengaged"
    cs = sm['carState']
    if cs.gasPressed or cs.brakePressed or cs.steeringPressed:
      return "driver override"
    lead = sm['radarState'].leadOne
    # headway-scaled: 6 m floor for lot speeds, ~1 s of headway at street speeds. demod has no
    # car-following — desire demos assume a clear road; this is the backstop, not ACC.
    if lead.status and lead.dRel < max(LEAD_ABORT_DIST_M, cs.vEgo * LEAD_ABORT_HEADWAY_S):
      return "lead detected"
    # deliberately NO client-heartbeat check here: maneuvers are distance/time-bounded and end
    # stopped on their own, so they run to completion even if WiFi drops. The heartbeat only
    # tethers the unbounded CRUISE hold (see the CRUISE state in update()).
    return ""

  # --- longitudinal profiles ------------------------------------------------
  def _accel_to_stop_at(self, target_m: float, v_ego: float, cruise_ms: float) -> tuple[float, bool]:
    dist_remaining = target_m - self.odo
    # decel-limited approach speed so we arrive stopped at the target
    v_curve = np.sqrt(2.0 * DECEL_MAX * max(dist_remaining, 0.0))
    v_target = float(np.clip(min(cruise_ms, v_curve), 0.0, cruise_ms))
    accel = float(np.clip((v_target - v_ego) / T_TRACK, -DECEL_MAX, ACCEL_MAX))
    should_stop = dist_remaining <= STOP_DIST_M and v_ego < self.CP.vEgoStopping
    return accel, should_stop

  def _accel_speed_hold(self, v_ego: float, cruise_ms: float) -> float:
    return float(np.clip((cruise_ms - v_ego) / T_TRACK, -DECEL_MAX, ACCEL_MAX))

  # --- main step ------------------------------------------------------------
  def update(self, sm) -> tuple[float, bool, bool, float, str]:
    """Advance the state machine. Returns (accel, should_stop, lat_active, curvature, desire)."""
    v_ego = max(sm['carState'].vEgo, 0.0)
    long_active = sm['carControl'].longActive

    cmd = self._read_command()
    if cmd is not None:
      self._handle_command(cmd)

    accel, should_stop, lat_active, curvature, desire = 0.0, True, False, 0.0, ""

    if self.state == State.ARMED:
      # wait until the safety driver has engaged, then start from the current position
      if long_active:
        self.odo = 0.0
        self.elapsed = 0.0
        self.seen_lane_change = False
        self.state = State.EXECUTING
        cloudlog.info(f"demod: EXECUTING {self.maneuver.kind}")

    if self.state == State.EXECUTING:
      self.odo += v_ego * DT_MDL
      self.elapsed += DT_MDL
      fault = self._check_guardrails(sm)
      if fault:
        self.fault = fault
        self.state = State.ABORT
        cloudlog.warning(f"demod: ABORT ({fault})")
      elif self.maneuver.is_desire:
        # model-executed: hold speed, inject the desire, let the driving model steer
        accel = self._accel_speed_hold(v_ego, self.maneuver.cruise_ms)
        should_stop = False
        desire = self.maneuver.desire
        if self.maneuver.kind == "lanechange":
          lcs = sm['modelV2'].meta.laneChangeState
          if lcs in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
            self.seen_lane_change = True
          if self.seen_lane_change and lcs == LaneChangeState.off:
            self.state = State.CRUISE
            cloudlog.info("demod: lane change complete -> CRUISE")
          elif self.elapsed > LANE_CHANGE_TIMEOUT_S:
            self.fault = "lane change not executed (blocked or below speed)"
            self.state = State.ABORT
        else:  # turn: hold the desire for hold_s, then hand back to the model
          if self.elapsed > self.maneuver.hold_s:
            self.state = State.CRUISE
            cloudlog.info("demod: turn hold done -> CRUISE")
      else:
        accel, should_stop = self._accel_to_stop_at(self.maneuver.target_m, v_ego, self.maneuver.cruise_ms)
        # scripted lateral only inside the S-curve; the model steers everywhere else
        lat_active = self.maneuver.in_lat_window(self.odo)
        curvature = self.maneuver.curvature(self.odo)
        if self.odo >= self.maneuver.target_m and v_ego < self.CP.vEgoStopping:
          self.state = State.DONE
          cloudlog.info("demod: DONE")

    if self.state == State.CRUISE:
      # speed held on model lateral until the next command (stop / another maneuver)
      self.odo += v_ego * DT_MDL
      fault = self._check_guardrails(sm)
      if fault:
        self.fault = fault
        self.state = State.ABORT
        cloudlog.warning(f"demod: ABORT ({fault})")
      elif not self._client_alive():
        # CRUISE is the only unbounded state (speed held until the next command), so it keeps
        # a connection tether: no heartbeat -> controlled stop, not an abort fault
        self.fault = "client lost"
        self.state = State.STOPPING
        cloudlog.warning("demod: client lost in CRUISE -> controlled stop")
      else:
        accel = self._accel_speed_hold(v_ego, self.maneuver.cruise_ms)
        should_stop = False

    if self.state in (State.STOPPING, State.ABORT):  # controlled stop; lateral on the model
      accel, should_stop = self._accel_to_stop_at(self.odo, v_ego, self.maneuver.cruise_ms if self.maneuver else 0.0)
      lat_active = False
      curvature = 0.0
      desire = ""
      if v_ego < self.CP.vEgoStopping:
        should_stop = True
        self.state = State.IDLE  # an abort's fault stays visible in status until the next command

    return accel, should_stop, lat_active, curvature, desire

  def _handle_command(self, cmd: dict) -> None:
    kind = cmd.get("type")
    if kind == "abort":
      if self.state in (State.ARMED, State.EXECUTING, State.CRUISE, State.STOPPING):
        self.fault = "operator abort"
        self.state = State.ABORT
        cloudlog.warning("demod: ABORT (operator)")
      return

    if kind == "stop":
      if self.state in (State.ARMED, State.EXECUTING, State.CRUISE):
        self.fault = ""
        self.state = State.STOPPING
        cloudlog.info("demod: STOPPING (operator)")
      return

    # accept a new motion when idle/finished, or chain one from CRUISE
    if self.state not in (State.IDLE, State.DONE, State.CRUISE):
      cloudlog.warning(f"demod: ignoring {kind} while {self.state.value}")
      return

    maneuver, err = build_maneuver(cmd)
    if maneuver is None:
      self.fault = f"rejected: {err}"
      cloudlog.warning(f"demod: rejected command {cmd}: {err}")
      return
    if maneuver.is_desire:
      prereq = self._desire_prereqs_ok(maneuver.kind)
      if prereq:
        self.fault = f"rejected: {prereq}"
        cloudlog.warning(f"demod: rejected {maneuver.kind}: {prereq}")
        return

    self.maneuver = maneuver
    self.odo = 0.0
    self.fault = ""
    self.state = State.ARMED
    cloudlog.info(f"demod: ARMED {maneuver.kind}")

  def status(self, v_ego: float) -> dict:
    m = self.maneuver
    return {
      "state": self.state.value,
      "kind": m.kind if m else "",
      "desire": m.desire if m and self.state == State.EXECUTING and m.is_desire else "",
      "odo_ft": round(self.odo * M_TO_FT, 1),
      "target_ft": round(m.target_m * M_TO_FT, 1) if m else 0.0,
      "offset_ft": round(m.offset_m * M_TO_FT, 1) if m else 0.0,
      "v_mph": round(v_ego * CV.MS_TO_MPH, 1),
      "fault": self.fault,
    }


def main():
  params = Params()
  cloudlog.info("demod is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)

  # poll on carState (card, 100 Hz) rather than modelV2 so a modeld stall can't stop our
  # longitudinalPlan stream (which selfdrived watches for commIssue); modelV2 is only read
  # for lane-change completion tracking
  sm = messaging.SubMaster(['carState', 'carControl', 'controlsState', 'selfdriveState', 'radarState', 'modelV2'],
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
    accel, should_stop, lat_active, curvature, desire = ctrl.update(sm)

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

    # demoControl: scripted lateral + injected desire (consumed by controlsd and modeld)
    demo_send = messaging.new_message('demoControl')
    demo_send.valid = True
    dc = demo_send.demoControl
    dc.active = lat_active
    dc.desiredCurvature = float(curvature)
    dc.state = ctrl.state.value
    dc.odometer = float(ctrl.odo)
    dc.desire = desire
    pm.send('demoControl', demo_send)

    # driverAssistance (kept valid so controlsd's hud checks pass, like maneuversd)
    assistance_send = messaging.new_message('driverAssistance')
    assistance_send.valid = True
    pm.send('driverAssistance', assistance_send)

    # on-screen overlay
    alert_send = messaging.new_message('alertDebug')
    alert_send.valid = True
    st = ctrl.status(v_ego)
    alert_send.alertDebug.alertText1 = f"DEMO {st['state']}  {st['odo_ft']:.0f} ft  {st['v_mph']:.0f} mph"
    alert_send.alertDebug.alertText2 = st['fault'] if st['fault'] else (st['desire'] or st['kind'] or 'ready')
    pm.send('alertDebug', alert_send)

    # status for the web UI: at most 4 Hz, and only when something changed (params.put fsyncs)
    if frame % 25 == 0:
      status_json = json.dumps(st)
      if status_json != last_status:
        last_status = status_json
        # JSON param: put takes the dict itself. Non-blocking: a blocking put fsyncs, which
        # under onroad IO load can stall this 20 Hz loop and starve longitudinalPlan.
        params.put_nonblocking("DemoStatus", st)


if __name__ == "__main__":
  main()
