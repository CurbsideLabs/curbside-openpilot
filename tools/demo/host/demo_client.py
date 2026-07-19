#!/usr/bin/env python3
"""
Curbside demo host client — runs on the laptop/phone side, talks to demoweb on the comma 4.

Implements the full command + watchdog contract from tools/demo/README.md:
every motion command returns a cmd_id, and the commanding client must POST
/heartbeat {"cmd_id": N} at least every ~1 s (this client beats at 2 Hz) until the maneuver ends, or demod
aborts with "client lost". This module owns that heartbeat loop in a background
thread so the AI agent (or a human at the CLI) only has to issue commands.

Library use (what the Curbside agent imports):

    from demo_client import DemoClient
    client = DemoClient("192.168.43.1", token="<DemoAuthToken>")
    client.forward(distance_ft=50)
    final = client.wait()            # blocks until DONE / IDLE / ABORT / CRUISE
    client.stop()                    # graceful stop (ends a CRUISE)

CLI use (manual testing / filming):

    ./demo_client.py --host 192.168.43.1 --token XXXX status
    ./demo_client.py --host 192.168.43.1 --token XXXX forward --distance-ft 50
    ./demo_client.py --host 192.168.43.1 --token XXXX lanechange left
    ./demo_client.py --host 192.168.43.1 --token XXXX stop | abort

Motion subcommands stay in the foreground, heartbeating and printing status until
the maneuver reaches a terminal state. Ctrl-C sends a graceful stop. If this
process dies for any reason, demod's watchdog stops the car within ~6 s.
"""
import argparse
import json
import sys
import threading
import time
import urllib.request
import urllib.error

TERMINAL_STATES = ("IDLE", "DONE", "ABORT")
CRUISE_STATES = ("CRUISE",)
HEARTBEAT_PERIOD_S = 0.5  # 2 Hz against demod's 6 s watchdog: several beats can drop before an abort


class DemoClientError(Exception):
  pass


class DemoClient:
  def __init__(self, host: str, token: str, port: int = 5555, timeout: float = 2.0):
    self.base = f"http://{host}:{port}"
    self.token = token
    self.timeout = timeout
    self._cmd_id: int | None = None
    self._lock = threading.Lock()
    self._hb_thread: threading.Thread | None = None
    self._hb_stop = threading.Event()

  # --- http ----------------------------------------------------------------
  def _request(self, method: str, path: str, body: dict | None = None) -> dict:
    data = json.dumps(body or {}).encode() if method == "POST" else None
    req = urllib.request.Request(self.base + path, data=data, method=method)
    req.add_header("Content-Type", "application/json")
    if method == "POST":
      req.add_header("X-Demo-Token", self.token)
    try:
      with urllib.request.urlopen(req, timeout=self.timeout) as resp:
        return json.loads(resp.read().decode() or "{}")
    except urllib.error.HTTPError as e:
      detail = e.read().decode(errors="replace")
      raise DemoClientError(f"{method} {path} -> HTTP {e.code}: {detail}") from e
    except (urllib.error.URLError, TimeoutError, OSError) as e:
      raise DemoClientError(f"{method} {path} failed: {e}") from e

  # --- heartbeat -----------------------------------------------------------
  def _heartbeat_loop(self):
    failures = 0
    while not self._hb_stop.wait(HEARTBEAT_PERIOD_S):
      with self._lock:
        cmd_id = self._cmd_id
      if cmd_id is None:
        continue
      try:
        resp = self._request("POST", "/heartbeat", {"cmd_id": cmd_id})
        if resp.get("status") != "ok":
          failures += 1
          if failures in (1, 10):
            print(f"\n[heartbeat] rejected by demoweb as {resp} — CRUISE would auto-stop; "
                  "scripted maneuvers unaffected", file=sys.stderr)
        else:
          failures = 0
      except DemoClientError as e:
        # transient WiFi drop: keep trying; maneuvers finish on their own, and demod
        # gracefully stops a CRUISE ~6 s after the last beat that landed
        failures += 1
        if failures in (3, 10):
          print(f"\n[heartbeat] {failures} consecutive failures ({e}) — check WiFi; "
                "scripted maneuvers unaffected", file=sys.stderr)

  def _start_heartbeat(self, cmd_id: int):
    with self._lock:
      self._cmd_id = cmd_id
    if self._hb_thread is None or not self._hb_thread.is_alive():
      self._hb_stop.clear()
      self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
      self._hb_thread.start()

  def _end_heartbeat(self):
    with self._lock:
      self._cmd_id = None
    self._hb_stop.set()

  # --- commands ------------------------------------------------------------
  def _motion(self, path: str, body: dict) -> int:
    resp = self._request("POST", path, {k: v for k, v in body.items() if v is not None})
    cmd_id = resp.get("cmd_id")
    if resp.get("status") != "ok" or cmd_id is None:
      raise DemoClientError(f"command rejected: {resp}")
    self._start_heartbeat(cmd_id)
    return cmd_id

  def forward(self, distance_ft: float, cruise_mph: float | None = None) -> int:
    return self._motion("/demo/forward", {"distance_ft": distance_ft, "cruise_mph": cruise_mph})

  def pullover(self, travel_ft: float | None = None, offset_ft: float | None = None,
               runout_ft: float | None = None, cruise_mph: float | None = None) -> int:
    return self._motion("/demo/pullover", {"travel_ft": travel_ft, "offset_ft": offset_ft,
                                           "runout_ft": runout_ft, "cruise_mph": cruise_mph})

  def pullout(self, forward_ft: float | None = None, offset_ft: float | None = None,
              runout_ft: float | None = None, cruise_mph: float | None = None) -> int:
    return self._motion("/demo/pullout", {"forward_ft": forward_ft, "offset_ft": offset_ft,
                                          "runout_ft": runout_ft, "cruise_mph": cruise_mph})

  def arcturn(self, direction: str, radius_ft: float | None = None, angle_deg: float | None = None,
              lead_ft: float | None = None, tail_ft: float | None = None,
              cruise_mph: float | None = None) -> int:
    """Scripted (dead-reckoned) turn from a stop: optional straight lead-in, constant-radius
    arc through angle_deg, straight tail-out, ends stopped. Deterministic — no model perception
    involved in the arc itself."""
    return self._motion("/demo/arcturn", {"direction": direction, "radius_ft": radius_ft,
                                          "angle_deg": angle_deg, "lead_ft": lead_ft,
                                          "tail_ft": tail_ft, "cruise_mph": cruise_mph})

  def lanechange(self, direction: str, cruise_mph: float | None = None) -> int:
    return self._motion("/demo/lanechange", {"direction": direction, "cruise_mph": cruise_mph})

  def turn(self, direction: str, hold_s: float | None = None, cruise_mph: float | None = None) -> int:
    return self._motion("/demo/turn", {"direction": direction, "hold_s": hold_s, "cruise_mph": cruise_mph})

  def stop(self) -> None:
    """Graceful controlled stop (no fault). Ends a CRUISE. Keeps no heartbeat afterwards."""
    self._request("POST", "/demo/stop")
    self._end_heartbeat()

  def abort(self) -> None:
    """Immediate controlled stop, recorded as an abort."""
    self._request("POST", "/abort")
    self._end_heartbeat()

  def status(self) -> dict:
    return self._request("GET", "/status")

  def wait(self, stop_states: tuple = TERMINAL_STATES + CRUISE_STATES,
           poll_s: float = 0.5, on_status=None) -> dict:
    """Block until demod reaches one of stop_states; returns the final status dict.

    CRUISE counts as done by default (a completed lane change / turn parks there,
    speed held) — chain the next command or call stop(). The heartbeat keeps
    running through CRUISE; it only ends on stop()/abort() or a terminal state.
    """
    st = {}
    while True:
      try:
        st = self.status()
      except DemoClientError:
        time.sleep(poll_s)
        continue
      if on_status:
        on_status(st)
      state = st.get("state", "")
      if state in stop_states:
        if state in TERMINAL_STATES:
          self._end_heartbeat()
        return st
      time.sleep(poll_s)

  def close(self):
    self._end_heartbeat()


# --- CLI --------------------------------------------------------------------

def _print_status(st: dict):
  turn = f"{st['turn_deg']:3.0f}° " if st.get("turn_deg") else ""
  line = (f"\r{st.get('state', '?'):9s} {st.get('kind', ''):10s} "
          f"{st.get('odo_ft', 0):6.1f} ft  {st.get('v_mph', 0):5.1f} mph  {turn}"
          f"{st.get('fault', '')}")
  sys.stdout.write(line.ljust(100))
  sys.stdout.flush()


def _run_motion(client: DemoClient, start):
  cmd_id = start()
  print(f"command accepted (cmd_id={cmd_id}); heartbeating — Ctrl-C for graceful stop")
  try:
    final = client.wait(on_status=_print_status)
    print()
    state = final.get("state")
    if state in CRUISE_STATES:
      print("maneuver complete, holding CRUISE — heartbeat continues; Ctrl-C (or `stop`) to stop the car")
      final = client.wait(stop_states=TERMINAL_STATES, on_status=_print_status)
      print()
    print(f"final: {json.dumps(final)}")
    return 0 if final.get("state") != "ABORT" else 1
  except KeyboardInterrupt:
    print("\nCtrl-C: sending graceful stop")
    client.stop()
    return 130


def main():
  p = argparse.ArgumentParser(description="Curbside demo host client")
  p.add_argument("--host", required=True, help="comma 4 IP (192.168.43.1 on device hotspot)")
  p.add_argument("--token", required=True, help="DemoAuthToken from the device")
  p.add_argument("--port", type=int, default=5555)
  sub = p.add_subparsers(dest="cmd", required=True)

  sub.add_parser("status")
  sub.add_parser("watch")
  sub.add_parser("stop")
  sub.add_parser("abort")

  f = sub.add_parser("forward")
  f.add_argument("--distance-ft", type=float, required=True)
  f.add_argument("--cruise-mph", type=float)

  po = sub.add_parser("pullover")
  po.add_argument("--travel-ft", type=float)
  po.add_argument("--offset-ft", type=float)
  po.add_argument("--runout-ft", type=float)
  po.add_argument("--cruise-mph", type=float)

  pu = sub.add_parser("pullout")
  pu.add_argument("--forward-ft", type=float)
  pu.add_argument("--offset-ft", type=float)
  pu.add_argument("--runout-ft", type=float)
  pu.add_argument("--cruise-mph", type=float)

  at = sub.add_parser("arcturn")
  at.add_argument("direction", choices=["left", "right"])
  at.add_argument("--radius-ft", type=float)
  at.add_argument("--angle-deg", type=float)
  at.add_argument("--lead-ft", type=float)
  at.add_argument("--tail-ft", type=float)
  at.add_argument("--cruise-mph", type=float)

  lc = sub.add_parser("lanechange")
  lc.add_argument("direction", choices=["left", "right"])
  lc.add_argument("--cruise-mph", type=float)

  tn = sub.add_parser("turn")
  tn.add_argument("direction", choices=["left", "right"])
  tn.add_argument("--hold-s", type=float)
  tn.add_argument("--cruise-mph", type=float)

  args = p.parse_args()
  client = DemoClient(args.host, args.token, port=args.port)

  try:
    if args.cmd == "status":
      print(json.dumps(client.status(), indent=2))
    elif args.cmd == "watch":
      while True:
        _print_status(client.status())
        time.sleep(0.5)
    elif args.cmd == "stop":
      client.stop()
      print("stop sent")
    elif args.cmd == "abort":
      client.abort()
      print("abort sent")
    elif args.cmd == "forward":
      return _run_motion(client, lambda: client.forward(args.distance_ft, args.cruise_mph))
    elif args.cmd == "pullover":
      return _run_motion(client, lambda: client.pullover(args.travel_ft, args.offset_ft, args.runout_ft, args.cruise_mph))
    elif args.cmd == "pullout":
      return _run_motion(client, lambda: client.pullout(args.forward_ft, args.offset_ft, args.runout_ft, args.cruise_mph))
    elif args.cmd == "arcturn":
      return _run_motion(client, lambda: client.arcturn(args.direction, args.radius_ft, args.angle_deg,
                                                        args.lead_ft, args.tail_ft, args.cruise_mph))
    elif args.cmd == "lanechange":
      return _run_motion(client, lambda: client.lanechange(args.direction, args.cruise_mph))
    elif args.cmd == "turn":
      return _run_motion(client, lambda: client.turn(args.direction, args.hold_s, args.cruise_mph))
  except KeyboardInterrupt:
    print()
  except DemoClientError as e:
    print(f"error: {e}", file=sys.stderr)
    return 1
  return 0


if __name__ == "__main__":
  sys.exit(main())
