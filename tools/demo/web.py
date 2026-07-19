#!/usr/bin/env python3
"""
Curbside demo web server.

aiohttp server that runs on the comma 4 when ``DemoScriptMode`` is set. It exposes the scripted
motions as HTTP endpoints (triggerable from a phone on the same WiFi, or by the offboard AI planner
which POSTs the identical endpoints with computed parameters) and serves a phone-friendly control
page with a big ABORT button and live status.

Auth: every POST requires the ``X-Demo-Token`` header matching the ``DemoAuthToken`` param. The
token is generated on first start and logged; fetch it once over SSH with
``python3 -c "from openpilot.common.params import Params; print(Params().get('DemoAuthToken'))"``
and enter it in the control page (it persists in the browser).

Heartbeat contract: issuing a motion command returns a ``cmd_id``; the commanding client POSTs
``/heartbeat {"cmd_id": ...}`` periodically. Maneuvers execute to completion regardless of the
connection (they are bounded and end stopped) — the heartbeat only tethers the unbounded CRUISE
hold, which demod gracefully stops ~6 s after the last beat. Only the heartbeat of the client
driving the *active* command counts — status polls do not feed it.

Commands are handed to demod via the ``DemoCommand`` param with a monotonically increasing ``seq``
(demod consumes by sequence number, so no read/remove race); live state is read back from
``DemoStatus``.
"""
import json
import math
import secrets
import time

from aiohttp import web

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

DEMODIR = f"{BASEDIR}/tools/demo"
PORT = 5555

params = Params()

with open(f"{DEMODIR}/static/index.html") as f:
  INDEX_HTML = f.read()

# in-memory session state (single-process server)
_active_cmd_id: int | None = None


def _get_token() -> str:
  token = params.get("DemoAuthToken")
  if not token:
    token = secrets.token_urlsafe(16)
    params.put("DemoAuthToken", token)
  return token


TOKEN = None  # set in main() after Params is writable


def _beat() -> None:
  # CLOCK_MONOTONIC is system-wide; demod compares it against WATCHDOG_TIMEOUT_S.
  # Non-blocking: a blocking put fsyncs, and onroad (loggerd writing three video streams)
  # that can stall the whole aiohttp event loop long enough to starve the watchdog.
  params.put_nonblocking("DemoHeartbeat", str(time.monotonic()))


_seq = int(time.monotonic() * 1000)  # unique across web.py restarts within a boot


def _send_command(cmd: dict) -> int:
  global _seq
  _seq += 1
  cmd["seq"] = _seq
  params.put("DemoCommand", cmd)  # JSON param: put takes the dict itself
  return _seq


@web.middleware
async def auth_middleware(request: 'web.Request', handler):
  if request.method == "POST" and request.headers.get("X-Demo-Token") != TOKEN:
    return web.json_response({"error": "missing or bad X-Demo-Token"}, status=401)
  return await handler(request)


def _num(body: dict, key: str) -> float | None:
  """Optional finite float from client JSON; raises 400 on garbage (incl. NaN/inf)."""
  if key not in body:
    return None
  try:
    v = float(body[key])
  except (TypeError, ValueError):
    v = math.nan
  if not math.isfinite(v):
    raise web.HTTPBadRequest(text=json.dumps({"error": f"{key} must be a finite number"}),
                             content_type="application/json")
  return v


def _motion_command(kind: str, body: dict, keys: list[str], extra: dict | None = None) -> 'web.Response':
  global _active_cmd_id
  # pass through only the keys the client provided — demod owns the defaults
  cmd: dict = {"type": kind, **(extra or {})}
  for key in keys:
    v = _num(body, key)
    if v is not None:
      cmd[key] = v
  cmd_id = _send_command(cmd)
  _active_cmd_id = cmd_id
  _beat()
  return web.json_response({"status": "ok", "cmd_id": cmd_id})


async def index(request: 'web.Request'):
  return web.Response(content_type="text/html", text=INDEX_HTML)


async def status(request: 'web.Request'):
  # read-only; deliberately does NOT feed the watchdog (only /heartbeat from the commanding
  # client does, so an observer's status poll can't mask a dead operator)
  data = params.get("DemoStatus")  # JSON param: parsed dict (or None)
  return web.json_response(data if isinstance(data, dict) else {})


async def forward(request: 'web.Request'):
  return _motion_command("forward", await _json(request), ["distance_ft", "cruise_mph"])


async def pullover(request: 'web.Request'):
  return _motion_command("pullover", await _json(request), ["travel_ft", "offset_ft", "runout_ft", "cruise_mph"])


async def pullout(request: 'web.Request'):
  return _motion_command("pullout", await _json(request), ["forward_ft", "offset_ft", "runout_ft", "cruise_mph"])


def _direction(body: dict) -> str:
  d = body.get("direction")
  if d not in ("left", "right"):
    raise web.HTTPBadRequest(text=json.dumps({"error": "direction must be 'left' or 'right'"}),
                             content_type="application/json")
  return d


async def lanechange(request: 'web.Request'):
  body = await _json(request)
  return _motion_command("lanechange", body, ["cruise_mph"], extra={"direction": _direction(body)})


async def turn(request: 'web.Request'):
  body = await _json(request)
  return _motion_command("turn", body, ["cruise_mph", "hold_s"], extra={"direction": _direction(body)})


async def stop(request: 'web.Request'):
  global _active_cmd_id
  _send_command({"type": "stop"})
  _active_cmd_id = None
  _beat()
  return web.json_response({"status": "ok"})


async def heartbeat(request: 'web.Request'):
  body = await _json(request)
  cmd_id = body.get("cmd_id")
  if _active_cmd_id is not None and cmd_id == _active_cmd_id:
    _beat()
    return web.json_response({"status": "ok"})
  cloudlog.warning(f"demoweb: stale heartbeat rejected: got cmd_id={cmd_id!r}, active={_active_cmd_id!r}")
  return web.json_response({"status": "stale", "active_cmd_id": _active_cmd_id})


async def abort(request: 'web.Request'):
  global _active_cmd_id
  _send_command({"type": "abort"})
  _active_cmd_id = None
  _beat()  # let demod act on the abort itself rather than racing the watchdog
  return web.json_response({"status": "ok"})


async def _json(request: 'web.Request') -> dict:
  if request.can_read_body:
    try:
      body = await request.json()
      return body if isinstance(body, dict) else {}
    except (ValueError, json.JSONDecodeError):
      return {}
  return {}


def main():
  global TOKEN
  TOKEN = _get_token()
  cloudlog.info(f"demoweb auth token: {TOKEN}")

  app = web.Application(middlewares=[auth_middleware])
  app.router.add_get("/", index)
  app.router.add_get("/status", status)
  app.router.add_post("/demo/forward", forward)
  app.router.add_post("/demo/pullover", pullover)
  app.router.add_post("/demo/pullout", pullout)
  app.router.add_post("/demo/lanechange", lanechange)
  app.router.add_post("/demo/turn", turn)
  app.router.add_post("/demo/stop", stop)
  app.router.add_post("/heartbeat", heartbeat)
  app.router.add_post("/abort", abort)
  app.router.add_static('/static', f"{DEMODIR}/static")
  cloudlog.info(f"demoweb listening on :{PORT}")
  web.run_app(app, access_log=None, host="0.0.0.0", port=PORT)


if __name__ == "__main__":
  main()
