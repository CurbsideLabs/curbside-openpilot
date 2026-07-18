# Curbside Demo — scripted motion

Scripted self-driving demo motions on the Palisade (HDA2, comma 4). See `PLAN.md` for the full
design and rationale. This directory implements the code portions of Phases 1–3 and the WiFi UI.

## What's here

| File | Role |
|---|---|
| `demod.py` | Daemon that **replaces plannerd** when `DemoScriptMode` is set. Runs the distance-based state machine (IDLE → ARMED → EXECUTING → DONE/ABORT), publishes `longitudinalPlan` (scripted accel), a `longitudinalPlanSP` keep-alive (required by selfdrived to engage), and `demoControl` (scripted curvature). |
| `web.py` | aiohttp server (port **5555**) exposing the trigger endpoints and serving the phone control page. All POSTs require the `X-Demo-Token` header. |
| `static/index.html` | Phone-friendly UI: token entry, big FORWARD / PULL OVER / PULL OUT buttons, live status, ABORT. |

Touch-points in the rest of the tree:
- `cereal/custom.capnp` / `log.capnp` / `services.py` — the `demoControl` message (reuses the reserved slot `@136`).
- `common/params_keys.h` — `DemoScriptMode`, `DemoCommand`, `DemoStatus`, `DemoHeartbeat`, `DemoAuthToken`.
- `system/manager/process_config.py` — gates plannerd off and demod/demoweb on when `DemoScriptMode` is set. `DemoScriptMode` takes precedence over `LongitudinalManeuverMode` (maneuversd will not start while demo mode is on — exactly one longitudinalPlan publisher at a time).
- `selfdrive/controls/controlsd.py` — small override: while `demoControl.active`, use its curvature instead of the model's. If demod dies mid-maneuver the current curvature is held (predictable arc) until selfdrived's commIssue disengages, rather than silently snapping back to the model's path. Curvature rate limits, the torque controller, and panda safety are untouched.

## Lateral behavior

Scripted curvature applies **only inside a maneuver's S-curve window**. Plain FORWARD runs and the
straight travel segment of a pull-over leave steering on the driving model (lane keeping / road
following, which works on unmarked roads). During an abort, lateral is released to the model and
the car brakes to a controlled stop.

## Enabling

```bash
# on the device
python3 -c "from openpilot.common.params import Params; Params().put_bool('DemoScriptMode', True)"
# fetch the API token (generated on first demoweb start, persistent)
python3 -c "from openpilot.common.params import Params; print(Params().get('DemoAuthToken'))"
```
`DemoScriptMode` clears on manager start and offroad transition, so it must be re-set per session
(same lifecycle as `LongitudinalManeuverMode`). The safety driver engages once (SET press); every
motion below is then remote-triggered (Phase 4 Stage A).

## Endpoints (`http://<device-ip>:5555`)

All POSTs require header `X-Demo-Token: <token>`. Distances in feet, speeds in mph; omitted fields
use demod's defaults (`DEFAULT_*` in `demod.py` — the single source of truth).

| Endpoint | Body | Purpose |
|---|---|---|
| `POST /demo/forward` | `{distance_ft, cruise_mph}` | Drive forward N feet (model steering), stop. |
| `POST /demo/pullover` | `{travel_ft, offset_ft, runout_ft, cruise_mph}` | Forward on model steering, then S-curve to the curb, stop. |
| `POST /demo/pullout` | `{forward_ft, offset_ft, runout_ft, cruise_mph}` | S-curve away from the curb, then forward on model steering, stop. |
| `POST /heartbeat` | `{cmd_id}` | Watchdog keep-alive for the **active** command (see below). |
| `POST /abort` | — | Immediate controlled stop. |
| `GET /status` | — | State machine + odometry + faults. Read-only; does **not** feed the watchdog. |

Motion commands return `{"status": "ok", "cmd_id": N}`. **Watchdog contract:** the commanding
client must POST `/heartbeat {"cmd_id": N}` at least every ~1 s until the maneuver reaches
DONE/IDLE, or demod aborts with `client lost` ~3 s after the last beat. Only the active command's
heartbeat counts — observers polling `/status` cannot mask a dead operator. The offboard AI planner
(Phase 5) must implement the same heartbeat loop.

`offset_ft` is signed: positive pulls toward the right-hand curb, negative toward the left
(`CURB_CURVATURE_SIGN` in `demod.py` calibrates which physical side "positive" is — verify on the
car before the first run).

## Guardrails (enforced in `demod`)

- Hard caps: max speed 15 mph, max distance 300 ft per command, max scripted curvature 0.15 1/m —
  commands whose offset/runout geometry needs sharper steering are rejected with the minimum
  feasible `runout_ft` in the fault message.
- Input validation rejects NaN/inf at both the web layer (HTTP 400) and demod.
- Abort on a radar lead within 6 m, driver gas/brake/steer override, disengage, or a stale client
  heartbeat (>3 s) mid-maneuver.
- Commands are consumed by sequence number (never deleted), so an ABORT can't be lost to a race.
- `shouldStop=False` while moving lets controlsd emit `cruiseControl.resume` automatically, so a
  remote "go" from a standstill works with stock controls once engaged.

## Tuning notes

- The S-curve is a single sinusoid `kappa(s) = A·sin(2π·s/L)` giving a net-zero heading change and
  a lateral offset of `A·L²/(2π)`. Defaults (9 ft offset over a 50 ft run-out) peak at
  `A ≈ 0.074 1/m` (~12° road-wheel angle) — comfortably inside the 0.15 cap. Shorter `runout_ft`
  is sharper; the cap rejects anything the torque controller couldn't track.
- Low-speed steering authority is the main unknown (plan Phase 3): tune `offset_ft`/`runout_ft`
  on the lot before street runs.
