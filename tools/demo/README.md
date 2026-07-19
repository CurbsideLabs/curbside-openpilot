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
- `selfdrive/controls/lib/desire_helper.py` + the three modeld variants — Plan 2 desire injection: `demoControl.desire` is OR'd in as a **virtual blinker** before `DesireHelper.update`, so lane changes and turns are executed by the driving model with every existing gate (lateral active, speed window, blind-spot block, AutoLaneChange mode, LaneTurnDesire ceiling) applying unchanged.

## Lateral behavior

Scripted curvature applies **only inside a maneuver's S-curve window**. By default, plain FORWARD
runs and the straight segments of a pull-over/pull-out leave steering on the driving model (lane
keeping / road following — good on real roads with structure). **On open lots with no lane lines
or edges the model hunts and the car weaves** — pass `hold_straight: true`
(`--hold-straight` in the CLI) to keep scripted lateral active through the straight segments
instead (curvature 0, wheel held straight). Note hold-straight is open-loop: it does not correct
heading drift, so square the car up before long runs. During an abort, lateral is released to
the model and the car brakes to a controlled stop.

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
| `POST /demo/arcturn` | `{direction, radius_ft, angle_deg, lead_ft, tail_ft, cruise_mph}` | **Scripted** (dead-reckoned) turn from a stop: optional straight lead-in, constant-radius arc through `angle_deg` (default 90°, 20–120°), straight tail-out, ends stopped. Deterministic — no model perception in the arc. Defaults: **75 ft radius** (floor ~73 ft: lot logs show stock `STEER_MAX` torque saturates at ~0.047 1/m at arc speed — tighter radii physically stall; raising the limit needs opendbc + panda firmware changes), 15 ft tail, **6 mph** (floor 5 mph: torque steering tracks lateral accel = curvature·v², so below ~5 mph the EPS has no error signal and unwinds mid-arc). A 90° turn sweeps ~115 ft of arc — plan the space accordingly. |
| `POST /demo/lanechange` | `{direction, cruise_mph}` | **Model-executed** lane change (Plan 2): demod holds cruise speed and injects the desire as a virtual blinker; the driving model plans and steers the change with all gates (≥21 mph, blind spot, ALC mode) intact. Ends in CRUISE. |
| `POST /demo/turn` | `{direction, hold_s, cruise_mph}` | **Model-executed** turn via sunnypilot LaneTurnDesire: desire held for `hold_s` (default 8 s) at low speed; the model executes. Ends in CRUISE. |
| `POST /demo/stop` | — | Graceful controlled stop (no fault) — ends a CRUISE. |
| `POST /heartbeat` | `{cmd_id}` | Keep-alive for the **active** command; only tethers the CRUISE hold (see below). |
| `POST /abort` | — | Immediate controlled stop, recorded as an abort. |
| `GET /status` | — | State machine + odometry + faults. Read-only; does **not** feed the heartbeat. |

**Model-executed maneuver prerequisites** (rejected with guidance otherwise): lane changes need
nudgeless auto-lane-change (`Params().put("AutoLaneChangeTimer", "1")`); turns need
`Params().put_bool("LaneTurnDesire", True)` (plus `LaneTurnValue` ≥ the turn cruise speed).
Desire maneuvers end in a **CRUISE** state — speed held, model steering — so maneuvers can be
chained; send `/demo/stop` (or the next command) to continue. **Clear-road assumption:** demod has
no car-following; the headway-scaled lead abort (~1 s) is a backstop, not ACC — run these on an
empty road.

Motion commands return `{"status": "ok", "cmd_id": N}`. **Heartbeat contract:** maneuvers
execute to completion regardless of the connection — they are distance/time-bounded and end
stopped, so a WiFi drop mid-maneuver does not abort them. The commanding client POSTs
`/heartbeat {"cmd_id": N}` periodically (the bundled client beats at 2 Hz) because the one
unbounded state, **CRUISE** (speed held after a lane change / turn), gracefully stops ~6 s after
the last beat — a dead operator can't leave the car cruising. Only the active command's
heartbeat counts — observers polling `/status` cannot mask a dead operator. The offboard AI
planner (Phase 5) should implement the same heartbeat loop.

`offset_ft` is signed: positive pulls toward the right-hand curb, negative toward the left
(`CURB_CURVATURE_SIGN` in `demod.py` calibrates which physical side "positive" is — verify on the
car before the first run).

## Guardrails (enforced in `demod`)

- Hard caps: max speed 30 mph, max distance 2000 ft per command. S-curve geometry is validated
  against both the measured steering-authority ceiling (~0.045 1/m) and lateral accel at the
  commanded cruise (~2 m/s² — sharper needs a longer `runout_ft` or a lower speed); rejections
  include the minimum feasible `runout_ft`.
- The lead-vehicle abort distance scales with the controlled-stop braking distance (~60 m at
  30 mph), not just headway — still a clear-road backstop, not ACC.
- Input validation rejects NaN/inf at both the web layer (HTTP 400) and demod.
- Abort on a radar lead within 6 m, driver gas/brake/steer override, or disengage.
- CRUISE (the only unbounded state) additionally requires a live client heartbeat: >6 s stale
  triggers a graceful controlled stop. Bounded maneuvers run to completion without the client.
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
- Arc turn: commands a trapezoidal curvature (2 m ramp-in, constant 1/radius plateau) but ends
  the arc on **measured heading**, not distance: demod integrates `controlsState.curvature` over
  distance travelled and holds the arc until the achieved angle reaches `angle_deg` (ramping out
  over the last ramp's worth of heading), then runs the straight `tail_ft` and stops. EPS lag at
  crawl speed makes the *path* wider than the nominal radius, but the final heading is exact.
  Bail-out: if the heading can't complete within 2× the nominal arc length, the maneuver stops
  normally (never circles) and logs a warning. `GET /status` exposes live progress as `turn_deg`.
- Signs are **verified on this car** (lot test 2026-07-18): positive script curvature = right ⇒
  `CURB_CURVATURE_SIGN = +1.0`, `TURN_LEFT_SIGN = -1.0`. Don't re-flip without re-testing.
