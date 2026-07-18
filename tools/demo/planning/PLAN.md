# Curbside Demo — Scripted Motion Plan (`pal23sp-hda2-c4`)

**Goal:** Mock self-driving demos on the 2023 Palisade (HDA2, comma 4, sunnypilot branch `pal23sp-hda2-c4`). An AI layer analyzes the scene and tells the car where to pull over; the actual motions are scripted and triggered over WiFi through a UI.

**Demo motions (proof of concept for the high-level autonomy planner):**
1. On WiFi trigger, drive forward ~50 ft autonomously, then pull over to the curb.
2. Pull out from the curb autonomously, then drive forward ~50 ft.

**Scope decisions (2026-07-17):**
- Venue: develop in a closed lot (~5–15 mph), demo on a quiet street — plan covers both.
- Trigger: HTTP over WiFi (default); AI scene analysis runs offboard (laptop) unless decided otherwise.
- Engagement: fully remote start desired — staged approach below (Stage A: one human SET press per session; Stage B: true zero-touch).
- Openpilot (alpha) longitudinal status on the car: **unconfirmed — Phase 0 must verify.**

---

## Verdict

**Feasible.** The branch already contains working precedents for every piece:

- `tools/longitudinal_maneuvers/maneuversd.py` — comma's daemon that **replaces plannerd** (gated by the `LongitudinalManeuverMode` param in `system/manager/process_config.py`) and publishes `longitudinalPlan` with arbitrary scripted accel profiles. This is exactly the "drive forward N feet" pattern.
- `tools/joystick/joystickd.py` — replaces controlsd and commands `actuators.curvature` / `steeringAngleDeg` directly (gated by `JoystickDebugMode`). Proves direct lateral command works.
- `tools/bodyteleop/web.py` — aiohttp web server pattern already running on-device in other modes.
- `controlsd` auto-resume: `CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not shouldStop` — remote "go" from a standstill already works with stock code once engaged.

**Car config:** fingerprints as `HYUNDAI_PALISADE_2023` (CAN/CAN-FD blended, radar SCC, torque-based steering, no minimum steer speed). Openpilot longitudinal ("alpha long") is available and **required** — with stock SCC longitudinal we cannot command acceleration.

**Key risks:**
- (a) Remote engagement from parked requires panda `controls_allowed` (see Phase 4 Stage B) — the only part that touches a real safety interlock.
- (b) Torque-based steering has limited authority at parking-lot speeds; pull-over sharpness needs empirical tuning.
- (c) Bypassing the planner loses lead-car collision avoidance — `demod` restores a radar-based abort as a guardrail.
- (d) Driver monitoring nags/disengages without a seated, forward-glancing person; demo-mode DM relaxation is a small local patch if needed.

---

## Architecture

One new daemon, **`demod`**, gated by a new `DemoScriptMode` param (same pattern as `LongitudinalManeuverMode`). When active it replaces `plannerd` and publishes:

- **`longitudinalPlan`** — speed/accel from a distance-based state machine (IDLE → ARMED → EXECUTING → DONE/ABORT). Odometry by integrating `carState.vEgo` (50 ft ≈ 15.2 m; wheel-speed dead reckoning is accurate to well under a foot at these distances). `shouldStop=False` at standstill makes controlsd send resume automatically.
- **`demoControl`** (new cereal message) — scripted desired curvature. A ~10-line patch to `controlsd` substitutes this for the model's `modelV2.action.desiredCurvature` when a demo lateral maneuver is active, keeping the existing closed-loop torque controller, curvature rate limits, and panda safety fully intact.

**WiFi trigger + UI:** aiohttp server on the comma 4:

| Endpoint | Purpose |
|---|---|
| `POST /demo/forward {distance_ft}` | Drive forward N feet, stop |
| `POST /demo/pullover {travel_ft, offset_ft}` | Forward + S-curve to curb, stop |
| `POST /demo/pullout` | Mirror S-curve away from curb + forward 50 ft |
| `POST /abort` | Immediate controlled stop |
| `GET /status` | State machine + odometry + faults |

Plus a phone-friendly web page (big buttons, live status, ABORT). The offboard AI planner calls the identical endpoints with computed parameters. On-screen status uses the `alertDebug` overlay like maneuversd does.

**Guardrails in `demod`:**
- Abort on radar lead within threshold (restores collision stop).
- Abort on driver brake/gas/steer override.
- Hard caps: max speed (~15 mph), max distance per command.
- Watchdog: controlled stop if the trigger client disappears mid-maneuver.
- ABORT button on the UI.

---

## Phases

### Phase 0 — Baseline verification (half a day, on car)
- Confirm fingerprint `HYUNDAI_PALISADE_2023`.
- **Confirm alpha/openpilot longitudinal toggle is enabled and working** (stop-and-go in a lot). Hard prerequisite.
- Run stock `LongitudinalManeuverMode` as-is: free end-to-end test of "scripted accel through real actuators" with zero custom code.

### Phase 1 — `demod` + "forward 50 ft and stop" (2–3 days)
- New daemon + `process_config.py` entry + params.
- Trapezoidal speed profile from distance-to-go; stop; done.
- Lateral stays on the stock model this phase (holds straight/lane-keeps).
- Test: bench replay → closed lot.

### Phase 2 — WiFi trigger + web UI (1–2 days, parallel with Phase 1)
- aiohttp server, endpoints above, phone page with status + abort.
- **Milestone:** driver engages once; someone outside the car triggers "forward 50 ft" from a phone.

### Phase 3 — Scripted lateral: pull over / pull out (~1 week incl. car time; tuning-heavy)
- controlsd override patch + curvature profile generator.
- Distance-parameterized S-curve (two opposing arcs), ~2.5–3.5 m lateral offset over a configurable run-out, composed with a low-speed profile, ending stopped at the curb. Pull-out = mirror image + 50 ft forward.
- Tune offset-vs-distance aggressiveness on the lot before street runs (low-speed steering authority is the main unknown).

### Phase 4 — Remote start (staged)
- **Stage A (zero extra risk, works with the above):** safety driver engages once (SET press) at session start; every subsequent motion — including from a full stop at the curb — is remote-triggered with nobody touching controls.
- **Stage B (true zero-touch, stretch):** panda sets `controls_allowed` only on seeing real cruise-button presses from the car, so synthetic engagement needs a small panda safety patch + a selfdrived hook. Doable (already on forked firmware) but weakens a real safety interlock — demo with Stage A first.
- DM: seated person glancing forward keeps it satisfied; otherwise add demo-mode DM timer relaxation.

### Phase 5 — AI hook
- Camera snapshot/stream endpoint from the device; offboard AI analyzes the scene and POSTs `{travel_ft, offset_ft}` to the same API the buttons use.
- Scripted maneuvers become the execution layer for the high-level planner — the proof-of-concept story.

---

## Repo mechanics

- Branch **`curbside-demo`** off `pal23sp-hda2-c4`.
- New code in `tools/demo/` (or `sunnypilot/`) + three small touch-points: `process_config.py`, controlsd override, cereal message — keeps upstream rebases cheap.

## Open items before starting
1. Confirm alpha-long toggle status on the car (Phase 0).
2. Decide whether Stage-A engagement (one human SET press per session) is acceptable for demo day, or Stage B is required.
