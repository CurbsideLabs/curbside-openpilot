# Curbside Demo — Plan 2: Real-Autonomy Extensions

> **Status (2026-07-18):** W1 (desire injection), W2 (lane change), and W3 (turns) are implemented —
> see `README.md` for endpoints, prerequisites, and the CRUISE/stop chaining semantics. W2/W3 need
> on-car validation. W4 (mission executor) and W5 (AI integration) are not started.

Second-phase plan. Builds on the primitives, daemon, and HTTP API from [PLAN.md](PLAN.md) (Phases 0–5). Where Plan 1 delivers deterministic *scripted* motions (pull out, forward N meters, pull over), this plan replaces hardcoded motion with the driving model's own perception and execution wherever the stack genuinely supports it, and composes everything into longer missions.

**Goal:** demos that are honestly "real autonomy" — the model perceives and executes; our layer (and eventually the AI planner) only makes decisions.

---

## Capability assessment (verified on `pal23sp-hda2-c4`)

| Capability | Native support | Mechanism |
|---|---|---|
| Lane keeping, unmarked roads | **Yes** | End-to-end model; road edges/curbs/parked cars are inputs, lane lines not required |
| Lane change | **Yes — fully real** | `desire_helper.py`: blinker ≥ 20 mph → model plans/executes with real perception; BSM-gated (Palisade has BSM). Sunnypilot nudgeless/timed modes in `sunnypilot/selfdrive/controls/lib/auto_lane_change.py` |
| Intersection turns | **Partial — real-ish** | Sunnypilot `lane_turn_desire.py`: below configurable speed (`LaneTurnValue`, capped at 20 mph), blinker feeds `turnLeft`/`turnRight` desire to the model, which executes the turn with real perception. Intersection-dependent reliability |
| Destination driving | **No** | Navigate-on-openpilot removed upstream; sunnypilot `mapd`/`navd` = OSM speed limits + map display only, no turn-by-turn steering decisions |

Conclusion: lane changes and turns can be *triggered* by us but *executed* by the model. Destination-level behavior must be composed by a mission layer — which is exactly the project's high-level-planner story.

---

## Demo ladder (least → most real)

1. Scripted pull-over / pull-out (Plan 1)
2. Model-driven cruise on unmarked residential road (Plan 1, lateral on model)
3. WiFi-triggered **real lane change** (this plan, W2)
4. WiFi-triggered **model-executed turn** (this plan, W3)
5. **GPS-waypoint mission** chaining all primitives to a "destination" (this plan, W4–W5)

---

## Workstreams

### W1 — Desire injection framework (foundation, ~2–3 days)
The model executes lane changes and turns from "desires" derived from blinker + speed in `desire_helper.py` / `lane_turn_desire.py`. Today the only input is the physical blinker stalk.

- Extend the `demoControl` cereal message (from Plan 1) with a commanded desire: `none | laneChangeLeft | laneChangeRight | turnLeft | turnRight`.
- Patch the desire path to OR in the demo-commanded desire alongside the blinker input (same gating: speed windows, blind-spot checks stay active — we inject *intent*, never bypass the safety gates).
- Also drive the physical turn signals when a desire is injected (legal requirement on public roads, and it visually sells the demo).
- New endpoints: `POST /demo/lanechange {direction}`, `POST /demo/turn {direction}`.

### W2 — Real lane change demo (~2 days incl. car time)
- Requires ≥ 20 mph (`LANE_CHANGE_SPEED_MIN`) — plan the demo on a road where 25–30 mph is comfortable.
- Configure sunnypilot auto-lane-change (nudgeless) so no steering nudge is needed after the injected desire.
- BSM gating verified live: trigger with a car in the blind spot → change must be refused.
- Status reporting: expose `laneChangeState` (off/preLaneChange/changing/finishing) via `GET /status` so the UI and AI planner can sequence.
- Abort: cancelling the desire mid-`preLaneChange` cleanly returns to lane keeping; test cancel during `changing` (model finishes or recenters — characterize).

### W3 — Model-executed turns (~3–5 days, validation-heavy)
- Enable `LaneTurnDesire` + set `LaneTurnValue` (speed ceiling for turn desires) via params.
- Trigger through W1 injection; the model executes the intersection turn with real perception.
- **Validation is the work:** behavior is intersection-dependent (the model must see the cross street). Test at the specific demo intersections, both directions, multiple approach speeds. Characterize failure modes (goes straight, wide line, hesitates).
- Fallback: Plan 1's scripted-curvature 90° turn (dead-reckoned) for any intersection where the model is unreliable — mission layer (W4) can select per-waypoint which mode to use.

### W4 — Mission executor (~1 week)
Composes primitives into a "drive to destination" demo. Decisions scripted/AI-made; driving between decisions is genuinely the model's.

- Mission = ordered list of primitives with trigger conditions:
  `[{followRoad, until: gps_waypoint | distance}, {laneChange, dir}, {turn, dir, mode: model|scripted}, {pullOver, offset}, {stop}]`
- Executor runs in `demod` (or offboard in the AI planner — same API): monitors GPS (`liveLocationKalman` / device GNSS) + wheel odometry, fires the next primitive when its trigger condition is met.
- `POST /mission` uploads a mission; `GET /status` streams current leg, position, distance-to-next-decision; `POST /abort` unchanged (always a controlled stop).
- Optional: derive waypoints from an OSM route (mapd data is already on-device) so a "destination" can be picked on a map.
- Guardrails additive to Plan 1's: per-leg distance/time budget (abort if exceeded — catches a missed turn), GPS-vs-odometry sanity check, max mission length.

### W5 — AI planner integration (~2–3 days, mostly offboard)
- The mission API becomes the contract for the high-level planner: scene analysis (camera snapshot/stream endpoint from Plan 1 Phase 5) selects the pull-over spot and emits `{travel, offset}`; route logic emits the mission legs.
- Pitch framing: our planner is the decision layer; openpilot's model is the execution layer. Lane keeping, lane changes, and turns are real perception-driven autonomy — only the decision sequence and the final curb approach are ours.

---

## Safety notes (additive to Plan 1 guardrails)

- Desire injection never bypasses model/BSM gating — we command intent, the model retains authority to refuse or abort.
- Turns and lane changes on public roads: signals on (W1), safety driver supervising, speeds within posted limits, pre-validated locations only.
- Mission executor treats *any* unexpected state (GPS jump, leg timeout, model refuses maneuver twice) as abort-to-stop, never retry-and-hope.

## Sequencing & prerequisites

- W1 depends on Plan 1 Phases 1–2 (`demod`, `demoControl`, HTTP API). W2/W3 depend on W1. W4 depends on W2 (and W3 for turn legs). W5 depends on W4.
- W2 (lane change) is the cheapest "real autonomy" win — schedule it immediately after Plan 1's first milestone.
- Rough total: ~3 weeks of dev + car time, parallelizable with two people (one on-car tuning, one on executor/UI).

## Open items

1. Pick and drive the candidate demo route: needs a 25–30 mph segment (lane change), validated intersections (turns), and a pull-over-friendly curb.
2. Decide where the mission executor lives: on-device in `demod` (self-contained) vs. offboard in the AI planner (simpler iteration). Default: on-device execution, offboard mission authoring.
3. Confirm sunnypilot param names/values for nudgeless lane change and `LaneTurnDesire`/`LaneTurnValue` on this branch during W1 bring-up.
