# Curbside Demo — Field Runbook

Operational guide for deploying the `curbside-demo` branch to the comma 4 and running the
scripted-motion shakedown through the filmed demo. Companion to [PLAN.md](PLAN.md) /
[PLAN2.md](PLAN2.md) (design) and `../README.md` (API reference).

Every command is labeled with where it runs:

- **[car]** — physical actions by the safety driver in the Palisade
- **[ssh]** — a terminal on the laptop, SSH'd into the comma 4
- **[laptop]** — a terminal on the laptop in this repo, running `tools/demo/host/demo_client.py`

Safety roles: the **driver's brake pedal is the primary abort** — it works at two independent
layers (demod's `driver override` guardrail, and panda/stock disengagement below all demo code)
and needs no WiFi. The laptop provides the operational stop paths (Ctrl-C = graceful stop,
`abort` subcommand = immediate controlled stop). Maneuvers are bounded and finish on their own
even if WiFi drops mid-run; only a CRUISE hold (speed held after a lane change / turn) depends
on the connection — it gracefully stops ~6 s after the last heartbeat, so a dead laptop can't
leave the car cruising. Note the asymmetry: a brake-press abort also
**disengages** openpilot (driver must press SET again before the next take), while laptop
`stop`/`abort` leave the car engaged and ready for the next command. Brake = safety,
laptop = retakes.

---

## 1. Deploy to the comma 4 (one-time per code change)

Not a firmware flash — replace the openpilot checkout on the device:

```bash
git push origin curbside-demo        # from the dev machine
ssh comma@<device-ip>
cd /data/openpilot
git remote add curbside https://github.com/CurbsideLabs/curbside-openpilot.git  # if needed
git fetch curbside curbside-demo && git checkout curbside-demo
git submodule update --init --recursive
sudo reboot
```

First boot after checkout recompiles with scons (the capnp change forces a cereal rebuild) —
allow 10–20 min on device power; don't interrupt it. Clean-slate alternative: factory reset and
enter `installer.comma.ai/CurbsideLabs/curbside-demo` as the custom software URL during setup.

## 2. One-time prerequisites

1. SSH enabled: comma Settings → Network → Advanced → Enable SSH, GitHub username set under
   SSH keys so the laptop's key is authorized.
2. Laptop has this repo (`demo_client.py` is stdlib-only; any Python 3.10+).
3. **Phase 0 verification (hard prerequisite, per PLAN.md):** car fingerprints as
   `HYUNDAI_PALISADE_2023`, and the **openpilot (alpha) longitudinal toggle is enabled and
   actually controlling speed** (verify stop-and-go behind a lead in a lot). Without alpha long,
   stock SCC won't accept accel commands and none of this works.

## 3. Network

**Primary: phone hotspot.** The comma joins the phone's hotspot (Settings → Network) like any
WiFi network; the laptop joins the same hotspot. Both iOS and Android hotspots allow
client-to-client traffic, so SSH and demoweb (port 5555) work laptop→comma. Everything gets
internet (route uploads; cloud APIs for the agent).

- The comma's IP is DHCP-assigned (iPhone: `172.20.10.x`, Android: usually `192.168.x.x`).
  Read it at comma Settings → Network → Advanced — **check it each session, it can change.**
  No mDNS on AGNOS; use the IP.
- Keep the hotspot phone plugged in and its hotspot screen open — iPhones drop idle hotspots
  when the screen sleeps. Once a command is active, the 2 Hz heartbeats keep the link busy. A
  hotspot blip mid-maneuver is harmless: the maneuver finishes on its own; only a CRUISE hold
  would gracefully stop (`client lost`) ~6 s after the last beat.
- If the comma can't see an iPhone hotspot, enable "Maximize Compatibility" (2.4 GHz).
- Cell signal doesn't matter for control — heartbeats/SSH/demoweb are all LAN-local.

**Fallback: comma's own hotspot** (fixed IP `192.168.43.1`, no sleep behavior, no internet).
The two modes are mutually exclusive — joining a hotspot turns the comma's own hotspot off.

## 4. Session bring-up (repeat at the start of every drive)

Order matters: demod/demoweb only run while the device is **onroad** (ignition on), and
`DemoScriptMode` **clears every time the device goes offroad**, so it must be re-set per session.

1. **[car]** Park in the lot, ignition on, let the comma boot to the road view.
2. Laptop (and comma, if using phone hotspot) on the same network; note the comma's IP.
3. **[ssh]** Enable demo mode — while **disengaged and stopped** (the plannerd→demod swap
   briefly interrupts `longitudinalPlan`):
   ```bash
   ssh comma@<comma-ip>
   cd /data/openpilot
   python3 -c "from openpilot.common.params import Params; Params().put_bool('DemoScriptMode', True)"
   ```
4. **[ssh]** Confirm both daemons are up and grab the auth token (generated on demoweb's first
   ever start, persistent afterwards):
   ```bash
   ps aux | grep -E "demod|demo.web" | grep -v grep
   python3 -c "from openpilot.common.params import Params; print(Params().get('DemoAuthToken'))"
   ```
5. Verify the device screen shows the demo overlay: `DEMO IDLE 0 ft 0 mph` / `ready`. That
   overlay is ground truth for demod's state all session.
6. **[laptop]** Confirm the API end to end:
   ```bash
   cd tools/demo/host
   export DEMO_HOST=<comma-ip>
   export DEMO_TOKEN=<paste token>
   ./demo_client.py --host $DEMO_HOST --token $DEMO_TOKEN status
   ```
   Expect `{"state": "IDLE", ...}`. 401 → wrong token. Connection refused → demoweb isn't
   running (did the device go offroad after you set `DemoScriptMode`?).

One-time param setup for Plan-2 (model-executed) maneuvers, if testing those:
```bash
# [ssh]
python3 -c "from openpilot.common.params import Params; p=Params(); p.put('AutoLaneChangeTimer','1'); p.put_bool('LaneTurnDesire', True); p.put('LaneTurnValue','10')"
```

## 5. First motion — forward 20 ft at walking speed

1. **[car]** Driver: seatbelt on, door closed, eyes forward (DM is fully active — looking down
   at a phone earns monitoring nags and eventually a disengage; expected behavior, not a bug).
   Foot near but not on the brake. Press **SET** at standstill to engage. The car holds the
   stop (demod `IDLE` commands `shouldStop=True`).
2. **[laptop]**
   ```bash
   ./demo_client.py --host $DEMO_HOST --token $DEMO_TOKEN forward --distance-ft 20 --cruise-mph 3
   ```
   The CLI prints `command accepted (cmd_id=N)` and heartbeats. demod goes `ARMED` →
   `EXECUTING` (instantly, since already engaged), `shouldStop` flips false, controlsd sends
   `cruiseControl.resume`, and the car creeps forward and stops itself ~20 ft later. The CLI
   live-prints state/odometer/speed and exits on `DONE`.
3. Check: it moved with nobody touching pedals, stopped within ~a foot of target, and both the
   screen overlay and CLI ended at `DONE`.

Ordering discipline: a command sent *before* engagement parks demod in `ARMED` and launches
the instant the driver presses SET. Legal but surprising — **always engage first, then command.**

## 6. Abort drills (all four before any bigger motion)

Run `forward --distance-ft 100 --cruise-mph 5` for each drill and kill it a different way mid-run:

1. **[car]** Driver taps the brake → guardrail abort, fault `driver override`, and openpilot
   disengages (re-engage with SET for the next drill). This is the real safety net; everyone
   in the car should see it work.
2. **[laptop]** Ctrl-C in the CLI → graceful `stop`, car stops with no fault, stays engaged.
3. **[laptop]** Connection independence: start a run, then hard-kill the client (`kill -9`
   from another terminal, or close the laptop lid). The maneuver **keeps going and completes
   normally** — that's by design; scripted maneuvers are bounded and end stopped. (The CRUISE
   tether — auto-stop ~6 s after the last heartbeat — gets its drill in the lane-change street
   session, since only desire maneuvers end in CRUISE.)
4. Lead abort: at creep speed, with the driver covering the brake, have someone walk to
   ~5 m ahead of the car → `lead detected` abort. The 6 m floor is a backstop, not ACC —
   see it fire once so nobody mistakes it for car-following.

## 7. Calibrate `CURB_CURVATURE_SIGN` (expect to flip it)

The shipped `+1.0` most likely steers **left** for a positive offset (openpilot uses the ISO
convention: positive curvature = left turn), i.e. backwards from the README's
"positive = right-hand curb".

1. Line up with ≥ 4 car-widths clear on **both** sides.
2. **[laptop]** Minimal S-curve at walking pace:
   ```bash
   ./demo_client.py --host $DEMO_HOST --token $DEMO_TOKEN pullover --travel-ft 10 --offset-ft 3 --runout-ft 50 --cruise-mph 3
   ```
3. First half of the S with positive `offset_ft` should move **right**. If it moves left:
   ```bash
   # [ssh]
   sed -i 's/^CURB_CURVATURE_SIGN = 1.0/CURB_CURVATURE_SIGN = -1.0/' /data/openpilot/tools/demo/demod.py
   sudo reboot   # required: the manager pre-imports process modules and forks, so a killed
                 # demod restarts with the OLD in-memory code — only a reboot loads edits
   ```
   Confirm the overlay returns to `DEMO IDLE`, rerun, verify it goes right. **Mirror the fix
   back into the repo** so the next deploy doesn't regress it.

## 8. Work up to demo-scale maneuvers

Each rung only after the previous one is boring:

```bash
# the filmed move (Demo Scripts.md scene at 2:00–2:45): spot A -> spot B
./demo_client.py --host $DEMO_HOST --token $DEMO_TOKEN forward --distance-ft 50 --cruise-mph 5

# full pull-over: 50 ft straight (model steering), then 9 ft offset over a 50 ft S-curve
./demo_client.py --host $DEMO_HOST --token $DEMO_TOKEN pullover --travel-ft 50 --offset-ft 9 --runout-ft 50 --cruise-mph 5

# pull-out from the curb, then 50 ft forward
./demo_client.py --host $DEMO_HOST --token $DEMO_TOKEN pullout --forward-ft 50 --offset-ft 9 --runout-ft 50 --cruise-mph 5
```

Tuning:
- Too-lazy drift toward the curb → shorten `--runout-ft` (sharper S; demod rejects anything
  needing curvature over the 0.15 1/m cap and reports the minimum feasible runout in the fault).
- Overshoot/weave → lengthen `--runout-ft`.
- `--offset-ft` should match the measured lane-to-curb distance at the venue, not a guess.
- Chalk-mark start and target spots so runs are comparable and the footage is repeatable.

**Not part of the lot session:**
- **Lane changes** need sustained ≥ 21 mph, and (known issue) the command currently only
  triggers if issued while *already* at speed — DesireHelper requires a blinker rising edge
  above 20 mph, so a desire injected while below the threshold never fires and times out after
  20 s. Street test, after that fix lands in `demod.py`.
- **Turns** need a validated intersection, and the `hold_s` clock starts at command time
  (including any acceleration from a stop), so send the command as the car approaches the
  intersection mouth — timing practice matters. Also `LaneTurnValue` must be ≥ the turn cruise
  speed (default 8 mph) or the desire is silently gated.

## 9. Filming the real-car scene (Demo Scripts.md, 2:00–2:45)

Mark spots A and B (~50 ft apart, same lane). Car engaged and stopped at A, rider speaks, the
agent (or the operator, faking it for this cut) runs `client.forward(distance_ft=50)`, the
agent voice line plays, the car pulls to B and stops. Rider-facing agent audio and
parsed-intent captions are host-side/post-production — the car-side loop is real, which is
what the script's honesty framing needs.

## 10. Wrap-up

- Ignition off clears `DemoScriptMode` — the car is stock sunnypilot on next start unless
  Section 4 is repeated.
- Every run is logged as a normal route on the device; review via comma connect or SSH.
- Session planning: budget Sections 4–6 as the whole first outing (an hour goes fast in a
  lot), and don't skip the abort drills to get to the fun part — they're the reason a later
  surprise stays a non-event.
