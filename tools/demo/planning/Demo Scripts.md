# Draft 1 

## **0:00 – 0:18 · Bridge \+ setup**

**\[OPEN: match the founder video's closing frame — introduce simulations through CARLA, briefly mention technical stack**

**VO (Tarun):** This is Curbside running on our car with a Comma 4, our equivalent of a robotaxi. Curbside is an intelligence layer for robotaxis that understands rider intent and dynamically directs the vehicle, replacing rigid, hardcoded destination planning.

---

## **0:18 – 1:15 · Scenario 1 — Pickup mismatch (the hero)**

**\[Tarun and Vikram are in the car, and they’re approaching Entrance 1 of Stoneridge Mall\]**

**VO:** The car's been routed to the mall’s front entrance. But the rider isn't there.

**\[Screen: rider audio indicator lights up; utterance plays as caption.\]**

**RIDER (audio):** "I'm at the back, by JC Penny."

**\[Screen: split for \~5 sec — left: live camera feed; right: map, a secondary access point highlights. Label: "Matching speech → camera → map".\]**

**VO:** The agent takes what the rider said, finds JC Penny through  the maps API, and matches it to a real access point on the map.

**\[Screen: GPS pin slides from front entrance to the rear access point. A structured command card appears: `reroute → rear_access {lat, lng}`.\]**

**VO:** It refines the pickup coordinate and sends a single reroute command to the fleet's planner. It never touches the driving model.

**\[Screen: agent response plays; car begins moving toward rear.\]**

**AGENT (audio):** "Got it — rerouting to the rear entrance. I'll be there in about 45 seconds."

**\[Screen: car arrives at JC Penny. Small green label: "Resolved in 40s · no operator call".\]**

**VO:** No app, no pin-drag, no human operator. The rider just talked to the car, and the car understood.

---

## **1:15 – 2:00 · Scenario 2 — The safety gate (knows when to say no), Stoneridge Mall Drive**

**\[Screen:. Same car, now mid-ride on a busy multi-lane road. Label: "In transit — arterial road".\]**

**VO:** Communication only works if the car also knows when not to act.

**\[Screen: rider audio lights up.\]**

**RIDER (audio):** "Just stop here and let me out."

**\[Screen: the request hits a visible gate UI — three checks evaluate fast: SAFE ✗ / LEGAL ✗ / FEASIBLE ✓. The SAFE and LEGAL checks flash red. Label: "Live traffic lane, with a fire lane — no legal stop".\]**

**VO:** Every request runs through a safety gate first. Here, the camera and map both show a live traffic lane with no legal place to stop. So the agent refuses — and explains why.

**\[Screen: a red "REQUEST DECLINED" card; then the agent offers an alternative as the map highlights a pull-over spot ahead.\]**

**AGENT (audio):** "I can't stop here safely. There's a pull-over spot about 200 feet ahead, would you be ok with that?”

**RIDER: “**Sure”

 **\[Screen: car continues to the safe spot and stops. Green label: "Unsafe stop avoided · alternative offered".\]**

**VO:** Curbside assesses every single request and accommodates the rider within what's actually safe.

---

## **2:00 – 2:45 · Real-world demo (same software, real car)**

**\[Screen: hard cut to real footage. Lower-third label: "Real vehicle · same software · early test". Show the car stationary at a marked spot A, a second marked spot B a short distance ahead in the same lane.\]**

**VO:** This is the same agent running on a real vehicle. Early test, deliberately simple — one lane, one straight move — but it's the real loop, end to end.

**\[Footage: person in the rider seat speaks.\]**

**RIDER (live):** "Can you pull up to the spot ahead?"

**\[Footage: on-screen caption shows the agent's parsed intent \+ command: `reroute → spot_B`. Then the agent's spoken reply plays through the cabin.\]**

**AGENT (audio):** "Sure — moving up to the spot ahead now."

**\[Footage: the car drives forward in a straight line from spot A and stops at spot B. Keep it real and unedited — the honesty is the point.\]**

**VO:** Rider spoke, the agent understood it, decided, and the car moved. Same software as the simulation, running in the real world.

---

## **2:45 – 3:00 · Close**

**\[Screen: return to rider's-seat windshield POV. Clean.\]**

**VO (Tarun):** Robotaxis can already drive. We're building the layer that lets them communicate — and we're just getting started.

**\[End card: "Banana Taxi" wordmark.\]**

