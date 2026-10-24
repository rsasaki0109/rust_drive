# People and taxi fleet

[Open the fleet](https://rsasaki0109.github.io/rust_driving/?mode=fleet).
The driving game's start screen also links here.

![Recorded people walking, requesting taxis, boarding and arriving](../assets/taxi-fleet-demo.gif)

Press **START** to run or pause. **CITY VIEW** shows the neighbourhood;
**FOLLOW TAXI** follows a taxi selected by clicking a person's request card.
Escape also pauses. Reload starts the same authored morning again.

## What is simulated

Seventy-two individual travel plans are released over about four minutes.
Commuting, shopping, train connections, appointments and meetups have different
origins, destinations, departure times, walking speeds, patience and boarding
times. A train-arrival pulse creates a small burst of demand. Each person walks
along the sidewalk to a pickup point. Requests can be assigned while the person
is walking; a taxi must then wait for them to reach the curb.

Eight one-rider taxis serve eight curb stops among ordinary city traffic.
The dispatcher processes the oldest requests first and chooses an available
taxi by legal road distance, including the directed pickup approach. Taxis use
the existing traffic steering, acceleration, car following and junction
admission. They cannot reverse across a street to collect a nearby rider.

Boarding starts only within 1.1 m of the pickup pose at under 0.25 m/s.
A person then walks to the door and completes their individual boarding dwell.
Dropoff has the same stopped-arrival requirement. The person walks back to the
sidewalk before the taxi becomes available again, then continues along the
sidewalk to their destination. A request that exceeds the person's patience
is cancelled and releases its assignment. Counts come from these events;
passing time alone never completes a trip.

The colours identify trip purposes. Taxi rings indicate available, pickup,
occupied and dropoff states. The sidebar shows three current people; it is
a sample of the active agents, not the full dispatch queue.

## Reproduce and inspect

With the pinned toolchains and normal browser-playground prerequisites:

~~~sh
npm ci --prefix web --ignore-scripts
bash scripts/build-playground.sh
npm --prefix web test
npm --prefix web run preview -- --host 127.0.0.1 --port 4173
~~~

In another terminal (Python 3.12, Playwright 1.56.0, Pillow 12.0.0 and Chromium):

~~~sh
python web/tests/fleet_smoke.py \
  --url http://127.0.0.1:4173/ \
  --output web/test-results/fleet \
  --record assets/taxi-fleet-demo.gif
~~~

The capture starts through UI controls and advances the real rendered simulation
with the existing deterministic browser clock. It pre-rolls to a real dropoff,
then records 280 frames at 100 ms each: 28 seconds at normal playback speed.
It does not write agent poses or counters. The longer 120-second browser check
also requires completed trips, abandonments, exclusive assignments and working
pause, follow and mobile controls.

The [receipt](../assets/taxi-fleet-demo.json) records the source commit and
SHA-256 hashes, frame hashes, actual event times, recording duration and metrics.
The [poster](../assets/taxi-fleet-demo.png) is one unmodified recorded frame.
The **Taxi fleet evidence** workflow can record a same-repository feature branch;
it checks the branch has not moved before committing the media.
The normal **Browser playground** workflow also tests the fleet before Pages
deployment.

## Scope

This is an optional authored arcade scenario, not demand fitted to travel
surveys or a real Tokyo population. Its eight stops are fictional local
destinations in the existing city. It models individual first/last walking legs,
not general pedestrian crossing behaviour. There is one rider per cab;
pooling, multi-person parties, fares, shifts, batteries and predictive fleet
rebalancing are not implemented. Routing and dispatch use simulation state
inside the playground and are independent of the Rust autonomous-driving stack.
These GIFs do not establish perception, localization or real-vehicle performance.
