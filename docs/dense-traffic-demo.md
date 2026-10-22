# Dense traffic demonstration

This native demonstration shows **24 reactive vehicles in three eight-vehicle queues**,
plus the blue ego vehicle. Each participant is simulated and recorded by the
pinned native RNE backend. Hatchbacks, sedans and vans are original SI display
meshes; the perspective camera fits every vehicle throughout the recording.
The cars have individual following calibrations and uneven initial spacing;
their motion comes from recorded simulation poses.

![Recorded native 24-vehicle traffic simulation](../assets/dense-traffic-demo.gif)

The authored straight test corridor is 20 m wide. Queue centres lie at −7, 0
and +7 m, and three static circular obstructions stop the traffic downstream.
Each actor uses a 2.8 m circular proxy and a finite 45 m proximity observation.
The deterministic fixture varies desired speed (2.8–5.0 m/s), initial speed
(1.0–3.8 m/s), minimum gap (3.4–4.3 m), time headway (1.0–1.8 s), acceleration
(0.9–1.8 m/s²) and comfortable braking (1.3–2.1 m/s²) by vehicle. The three
obstructions lie at different longitudinal positions (132, 137 and 151 m).
Each following policy reacts to its own measured gap; different speeds and
braking waves emerge without rendering offsets or scripted pose jitter.
Ego runs the existing
sensor-driven pipeline. The expected outcome is a stopped queue, not arrival
at the road endpoint.
LiDAR uses first returns, so vehicles can occlude one another. The camera shows
recorded simulator-world poses; it does not imply that all 24 cars are tracked.

Lane dividers, arrows and buildings are display geometry. This does not add
lane changing, lane-aware routing, arbitrary junction negotiation, contact
response or calibrated production-vehicle sensing. Actor metadata never enters
the ego driver. The [Japanese-city recording](japanese-city-demo.md) and
[original three-vehicle queue](vehicle-fleet.md) remain available.

## Evidence

[Acceptance](../assets/dense-traffic-demo-results.json) covers native seeds
7, 1 and 42. Each complete 40-second episode retains all 24 reactive identities,
all 27 physical objects, 801 sensor-only replay ticks, fixed queue order and
at least 15 m actual travel per vehicle (28.59 m measured minimum).
Independent checks reconstruct actor
integration and finite-range proximity readings from the recorded positions.
The complete fleet and ego must remain stopped together for five terminal
seconds (10.8 s measured); collisions and road violations must remain zero.

A motion-diversity check compares matching queue ranks across the three lanes
from 5 to 20 s, removing their different initial positions. Speeds are derived
from physical position differences, rather than reported speed labels. Mean
cross-lane speed range must reach 0.3 m/s and mean travel range must reach 2 m;
this recording measures approximately 0.585 m/s and 7.44 m. Copying one lane's
motion to the others is rejected even with the original varied telemetry.
Together with the existing corruption cases, ten rejections are checked per seed.

The checker also sweeps expanded display envelopes between recorded poses,
retaining the 1 m clearance floor with additional inter-sample reserves.
The renderer verifies actual evaluated mesh bounds against these envelopes,
all sampled poses and the complete camera framing. This is bounded planar
simulation evidence, not a full contact-enabled vehicle-body model.

The preceding [uniform-traffic recording](https://github.com/rsasaki0109/rust_drive/blob/20cd85b03d7b8b9d16b6cb0c16d02e8d1764b2d1/docs/dense-traffic-demo.md)
used identical calibrations, spacing and stop positions in all three lanes.
It measures zero cross-lane motion diversity under the new check. Its
[initial trial](../assets/dense-traffic-initial-trial.json) remains a historical
failure: that different fixture left seed 1 with only 2.3 terminal stopped
seconds at 40 s and required a 46 s observation. The new, individually
configured queues settle earlier; the five-second stop and 1 m clearance
gates and the driving algorithms are unchanged.

The [GIF provenance](../assets/dense-traffic-demo.json) records the actual trace,
scenario, physical summary, engine revision, rendering parameters and source
hashes. The full timeline is shown at 3× playback.

## Reproduction

Install the pinned Rust toolchains, set up the engine with
`bash scripts/setup-rne.sh`, install Blender, and install
`scripts/requirements-demo.txt` in your Python environment.

```sh
bash scripts/dense-traffic-demo.sh artifacts/dense-traffic-demo
```

This captures and replays all three seeds, runs the independent checker, then
renders `artifacts/dense-traffic-demo/demo.gif` with Cycles CPU. To intentionally
refresh the README GIF, supply `assets/dense-traffic-demo.gif` as the second
argument. Regenerating requires a fresh output directory. The camera and the
`--traffic-lanes` option affect visualization only; old rendering modes retain
their existing layouts.
