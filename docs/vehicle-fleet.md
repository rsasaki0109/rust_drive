# Multiple vehicles in the RNE demo

The new `traffic-fleet-queue` fixture has three reactive participants ahead of ego on a straight suburban test road. They start at 25, 50 and 75 m at 4 m/s, observe finite-range gaps, and brake behind a static circular obstacle at 110 m. Ego runs the existing RustDrive pipeline from noisy GNSS, wheel/gyro and first-return planar LiDAR. The expected outcome is a stopped queue, not arrival at the road endpoint.

![Actual RNE three-vehicle queue rendered in 3D](../assets/traffic-fleet-demo.gif)

| Participant | Display model | Color |
|---|---|---|
| Ego | Hatchback with decorative roof sensor | Blue |
| Actor 0 | Sedan with lower cabin and separate rear deck | Amber |
| Actor 1 | Cargo van with tall cabin and cargo panels | Ivory |
| Actor 2 | Pickup with open bed, rails and tailgate | Green |

Meshes and materials are original procedural assets. These labels select render geometry only: all three actors retain the fixture's 1 m circular radius and the shared one-dimensional bounded-acceleration traffic integrator. No class-aware perception, separate truck/van dynamics, lane changes, traffic lights or road priority are implied. Ego still uses pinned native RNE dynamics and Rapier LiDAR queries. Trees and buildings remain display-only.

## Reproduce the complete episode

Set up RNE with `bash scripts/setup-rne.sh`, install Blender and activate the Pillow environment documented in the README. No GPU is required.

```sh
bash scripts/rne-3d-demo.sh artifacts/fleet/demo.gif \
  scenarios/traffic-fleet-queue.json artifacts/fleet \
  --traffic-models sedan van pickup --camera traffic \
  --scene-output artifacts/fleet/scene.blend
```

This executes the native dynamic plant with seed 7, verifies all sensor ticks, renders the complete timeline, and exports an editable scene at the last state. Open `artifacts/fleet/demo.gif` or `blender artifacts/fleet/scene.blend`. The scene is a snapshot, not a baked animation. Simulation and replay run without Blender/Python; full CPU rendering takes several minutes.

Individual commands are also available:

```sh
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --scenario scenarios/traffic-fleet-queue.json --plant dynamic --seed 7 \
  --output artifacts/fleet-native
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/fleet-native/sensors.jsonl --output artifacts/fleet-native/replay
python3 scripts/render_demo_3d.py artifacts/fleet-native/run.json \
  --output assets/traffic-fleet-demo.gif --traffic-models sedan van pickup \
  --camera traffic --scene-output artifacts/3d/fleet-scene.blend
```

For a quick preview of an existing trace, append `--preview-time 30 --samples 4 --threads 2`; the output is a PNG. `--traffic-models` cycles through the selected models in stable actor appearance order. `--camera traffic` frames ego and active reactive vehicles; the default camera follows ego. Both options affect rendering only.

## Acceptance and evidence

The episode retains a 65 s evaluation and a 1 m minimum ego-clearance floor. Independent acceptance reconstructs bounded actor speed/distance integration and finite-range proximity observations at every 50 ms tick, computes swept circular separation, requires all three actor identities throughout, preserves their single-lane order, requires at least 15 m travel per actor and at least five continuous terminal seconds with all vehicles stopped. Full sensor-only replay must match every pipeline output. These are fixture checks, not a universal safe following-distance specification.

In native seed 7, all 1301 ticks replay, ego clearance is 4.554560 m, minimum actor-pair clearance is 2.995975 m and the complete queue remains stopped for 41.0 s. Zero ego/traffic collision or road-boundary violations occur. Dropping one actor from the terminal evidence or changing final ego speed to 0.15 m/s is rejected by the independent checker. Vehicle types do not enter the sensor log or pipeline configuration.

The [GIF provenance](../assets/traffic-fleet-demo.json) retains the input hash, complete physical summary, engine revision, renderer source hash, selected vehicle models, camera and checked scene states. The preceding opening/follower GIFs retain their captures from revision `5df62d1`; they were not regenerated for this addition. Full regression results are retained separately in [fleet results](../assets/fleet-results.json).

Across reference and native plants and seeds 1/7/42, all six fleet episodes pass with 1301 replayed ticks each. Minimum ego clearance across these six runs is 4.534712 m; minimum actor-pair clearance is 2.995975 m; terminal queue residence is 40.85–41.00 s. The complete suite passes 132 positive runs (63 reference, 69 native), retaining all prior fixtures/floors and separately verifying the two known short-range physical rejections.

The committed GIF contains 218 frames at 960 × 640, runs for 23.1 s including the final pause, and is approximately 3.8 MB. All 218 sampled scene states match the recorded poses. The traffic camera fits complete vehicle bounds within a five-percent screen margin, including the widely separated starting positions.
