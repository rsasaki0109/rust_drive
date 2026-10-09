# CPU-only Robot Native Engine integration

This standalone Cargo workspace embeds [RobotNativeEngine](https://github.com/rsasaki0109/RobotNativeEngine) native vehicle systems and Rapier LiDAR queries into the **same RustDrive pipeline** used by the reference simulator and replay. It is executable code with integration tests, not a mock or renderer-driven vehicle animation.

## Reproduce

From the RustDrive checkout, with Git, Rust, a C linker and network access:

```sh
bash scripts/setup.sh
bash scripts/setup-rne.sh
# Pillow is optional for running/testing; required for the final GIF step.
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r scripts/requirements-demo.txt
bash scripts/rne-demo.sh dynamic
bash scripts/rne-demo.sh kinematic
```

Setup fetches exactly [`df6007aa40315e81d12ae00fc1f60369e393a178`](https://github.com/rsasaki0109/RobotNativeEngine/commit/df6007aa40315e81d12ae00fc1f60369e393a178) (also in `rne-revision.txt`) into sibling `../RobotNativeEngine`, using a sparse checkout of crates and vendored dependencies. An existing checkout is preserved; setup exits if HEAD differs rather than resetting user work. The engine fixes are on `feat/rustdrive-sensor-contract`, based on upstream `81454814e997e6733f5bd1687d86a0cab03c1b03`.

Rust 1.95.0 is pinned here to match RNE; the default RustDrive workspace retains 1.90.0. The standalone lockfile keeps optional dependencies out of default builds. No `wgpu`/Vulkan renderer, window, GPU, ROS, Docker, model weights or simulator assets are needed. `rne_render` contains backend-neutral types required transitively by the sensor library; no GPU backend is linked.

Without Python:

```sh
cargo +1.95.0 test --manifest-path integrations/rne/Cargo.toml --locked
cargo +1.95.0 clippy --manifest-path integrations/rne/Cargo.toml --all-targets --locked -- -D warnings
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --scenario scenarios/mission.json --plant dynamic --seed 7 --output artifacts/rne-dynamic
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/rne-dynamic/sensors.jsonl --output artifacts/rne-dynamic/replay
```

After fetching/building, add `--offline` to Cargo commands to use retained dependencies. Raw run/sensor/summary/replay files are generated in ignored `artifacts/`. To intentionally refresh the README media: `bash scripts/rne-demo.sh dynamic assets/rne-demo.gif`.

## Model and conversions

- RustDrive is planar ENU: x east, y north, positive counterclockwise yaw. RNE is Y-up: `(x,y)` maps to `(x,0.6,-y)`; yaw maps to a positive-Y quaternion. Tests exercise round-trip coordinates and orientation.
- `kinematic` uses RNE `AckermannDrive` and `ackermann_kinematics`, with the 8 m/s reference cruise setting.
- `dynamic` uses `VehicleDynamics` and `vehicle_dynamics`, with friction coefficient 0.9, steering time constant 0.08 s, and a 6 m/s cruise setting. This avoids claiming identical behavior/calibration across plants.
- Plants integrate ten 0.005 s substeps per 0.05 s control tick (200 Hz simulated integration). Acceleration is mapped to a bounded speed target; steering and acceleration/deceleration limits are explicit.
- RNE entities for the ego and circular scenario obstacles carry Rapier query colliders, synchronized at each acquisition. The ego is filtered from its own rays. Rapier is used for scene queries; native RNE systems integrate the vehicle. There is no second Rapier integration or physical contact response in this adapter.
- Checked RNE LiDAR samples 720 rays to 45 m with deterministic keyed Gaussian 0.008 m range noise. World-frame points are inverse-transformed through the acquisition mount into body x-forward/y-left. GNSS and wheel-speed/gyro remain explicit synthetic noisy measurements, not sensor-independent ground-truth estimates.
- A failed ray query becomes `lidar_failed`, triggering emergency braking. The fault test deliberately uses an invalid physics world; healthy empty scans remain valid. Scenario dropouts additionally exercise freshness handling.
- The evaluator scores native plant truth independently using swept circular footprints and the supplied route corridor. The GIF is a top-down Pillow visualization of actual telemetry, **not an RNE 3D renderer capture**.

## Engine improvements

The pinned RNE change adds an additive strict `sample_lidar_checked` API. Invalid configuration/mount or any backend ray failure returns an error instead of silently presenting a healthy empty/partial scan; permissive legacy APIs remain available.

It also fixes `RapierBackend::sync_from_ecs`: modified body poses must propagate to colliders **before** updating the query pipeline. Before the fix, a collider moved from 5 m to 10 m still returned the old 4 m ray range; after the fix it immediately returns 9 m without an artificial physics tick. A regression covers fixed and kinematic bodies. The stale geometry caused real integration collisions before repair; acceptance was not weakened to hide them.

The adapter uses crates.io Rapier 0.22.0 as supported by the backend manifest. The RNE workspace's own affected-crate tests additionally exercise its vendored Rapier patch. See [checked acquisition](https://github.com/rsasaki0109/RobotNativeEngine/blob/df6007aa40315e81d12ae00fc1f60369e393a178/docs/LIDAR_CHECKED_ACQUISITION.md).

## Verified boundaries

Eight adapter tests cover frame conversion, kinematic mission/blocked road, dynamic mission, acquisition-error braking, RNE-log recomputation, geometric occlusion, actual friction limits and multi-seed hazard runs. The affected RNE packages pass 150 tests. Full RNE workspace/rendering/platform CI is not claimed. Local scenario results and remaining limitations are in [validation](../../docs/validation.md).

This is a planar known-route research integration. It does not establish realistic tire calibration, road elevation/suspension/contact response, camera perception, traffic reasoning, full 3D autonomous driving, hardware throughput or road safety. CARLA remains unimplemented; its standard rendered/camera workflow normally requires a suitable GPU, while no-rendering changes available sensors and does not by itself prove CPU-only support.

RNE is dual MIT/Apache-2.0. Optional locked dependency licenses are inventoried in [THIRD_PARTY.md](THIRD_PARTY.md).

## Occlusion, crossing and low-friction regressions

Run `bash scripts/check-hazards.sh` from RustDrive for the 30-case reference/RNE suite and complete replay. The dynamic adapter accepts optional scenario `dynamics` calibration for friction and steering lag, adds a longitudinal `mu*g` actuation clamp, and supplies conservative fixed forward/braking/lateral limits to the shared planner. Reference/kinematic backends reject this calibration. Combined longitudinal/lateral friction coupling and online friction estimation are absent. [Measured outcomes and the repaired braking failure](../../docs/hazard-validation.md).

The integration currently passes eight tests, including multi-seed hazard runs, acquisition under real geometric occlusion, and measured acceleration/braking limits. The engine pin and standalone dependencies remain the same.

The current shared planner integrates bounded acceleration to compute arrival times, checks the retimed stopping path and its stationary hold, and uses a longer persistent lateral transition where low lateral authority requires it. The dynamic adapter caps calibrated forward acceleration at 2.0 m/s² and the existing conservative friction authority. No RNE engine revision changed for this extension. [Previous speed-planning methods and results](../../docs/speed-planning.md).

The current tracking extension adds smooth supplied-route interpolation, estimated-heading joins and braking headroom in the shared algorithm crates; the engine and plant parameters stay pinned. Both the RNE multi-seed test and hazard CLI now require at most 20 emergency ticks in the specified low-friction avoidance fixture. Recorded counts are 9/8/7 for seeds 1/7/42. [Current tracking evidence and remaining limitations](../../docs/tracking.md).

## Road-network demo

The adapter also runs authored graph routes selected by the shared simulator before departure. With Pillow active, `bash scripts/rne-demo.sh dynamic assets/rne-demo.gif scenarios/route-handover.json` reproduces the opening README closure-detour GIF. The three map fixtures run across seeds 1/7/42 in the expanded acceptance/replay suite. Known closures are map inputs; they are not LiDAR objects. [Routing results and limits](../../docs/routing.md).

Live fixtures also carry timestamped closure snapshots into the shared navigator; RNE world objects stay on their initial road. Stopped handover and reopening are tested with an explicit 4 m/s cruise / 1 m/s² planned lateral bound. The previously failed 6 m/s run without an optional curvature cap now passes as `route-handover-fast`; the original failed evidence is preserved. [Current avoidance results](../../docs/avoidance-continuity.md); [handover protocol and baseline](../../docs/handover.md).

GNSS bias-window fixtures use the shared simulator fault injector after native sensor acquisition. The engine/plant, sensor noise and control bounds remain unchanged. Fixed-obstacle spike/burst/persistent-fault cases pass across three seeds with at most 0.5 m position error. The original three-actor burst episode now passes, together with a 16-second post-arrival physical-hold variant. All 120 positive seeded runs include full replay and fixed clearance floors. The native engine pin and calibration are unchanged. [Current terminal-stopping measurements](../../docs/terminal-stopping.md); [GNSS baseline and historical failure](../../docs/gnss-robustness.md).

Optional traffic actors are owned by the shared simulator `TrafficWorld`: the RNE ego still uses native Ackermann dynamics and Rapier LiDAR, while actor scalar proximity sensing and one-dimensional bounded integration run independently of engine tire/contact systems. Stopped-lead resumption, follower braking/recovery and traffic queues pass across three seeds with full replay. Two narrow-road 65-second deadline failures remain separately rejected. No engine revision or plant calibration was changed. [Measured behavior, GIF and limits](../../docs/reactive-traffic.md).
