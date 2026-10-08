# Validation record

Recorded on 2026-10-08 in the Linux Codex cloud workspace. Default workspace: Rust 1.90.0; optional RNE: Rust 1.95.0. Pillow 12.3.0, release and debug profiles. Results below describe local execution, not a safety assessment or independently restored task.

## Checks executed

- `bash scripts/check.sh`: formatting, Clippy with warnings denied, **37 nonzero workspace tests**, locked release build, four CLI acceptance scenarios and full sensor-output replay for each passed. No tests failed or were ignored.
- Default offline tests/replay work after dependencies are retained. Mission tests run seeds 1, 7 and 42; exact same-seed traces are compared on this binary/platform.
- Regression coverage includes circular-object fitting, maneuver persistence, return to center, occlusion, EKF rejection/covariance, track expiration/velocity, occupancy rays, blocking and sensor timestamp faults.
- New pipeline tests cover healthy empty scans vs acquisition errors, duplicate/delayed freshness, malformed/future sensor data, transactional clock rejection/gaps and stale odometry. Log tests cover recomputation, altered commands, truncation/count mismatch, schema/truth-field rejection and failing output writers.
- CLI negative tests verify incomplete/colliding missions fail physical acceptance, invalid options/replay mismatches exit 2, and previous replay success reports are removed for corrupted or missing logs. These expected negative outcomes are not failing tests.
- `bash scripts/setup-rne.sh`: matched the fixed engine pin, fetched locked dependencies, built release and passed **5 adapter tests**. Formatting and all-target Clippy with warnings denied also passed for the standalone integration.
- RNE `cargo +1.95.0 test --locked -p rne_physics_rapier -p rne_sensor --jobs 4`: **150 tests** passed (42 Rapier, 103 sensor, 5 sensor integration). Affected-package Clippy passed. Existing vendored Rapier dependency warnings were emitted; they were not introduced by this change. Full RNE workspace/renderer validation was not run.
- `bash scripts/demo.sh assets/demo.gif` and `bash scripts/rne-demo.sh dynamic assets/rne-demo.gif` generated the README media from actual release-binary runs and verified their sensor logs. The kinematic RNE demo script also completed. GIF dimensions/frame counts and representative PNGs were inspected: reference 105 frames; RNE dynamic 132 frames; both 1200×720.

## Reference release-binary results (seed 7)

| Scenario | Acceptance | Goal | Colliding steps | Road violations | Min clearance (m) | Localization RMSE (m) | Final speed (m/s) | Replay ticks |
|---|---|---|---|---|---|---|---|---|
| mission | PASS | yes | 0 | 0 | 0.926 | 0.064 | 0.101 | 624 |
| blocked | PASS | no (expected) | 0 | 0 | 2.370 | 0.049 | 0.000 | 401 |
| lidar-fault | PASS | no (expected) | 0 | 0 | 47.736 | 0.055 | 0.000 | 321 |
| gnss-fault | PASS | no (expected) | 0 | 0 | 45.284 | 0.158 | 0.000 | 321 |

The reference mission reaches 220.24 m arc length in 31.15 **simulated seconds**. The LiDAR/GNSS fault scenarios record 195/189 emergency-control steps. Raw evidence is in ignored `artifacts/check/`; committed `assets/demo.json` records the displayed run. No wall-clock throughput result is claimed.

## RNE release-binary results (seed 7)

Engine: [`df6007aa40315e81d12ae00fc1f60369e393a178`](https://github.com/rsasaki0109/RobotNativeEngine/commit/df6007aa40315e81d12ae00fc1f60369e393a178), based on `81454814e997e6733f5bd1687d86a0cab03c1b03`. Both runs use the same supplied mission and shared pipeline, with plant-specific documented cruise settings.

| Plant | Acceptance / goal | Colliding steps | Road violations | Min clearance (m) | Localization RMSE (m) | Final speed (m/s) | Simulated seconds | Replay ticks |
|---|---|---|---|---|---|---|---|---|
| Kinematic (8 m/s cruise) | PASS / yes | 0 | 0 | 0.572 | 0.067 | 0.158 | 37.25 | 746 |
| Dynamic (6 m/s cruise) | PASS / yes | 0 | 0 | 0.947 | 0.082 | 0.106 | 39.20 | 785 |

The dynamic run reaches 220.31 m; maximum localization error is 0.252 m. Both RNE logs generated with Rust 1.95.0 also verified exactly using the Rust 1.90.0 default CLI locally. This is a tested pair, not a general portability guarantee. Raw evidence is under `artifacts/rne-{kinematic,dynamic}/`; committed `assets/rne-demo.json` contains the dynamic metrics and reproduction recipe.

The adapter tests additionally cover blocked-road stopping and acquisition-error braking. The RNE fix has a reproduced negative regression: a fixed/kinematic query collider moved from 5 m to 10 m previously returned its old 4 m ray range; after pose propagation it immediately returns 9 m without stepping. Before repair the integration encountered stale moving geometry and collisions. The repair passes the original acceptance criteria.

## Limits and unexecuted checks

- RNE runs native vehicle integration and CPU 3D ray queries with planar LiDAR. Rapier contact response is not used; swept circular collision/route scoring is independent. The GIF is top-down telemetry rendering, not a full 3D engine camera capture.
- CARLA is not installed, implemented or tested. No GPU is required for the implemented RNE path. CARLA's standard rendered/camera workflow generally needs a suitable GPU; no-rendering limits sensors and has not been validated here.
- No learned camera/semantic perception, 3D SLAM, global routing, traffic-rule reasoning, ROS 2 bridge or real vehicle actuator is implemented. The route and initial pose calibration are supplied.
- Occupancy mapping is diagnostic. Planning has three lateral targets, heuristic CV forecasts and constant-speed time approximation. RNE adds a native friction limit/steering lag, without establishing realistic vehicle calibration or joint trajectory feasibility.
- Replay verifies computation only; it cannot establish physical acceptance, timing, robust autonomy or operational safety. No certification, formal verification, hardware-in-the-loop or sensor/weather benchmark is claimed.
- GitHub Actions' three-platform default checks plus Linux visualization/RNE jobs passed remotely as recorded below. Full RNE rendering/platform CI remains unexecuted here.
- Published cloud snapshots have reconnected with the retained RNE source/tooling and passed the checks described below. Independent fresh-task restoration remains unverified.

See [capabilities](capabilities.md), [architecture](architecture.md), [sensor replay](sensor-replay.md) and [RNE integration](../integrations/rne/README.md).

## Hazard-suite extension (2026-10-09, Asia/Tokyo)

The later development tree passes 45 default-workspace tests, 8 RNE-adapter tests, formatting and Clippy in both workspaces. The default check script now executes six reference scenarios and their logs. The dedicated hazard command passes all 18 release-binary acceptance runs and full replay across seeds 1/7/42, including actual low-friction acceleration bounds. A low-friction braking overlap was reproduced and repaired without changing its fixture or acceptance criteria. Detailed metrics, source fingerprint and remaining model limitations: [hazard validation](hazard-validation.md) and [result snapshot](hazard-results.json).

The previously published environment reconnected with the original pinned source/tooling and passed 37 tests, 5 adapter tests, both release missions, 624/785-tick replay and RNE GIF regeneration. This verifies that reconnection; an independently created task remains untested. No new RNE engine revision was needed for the hazard extension.

## Observed GitHub Actions results

For hazard commit `560ea49cc4f9b89e1c91f62809a240603a11507e`, [run 37802806434, attempt 2](https://github.com/rsasaki0109/rust_drive/actions/runs/37802806434/attempts/2) completed successfully: Linux, macOS and Windows workspace checks, Linux GIF generation, and Linux CPU-only RNE integration including the 18-run hazard suite.

The first attempt failed to acquire a hosted macOS runner. Its visualization job failed before compiling the demo, while rustup added manifest-required components to the runner's existing Rust installation: `failed to install component: 'clippy-preview-x86_64-unknown-linux-gnu', detected conflict: 'bin/cargo-clippy'`. Retrying only failed jobs passed without application changes. CI now installs toolchains under a job-specific `RUSTUP_HOME` in `runner.temp`, with the manifest-required Rust 1.90.0 components installed explicitly, to avoid using a runner image's pre-existing toolchain files. This change does not address hosted-runner capacity.

Locally, installing Rust 1.90.0 plus Clippy/rustfmt into an empty temporary `RUSTUP_HOME` and running `bash scripts/demo.sh` passed: 624 replay ticks and a 105-frame GIF. The workspace check script also passed all 45 tests and six scenario/replay pairs. The opening README media is the existing verified 132-frame RNE dynamic run, with its original provenance preserved.

## Continuous planning extension

The revised planner passes **56 workspace tests**, Clippy, formatting, the locked release build and eight reference acceptance/replay scenarios. The RNE adapter passes its **8 tests**, formatting, Clippy and release build. All **30 seeded reference/RNE hazard runs** pass physical acceptance and full replay with zero collisions and road violations; low-friction longitudinal bounds remain checked. [Methods, regressions and updated results](swept-planning.md).

The earlier planner failed the new between-sample oncoming regression in an isolated temporary test harness. Opposing scheduled crossings also reproduced a physical collision during development; the fixture and evaluation criteria were retained while repairing stop behavior and the trajectory's connection to the current estimate. The earlier numeric tables above are baseline records; the new compact snapshot and regenerated media describe this later implementation.

Run [37810379211](https://github.com/rsasaki0109/rust_drive/actions/runs/37810379211), before this algorithm extension, passed Linux, Windows, GIF generation and RNE. Its macOS job was cancelled because no hosted runner was acquired; annotations reported ARM runner capacity constraints. The workflow now selects the officially supported `macos-15-intel` image for macOS x64 coverage. This selects a different pool; it does not guarantee runner availability or establish current ARM coverage.
