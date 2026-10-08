# Validation record

Recorded on 2026-10-08 in the Linux Codex cloud workspace. Rust 1.90.0, release and debug profiles, Pillow 12.3.0. This record describes the verified source tree and local cloud execution. It does not establish a GitHub release or a fresh-task restoration result.

## Checks executed

- `bash scripts/setup.sh`: succeeded, including pinned tool activation, locked dependency fetch, release build and workspace tests.
- `bash scripts/check.sh`: formatting, Clippy with warnings denied, workspace tests, release build and all four CLI acceptance scenarios succeeded.
- `cargo test --workspace --locked --offline`: succeeded; no external service is required once dependencies are retained.
- 26 nonzero unit/integration tests passed; no failures or ignored tests. Binary and doc-test targets with zero tests are not counted as validation.
- Mission integration testing runs seeds 1, 7 and 42. Exact same-seed trace regeneration is tested on the current binary/platform.
- Regression tests cover circular-object center fitting, persistent maneuver progress, return to road center, occlusion, EKF outlier rejection / covariance, track velocity / expiration, occupancy rays, blocking and timestamp faults.
- A deliberate collision correctly fails simulator acceptance; the CLI correctly exits 1 and retains evidence for incomplete missions, and exits 2 for invalid options. These are expected negative-test outcomes, not unresolved failing tests.
- Isolated visualization venv installed at `/workspace/.rustdrive-tools/demo-venv`. `bash scripts/demo.sh assets/demo.gif` exercises release CLI and renderer. GIF dimensions/frame count are checked by Pillow and a representative PNG was inspected.

## Release-binary scenario results (seed 7)

| Scenario | Acceptance | Goal | Colliding steps | Road violations | Min clearance (m) | Localization RMSE (m) | Final speed (m/s) |
|---|---|---|---|---|---|---|---|
| mission | PASS | yes | 0 | 0 | 0.926 | 0.064 | 0.101 |
| blocked | PASS | no (expected) | 0 | 0 | 2.370 | 0.049 | 0.000 |
| lidar-fault | PASS | no (expected) | 0 | 0 | 47.736 | 0.055 | 0.000 |
| gnss-fault | PASS | no (expected) | 0 | 0 | 45.284 | 0.158 | 0.000 |

The mission completed in 31.15 **simulated seconds**, reaching 220.24 m arc length. This is not a wall-clock benchmark. The fault scenarios record 195 LiDAR-fault and 189 GNSS-fault emergency-control steps, respectively. The blocked-road vehicle stops before the obstacle.

Raw per-frame traces and summary files for these checks are under ignored `artifacts/check/`. The intentionally committed `assets/demo.json` records the displayed mission's metrics. No benchmark supports a real-time, hardware throughput, operational safety or accuracy generalization beyond these tests.

## Limitations and unexecuted checks

- CARLA is not installed, implemented or tested. The end-to-end demo uses the project's real 2D reference simulator.
- No camera AI, semantic perception, 3D SLAM, traffic-rule behavior, global routing, ROS 2 bridge or real vehicle actuator is implemented.
- Occupancy mapping is diagnostic and does not feed collision planning. The map and initial heading are supplied.
- Planning is limited to three fixed lateral targets, heuristic CV forecasts and constant-speed time approximation. Collision margins are not probability-calibrated; control/dynamics do not model tire limits or actuator delay.
- No safety certification, formal verification, realistic sensor weather/noise benchmark or hardware-in-the-loop result is claimed.
- macOS / Windows jobs and GIF generation are configured in GitHub Actions, but remote workflow execution has not been observed.
- After the user published the cloud environment, all 26 tests, the release-binary mission and GIF regeneration passed again in the reconnected environment.
- GitHub publication is separate from these local validation results. A tagged release and restoration in an independently created task have not been validated.

See [capabilities](capabilities.md), [architecture](architecture.md) and [roadmap](roadmap.md) for boundaries and the next acceptance gates.
