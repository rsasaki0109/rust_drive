# Hazard scenario validation

Recorded 2026-10-09 (Asia/Tokyo) on the Linux cloud environment after publication/reconnection. Default Rust 1.90.0, RNE integration Rust 1.95.0. CPU-only simulation; no graphics context or server required.

## Reproduce the actual runs

From the development branch `feat/shared-pipeline-rne`, with the retained environment or after `bash scripts/setup.sh` and `bash scripts/setup-rne.sh`:

```sh
bash scripts/check.sh
bash scripts/check-hazards.sh
```

The current command builds both release binaries and runs **30 cases**: four reference scenarios and six RNE dynamic scenarios, each with seeds 1, 7 and 42. It verifies every sensor-log tick with the default replay CLI, compares the verified tick count against physical evaluation steps, and checks physical acceptance separately. Python uses only the standard library. GPU, Pillow and RNE rendering are unnecessary. This page retains the initial 18-run results; [continuous planning validation](swept-planning.md) records the expanded suite and revised planner.

`artifacts/hazards/report.json` retains acceptance, replay, parameters and a fingerprint of the Rust sources, manifests, lockfiles, toolchain pins and scenarios. Raw traces, logs and replay output live beneath the same directory. The [original compact result snapshot](hazard-results.json) retains the earlier implementation's metrics/fingerprint without workspace-specific paths; it is distinct from the [expanded suite snapshot](swept-planning-results.json). The runner removes previous success reports before executing fresh runs and exits 1 for failed acceptance/replay, 2 for setup/I/O/input errors.

To check only the reference backend after building the default workspace:

```sh
python3 scripts/check_hazards.py --backend reference --seeds 1 7 42
```

## Fixtures and the repaired failure

- `occluded-crossing`: a parked circle at route s=65 m, lateral 2.8 m, radius 1.6 m initially hides a second circle at s=80 m, lateral 5 m. The second actor exists from time zero and starts crossing at 13 s. LiDAR emits only first returns. A separate acquisition fixture at x=40 m places both actors within 45 m range and verifies that the hidden actor has no returns; moving the mount past the occluder makes its surface visible. This distinguishes geometric occlusion from range exclusion.
- `cut-in`: a circle exists at s=40 m, lateral 6 m; it starts a 2.5 m/s lateral crossing at 5 s. The 2.1 m road half-width prevents a full lateral avoidance lane. It is a scheduled lateral crossing, not an interactive traffic agent or a semantic pedestrian model.
- `low-friction`: a curved road (6 m sine amplitude), center obstacle and dynamic vehicle with friction coefficient 0.2 and steering lag 0.15 s.
- `low-friction-stop`: the same calibration on a narrow road, with an obstacle at s=60 m. Stopping, rather than goal completion, is required.

Before adding calibrated planning limits, the low-friction stop reproduced **0.0207 m of overlap** and failed acceptance: the planner assumed 2.5 m/s² braking while the plant could apply only 1.962 m/s². The unchanged fixture now stops about 3.5 m clear. Acceptance criteria were retained.

The RNE adapter bounds actual longitudinal acceleration/deceleration by `mu*9.81`, while RNE's native model limits lateral tire forces. Explicit fixed calibration supplies conservative planning limits `min(2.5, 0.6*mu*9.81)` in m/s², rather than reading runtime physical truth. The planner uses that deceleration in obstacle/goal speed envelopes and bounds path speed by sampled curvature (`v²*|curvature| <= lateral_acceleration`). It prioritizes unblocked candidates when those limits are enabled. Default uncalibrated reference/nominal RNE behavior is retained.

## Initial 18-run results

All runs passed with zero colliding evaluation steps and zero road-boundary violations. Worst clearance and maximum localization error aggregate all three seeds; time and replay count are seed 7 values. Time is **simulated**, not a wall-clock benchmark.

| Backend | Scenario | Passing seeds | Worst clearance (m) | Max localization error (m) | Seed 7 time (s) | Seed 7 replay ticks |
|---|---|---|---|---|---|---|
| reference | cut-in | 3/3 | 0.657 | 0.182 | 21.30 | 427 |
| reference | occluded-crossing | 3/3 | 1.844 | 0.182 | 20.90 | 419 |
| rne-dynamic | cut-in | 3/3 | 0.629 | 0.207 | 22.90 | 459 |
| rne-dynamic | low-friction | 3/3 | 0.837 | 0.278 | 28.60 | 573 |
| rne-dynamic | low-friction-stop | 3/3 | 3.493 | 0.209 | 40.00 | 801 |
| rne-dynamic | occluded-crossing | 3/3 | 1.240 | 0.292 | 25.75 | 516 |

For both low-friction fixtures, measured frame-to-frame longitudinal acceleration is bounded by 1.962 m/s² within floating-point tolerance. A separate actuation test verifies both acceleration and emergency braking at the 20 Hz control boundary. Tests reject friction calibration on reference/kinematic plants rather than silently ignoring it.

At the original revision, local formatting and Clippy passed for both workspaces; **45 default-workspace tests** and **8 RNE-adapter tests** passed. Additional regressions cover delayed motion, invalid calibration, braking/goal envelopes, curvature limits, and collisions from newly active actors or overlap on the final evaluation tick. Existing object sweeps and current/terminal overlaps contribute at most one collision count per evaluated tick; newly active objects are checked at their first sampled endpoint rather than swept backward before existence.

After the earlier cloud environment publication, the 37-test/5-adapter-test baseline and reference/RNE runs replayed successfully. The original hazard extension subsequently passed [all five remote CI jobs on attempt 2](https://github.com/rsasaki0109/rust_drive/actions/runs/37802806434/attempts/2). Independent fresh-task restoration remains untested.

## Remaining limits

These fixtures cover supplied planar roads and circular, scheduled actors. They do not establish traffic-rule behavior, universal collision avoidance, online friction estimation, camera/3D perception, or realistic physical calibration. Motion limits are known fixed calibration; spatially varying or unknown friction is not supported. Longitudinal and lateral friction limits are separate; a combined friction ellipse and slip-aware MPC are not implemented. RNE blends into kinematics near standstill. Curvature is sampled, speed is conservatively constant across a short horizon, and forecasts use a constant-speed time approximation; this is not joint dynamic trajectory optimization.

Swept evaluation assumes linear relative motion between 20 Hz samples for already active actors. Newly active actors receive endpoint checks; continuous birth-time collision handling and substep road containment remain future work. Replay proves matching computation, not physical acceptance or operational safety.

## Render the occlusion run

With the visualization venv active:

```sh
python3 scripts/render_demo.py artifacts/hazards/rne-dynamic/occluded-crossing/seed-7/run.json \
  --output artifacts/hazards/occlusion.gif
```

The top-down animation uses actual telemetry. The committed `assets/hazard-demo.gif` and sidecar provenance were generated from this run. No trajectory or success metric is scripted by the renderer.
