# Sensor contract and replay

`rustdrive-pipeline` exposes `PipelineConfig`, `SensorFrame`, `DrivingPipeline` and `PipelineOutput`. Both reference simulation and RNE call the same stateful synchronous library. No executor or middleware is needed.

## Input boundary

Configuration supplies a validated arc-length route, calibrated initial pose, vehicle geometry/limits, nominal time step and cruise speed and optional calibrated forward/braking/lateral acceleration limits. Missing optional limits select default planning limits; invalid supplied limits fail construction. The observed frame contains:

- `time`: finite, nonnegative, strictly increasing seconds on one simulation clock.
- `odometry`: optional acquisition-stamped speed and yaw rate; required at 20 Hz.
- `gnss`: optional acquisition-stamped noisy position/variance; generated at 5 Hz.
- `lidar`: optional acquisition-stamped **body-frame** x-forward/y-left returns; generated at 10 Hz.
- `navigation_update`: optional external closure snapshot, with acquisition stamp, increasing revision and complete closed-edge list; configured map required for route handling.
- `lidar_failed`: explicit acquisition failure (default false), distinct from healthy zero returns.

Absent samples do not refresh last accepted timestamps. Duplicate, out-of-order, future and invalid samples cannot refresh them either. Odometry older than 0.15 s, LiDAR older than 0.35 s or accepted GNSS older than 0.75 s brakes. Clock gaps over 0.25 s and excessive localization uncertainty brake. Regressing/non-finite clocks return an error before state mutation; adapters must stop on errors. This research health policy is intentionally conservative and does not support GNSS-denied navigation.

Ground-truth poses, object identities and physical collision results are not input fields. Unknown top-level fields are rejected. Synthetic sensors necessarily observe the simulator world; their noisy measurements cross this boundary.

## Log schema 1

`sensors.jsonl` is separate from the evaluation/display `run.json`. It contains one JSON object per line:

1. `{"kind":"header","header":{"schema_version":1,"source":"...","config":...}}`
2. One or more `{"kind":"tick","tick":{"input":...,"expected":...}}` records.
3. Mandatory `{"kind":"end","ticks":N}` count footer.

The expected record contains the complete estimate, tracks, forecasts, trajectory, command, emergency state, health and position variance. It never enters the pipeline. Replay constructs fresh state from the header, feeds only inputs, and compares reserialized outputs exactly. Serde's float-roundtrip parsing preserves recorded f64 values. No tolerance or success shortcut hides differences.

```sh
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/demo/sensors.jsonl --output artifacts/replay
```

Exit 0 means every tick matched and the count footer completed. Exit 2 means a mismatch, schema/input/clock error, empty/truncated log, extra data, or I/O failure. The reader limits each line to 8 MiB. It streams outputs and propagates flush errors. A previous `replay.json` is removed before reading the log; a new success report is written only after verification completes. `outputs.jsonl` may contain a partial prefix after failure and must not be interpreted as a verified run.

## Evidence and limits

Unit/integration tests check actual recomputation, changed commands, empty/truncated/dropped tick streams, schema/version/truth-field rejection and failing output writers. CLI tests verify exit 2 and removal of a stale success report for corrupted and missing input. Reference and RNE-generated logs are verified end to end.

Exact repeatability is tested on this Linux build/platform. The locally recorded RNE Rust 1.95.0 dynamic log also verified with the Rust 1.90.0 default CLI; general cross-platform/toolchain bitwise reproducibility is not established. Schema changes require a future migration/version decision.

Schema 1 describes the record format, not an algorithm revision. Changes such as the continuous candidate sweeps and stop/wait policy can change expected trajectories and commands. Regenerate recordings with the revised pipeline before expecting exact replay; old recordings may correctly report a mismatch. Algorithm-version migration and historical-binary replay are not implemented.

Replay verifies deterministic computation, **not** collision avoidance, goal completion or real-time execution. An incomplete or colliding physical run can have a fully reproducible log. Physical acceptance is scored independently in `summary.json`; sensor logs intentionally contain no truth evaluator inputs.

Acceleration calibration now includes `max_acceleration_m_s2` (0.1–2.0 m/s²). Older motion-limit objects without this field deserialize with 2.0 m/s²; explicitly supplied non-finite or out-of-range limits are rejected. Schema 1 remains readable, but revised speed/timing/control outputs require regenerated recordings for exact replay.

## Map-aware navigation replay

Optional `PipelineConfig.navigation` records the known graph/start/goal/initial closures and must match the initial route. Map-configured logs record `SensorFrame.navigation_update` snapshots and complete `PipelineOutput.navigation` state. Replay reruns initial routing, each accepted snapshot, stop-before-divergence transitions and route switching. It does not substitute expected routes or navigator states. Existing resolved-route logs omit these optional fields and still replay local computation without Dijkstra. A changed closure snapshot is covered by a mismatch test. Schema 1 remains readable; exact outputs are tied to this implementation. [Contract and physical evidence](handover.md).


For road-network commit `aa766836d6f79fc58c4cde5cc6560b019a8cbac1`, [run 37856849026](https://github.com/rsasaki0109/rust_drive/actions/runs/37856849026) completed all five jobs successfully.

## Live closure handover baseline (2026-10-09, Asia/Tokyo)

Local checks pass **97 workspace tests**, **11 RNE tests**, formatting, Clippy with warnings denied and locked release builds. The reference script passes fourteen scenario/replay pairs. The positive seeded suite passes **66 runs**, including 18 live-navigation runs, with zero collisions, road violations and closed-edge entry violations, full replay and unchanged per-fixture clearance/normal steering-rate constraints. Tests also cover protocol faults, late-notification failure, state retention, steering continuity and changed-snapshot replay detection.

The live fixtures explicitly use 4 m/s cruise and a 1 m/s² planned lateral bound. A separately reproduced RNE seed-7 run at 6 m/s with curvature limits omitted fails goal acceptance while staying collision-free; its CLI returns 1 and its 1401 sensor ticks still replay. One of the eleven RNE tests asserts this retained failure; it is not a successful driving episode. [Methods, actual measurements, failed summary and limitations](handover.md).

The opening README GIF is now the actual RNE live-closure run, showing braking, a pending route and stopped handover. Traffic priority and continuous moving handover remain unimplemented. For live-handover commit `cc9ba4d05c7b90685969203b7e0c8d34696e5256`, [run 37860843074](https://github.com/rsasaki0109/rust_drive/actions/runs/37860843074) completed successfully. This section records that baseline; the current extension below supersedes its counts and repairs its retained failed mission.

## Avoidance continuity (2026-10-09 UTC)

Local formatting, Clippy with warnings denied, locked release builds, **100 workspace tests** and **11 RNE tests** pass. The reference script passes fifteen scenario/replay pairs. The **72-run** suite includes 24 live-navigation runs and retains all prior clearance and normal steering-rate gates; collision, road-boundary and closed-edge entry counts are zero. Full logs recompute successfully. The former 6 m/s deadlock is now a positive fixture with the same world, duration, speed, update and acceptance criteria. Across three seeds, RNE completion takes 40.20–40.40 s with at least 1.458 m clearance; the corresponding reference runs also pass. [Methods, baseline comparison and complete snapshot](avoidance-continuity.md).

The README opening GIF is the actual RNE seed-7 6 m/s run: 40.25 s, 806 replayed ticks, 135 frames at 1200 × 720, and recorded scenario/metrics/route history matching the validated suite. Remote CI for this new revision is reported separately from the preceding observed run. These results remain limited to the authored simulation cases.

For avoidance commit `e851c2ac3eb9a0bab7f995c97fa0fb7489400cc5`, [run 37864100417](https://github.com/rsasaki0109/rust_drive/actions/runs/37864100417) completed all five jobs successfully.

## GNSS robustness baseline (2026-10-09, Asia/Tokyo)

Local formatting, Clippy with warnings denied, locked release builds, **109 workspace tests** and **13 RNE tests** pass. The reference script passes eighteen scenario/replay pairs. The seeded suite passes **90 positive scenarios**, including 18 GNSS-fault cases, with full replay, zero collision/road/closed-edge violations and all prior profile/clearance/steering constraints. New independent checks bound position error to 0.5 m and verify actual injected GNSS, observed/accepted timestamps, rejection without correction, stale braking, physical stop/recovery and control continuity.

At GNSS baseline `e35c483fa6b9d6e33240fc4871b19648d65a768d`, a separate RNE `gnss-burst-traffic` seed-7 episode failed physical acceptance: 32 collision ticks, no goal, −1.049 m minimum clearance. Its 1301 sensor ticks fully replay and the CLI correctly returns 1; this is retained failure evidence, not a successful driving run. One RNE test and a separate suite record assert its explicit rejection. [Full measurements, actual GIF and limitations](gnss-robustness.md). All five jobs passed for that baseline in [run 37871189925](https://github.com/rsasaki0109/rust_drive/actions/runs/37871189925). The terminal-stopping extension below supersedes its counts and repairs that traffic failure.

## Terminal stopping baseline (2026-10-09, Asia/Tokyo)

Local formatting, Clippy with warnings denied, locked builds, **115 workspace tests** and **13 RNE tests** pass. The reference script passes twenty scenario/replay pairs. All **102 positive seeded runs**, including 30 GNSS-fault runs, pass with full replay, zero collision/road/closure violations and unchanged physical clearance, profile and steering gates. Twelve added runs cover the original compound traffic world and its 16-second post-arrival residence variant in both plants and all three seeds.

The known e35c483 collision evidence remains historical in `gnss-results.json`; the suite now checks physical success rather than expecting that failure. A distance-only intermediate repair reached the goal before traffic caught up, but failed an eight-second post-arrival experiment with 28 collision ticks. The complete repair reserves a feasible side stopping place and passes the stricter sixteen-second experiment. An independent checker projects truth onto the original polyline, checks continuous sampled goal residence/standstill, limits movement to 0.5 m, verifies clearance and confirms the lead reached the original endpoint. A negative simulation test proves that a post-arrival collision cannot be hidden by early goal completion. [Evidence, commands and limitations](terminal-stopping.md). All five jobs passed for terminal commit `4b6d2049f39037d7ae2fb958b907bf979511ea97` in [run 37875855573](https://github.com/rsasaki0109/rust_drive/actions/runs/37875855573). The reactive-traffic extension below supersedes these counts.

## Reactive traffic baseline (2026-10-09, Asia/Tokyo)

Local formatting, warnings-denied Clippy, locked builds, **123 workspace tests** and **15 RNE tests** pass. The reference script verifies 23 scenario/replay pairs. The seeded suite passes **120 positive runs**, retaining all previous 102 cases and adding 18 reactive-traffic cases across both plants and seeds 1/7/42. Positive cases satisfy zero ego/traffic/road/closure violations, complete replay and unchanged prior profile/steering/clearance gates. New checks independently reconstruct actor range/closing observations, bounded speed/distance integration, stop/resume/queue behavior and circular actor-pair sweeps. Negative tests establish failure on actor collisions, late sensing and endpoint overrun.

Two additional RNE short-fault episodes (`traffic-follower-deadline`, seeds 1 and 42) remain collision-free but miss their 65-second goal-residence deadline. They return 1 and replay all 1301 ticks; the report and native regression explicitly require their physical rejection. These failures are excluded from the positive count. The longer positive follower case uses a [10,20) fault and 80-second evaluation to exercise real standstill and full residence; it does not repair the short deadline case. Conservative repeated ego stops near the goal remain unimplemented interaction-planning work. [Complete evidence, source fingerprint and GIF](reactive-traffic.md). Remote CI for this new revision is reported separately.


## Observed-braking prediction (2026-10-09 UTC)

Local formatting, Clippy with warnings denied, locked builds, **128 workspace tests** and **16 RNE tests** pass. The reference script passes **24 scenario/replay pairs**. All **126 positive runs** (60 reference and 66 RNE) pass with complete sensor-only replay and unchanged physical clearance, profile, normal-steering and containment gates. The original 65-second follower fixture now completes eight seconds of physical residence in both plants across seeds 1/7/42, including the two previously failed native seeds. The actor world, fault window, deadline, native pin and plant calibration were not changed.

An independent sensor-log checker verifies evidence-supported braking, no lateral invention/reversal/acceleration, bounded deceleration, one-second persistence and reacceleration fallback. The two historical deadline failures remain in `traffic-results.json`. Two new five-meter follower sensing cases still return 1 for failure of the original 1 m clearance floor, replay all 1301 ticks and are excluded from the positive count. [Actual RNE GIF, measured comparison, current evidence and remaining limitations](observed-braking.md). Publication CI for this revision is reported separately from the preceding observed [all-five-job success on `4620878`](https://github.com/rsasaki0109/rust_drive/actions/runs/37879754523).

## CPU 3D replay (2026-10-09 UTC)

The README now includes two Blender Cycles CPU GIFs of successful native RNE recordings. The closure-detour recording completes at 40.40 s; all 136 sampled scene states match recorded ego/object transforms. The follower recording completes at 64.25 s; all 216 sampled states match. Encoded GIFs are 960 × 640, approximately 2.6 MB each, with checked total playback durations of 14.9 and 22.9 seconds including the final pause. Input hashes, engine pin and physical summaries are in their committed provenance JSON files. Blender 4.3.2 was used, without a GPU.

The complete wrapper was additionally exercised on a fresh native closure-detour run: physical acceptance passed, all 809 sensor ticks replayed and a CPU-rendered preview passed scene checks. Failed physical logs are rejected by the renderer. Local `scripts/check.sh` passes formatting, warnings-denied Clippy, locked builds, all 128 workspace tests and 24 reference scenario/replay pairs. Python compilation, shell syntax and workflow lint pass. CI adds a real native single-frame CPU rendering check; full GIFs were generated and visually inspected locally. This changes visualization only, with planar physics and the existing independent acceptance retained. [Commands and boundaries](3d-demo.md).
