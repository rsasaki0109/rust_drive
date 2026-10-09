# Sensor contract and replay

`rustdrive-pipeline` exposes `PipelineConfig`, `SensorFrame`, `DrivingPipeline` and `PipelineOutput`. Both reference simulation and RNE call the same stateful synchronous library. No executor or middleware is needed.

## Input boundary

Configuration supplies a validated arc-length route, calibrated initial pose, vehicle geometry/limits, nominal time step and cruise speed and optional calibrated forward/braking/lateral acceleration limits. Missing optional limits select default planning limits; invalid supplied limits fail construction. The observed frame contains:

- `time`: finite, nonnegative, strictly increasing seconds on one simulation clock.
- `odometry`: optional acquisition-stamped speed and yaw rate; required at 20 Hz.
- `gnss`: optional acquisition-stamped noisy position/variance; generated at 5 Hz.
- `lidar`: optional acquisition-stamped **body-frame** x-forward/y-left returns; generated at 10 Hz by default. Delivery time is the enclosing frame's `time`, not a replacement for the scan's acquisition stamp.
- `navigation_update`: optional external closure snapshot, with acquisition stamp, increasing revision and complete closed-edge list; configured map required for route handling.
- `lidar_failed`: explicit acquisition failure (default false), distinct from healthy zero returns.

Absent samples do not refresh last accepted timestamps. Duplicate, out-of-order, future and invalid samples cannot refresh them either. Odometry older than 0.15 s, LiDAR older than 0.35 s or accepted GNSS older than 0.75 s brakes. Clock gaps over 0.25 s and excessive localization uncertainty brake. Regressing/non-finite clocks return an error before state mutation; adapters must stop on errors. This research health policy is intentionally conservative and does not support GNSS-denied navigation.

Ground-truth poses, object identities and physical collision results are not input fields. Unknown top-level fields are rejected. Synthetic sensors necessarily observe the simulator world; their noisy measurements cross this boundary.

## Bounded acquisition-time LiDAR reprojection

Fresh monotonic scans use the EKF pose estimate at acquisition, rather than the pose at delivery, for both detection coordinates and occupancy-grid ray origins. The private history starts at the first pipeline step, after its odometry prediction and GNSS correction; no earlier pose is fabricated. A lookup accepts at most 0.35 s of age and retains at most 64 estimates, with one predecessor for interpolation at the age boundary. An exact stored timestamp returns its original pose. Otherwise position and the shortest wrapped yaw arc interpolate between surrounding estimates whose gap is at most 0.25 s. There is no extrapolation. A new scan with expired or uncovered history adds `InvalidLidar`, brakes, and leaves tracks, map and last accepted scan stamp unchanged. Duplicate/older accepted scan stamps remain ignored without refreshing health.

The acquisition stamp is also retained for tracking and freshness. Priority-crossing permission still requires an accepted scan no more than 0.15 s old; the broader 0.35 s history/health bound does not relax it. A transport delay can therefore trigger withholding permission or braking even when a scan can be geometrically transformed.

This is bounded reprojection through past estimates, not a full delayed-sensor estimator. Later GNSS corrections do not retroactively smooth history; delayed odometry/GNSS fusion, per-point LiDAR deskew and covariance propagation are absent. Forecasts still start at the last acquired track position instead of being propagated to delivery time. Do not infer general latency tolerance from an accepted replay or the authored timing fixtures.

## Simulator-only LiDAR delivery injection

An optional scenario `sensor_timing` object configures delivery to the shared driver. Ticks are 0.05 s of simulated time:

```json
{
  "sensor_timing": {
    "lidar_period_ticks": 2,
    "lidar_delay_ticks": 1,
    "lidar_failure_windows": [{"from": 14.05, "until": 14.30}]
  }
}
```

`lidar_period_ticks` must be even and in 2–10: delivered observation cadence ranges from 10 Hz to 2 Hz. `lidar_delay_ticks` is 0–6, or 0–0.30 s. These are supported injection ranges, not guaranteed operating tolerances. The existing adapters continue acquiring at 10 Hz; the common simulator layer selects scans, queues their unchanged body-frame points/stamps, and delivers them later. It does not reduce native ray-query work or measure wall-clock latency. Omitting the object retains the previous transport path.

At most 32 finite, sorted, nonoverlapping failure windows within the scenario duration are allowed. A `[from, until)` window immediately emits `lidar_failed`, suppresses a scan and flushes queued acquisitions. A backend acquisition error has the same queue-flushing behavior. Recovery must acquire a new scan; an old queued scan cannot restore health. This models an explicit sensor failure report rather than an undetected communications outage. Dropout and freshness tests remain separate.

The scenario's timing/delay/failure schedule stays outside the operational replay header. Logs contain delivered observations with their actual acquisition stamps and explicit failure flags. Replay rebuilds acquisition-pose history from those observations and recomputes every output; neither expected history nor simulator poses are injected. Schema 1 stays readable, but changed delayed-scan arithmetic requires fresh expected recordings.

```sh
# 100 ms delayed observations, with the original acquisition stamps.
cargo run --release --locked --bin rustdrive -- run \
  --scenario scenarios/intersection-delay-two.json --seed 7 \
  --output artifacts/intersection-delay
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/intersection-delay/sensors.jsonl \
  --output artifacts/intersection-delay/replay

# 50 ms delay and an explicit transient acquisition failure.
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --plant dynamic --scenario scenarios/intersection-lidar-recovery.json --seed 7 \
  --output artifacts/rne-intersection-recovery
```

The physical clearance/priority gates and replay checks are independent. [Intersection fixtures and measured acceptance](intersections.md).

## Log schema 1

`sensors.jsonl` is separate from the evaluation/display `run.json`. It contains one JSON object per line:

1. `{"kind":"header","header":{"schema_version":1,"source":"...","config":...}}`
2. One or more `{"kind":"tick","tick":{"input":...,"expected":...}}` records.
3. Mandatory `{"kind":"end","ticks":N}` count footer.

The expected record contains the complete estimate, tracks, forecasts, trajectory, command, emergency state, health, position variance and optional navigation state. It never enters the pipeline. Replay constructs fresh state from the header, feeds only inputs, and compares reserialized outputs exactly. Serde's float-roundtrip parsing preserves recorded f64 values. No tolerance or success shortcut hides differences.

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

## GNSS acceptance diagnostics

Optional output `localization` reports last observed/accepted GNSS stamps, the latest valid-new-fix decision and finite NIS, and accepted/rejected counts. Rejected fixes cannot refresh health; their receipt prevents subsequent duplicate/older observations from being processed. Invalid input continues to use health diagnostics. Replay recomputes the joint gate and all diagnostics. Changing a GNSS observation or its expected counters is covered by mismatch tests. Fault-window labels exist only in simulator configuration; recorded sensor observations contain the actual bias without labels. Schema 1 remains readable, but new correction arithmetic and output diagnostics require regenerated expected outputs. Boxed record payloads preserve JSON encoding. [GNSS baseline recovery](gnss-robustness.md) and [current terminal stopping](terminal-stopping.md).

`goal_hold_seconds` belongs to simulator acceptance and is absent from the pipeline/replay header. Replaying an extended episode recomputes all post-arrival sensor outputs and commands; it does not independently establish the physical residence or clearance. Those checks use evaluator truth. Changed goal-profile arithmetic and terminal preferences require fresh expected outputs; no sensor-log format change is introduced.

Optional `run.json` frame `traffic` contains simulator actor truth and preceding proximity observation/command, with full 20 Hz truth frames for reactive episodes. This is not part of the version-1 sensor log or pipeline input. Replay verifies only the recomputed ego stack; independent physical checks reconstruct actor sensing, bounded integration and circular pair clearance from evaluator truth. Scenario following parameters and stop windows never enter the header. [Boundary and actual results](reactive-traffic.md).

Observed-braking forecasts keep track history inside the shared pipeline. Replay reconstructs that history from newly accepted LiDAR-derived tracks; expected tracks and forecasts are never fed back as inputs. Forecast fields and log schema remain unchanged. Old constant-velocity recordings may correctly mismatch; use the historical implementation for historical replay, or regenerate current recordings. [Observation support, physical results and limits](observed-braking.md).
