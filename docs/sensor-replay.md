# Sensor contract and replay

`rustdriving-pipeline` exposes `PipelineConfig`, `SensorFrame`, `DrivingPipeline` and `PipelineOutput`. Both reference simulation and RNE call the same stateful synchronous library. No executor or middleware is needed.

## Input boundary

Configuration supplies a validated arc-length route, calibrated initial pose, vehicle geometry/limits, nominal time step and cruise speed and optional calibrated forward/braking/lateral acceleration limits. Missing optional limits select default planning limits; invalid supplied limits fail construction. The observed frame contains:

- `time`: finite, nonnegative, strictly increasing seconds on one simulation clock.
- `odometry`: optional acquisition-stamped speed and yaw rate; required at 20 Hz.
- `gnss`: optional acquisition-stamped noisy position/variance; generated at 5 Hz.
- `lidar`: optional acquisition-stamped **body-frame** x-forward/y-left returns; generated at 10 Hz by default. Delivery time is the enclosing frame's `time`, not a replacement for the scan's acquisition stamp.
- `navigation_update`: optional external closure snapshot, with acquisition stamp, increasing revision and complete closed-edge list; configured map required for route handling.
- `lidar_failed`: explicit acquisition failure (default false), distinct from healthy zero returns.

Absent samples do not refresh last accepted timestamps. Duplicate, out-of-order, future and invalid samples cannot refresh them either. Odometry older than 0.15 s, LiDAR older than 0.35 s or accepted GNSS older than 0.75 s brakes. Clock gaps over 0.25 s and excessive localization uncertainty brake. Regressing/non-finite clocks return an error before state mutation; adapters must stop on errors. This research health policy is intentionally conservative and does not support GNSS-denied navigation.

Malformed odometry retains `InvalidOdometry` braking until a valid acquisition
strictly later than the latest failure's delivery time arrives within the normal
0.15 s age limit. Missing, duplicate, pre-fault and expired observations cannot
clear this hold or replace the last accepted measurement. Another malformed
observation advances the recovery cutoff. Estimation may continue from the
accepted cache within its normal age limit while commands remain emergency
braking. Ordinary missing samples without a preceding malformed observation
retain their existing age-based grace and recovery behavior.

Malformed GNSS retains `InvalidGnss` braking until the EKF accepts a valid fix
acquired strictly after the latest failure's delivery time and within the normal
0.75 s age limit. Missing, duplicate, pre-fault and expired fixes cannot clear
the hold or enter GNSS correction. A new malformed observation advances the
cutoff. A post-fault innovation rejection keeps the hold; accepted map updates
also cannot clear it. Normal missing-fix grace, bounded map coverage of an
ordinary outage and the existing consecutive-innovation policy are unchanged.
A single structurally valid innovation outlier alone does not create this
malformed-observation hold.

An invalid infrastructure-signal snapshot retains `InvalidTrafficSignal` braking
until a complete valid snapshot arrives with an acquisition time strictly after
the latest failure's delivery time and an age within 0.5 s. Missing, duplicate,
pre-fault and expired snapshots cannot replace the accepted snapshot or clear
the fault. Every malformed snapshot advances the cutoff, including one carrying
an old timestamp. A fresh post-fault red or unknown snapshot clears the format
fault but still imposes its normal stop constraint. The fault also keeps
uncommitted intersection clear-confirmation timers reset; only recovery can
start a new full confirmation interval.

Schema 1 remains readable, but earlier recordings spanning invalid sensor,
signal or navigation observations may require regenerated expected outputs for
the extended braking hold.

Every LiDAR input mode retains `InvalidLidar` braking after an explicit acquisition
failure, malformed or conflicting representation, or missing acquisition-pose
transform. Recovery requires a complete valid acquisition whose stamp is strictly
later than the most recent failure's delivery time. Absent samples, duplicates
and pre-fault acquisitions do not clear this hold even while the last accepted
scan remains within its normal age limit. A valid delayed post-fault acquisition
can recover within the existing age/history bounds; a successful empty scan is
valid. A simultaneous scan and failure flag cannot recover, and another failure
advances the required recovery epoch. Rejected scans do not update tracks,
occupancy or the last accepted acquisition stamp. This applies to ordinary 2D
LiDAR as well as the existing multi-height and XYZ paths.

The ordinary 2D hold repairs premature recovery on the frame after a fault.
The JSONL schema remains 1, but prior ordinary-input recordings spanning faults
may have different expected health/commands and must be recorded again for the
updated behavior. Historical research packets are retained without rewriting
their expected outputs. Six public-pipeline regression tests cover the hold,
malformed returns, contradictory inputs, repeated failures, missing transforms,
delayed recovery and full sensor-only replay including rejection of a forged
pre-fault recovery stamp. This simulation policy does not establish real-vehicle
safety or sensor-fault rates.

The reusable pure-pursuit controller clears both its steering memory and PI
speed integral whenever it emits an emergency command, including invalid first
segment durations and moving single-point trajectories. The next valid call
therefore starts from the emitted zero steering with the existing 0.7 rad/s
normal steering-rate limit and no stale integral acceleration. The pipeline
also synchronizes controller state after its independent command guard.
Finite trajectory inputs can still overflow during feedforward, proportional
feedback, integral accumulation or their unsaturated sum. These intermediate
results are checked before actuator/integral clamps; a nonfinite result emits
the same emergency command and clears state. Finite saturation and the existing
stationary single-point hold retain their behavior. This repairs standalone
control-library error handling; it adds no physical steering-actuator model or
real-vehicle validation.

The controller also rejects nonfinite intermediate displacements, distances
and circle-intersection arithmetic when selecting its pursuit target. Finite
coordinates can still overflow these calculations; interpolation clamps must
not turn that failure into a normal acceleration command. Rejection emits the
same state-clearing emergency command, so the next valid trajectory resumes
from zero steering and integral feedback. Large coordinates whose required
calculations remain finite retain normal control and feedback.

Each standalone controller call also checks the vehicle calibration it consumes:
wheelbase must be finite and positive in meters, and the steering-angle limit
must be finite and strictly between 0 and pi/2 radians. Invalid values return
the state-clearing emergency command before angle clamps can panic. This is
checked on every call because the public vehicle settings may change after
construction. A nonfinite computed steering angle also stops and clears state.
Repairing the calibration resumes from zero steering and integral feedback.
These geometry checks accept valid small-vehicle calibrations; the pipeline's
separate simulation-specific size and steering limits remain stricter. Vehicle
radius is unused by this controller and remains a planning/pipeline input.

When a valid runtime steering-limit reduction excludes the last emitted
steering command, the controller emits one state-clearing emergency command.
The next valid call resumes from zero within the new angle and normal rate
limits. A reduction that still contains the last command, or an increase,
preserves steering continuity and PI feedback. Final normal steering is bounded
again after the rate update so rounding near saturation cannot exceed the angle
limit and trigger a false emergency on the next call.

Every declared trajectory point has a finite, nonnegative time in seconds from
the current sensor frame, and every successive point must have a strictly
later time. The first point must be at time 0: the controller uses its speed as
the current target and the first segment for acceleration feedforward. It does
not extrapolate a future-only trajectory back to the current frame, including
for stationary holds. The standalone controller validates the entire timeline
before computing a normal command, including points beyond its preview.
A future-only trajectory, a repeated or reversed later timestamp, or a negative
relative time emits the state-clearing emergency command. Valid nonuniform
time intervals and stationary holds at repeated positions with increasing times
remain supported; a single stationary point at time 0 is also valid.

The standalone command guard requires finite summed x/y position variance in
the inclusive range 0–4 m², matching the pipeline's localization health check.
A negative variance is invalid and returns the emergency command rather than
being treated as high confidence or clamped to zero. Both range boundaries
remain valid; healthy inputs retain the existing command bounds and sensor-age
checks.

Standalone EKF prediction checks the computed pose and all entries of the 3×3
covariance before applying either result. Overflow leaves pose, speed,
covariance and GNSS acceptance diagnostics unchanged, allowing the next valid
prediction and correction to proceed. This includes the zero-offset chassis
prediction that delegates to the default predictor. The existing void prediction
API supplies no rejection status; embedding applications still need to validate
odometry and enforce their health policy. The shared pipeline already bounds
odometry and prediction time steps before calling the estimator.

Ground-truth poses, object identities and physical collision results are not input fields. Unknown fields are rejected, including inside nested measurements, coordinates, configuration and expected outputs. Synthetic sensors necessarily observe the simulator world; their noisy measurements cross this boundary.

## Bounded acquisition-time LiDAR reprojection

An optional [multi-height input](multi-height-lidar.md) uses `SensorFrame.multi_height_lidar` instead of `lidar`. It records one acquisition stamp and the raw measured body-XY points of every calibrated horizontal plane. The header supplies expected plane heights and a vertical vehicle interval; it contains no scene geometry. The pipeline validates the complete bundle, projects relevant heights and applies the same acquisition-pose lookup. Missing/invalid planes latch braking until a new complete post-fault acquisition. Simulator timing transport queues and flushes whole bundles. The added optional fields are omitted for ordinary runs, preserving previous schema-1 logs; earlier binaries that do not know these fields cannot replay the new mode.

The optional [inclined XYZ input](lidar-3d.md) uses `SensorFrame.lidar3d` and header `PipelineConfig.lidar3d` exclusively. Each actual native return retains a firing ordinal and body-forward/body-left/road-datum-up point. The calibration supplies the uniform ring geometry, range limits, mount height and collision-height interval. Replay validates every return, including excluded overhead points, before deterministic height selection and 5 cm XY deduplication. The same acquisition-pose transform and post-fault freshness latch apply; timing queues and drops a complete cloud atomically. An empty successful cloud differs from an acquisition failure. Ordinary, horizontal multi-height and XYZ representations cannot be mixed. The header contains no boxes, native world poses or truth labels, and the optional fields remain omitted in preceding modes. The acquisition is instantaneous; no per-beam motion deskew is claimed.

Fresh monotonic scans use the continuous wheel/gyro pose at acquisition, rather than the pose at delivery. The private history starts at the first pipeline step; no earlier pose is fabricated. It accumulates only accepted EKF prediction increments and excludes GNSS/fixed-map correction increments. A lookup accepts at most 0.35 s of age and retains at most 64 estimates, with one predecessor for interpolation at the age boundary. An exact stored timestamp returns its original continuous-frame pose. Otherwise position and the shortest wrapped yaw arc interpolate between surrounding estimates whose gap is at most 0.25 s. There is no extrapolation. Detections and motion history remain in that continuous frame; published tracks, predictions and occupancy-grid ray origins use its transform into the current localized world. A new scan with expired or uncovered history adds `InvalidLidar`, brakes, and leaves continuous-frame tracks, map and last accepted scan stamp unchanged. Duplicate/older accepted scan stamps remain ignored without refreshing health. A localization correction may reproject cached published tracks without renewing their observation times.

The acquisition stamp is also retained for tracking and freshness. Priority-crossing permission still requires an accepted scan no more than 0.15 s old; the broader 0.35 s history/health bound does not relax it. A transport delay can therefore trigger withholding permission or braking even when a scan can be geometrically transformed.

This is bounded reprojection through continuous odometry history, not a full delayed-sensor estimator. Current GNSS/map corrections update the world transform without becoming object velocity. They do not correct odometry drift within that history. Delayed odometry/GNSS fusion, per-point LiDAR deskew and covariance propagation remain absent, and historical occupancy cells are not rebuilt after a correction. Current-time motion forecasts separately propagate acquired tracks to the control clock, as described below. Neither step establishes general latency tolerance from an accepted replay or the authored timing fixtures. [Coordinate correction and replay details](tracker-localization-frame.md).

## Per-track observation intervals

Track velocity correction uses the elapsed acquisition time since that identity
was last observed, including intervening missed detections. Empty scans or
other visible objects cannot shorten this interval. The 0.6 s observation expiry
and zero initial velocity for a new identity remain in force. [The timing
repair and independent log audit](tracker-observation-time.md) document this
contract and the expected mismatch for historical logs affected by the old
scan-interval correction.

The pipeline uses `Tracker::try_update` to distinguish accepted acquisitions
from tracking rejection. Rejection preserves tracks, occupancy and the last
accepted LiDAR stamp, activates `InvalidLidar` braking and requires a new valid
post-fault acquisition. The standalone checked API also rejects duplicate/older
times and invalid detections without partially updating or expiring tracks.
The legacy `update` method returns cached tracks on rejection; callers must not
interpret that return value alone as a successful scan. [API and failure
checks](tracker-observation-time.md#rejected-acquisitions).

## Forecast time origin

Tracks retain their acquisition-time position, velocity and `last_seen` stamp. The shared pipeline calls `ObservedBraking` with the enclosing frame's current `time`; every returned forecast begins at that control time, with 41 positions spaced 0.2 s apart through the next eight seconds. Its samples evaluate acquired motion at `age + future_offset`, where `age = time - last_seen`. Tracks and acquisition stamps are not overwritten by extrapolated estimates. Both the collision planner and priority-zone sweeps consume these same current-time forecasts.

For constant velocity, the first predicted position is `track.position + track.velocity * age`. The default retains the 0.7 m/s static deadband. Optional `confirm_slow_motion: true` requires four directionally consistent position/velocity observations over approximately 0.6 s before propagating slow motion; unsupported tracks remain stationary. The [experimental contract and adoption limit](slow-motion-prediction.md) specify minimum speed, displacement, timing and the retained follower-deadline failure. The optional flag defaults to false and is omitted from legacy-compatible headers in that case. A supported braking hypothesis integrates from acquisition through the elapsed age before producing the first sample. Its one-second braking budget starts at acquisition and is not renewed on intervening control ticks; any remaining forecast coasts at the reduced nonnegative speed. Braking support expires once observation age exceeds 0.15 s, selecting the current-time constant-velocity fallback instead. Invalid, non-finite or future-stamped track motion produces an empty forecast, which downstream planning rejects and brakes on; old finite tracks remain subject to existing tracking and sensor-health policies.

This is a single observed-motion extrapolation, without uncertainty growth, covariance propagation, actor intent or multiple hypotheses. The public forecast fields remain unchanged: the origin is the enclosing pipeline output time, not a new serialized timestamp. The standalone `ConstantVelocity::predict` trait still returns acquisition-relative samples; the shared pipeline's `ObservedBraking::predict(tracks, time)` establishes the current-time contract. Schema 1 remains readable, but earlier acquired-origin forecast recordings can correctly mismatch when replayed by this implementation. Regenerate expected recordings for current arithmetic; historical algorithm migration is not implemented. Earlier accidental lag from acquired-origin forecasts is not retained as a clearance mechanism; empirical collision margins are separate from acquisition age and do not establish a certified prediction/control error bound.

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
cargo run --release --locked --bin rustdriving -- run \
  --scenario scenarios/intersection-delay-two.json --seed 7 \
  --output artifacts/intersection-delay
cargo run --release --locked --bin rustdriving -- replay \
  --log artifacts/intersection-delay/sensors.jsonl \
  --output artifacts/intersection-delay/replay

# 50 ms delay and an explicit transient acquisition failure.
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --plant dynamic --scenario scenarios/intersection-lidar-recovery.json --seed 7 \
  --output artifacts/rne-intersection-recovery
```

The physical clearance/priority gates and replay checks are independent. The final current-time prediction sweep passes 276 physical episodes with all 228,961 sensor ticks fully recomputed, including 24 timing-injection intersection episodes. This verifies the authored cases, not the entire supported injection range or real-time resource latency. [Intersection fixtures and measured acceptance](intersections.md#repaired-late-conflict) and [source/checker fingerprints](../assets/prediction-epoch-results.json).

## Log schema 1

Optional `lane_change.return_on_conflict` supplies surveyed source-lane geometry
and a maximum early-return length. Fresh measured motion supplies initial
curvature; accepted scans/forecasts supply source confirmation and conflicts.
The selected return request and clearance diagnostics are recomputed from those
inputs. Original destination geometry remains in the header; expected return
decisions never enter the driver. [Early return contract](lane-change-returns.md).

Optional `resume_after_return` inside that policy starts a new observation epoch
after the measured return stop. It reconstructs one stationary-pose connector
to the original target tail using fresh post-stop scans and motion. Resume
requests and decisions remain comparison outputs; they are never inputs to the
driver. Missing configuration preserves the return-and-hold serialization.
[Resume contract and independent audit](lane-change-resumes.md).

An early resume hold may temporarily select a source-contained prefix of the
fixed connector. Stop/fault/clear epochs and release decisions are recomputed
from observations in replay. `launch_hold` diagnostics are absent until a hold
occurs; they are comparison outputs, never authorization inputs.
[Launch hold contract](lane-change-launch-holds.md).

Successive source holds independently reset their stop/clear epochs. Optional
`cycle` and `previous_holds` diagnostics identify the current hold and at most
four completed holds; they are omitted for the preceding single-hold format.
An optional `limit_event` reports a later conflict after stopping-room commitment.
Replay recomputes its commitment budget and observation from inputs. These
diagnostics remain comparison evidence rather than operational permissions.

An optional `lane_change` header configuration supplies the authored target-lane
axis and connector entry/exit arc lengths. The shared observed-gap gate derives
waiting/ready/changing/completed state and front/rear gap diagnostics from the
usual accepted LiDAR forecasts and sensor health; its complete output is compared
exactly during replay. Simulator `world_motion` actor paths and schedules remain
outside the operational header. Old logs omit these optional fields and retain
their encoding. [Admission contract and limits](lane-change-gaps.md).

`sensors.jsonl` is separate from the evaluation/display `run.json`. It contains one JSON object per line:

1. `{"kind":"header","header":{"schema_version":1,"source":"...","config":...}}`
2. One or more `{"kind":"tick","tick":{"input":...,"expected":...}}` records.
3. Mandatory `{"kind":"end","ticks":N}` count footer.

The expected record contains the complete estimate, tracks, forecasts, trajectory, command, emergency state, health, position variance and optional navigation state. It never enters the pipeline. Replay constructs fresh state from the header, feeds only inputs, and compares reserialized outputs exactly. Serde's float-roundtrip parsing preserves recorded f64 values. No tolerance or success shortcut hides differences.

Shared transport structs and navigation diagnostics reject unknown JSON fields
instead of silently dropping them, both inside replay and when decoded directly.
This catches misspelled or additional measurement, calibration and diagnostic
fields before the affected record is processed. Known optional fields still
accept explicit `null`, and existing defaulted fields may still be omitted.
Valid schema-1 logs retain their encoding and recomputed outputs.

`SensorLog::write` enforces the same 8 MiB encoded-line limit as the reader,
including JSON escape sequences and the final newline. It reuses one bounded
record buffer and serializes borrowed header/tick data. An oversized record
returns an error before any of that record reaches the destination; earlier
complete records may remain without a footer and do not form a verified log.
Destination write and final flush errors are still propagated. Valid records
retain their existing schema and byte encoding.

Before JSON encoding, the writer also rejects NaN and either infinity in every
serialized numeric field, including nested sequences and `Some` values. JSON's
normal conversion of nonfinite numbers to `null` must not erase optional
calibration or diagnostics, or produce unreadable required numeric fields.
Errors identify the header or tick index and leave that record unwritten.
Legitimate absent options, text such as `"NaN"`, negative zero, subnormal values
and other finite numbers keep their original meaning and encoding.

Before writing any record, the writer preflights every frame clock: it must be
finite, nonnegative and strictly increasing. Each expected output time must have
the same f64 bits as its input frame time, including the sign of zero. The header
must use the supported schema, provide a nonempty source, and contain a
configuration that can initialize a fresh pipeline. Invalid clocks or headers
are rejected before any log bytes are written. Nonzero start times, nonuniform intervals,
ClockGap health events and finite invalid sensor-acquisition stamps remain
recordable. Replay still recomputes and compares every complete output.

```sh
cargo run --release --locked --bin rustdriving -- replay \
  --log artifacts/demo/sensors.jsonl --output artifacts/replay
```

Exit 0 means every tick matched and the count footer completed. Exit 2 means a mismatch, schema/input/clock error, empty/truncated log, extra data, or I/O failure. The reader limits each line to 8 MiB. It streams outputs and propagates flush errors. After the output-path checks below pass, a previous `replay.json` is removed before reading the log; a new success report is written only after verification completes. `outputs.jsonl` may contain a partial prefix after failure and must not be interpreted as a verified run.

Before replacing either replay artifact, the CLI rejects output destinations
that already refer to the input log. A collision returns exit 2 and preserves
the input and existing replay artifacts. Canonical paths catch direct and
symbolic-link aliases; on Unix, hard-link aliases are also rejected. A distinct
input log in the same directory as the replay outputs remains supported.

After these checks, the CLI unlinks existing artifact files and creates fresh
files with exclusive creation. This replaces symbolic links (including dangling
links) and hard links without writing to their targets. Links between the two
artifact names cannot merge the command stream and success report. Directories
at either artifact path cause an I/O error. The output directory and its ancestors
must remain stable during replay; concurrent filesystem changes are not covered.

## Evidence and limits

Unit/integration tests check actual recomputation, changed commands, empty/truncated/dropped tick streams, schema/version/truth-field rejection and failing output writers. CLI tests verify exit 2 and removal of a stale success report for corrupted and missing input. Reference and RNE-generated logs are verified end to end.

Exact repeatability is tested on this Linux build/platform. The locally recorded RNE Rust 1.95.0 dynamic log also verified with the Rust 1.90.0 default CLI; general cross-platform/toolchain bitwise reproducibility is not established. Schema changes require a future migration/version decision.

Schema 1 describes the record format, not an algorithm revision. Changes such as the continuous candidate sweeps and stop/wait policy can change expected trajectories and commands. Regenerate recordings with the revised pipeline before expecting exact replay; old recordings may correctly report a mismatch. Algorithm-version migration and historical-binary replay are not implemented.

Replay verifies deterministic computation, **not** collision avoidance, goal completion or real-time execution. An incomplete or colliding physical run can have a fully reproducible log. Physical acceptance is scored independently in `summary.json`; sensor logs intentionally contain no truth evaluator inputs.

Acceleration calibration now includes `max_acceleration_m_s2` (0.1–2.0 m/s²). Older motion-limit objects without this field deserialize with 2.0 m/s²; explicitly supplied non-finite or out-of-range limits are rejected. Schema 1 remains readable, but revised speed/timing/control outputs require regenerated recordings for exact replay.

## Map-aware navigation replay

Optional `PipelineConfig.navigation` records the known graph/start/goal/initial closures and must match the initial route. Map-configured logs record `SensorFrame.navigation_update` snapshots and complete `PipelineOutput.navigation` state. Replay reruns initial routing, each accepted snapshot, stop-before-divergence transitions and route switching. It does not substitute expected routes or navigator states. Existing resolved-route logs omit these optional fields and still replay local computation without Dijkstra. A changed closure snapshot is covered by a mismatch test. Schema 1 remains readable; exact outputs are tied to this implementation. [Contract and physical evidence](handover.md).

Malformed newer navigation updates retain `Fault` and `InvalidNavigation`
braking until a valid snapshot arrives with an acquisition time strictly after
the latest failure's delivery time. The existing one-second delivery-age limit and increasing
accepted revision/stamp requirements still apply. A valid pre-fault update
cannot clear the fault or alter accepted closures, even with a higher revision.
Each new malformed update advances the cutoff and cancels any pending handover.
Recovery starts its own three healthy stopped samples before a route change;
a valid snapshot that leaves no usable route remains `Blocked`.

Route handover counts three distinct accepted odometry acquisitions with both
measured and estimated absolute speed at most 0.05 m/s. Cached, duplicate or
reordered observations do not advance the count, including a cached stop from
before the closure request. The existing 0.15-second odometry age bound still
applies. Within that bound, a healthy cached stop can retain earlier confirmation;
an unhealthy frame resets the count and requires acquisitions strictly after
that frame before confirmation resumes. Corridor and heading checks remain
required on every frame. Unconfigured signal input invalidates confirmation
before the route can switch. Replay recomputes these decisions from recorded
motion; older logs that switched using repeated motion need new expected outputs.

## Stop-sign motion evidence

An unserved stop sign needs two seconds supported by distinct accepted odometry
acquisitions, each at most 0.05 seconds old and at most 0.05 seconds apart. Both
measured and estimated absolute speed must be at most 0.05 m/s, with the existing
healthy sensing, near-line margin, heading and corridor checks. Confirmation
starts at the first qualifying delivery time and advances only to a newer motion
acquisition time. Cached, duplicate and reordered motion cannot advance `held_s`
or release the sign. An acquisition gap restarts confirmation even when no
pipeline call observed the gap. Unhealthy sensing resets it and requires motion
acquired after that frame; a sign already served remains released. Replay
recomputes the interval, and older logs that credited unobserved time need new
expected outputs. This is the fixed 20 Hz research policy, not a claim of
continuous physical standstill between observations.

## Intersection clear evidence

Uncommitted intersection permission needs clear scans acquired strictly after
the latest unhealthy, stale or unaligned permission check. A delayed scan from
before or at that control time cannot start a new confirmation interval.
Distinct accepted acquisitions must span at least one second, include at least
three scans and stay at most 0.2 seconds apart. A longer acquisition gap restarts
confirmation and revokes uncommitted permission, even when no intervening
pipeline call observed stale data. The existing 0.15-second scan-age limit,
forecast conflict checks and committed-crossing behavior still apply. Regular
5 Hz scans and 10 Hz scans with bounded delay remain supported; startup or
recovery may wait for an additional post-failure acquisition. Replay recomputes
this evidence, so logs that used earlier or interrupted clear intervals need
new expected outputs. These sampled checks do not establish continuous physical
visibility or general right-of-way safety.

## Acquisition-time GNSS replay

Optional `gnss_latency_compensation: true` in the sensor-only configuration
reconstructs a GNSS correction at its acquisition time and replays at most
0.5 seconds / 64 accepted wheel/gyro prediction intervals. The original NIS
gate, Joseph correction and acquisition-based freshness remain in force.
`gnss_latency` output diagnostics record compensated fixes, unavailable history,
current interval count and last accepted delay. Missing coverage brakes and
requires a covered post-fault acquisition. Synchronous accepted fixed-map
pose/covariance events can be retained and replayed too, with at most 64 map
events. Additional zero-omitted diagnostics report retained/replayed maps and
conflicting transactions. A conflict keeps the live state/history and original
observation clocks unchanged, latches invalid-GNSS braking and requires a new
post-fault acquisition. Delayed map scans remain unsupported.

The transport delay, fault schedule and physical truth are absent from the
header. Both the original configuration and raw delivered observations are
required for exact replay. Option-off records remain byte-identical; enabled
records require a reader that supports the new configuration and diagnostics.
[Independent chronological reconstruction and driving results](gnss-latency.md).
[Combined GNSS/map history and freshness contract](gnss-map-history.md).

## GNSS acceptance diagnostics

Optional output `localization` reports last observed/accepted GNSS stamps, the latest valid-new-fix decision and finite NIS, and accepted/rejected counts. Rejected fixes cannot refresh health; their receipt prevents subsequent duplicate/older observations from being processed. Invalid input continues to use health diagnostics. Replay recomputes the joint gate and all diagnostics. Changing a GNSS observation or its expected counters is covered by mismatch tests. Fault-window labels exist only in simulator configuration; recorded sensor observations contain the actual bias without labels. Schema 1 remains readable, but new correction arithmetic and output diagnostics require regenerated expected outputs. Boxed record payloads preserve JSON encoding. [GNSS baseline recovery](gnss-robustness.md) and [current terminal stopping](terminal-stopping.md).

The pipeline additionally emits health issue `GnssInnovationHold` after two consecutive strictly new `RejectedInnovation` decisions. A single innovation outlier does not trigger this additional hold. A newly `Accepted` fix resets the private streak; absent, duplicate, old or invalid fixes neither increment nor clear it. This brakes on sustained rejected measurements while preserving the existing NIS threshold 36 and accepted-fix-age limit of 0.75 s. Replay reconstructs the streak from sensor inputs; no streak counter, fault label or expected decision is supplied to the driver. The added health result and its commands can require fresh expected recordings, even though JSONL schema 1 remains unchanged. The policy is conservative simulator fault handling, not a guarantee of localization integrity or GNSS-denied operation.

`goal_hold_seconds` belongs to simulator acceptance and is absent from the pipeline/replay header. Replaying an extended episode recomputes all post-arrival sensor outputs and commands; it does not independently establish the physical residence or clearance. Those checks use evaluator truth. Changed goal-profile arithmetic and terminal preferences require fresh expected outputs; no sensor-log format change is introduced.

Optional `run.json` frame `traffic` contains simulator actor truth and preceding proximity observation/command, with full 20 Hz truth frames for reactive episodes. This is not part of the version-1 sensor log or pipeline input. Replay verifies only the recomputed ego stack; independent physical checks reconstruct actor sensing, bounded integration and circular pair clearance from evaluator truth. Scenario following parameters and stop windows never enter the header. [Boundary and actual results](reactive-traffic.md).

Observed-braking forecasts keep track history inside the shared pipeline. Replay reconstructs that history from newly accepted LiDAR-derived tracks; expected tracks and forecasts are never fed back as inputs. Forecast fields and log schema remain unchanged. Old constant-velocity recordings may correctly mismatch; use the historical implementation for historical replay, or regenerate current recordings. [Observation support, physical results and limits](observed-braking.md).
