# Sensor contract and replay

`rustdriving-pipeline` exposes `PipelineConfig`, `SensorFrame`, `DrivingPipeline` and `PipelineOutput`. Both reference simulation and RNE call the same stateful synchronous library. No executor or middleware is needed.

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
cargo run --release --locked --bin rustdriving -- replay \
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

Local formatting, warnings-denied Clippy, locked builds, **123 workspace tests** and **15 RNE tests** pass. The reference script verifies 23 scenario/replay pairs. The seeded suite passes **120 positive runs**, retaining all previous 102 cases and adding 18 reactive-traffic cases including green→yellow→red while approaching, across both plants and seeds 1/7/42. Positive cases satisfy zero ego/traffic/road/closure violations, complete replay and unchanged prior profile/steering/clearance gates. New checks independently reconstruct actor range/closing observations, bounded speed/distance integration, stop/resume/queue behavior and circular actor-pair sweeps. Negative tests establish failure on actor collisions, late sensing and endpoint overrun.

Two additional RNE short-fault episodes (`traffic-follower-deadline`, seeds 1 and 42) remain collision-free but miss their 65-second goal-residence deadline. They return 1 and replay all 1301 ticks; the report and native regression explicitly require their physical rejection. These failures are excluded from the positive count. The longer positive follower case uses a [10,20) fault and 80-second evaluation to exercise real standstill and full residence; it does not repair the short deadline case. Conservative repeated ego stops near the goal remain unimplemented interaction-planning work. [Complete evidence, source fingerprint and GIF](reactive-traffic.md). Remote CI for this new revision is reported separately.


## Observed-braking prediction (2026-10-09 UTC)

Local formatting, Clippy with warnings denied, locked builds, **128 workspace tests** and **16 RNE tests** pass. The reference script passes **24 scenario/replay pairs**. All **126 positive runs** (60 reference and 66 RNE) pass with complete sensor-only replay and unchanged physical clearance, profile, normal-steering and containment gates. The original 65-second follower fixture now completes eight seconds of physical residence in both plants across seeds 1/7/42, including the two previously failed native seeds. The actor world, fault window, deadline, native pin and plant calibration were not changed.

An independent sensor-log checker verifies evidence-supported braking, no lateral invention/reversal/acceleration, bounded deceleration, one-second persistence and reacceleration fallback. The two historical deadline failures remain in `traffic-results.json`. Two new five-meter follower sensing cases still return 1 for failure of the original 1 m clearance floor, replay all 1301 ticks and are excluded from the positive count. [Actual RNE GIF, measured comparison, current evidence and remaining limitations](observed-braking.md). Publication CI for this revision is reported separately from the preceding observed [all-five-job success on `4620878`](https://github.com/rsasaki0109/rust_drive/actions/runs/37879754523).

## CPU 3D replay baseline (2026-10-09 UTC)

Revision `1df32f6` introduced two Blender Cycles CPU GIFs of successful native RNE recordings. The closure-detour recording completes at 40.40 s; all 136 sampled scene states match recorded ego/object transforms. The follower recording completes at 64.25 s; all 216 sampled states match. The original GIFs were 960 × 640, approximately 2.6 MB each, with checked total playback durations of 14.9 and 22.9 seconds including the final pause. Input hashes, engine pin and physical summaries were recorded in their provenance JSON files. Blender 4.3.2 was used, without a GPU. The suburban model extension below replaces their visual assets.

The complete wrapper was additionally exercised on a fresh native closure-detour run: physical acceptance passed, all 809 sensor ticks replayed and a CPU-rendered preview passed scene checks. Failed physical logs are rejected by the renderer. Local `scripts/check.sh` passes formatting, warnings-denied Clippy, locked builds, all 128 workspace tests and 24 reference scenario/replay pairs. Python compilation, shell syntax and workflow lint pass. CI adds a real native single-frame CPU rendering check; full GIFs were generated and visually inspected locally. This changes visualization only, with planar physics and the existing independent acceptance retained. [Commands and boundaries](3d-demo.md).

## Suburban models and editable scenes (2026-10-09 UTC)

Both GIFs were regenerated with original detailed hatchbacks, raised pavements, trees, streetlights and campus buildings. All **136 / 216** sampled scene states pass the existing recorded-pose audit. Their source recordings, complete scenarios and physical summaries are unchanged from `1df32f6`; input hashes were compared with that revision. Resolution and total playback durations remain 960 × 640 and 14.9 / 22.9 seconds. GIF sizes are approximately **6.6 / 8.8 MB**. Provenance additionally identifies asset style `suburban-test-road-v2`, seed 1729, scenery counts and the renderer source hash. Full GIFs and previews were inspected locally.

Local formatting, warnings-denied Clippy, locked builds, **128 workspace tests** and **24 scenario/replay pairs** pass. A fresh native `mission` run passes physical acceptance, fully replays **796 ticks**, renders a CPU preview and exports an editable `.blend` snapshot. A separately exported closure scene reopens successfully in Blender, with its procedural objects/materials and CPU renderer retained and no external image references. Python compilation and workflow lint pass. CI also exports a native scene snapshot. Scenery remains visualization-only and is not added to RNE LiDAR or collision evaluation. [Models and reproduction](3d-demo.md).

## Multiple traffic vehicles (2026-10-09, Asia/Tokyo)

Local formatting, warnings-denied Clippy, locked builds, **128 workspace tests**, **16 RNE tests** and **25 reference scenario/replay pairs** pass. The full suite passes **132 positive runs** (63 reference, 69 native), retaining the previous 126 fixtures/seeds and adding six three-vehicle queue runs. All six replay all **1301 ticks**, preserve all three actor identities and single-lane order, satisfy bounded integration/reconstructed sensing and maintain a stopped queue for **40.85–41.00 s**. Minimum ego clearance is **4.534711 m** and minimum actor-pair clearance is **2.995975 m** across the six. The unchanged 1 m fixture floor applies. The two known short-range physical rejections remain separately verified and excluded from the positive count. [Compact measured evidence and source fingerprints](../assets/fleet-results.json).

The README reproduction wrapper additionally executes a fresh native seed-7 episode, replays all ticks and produces a CPU preview plus editable scene. The two native input traces have identical SHA-256 hashes locally. Removing a terminal actor or corrupting final ego standstill is rejected by the independent checker. Scene snapshots reopen with hatchback, sedan, van and pickup objects, original geometry/materials and Cycles CPU settings, without external image references. Vehicle types are display choices; shared planar traffic dynamics, ego algorithms and RNE pin are unchanged. [Commands and limits](vehicle-fleet.md).

The fleet GIF was generated with Blender 4.3.2 on CPU at 16 samples/pixel: **218 encoded frames**, **960 × 640**, **23.1 s playback** including the final pause, **3,763,123 bytes**. All **218 sampled scene states** match recorded ego/object transforms. The perspective traffic camera fits the complete vehicle bounds within a five-percent screen margin at each sampled state. Input and renderer hashes match the published provenance; decoded start/middle/end frames were inspected. The original opening/follower GIFs remain unchanged. Python compilation, shell syntax and workflow lint pass. CI adds a native fleet preview and editable scene alongside the mission preview.

## Mapped signal behavior and maturity plan (2026-10-09, Asia/Tokyo)

The first step toward the user's 50% engineering-maturity goal implements actual fixed-route signal stops and release, rather than visualization-only changes. Timestamped infrastructure snapshots, unique mapped IDs, accepted-age expiration, malformed-snapshot fault latching and healthy-estimate crossing commitment run through the shared driver. Phase schedules and acquisition-dropout labels remain outside its configuration. An independent physical-front evaluator rejects red/yellow/unknown crossings even if the simulator backend ignores the emitted braking command. The separate Python checker reconstructs infrastructure samples, freshness and true/observed permission, continuous close-line standstill and physical margins. [Contracts, commands and boundaries](traffic-signals.md); [subjective maturity estimate and capability gates](maturity.md).

Formatting, warnings-denied Clippy, locked builds, **139 workspace tests** and **31 reference scenario/replay pairs** pass. The six signal fixtures cover red→green, permanent red, expired green, observation recovery and two sequential signals, including green→yellow→red while approaching, across both plants and seeds 1/7/42. The smallest observed nonpermissive physical-front margin is **1.926626 m** and the shortest measured close-line hold is **2.65 s**, against fixed 1 m / 2 s gates. Native seed 7 waits **9.40 s** at the red line and completes at **38.60 s**; the two-signal native episode completes at **57.00 s**. Sensor-only replay matches every output.

The new CPU 3D preview renders the native red→green trace at 15 s, with the actual recorded pose, mapped stop marking and original procedural signal lamp. Its editable scene reopens with the mapped signal ID, CPU renderer and no external image references. Image/trace/renderer fingerprints are included in the measured snapshot. Existing GIFs are unchanged. Workflow checks additionally run the six reference signal episodes on all three platforms and render the native signal scene on CPU.

An intermediate final sweep exhausted the environment's disk while writing generated logs; it is an infrastructure failure and not counted as a physical scenario result. This turn's generated outputs were moved to the available `/tmp` filesystem; the initial successful report and disk-full diagnostic log were preserved, and complete final outputs were regenerated there. Historical logs, source files, locks and engine revision were preserved.

The complete final sweep passes **168 positive episodes** (81 reference, 87 native), including **36 signal-control episodes**, with all preceding gates retained and the two known short-range physical rejections separately verified. The final native suite passes **16 tests**. Source/checker/trace/image hashes match the [committed signal snapshot](../assets/signal-results.json). The moving green→yellow→red case also requires measured motion above 2 m/s before the phase change, followed by a close-line hold and permissive crossing.


## Mapped stop-sign extension (2026-10-09 UTC)

Local formatting, warnings-denied Clippy, locked builds, **147 workspace tests** and **16 native integration tests** pass. The required reference script verifies **36 scenario/replay pairs**. The complete seeded suite passes **198 positive runs** (96 reference / 102 native), retaining all preceding 168 and adding 30 stop-sign episodes. Two known short-range follower clearance failures remain physically rejected and excluded from the positive count. Existing collision, corridor, clearance, speed-profile, steering and replay gates are unchanged.

The new behavior is a continuous healthy two-second near-line stop, recent wheel-speed gating, low-speed brake retention, and release/passed state tracking. Five fixtures exercise one/two signs, an independent red signal, a sensed blockage and GNSS-bias recovery in both plants across seeds 1/7/42. A simulator evaluator uses actual front/speed; Python reconstructs every physical hold/crossing and enforces a fixed 1 m margin. Observed minimum unreleased margin is 1.931879 m and minimum close-line hold is 2.05 s. A backend ignoring control is rejected; modifying a physical frame to cross early without changing driver diagnostics is also rejected. [Compact evidence and commands](stop-signs.md).

The CPU 3D GIF uses the complete native seed-7 recording, a procedural octagonal stop sign and scene-transform auditing. It is a visual replay, not a camera sign detector or physical 3D scenery. Source/trace/renderer hashes and summaries accompany the assets. Publication CI is reported separately; the preceding signal commit `de0faac71b5ed81cd2c09f9ff3718dd5cec1cdb7` passed all five jobs in [run 37914908056](https://github.com/rsasaki0109/rust_drive/actions/runs/37914908056). Native pin and lockfiles are unchanged. General right of way, all-way stop order and sign compliance by traffic actors remain unimplemented.


## Mapped priority-crossing extension (2026-10-09 UTC)

The new fixed-route yield policy checks LiDAR-derived prediction sweeps against mapped conflict rectangles; it uses neither scheduled actor motions nor simulator occupancy as operational inputs. Healthy fresh distinct LiDAR acquisitions must show a continuous clear interval before route release. Entry commitment uses the healthy estimate; collision planning remains active. A separate physical evaluator computes actual circular-zone occupancy intervals from the first inside to the first outside 20 Hz sample, without entry/exit interpolation, and requires at least two seconds of temporal separation from priority traffic. Overlap and unsafe passage are independently rejected rather than inferred from driver status. Continuous collision sweeps remain a separate gate.

Five authored fixtures cover one crossing, successive opposite crossings, a stationary occupied junction, GNSS-fault recovery and a stop-sign combination. The required local script passes formatting, warnings-denied Clippy, locked builds, **160 workspace tests** and **41 reference scenario/replay pairs**. Native formatting, Clippy and **16 integration tests** pass. The final sweep passes **228 positive runs** (111 reference / 117 native), including **30 intersection runs**, while independently retaining the two known short-range clearance rejections outside the positive count. Every positive sensor log is fully recomputed. The preceding stop-sign revision `71ada62417766508560157b056eeb92e77be59a3` passed all five remote jobs in [run 37918880956](https://github.com/rsasaki0109/rust_drive/actions/runs/37918880956); publication CI for this extension is reported separately.

Across the 30 intersection runs, the minimum sampled physical priority gap is **4.50 s** against a **2 s** gate; minimum physical front-to-line margin while waiting is **1.926332 m** against a **1 m** gate. The shortest exercised continuous near-line hold is **2.40 s**. A control-ignoring backend is rejected by the Rust physical evaluator; an altered actual-occupancy trace is rejected by the independent Python checker. Timer tests include a fault during active clear confirmation, duplicate scans and a newly observed conflict before entry. [Measured fixtures and source fingerprints](intersections.md). Decorative perpendicular streets, yield signs and actor display headings belong only to the Blender replay; the physical domain remains planar.

The native seed-7 crossing completes at **33.20 s**, with **665 replayed ticks**. Its CPU GIF contains **112 encoded/audited scene states**, **960 × 640** pixels, **12.50 s playback** including the final pause and **5,439,126 bytes**. Input bytes match the final validated episode; input, renderer and GIF hashes match the compact evidence. Decoded start, waiting, post-crossing and final frames were inspected. The editable crossing scene reopens with **355 objects**, Cycles on CPU and no external image references. Python compilation and shell syntax pass. Native pin, both lockfiles and existing GIFs are unchanged.

## Acquisition-time LiDAR and varied crossings (2026-10-09 JST)

Body-frame LiDAR detections and map rays now use the acquisition-time EKF pose from bounded history, rather than the delivery-time pose. A translating/turning observation test preserves a stationary world circle with 100 ms delivery; stale or uncovered acquisitions reject without refreshing accepted age. No world poses enter operational sensing. This does not implement delayed GNSS/odometry fusion, retrospective smoothing, deskew, covariance propagation or delivery-time forecast rebasing. Simulator-only cadence/delay/failure schedules never enter the sensor-only replay header.

The required script passes formatting, warnings-denied Clippy, locked builds, **171 workspace tests** and **47 reference run/replay pairs**. Native formatting, Clippy and **16 tests** also pass. The complete suite passes **264 positive episodes** (129 reference / 135 native), including **66 intersection runs** and **18 timing-injection runs**. The new fixtures vary road width/length, speed, sequential zones, 5 Hz delivered observations, 50/100 ms delivery and a transient acquisition failure during active clear confirmation. Across intersection positives, minimum physical priority separation is **4.45 s**, waiting margin **1.900082 m**, and continuous near-line hold **2.40 s**; unchanged gates remain 2 s / 1 m. All sensor ticks replay. [Current evidence](../assets/sensor-timing-results.json).

Three failures are retained outside the positive count. Two native short-range follower cases still fail the collision-clearance floor. The new reference seed-7 late-conflict case passes the CLI's collision/zone summary and replays all **1098 ticks**, but independently fails the 1 m waiting-margin requirement at **0.267444 m**. At that revision its later-arriving second actor remained unrepaired; the earlier-arrival positive fixture is a different authored case. The reference-only known-failure execution path was also exercised with the actual CLI and replay. Changing acquisition stamps, hiding an error report or removing a scheduled scan causes independent timing rejection.

The prior intersection GIF input matches the corresponding newly validated trace byte-for-byte and the visual assets are unchanged. The native delayed-recovery recording renders a CPU preview at 14.1 s during emergency hold; its input matches the final accepted run. The exported 355-object Cycles CPU scene reopens without external image references. Python compilation and shell syntax pass; CI includes the new scenarios on three platforms, the full native sweep and the recovery preview. Publication CI is reported separately. Rust pins, lockfiles and the engine checkout are unchanged. Maturity remains a subjective **8%**; broader prediction timing and late-conflict behavior still require work.

## Current-time forecasts and repaired late crossings (2026-10-09 JST)

Motion predictions now start at the control clock while tracks retain their acquisition positions and timestamps. Both constant motion and supported braking use acquisition age plus forecast time; the one-second braking interval is not renewed by repeated control calls. Old braking evidence falls back to age-propagated constant velocity. Invalid, future or overflowing track states produce empty forecasts and fail closed in planning. The public `ConstantVelocity` trait remains acquisition-origin; this change applies to the pipeline's stateful predictor. Independent reconstruction checked **82,695 aged forecasts**, including **50,216 moving forecasts** rebased to the control clock.

An uncommitted `Proceeding` intersection now limits planned cruise speed by a response/braking-distance budget: half the configured planner deceleration, a 0.25 s response allowance and a nominal 2 m estimated front reserve. `Waiting` keeps its existing bounded stopping prefix; original entry commitment and physical gates are unchanged. The 0.5 m/s creep floor permits clear entry, so this is not a guaranteed reserve for arbitrary late or hidden actors. The original late-conflict scenario is byte-for-byte unchanged. Original and 100 ms-delayed versions pass both plants and all three seeds: **12 episodes**, with minimum waiting margins **1.939764 / 1.955410 m** against the unchanged 1 m floor. The original reference seed-7 margin was 0.267444 m at revision `0928be3`.

A full-sweep attempt detected a native GNSS-burst regression: seed 1 reached the goal but had 0.623638 m maximum position error, exceeding the unchanged 0.5 m independent gate during a strong countersteer and sustained rejected fixes. The final pipeline brakes after two strictly new consecutive innovation rejections and remains in `GnssInnovationHold` until a new fix is accepted. Single outliers, duplicate/older fixes and invalid/absent observations cannot invent a second rejection or clear the hold. NIS 36 and the accepted-fix 0.75 s freshness gate are unchanged. Native burst seeds 1/7/42 now have maximum errors **0.184327 / 0.222971 / 0.412726 m**; single-spike fixtures enter no innovation hold. Three altered logs with premature release, retained hold after acceptance or invented single-spike hold are independently rejected. This reduces the exercised fault's drift; it does not model native sideslip or establish a general localization bound.

Current-time forecasts also exposed a reference opposing-crossing regression: the previous circular sweep allowed nominal near-term gaps above its 0.3 m buffer, producing 0.544958 m physical clearance below the unchanged 0.7 m fixture floor. The final sweep adds an empirical motion reserve proportional to forecast velocity perpendicular to each candidate segment: up to 0.4 m at transverse speeds of at least 1 m/s, alongside the existing 0.3 m base and 0.06 m/s time growth capped at five seconds. Stationary segments use the estimated vehicle heading to distinguish crossing and parallel traffic; an unavailable heading conservatively uses total forecast speed. The same policy applies to normal candidates, retimed stops and stationary holds. Static and parallel forecasts retain their existing reserve; both this crossing and the static-detour regressions pass. This empirical allowance is not a certified uncertainty bound or a general clearance guarantee.

Final formatting, warnings-denied workspace/native Clippy, locked builds, **181 workspace tests**, **16 native tests** and **49 reference run/replay pairs** pass. The final source-fingerprinted sweep passes **276 positive episodes** (135 reference / 141 native), including **78 intersection episodes** and **24 timing-injection episodes**, with **228,961 fully recomputed sensor ticks**. Across intersection positives, minimum sampled physical priority separation is **8.95 s**, waiting margin **1.900082 m**, and near-line hold **2.20 s**. The existing 2 s / 1 m gates remain unchanged. Both native short-range follower failures are still physically rejected, replayed and excluded from the positive count. [Compact evidence and source/checker fingerprints](../assets/prediction-epoch-results.json).

The complete repaired native seed-7 episode takes **59.90 s / 1199 ticks**. Its new CPU 3D GIF contains **201 encoded / 201 audited scene states**, **960 × 640** pixels and **21.40 s** playback, including the final pause; size is **10,037,570 bytes**. Input bytes match the corresponding final validated episode, and input/renderer/GIF hashes accompany the recording. Decoded start, waiting, post-crossing and final frames were inspected. The exported two-zone scene reopens with **551 objects**, Cycles CPU and no external image references. This remains a planar physics recording with decorative 3D scenery, rather than physical 3D perception. Previous GIFs retain their historical source recordings.

Python compilation, shell syntax and workflow syntax checks pass. CI adds both late fixtures on three platforms and a repaired native crossing preview with editable scene export. The preceding `0928be385adb9230c5e4f856c8b447e5de7939aa` passed all five remote jobs in [run 37926106405](https://github.com/rsasaki0109/rust_drive/actions/runs/37926106405); publication CI for this revision is reported separately. Rust toolchains, lockfiles, RNE pin and engine checkout are unchanged. The subjective maturity estimate remains **8%**: broader physical 3D sensing, external map/data validation and traffic interaction remain future work.

## Native static cuboid scenes (2026-10-09 UTC)

Opt-in `--scene` installs upright, yaw-rotated cuboids in actual native Rapier queries. The existing 0.6 m operational LiDAR scan drives the shared pipeline; 0.15 m and 3.7 m horizontal scans are diagnostic only. The sidecar records native 200 Hz translation segments and 20 Hz body poses. A separate conservative upright-capsule guard uses actual box height and yaw, validates the discrete substep translation bound, and adds failures to the run summary. It does not apply contact forces or feed geometry to planning. [Contract and limitations](native-scenes.md).

The final independent suite passes **18 positive episodes**, with **six low-slab physical failures correctly rejected**, across both native plants and seeds 1/7/42. Ground and rotated barriers produce stops; the same barrier raised to a 3.5 m bottom height permits goal completion. Minimum positive capsule clearance is **1.15 m**, against a fixed **1 m** floor. Ground/rotated stops have at least **7.116577 m** clearance. All **13,630 sensor ticks** replay, including the failed physical runs. The independent checker verifies **136,084 motion samples**, **6,824 scan acquisitions**, all **14,739,840 ray ordinals** including misses, and **705,080 returns**. Maximum range residual is **0.038302 m**, under the fixed **0.06 m** noise tolerance. All **32 altered-evidence checks** reject missing colliders, changed height, corrupted ranges or forged clearance. [Raw hashes and measured results](../assets/native-scene-results.json).

The required workspace check passes **181 tests**, warnings-denied Clippy, formatting, locked release build and **49 reference scenario/replay pairs**. Native formatting, warnings-denied Clippy and all **23 native tests** also pass. Eight selected seed-7 reference/native-dynamic runs (opposing crossings, repaired late conflict, follower deadline and GNSS burst) have byte-identical run, sensor and summary outputs compared with `d06d9cb`. The preceding 276-positive full hazard sweep remains historical evidence at that revision; it was not rerun locally for this opt-in addition. The two historical short-range native follower failures remain documented separately. Publication CI retains the full hazard sweep and adds the 24 native-scene checks and an audited scene preview.

The low-slab cases reach the goal through the obstacle because the existing primary scan misses it. The diagnostic lower plane sees it and the independent capsule guard rejects every case with zero clearance. These failures remain outside the positive count. Auxiliary-plane fusion, volumetric perception, calibrated car-body collision geometry, terrain/contact response and external map/data validation remain future work. Rust toolchains, lockfiles, RNE revision and engine checkout are unchanged; maturity remains a subjective **8%**.

The accepted native dynamic seed-7 ground-barrier run supplies the new README GIF: **118 encoded / 118 audited scene states**, **960 × 640**, **13.10 s** playback including the final pause, **1,668,647 bytes**, rendered with Blender 4.3.2 Cycles CPU at eight samples per pixel. Trace and scene hashes match the final acceptance report; renderer hashes match the published sources. All 351 recorded trace poses agree with the sidecar, and the actual eight world-space cuboid mesh corners match physical dimensions/height/yaw at each sampled state. Decoded start/middle/end frames were inspected. The editable 201-object CPU preview scene reopens without external image references. Historical GIFs are unchanged.

## Operational multi-height projection (2026-10-09 UTC)

The explicit native `--multi-height` option now supplies synchronized measured body-XY planes at 0.15, 0.6 and 3.7 m through the sensor-only contract. The shared pipeline validates every plane against bounded calibration, selects the known capsule's vertical interval, removes duplicate 5 cm XY cells and applies its acquisition-time EKF pose. Relevant low returns affect detection, tracking, mapping and planning; overhead returns remain in the raw recording but are excluded by height calibration. No scene labels or physical cuboid geometry enter the driver. Malformed, mixed-mode or partial acquisitions latch braking until a complete acquisition strictly newer than the fault control-time epoch and the last accepted scan. Bundle timing/failure transport is atomic. [Contract and limitations](multi-height-lidar.md).

The required script passes formatting, warnings-denied Clippy, locked builds, **195 workspace tests** and **49 reference scenario/replay pairs**. Native formatting, warnings-denied Clippy and all **27 native release tests** pass. Eleven new pipeline tests cover low-only planning/map changes, overhead exclusion, deterministic deduplication, calibrated-height tolerance, invalid bundles, acquisition poses, fault recovery, replay and backward serialization. Three new transport tests verify bundle cadence/delay and failure flushing; four native tests exercise actual query-based low stopping, overhead passage, remaining blind geometry and primary/auxiliary acquisition failure.

The complete independent matrix passes **42 positive runs** (18 default / 24 multi-height) and verifies **12 retained physical failures** (six default low slabs / six multi-height sub-low slabs), across both plants and seeds 1/7/42. The previous 0.2 m high low slab now causes a sensed stop in all six multi-height episodes, with minimum capsule clearance **7.391197 m** against the unchanged **1 m** floor. Raised-barrier passage retains **1.15 m** clearance. All **31,466 sensor ticks** replay, including failed physical runs. Independent checks inspect **314,174 native motion samples**, **15,754 acquisitions**, every ray ordinal and **1,476,092 measured returns**; maximum range residual is **0.038302 m** under **0.06 m**. [Compact measured results and hashes](../assets/multi-height-results.json).

All **72 geometry mutations** and **50 raw-layer mutations** fail the independent oracle. Rust replay also rejects **46 of the 50** raw-layer edits; the remaining four edit an excluded overhead return or an isolated point without changing driving outputs. The separate geometric oracle still rejects their measurement inconsistency. Replay establishes repeated computation, rather than physical authenticity or sensor coverage.

All **24 default scene episodes** have byte-identical run, scene and sensor files compared with archived `1cc3802` evidence. Eight selected reference/native-dynamic seed-7 default episodes remain byte-identical to `d06d9cb`. Historical 276-positive full hazard evidence remains associated with that revision; this turn did not rerun that full matrix locally. The two historical short-range native failures remain documented and their native regression test still passes. Existing assets and original low-slab inputs retain their bytes; the new low-stop fixture changes only name/expected outcome, and a separate sub-low geometry retains the blind-zone failure.

This is sparse measured-height projection into planar perception and tracking. Obstacles between planes or below the lowest plane can still be missed; the sub-low cases intentionally reach the goal but fail offline capsule acceptance. The capsule extends below ground and is not a calibrated car body. Contact forces, terrain, volumetric object reconstruction and real-sensor validation remain future work. Engine revision, lockfiles and Rust pins are unchanged; subjective maturity remains **8%**. Publication CI retains the full hazard sweep and adds the new native matrix and actual low-stop scene preview.

The accepted native dynamic seed-7 low-stop recording supplies the new README GIF: **118 encoded / 118 audited scene states**, **960 × 640**, **13.10 s** playback with the final pause, **1,598,964 bytes**, at eight Cycles CPU samples per pixel. Recorded trace and scene hashes match the final report; renderer hashes match the current source. All 351 recorded body poses and each sampled mesh's eight world-space corners are checked. The editable 201-object CPU snapshot reopens without external image references; its slab is the actual **1 × 6 × 0.2 m** shape. Provenance correctly records three measured planes, two height-selected projection planes and the collision interval, rather than marking operational raw planes diagnostic-only. Decoded start/middle/end frames were inspected.

## Native inclined XYZ acquisition (2026-10-09 UTC)

The explicit `--lidar-3d` mode acquires real RNE/Rapier first returns from **720 azimuth columns × 16 elevation rings**, spanning ±15°. Typed sensor-only records retain actual body-forward/body-left/road-datum-up points and firing ordinals. Bounded calibration, unique ordinals, finite XYZ, measured ranges and calibrated beam directions are validated before measured-height selection and deterministic 5 cm XY projection. The existing planar detector, tracker, occupancy map and planner consume that projection at the acquisition-time EKF pose. Atomic transport and a fault latch require a complete valid acquisition strictly newer than both the fault control time and the last accepted scan before recovery. Driver inputs contain no physical cuboids or native truth poses. [Contract and limitations](lidar-3d.md).

The final independent matrix passes **36 positive episodes** and verifies **12 physical rejections**, across both native plants and seeds 1/7/42. Ground, rotated, raised, low, sub-low and mid-height cases exercise stopping or overhead passage. Six prior horizontal multi-height episodes still miss the between-plane beam and are physically rejected; six inclined near-high episodes remain blind before conservative capsule overlap. The new mid-height and sub-low stops retain at least **6.219278 m** and **4.727081 m** clearance respectively, against the unchanged **1 m** gate. Minimum overall positive clearance remains **1.15 m** for overhead passage. These are static upright-cuboid fixtures, without a new 3D moving-traffic acceptance claim.

All **28,861 sensor ticks** replay, including physical failures. The independent checker inspects **288,178 native motion samples**, **13,144 inclined acquisitions**, all **151,418,880 3D firing ordinals** including misses, and **2,233,363 XYZ returns**. Maximum range residual is **0.037587 m**, under the fixed **0.06 m** tolerance. It reconstructs expected height-selected voxel counts rather than inspecting an exported internal projection cloud; shared pipeline tests and full replay exercise actual operational projection. Horizontal diagnostic metadata and hit counts are comparison evidence in this matrix, rather than separately re-raycast measurements. [Raw hashes, commands, archives and results](../assets/lidar-3d-results.json).

All **112 XYZ evidence mutations** are rejected by the independent oracle. Rust replay rejects **96 of 98 sensor-log mutations**; deleting a single excluded overhead return in each plant leaves computed driving outputs unchanged, but still fails physical measurement correspondence. Eight baseline geometry and ten baseline raw-layer mutations also fail their independent checks. Six additional mutations of the accepted 3D mid-beam guard reject missing/raised colliders, forged clearance, omitted motion samples, changed clocks or observation poses. Each of the 48 generated evidence archives is verified against every original file SHA-256 and the full gzip CRC before compact-mode cleanup; the accepted dynamic seed-7 mid-beam recording remains expanded.

Required formatting, warnings-denied Clippy, locked release build, **208 workspace tests**, **32 native release tests** and **49 reference run/replay pairs** pass. All 54 preceding single-height/multi-height scene outputs and summaries match the `881e314` recordings, with full replay; eight selected reference/native-dynamic seed-7 default runs remain byte-identical to `d06d9cb`. The preceding 276-positive full hazard sweep remains historical evidence at that revision and was not rerun locally for this opt-in addition. CI retains that full sweep and the preceding 54-case matrix, adding the new 48-case checker, compressed evidence and a native mid-beam preview. The previous `881e314` publication has observed successful macOS/Windows jobs, with Linux/visual/native jobs still queued; publication CI for this increment is reported separately.

The accepted dynamic seed-7 mid-beam episode runs for **35 seconds / 701 ticks**, records **22,690 real XYZ returns**, and has zero hits in all three earlier horizontal diagnostic channels. Its first inclined return arrives at **4.9 s**. It stops with **6.770538 m** capsule-guard clearance, zero guard overlaps and zero final speed; 20 emergency ticks and up to three planar tracks from one cuboid remain documented. The new CPU GIF has **118 encoded / audited states**, **960 × 640** pixels, **13.10 s** playback and **1,807,636 bytes**. All 351 display poses and the actual cuboid's eight mesh corners are audited; decoded driving/stopped/final frames were inspected. The editable preview reopens with 201 objects, Cycles CPU and no external image references. Input hashes match the final accepted case. [GIF and provenance](../assets/lidar-3d-demo.json).

Native ego motion and downstream object processing remain planar. The query scenes contain no physical ground collider; there is no ground segmentation, so real road returns could become projected obstacles. Finite FOV and discrete rings do not establish continuous height coverage; the near-high guard overlap occurs before its first sensor return. Capsule geometry remains uncalibrated to a physical car, and there is no contact response, terrain handling, deskew, object semantics, external dataset validation or real-time claim. Rust toolchains, lockfiles, RNE pin and engine checkout are unchanged. Subjective OSS-relative maturity remains **8%**, with the broader 50% milestone still ahead.

## Measured road support, research body and external maps (2026-10-09 UTC)

The new opt-in native ground mode installs actual flat road cuboids in Rapier queries and feeds 180 × 16 inclined measured returns through the shared pipeline. A bounded measured-plane fit removes only locally supported near-plane returns; insufficient support latches braking. The optional authored 4.2 × 1.8 × 1.5 m body adds an independent upright-box swept guard and force-free actual Rapier static-obstacle overlap witnesses. Native Ackermann integration remains the sole motion integrator. Moving actors retain native capsule sensing and conservative circular traffic acceptance. [Contract and limitations](ground-lidar.md).

The required workspace check passes formatting, warnings-denied Clippy, locked builds, **233 tests** and **49 reference scenario/replay pairs**. Native release checks pass formatting, warnings-denied Clippy and **46 tests**, including an actual ignore-braking collision that both the swept guard and native overlap witnesses reject. The physical matrix completes **66 episodes**: 24 healthy goals, 24 sensed static-obstacle stops, 12 sensed moving-traffic stops and six separate confidence-failure stops. All **24,842 sensor ticks** are recomputed; independent reconstruction verifies **35,844,480 beam ordinals** and **16,284,733 XYZ returns**. All **238 mutations** are rejected by their applicable sensor replay or independent geometry/body oracle. [Full matrix, scopes and hashes](../assets/ground-results.json); [check logs and rendering evidence](../assets/ground-validation.json).

The final sweep uncovered a real low-slab scan where two least-squares refits reported a plane inconsistent with its final inlier mask: about 0.000142 m discrepancy exceeded the unchanged 0.0001 m independent gate. The original 1,354-return scan is retained as a regression fixture. The implementation now requires a fixed inlier mask within eight bounded refinements and fails confidence on nonconvergence. The complete final matrix passes with maximum independently reconstructed plane discrepancy below 2.1 × 10⁻¹⁸ m. No clearance, range, noise, residual or confidence gate was relaxed. The full matrix removes 572 actor returns and 757 static returns, chiefly near-plane surfaces; five removed lead returns belong to the selected GIF episode. Successful stopping does not imply perfect surface classification.

Two actual native capsule grazing hits differ from ideal f64 ray geometry in the pinned f32/GJK query implementation. A retained isolated native reproduction switches from hit to miss with a +50 µm perpendicular origin shift. The oracle counts the explicit empirical ≤100 µm capsule-boundary exceptions separately; it retains the 0.06 m range tolerance and rejects outside-band, wrong-range and hidden-behind-certain-geometry mutations. This empirical classification is not a certified floating-point bound or a continuous-coverage claim.

A bounded Rust OSM importer supplies genuine historical geographic centerlines, directed connectivity, explicit ENU projection and attributed ODbL data. Six integrated native ground/body episodes reach the 113.06 m imported road goal; three additional native dynamic episodes stop before a measured obstacle. All **12,914 ticks** replay, with **18,604,800 reconstructed beam ordinals**, **9,302,400 actual ground returns removed** and **59,510 static returns preserved**. Independent 200 Hz circumscribed-body corridor checks retain an extra 0.03 m inter-sample reserve; their minimum remaining margin is **0.065024 m**. These are explicitly simulated 3 m half-widths, not measured legal lanes. [External-map contracts and limitations](osm-import.md).

The nine OSM physical/oracle runs were originally verified on the immediately preceding ground-fitting source. After the fixed-point repair, all nine native episodes were regenerated and fully replayed on the final source; run, scene, raw sensor and recomputed-output hashes match exactly. [Published proof](../assets/osm-ground-results.json) preserves both fingerprints and this byte-identity reuse of the prior independent oracle, rather than relabeling historical runs.

An opt-in local-filleting mode additionally passes six sharp-corner reference runs without changing source coordinates, widths, speeds or deadlines. The genuine imported sharp branch still fails its goal deadline under native dynamics with zero recorded collisions/road violations; that required negative and its full replay remain separate from positive acceptance. [Corner results](../assets/local-corner-results.json); [independent corridor mutations](../assets/local-corner-mutations.json). The ground matrix and final-source legacy comparison retain all 54 earlier default/multi-height scene outcomes plus eight selected default reference/native outputs byte for byte. [Legacy comparison](../assets/ground-legacy-results.json). The preceding 276-positive hazard and 48 inclined-scan matrices remain associated with their historical revisions; the new CI also reruns them.

The new opening GIF is the accepted dynamic seed-7 ground/body traffic episode: **22 s / 441 ticks**, **75 decoded frames**, **960 × 640**, **8.8 s** encoded playback and **1,488,458 bytes**. Every recorded display pose and all sampled physical-road/body-envelope corners are audited. The moving lead is sensed and stopped behind with **5.715014 m** circular clearance; the scene has no static obstacle cuboids, so its static-body minimum is null. Decorative car geometry and its roof housing do not calibrate the research body or the 0.6 m sensor mount. [Provenance](../assets/ground-demo.json).

This integrates broader executable sensing, geometry and external-map behavior, supporting a subjective maturity estimate of **about 15%**. Sparse/narrow support false-blocking, near-plane object removal, finite-beam blind regions, native sharp-turn deadlock, planar motion, missing chassis contact response and absent real-world datasets remain explicit. It establishes no real-time throughput or road-safety claim. The Rust toolchains, lockfiles and RNE pin are unchanged.

## Low-speed chassis-reference turn repair (2026-10-10 JST)

The native dynamic imported branch now completes its unchanged goal criteria. The preceding failure used the chassis/COM pose with rear-axle motion assumptions: near 32 s, displacement course differed from body yaw by about 0.30 rad, estimated inward error understated actual error, and the recovery candidate left the original corridor. An explicit optional 1.5 m rear-axle offset now drives midpoint EKF translation from measured longitudinal speed and gyro, and a temporary course view for local planning/control. Body yaw remains the LiDAR/reference-pose orientation; no native lateral velocity or truth pose enters the pipeline. [Equations and limits](chassis-reference.md).

All **12 original reference/native dynamic corner cases** reach their goals with zero recorded collisions/road violations, and all **28,256 sensor ticks** replay. Native German-branch times are **78.65 / 79.70 / 78.50 s**, within the original 180 s deadline; minimum sampled original-circle corridor margin is **0.099672 m**. Native authored-detour times are **156.50 / 156.45 / 156.40 s**, within the original 250 s deadline. Emergency ticks of **9 / 19 / 9** remain in the German native cases. Original coordinates, physical widths, cruise settings, deadlines, collision/range/road gates and steering authority are unchanged. Three chassis-offset/gyro edits fail exact replay. [Current matrix and hashes](../assets/local-corner-chassis-results.json).

The required workspace check passes formatting, warnings-denied Clippy, locked builds, **240 tests** and **49 scenario/replay pairs**. Native formatting, warnings-denied release Clippy and all **46 tests** pass. Six new estimator/controller tests cover analytic left/right chassis displacement/covariance and course-to-steering inversion, invalid/overflow handling, saturation and exact zero-offset behavior; one new pipeline test covers explicit bounded calibration and serialization. All six reference corner run/sensor files retain their previous bytes. Eight selected default reference/native hazard cases and the accepted opening-GIF ground/body episode are regenerated byte-identically and fully replayed. The opening GIF remains unchanged. [Validation and default-regression proof](../assets/chassis-validation.json).

The preceding published CI completed its entire native job successfully, including the 276-hazard, native/multi-height/inclined/ground/body/OSM matrices and CPU scene previews. Its Windows Rust job failed before corner driving because checkout converted the pinned source XML/JSON to CRLF; Linux/macOS matrix jobs were canceled by fail-fast. Narrow `.gitattributes` rules preserve just those source bytes. An actual Git autocrlf checkout regression includes an unprotected CRLF control, runs the original map oracle, and rejects newline and appended-byte tampering without changing checksums. This local emulation does not establish fresh Windows CI success; the next publication run is reported separately.

This is a low-speed no-slip repair for the two clear circular-vehicle fixtures. Sampled narrow margins and emergency interruptions remain; arbitrary native bends, high-speed slip, obstacle interaction on these bends and rectangular-body corner acceptance are unverified. The former failure and pre-repair source fingerprint remain in the historical report. Other full matrices retain their preceding fingerprints and are rerun in CI. Rust/engine pins and lockfiles are unchanged. Subjective OSS-relative maturity remains **about 15%**.


## Optional measured research modules (2026-10-10, Asia/Tokyo)

The final local source passes the required `bash scripts/check.sh`: 301 workspace
tests, formatting, strict Clippy, locked release build and all 49 default
scenario/full-replay pairs. All 245 default raw output files are byte-identical
to the preceding baseline. Native release formatting, strict Clippy and all 46
tests pass. The opening ground/body run retains all six raw file hashes and its
441-tick replay; the README's first 3D GIF retains its original SHA. Root and RNE
lockfiles, Rust pins and RNE revision are unchanged. New optional integrations
have separate committed locks. [Exact validation record](../assets/research-cpu-validation.json).

Adaptive terrain uses only the original six calibration samples before its
source/configuration freeze. Their F1 is 0.8215; the nine already viewed sites
are regression data with F1 0.8133. Fresh Autzen fails the fixed gates: F1 0.3571,
recall 0.2174 and insufficient confidence. Only ground references are scored,
so precision 1.0 does not validate false-positive rejection. Independent parsing,
unit/index/AABB/metric reconstruction and corruption checks pass; integrity does
not mean accuracy. One optional native adaptive lead-stop run independently
passes 441-tick replay, measured ray/AABB checks and a 4.872 m bounded actor
clearance. Mixed ground/object components remain. [Terrain boundaries](adaptive-ground.md).

Bounded 6DOF registration is scored against actual recorded depth and mocap,
starting from identity. Original slow and fast protocols each reject all 11
pairs. Exact nearest-neighbor AABB pruning preserves brute-force correspondence
results in 1,800 authored queries without relaxing geometry or the work budget.
The previously viewed fast interval then accepts 10/11 as regression; a separately
frozen later interval accepts 7/11 (5/9 temporal-held-out), with three work-budget
and one ambiguity rejection retained. Accepted later pose errors reach 8.53 mm
and 0.02104 rad, but all data shares one room and the conditional covariance is
severely overconfident. No vehicle fusion, SLAM or real-time claim follows.
Independent pose/residual/covariance reconstruction and 14 mutations pass.
[Recorded protocols and failed uncertainty](recorded-rgbd.md).

Optional Rust-native CPU ONNX inference executes the official YOLOX nano model
through tract. One measured NASA portrait matches an independently supplied
reference person box with IoU 0.94865; repeated actual inference, a synthetic
blank and rejection checks pass. Four Rust tests, strict release Clippy and
formatting pass. One portrait does not validate driving-camera accuracy, metric
depth or control integration. [Sources, licenses and actual inference](../integrations/onnx/README.md).

The engineering estimate is about 20%, with the requested 30% still unmet.
Fresh terrain accuracy, partial recorded-motion acceptance, covariance
calibration, measured extrinsics and automotive camera evaluation remain open.
The research CI job retains failed accuracy outcomes while checking their
integrity; publication-run status is reported separately.


## Main-branch 3D road users (2026-10-10, Asia/Tokyo)

The previously published RNE stack and recorded 3D demos were merged into main
in [PR #1](https://github.com/rsasaki0109/rust_drive/pull/1). The following addition
uses actual native sensor/vehicle recordings with a lead, a crossing pedestrian
proxy and a cyclist proxy; render-only ID/model assignments add original
articulated human and bicycle meshes. All three seeds retain the fixed 1 m
clearance floor, zero collisions/road violations, stopping and 1,323 full sensor
replay ticks. Analytic ray, measured-ground, 200 Hz body/motion and actor-distance
checks pass; the pedestrian distance lower bound is 1.726 m. No avatar mesh
sensing, semantic classification, contact response or human intent is claimed.
[Physical results](../assets/vru-demo-results.json); [source, meshes and commands](road-users.md).

The final GIF is rendered on CPU from the seed-7 recording. All 75 scene states,
actor root positions, native ground/body geometry and renderer/trace hashes are
verified. The prior ground/body GIF is preserved. The required workspace check
again passes 301 tests and all 49 default scenario/replay pairs, with all 245
output files byte-identical to their prior baseline. Rust/RNE pins and locks are
unchanged. Model/renderer Python compilation and workflow actionlint pass.

The preceding publication's research CI failed because an archived manifest
resolved its raw directory beneath `integrations/rgbd/baselines/`. The workflow
now supplies `--raw data/tum-fr1-xyz/raw` explicitly. The actual failed command
was reproduced, then both archived oracles and the later 7/11 partial benchmark
were verified again; no pose, covariance, work or accuracy gate was relaxed.
Remote CI for the follow-up publication is reported separately. Visual asset
quality does not change the subjective maturity estimate of about 20%.


## RustDriving namespace and original road-user meshes (2026-10-10, Asia/Tokyo)

Cargo packages and CLI commands now use `rustdriving-*` / `rustdriving`. The
[rename validation](../assets/package-rename-validation.json) checks all four
lockfiles: only project package names change; external versions, dependencies
and checksums remain fixed. Historical recordings, source snapshots, versioned
schema identifiers and the original adaptive-ground freeze remain unchanged.
Adaptive evaluation after the import rename is an explicit regression on
previously viewed data, with the original measured-data failures retained.
The archived RGB-D oracle uses its original
[`Cargo-v1.lock`](../integrations/rgbd/baselines/Cargo-v1.lock).

The [vehicle audit](../assets/vehicle-display-audit.json),
[truck/dog audit](../assets/city-display-audit.json) and
[family audit](../assets/family-display-audit.json) independently measure
80 + 20 + 30 display poses. They check SI dimensions, radius-independent
geometry, round wheels and retained recorded roots. Family cases also measure
hand contact with the cane and stroller pushbar. The separate
[family showroom](../assets/family-models.json) is CPU-rendered display evidence,
not a driving run. [Physical city checks and limitations](road-users.md) remain
separate from mesh checks.

The completed urban recording passes seeds 1, 7 and 42: 2,763 sensor-only
replay ticks, 3,983,040 full-grid ray entries and 27,603 native 200 Hz poses.
Continuous clearance for every one of the 171 actor pairs stays at least 1 m;
the minimum is 1.003840 m. Each run crosses both signals on freshly observed
green and holds continuously behind the red-light queue for at least 8 seconds.
These are authored-scenario checks, not certification of general traffic rules.
Two failed development trials remain separately identified in the results.

The final CPU GIF contains 155 verified scene/vehicle-scale states at 960×640,
with 3× playback and a final hold (16.8 seconds encoded). The actual seed-7 trace,
native scene and renderer hashes are retained in its [provenance](../assets/city-demo.json).
The required workspace checks pass 306 tests and 49 scenario/replay pairs;
all 245 legacy output files remain byte-identical. Native integration checks
pass 55 release tests, strict Clippy and formatting. The opt-in physical-capsule
query correction also retains four byte-identical unflagged native outputs.
[Local validation](../assets/city-demo-validation.json) records these checks.

GitHub denied the requested repository rename with HTTP 403 (`Resource not
accessible by integration`). After the physical checks, only the Cargo
repository URL was corrected to the current `rust_drive` address. The local
validation records both source fingerprints, verifies the original fingerprint
after normalizing that one metadata field, and checks unchanged release-binary
hashes. Workspace checks and the locked native release build were rerun after
that correction. Package names remain `rustdriving-*`; remote CI status is
reported separately from these local results.
