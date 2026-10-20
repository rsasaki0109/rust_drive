# Development roadmap

RustDriving's long-term target is a practical independent autonomous driving OSS. Milestones are capability gates, not release dates or claims of parity with Autoware, Apollo or openpilot.

The near-term user goal is a **50% engineering maturity estimate**. [Capability waypoints and evidence requirements](maturity.md) define the development direction; the percentage is subjective and does not imply road safety or parity. The current estimate is about 29%; this remains a subjective simulation-prototype estimate. Bounded turn-rule routing and richer intersection recordings add reproducible behavior, while general driving, perception accuracy and planar/terrain limits remain.

## M0 — Executable Rust baseline (implemented)

- Cargo workspace with independent algorithm crates and no mandatory ROS.
- Genuine sensor-processing / localization / tracking / prediction / planning / control loop.
- Repeatable curved-road mission, blocked-road stop and sensor freshness faults.
- Actual-run GIF, English design and capability documentation, lockfile and CI definition.
- Local build/test/Clippy and acceptance runs pass. Publication, remote CI and external platform results remain separate.

## M1 — Broader validated simulation (in progress)

Implemented: shared sensor-only pipeline; versioned JSONL input/output replay with truncation/mismatch detection; CPU-only RNE adapter using native kinematic/dynamic vehicles and Rapier ray queries; native friction limits/steering lag; explicit acquisition-failure braking and out-of-order freshness regressions. Both plants pass the supplied mission locally. Hazard fixtures, delayed actor motion and calibrated braking/curvature speed limits are implemented ([initial evidence](hazard-validation.md)). The planner now sweeps synchronized trajectory/forecast polylines, joins lagging maneuvers from the current estimate, and stops and waits for blocked candidates. Multiple blocked alternatives and opposing scheduled crossings extend the suite ([details](swept-planning.md)). Acceleration-aware longitudinal profiles, curvature-dependent local speed caps, retimed collision checks and acceleration feedforward are also implemented ([current evidence](speed-planning.md)). Smooth route interpolation, estimated-heading joins, braking headroom and interpolated pursuit targets reduce emergency fallback in the seeded low-friction fixture ([tracking evidence](tracking.md)). Directed road graphs, deterministic Dijkstra, destination selection and pre-departure closure detours now run in both backends, with minimum-clearance regression floors ([routing evidence](routing.md)). Stopped route handover after live closure snapshots, no-route holding and reopening are now exercised with preserved localization/tracking state and complete map-aware replay ([handover results](handover.md)). A center-return preference based on observed centerline occupancy discourages premature abandonment of unfinished lateral shifts and repairs the seeded 6 m/s detour deadlock ([evidence](avoidance-continuity.md)). Joint GNSS innovation gating, Joseph updates, accepted/observed diagnostics and bounded spike/burst/persistent-fault validation are implemented ([GNSS baseline](gnss-robustness.md)). Candidate-arc-length goal stopping, a terminal side preference for observed approaching traffic and post-arrival physical residence repair that baseline traffic failure ([terminal evidence](terminal-stopping.md)). Optional reactive traffic now follows scalar proximity measurements with bounded one-dimensional motion; stopped leads, resumption, follower braking and traffic queues run in both plants, with independent actor-pair acceptance ([evidence and deadline failures](reactive-traffic.md)). Sustained observed-braking prediction also repairs the unchanged short follower deadline ([current evidence](observed-braking.md)); repeated conservative stopping remains. Mapped stop-line control from timestamped infrastructure signal snapshots is also implemented, with red/yellow/unknown holding, green release, stale-feed recovery, independent physical crossing checks and complete replay ([evidence](traffic-signals.md)). Mapped stop signs now add continuous healthy standstill, brake retention, multiple signs, mixed controls and GNSS-fault recovery with independent physical checks ([evidence](stop-signs.md)). Fixed-route priority crossings additionally use observed predicted zone occupancy, fresh-scan clear dwell, brake retention and independent physical temporal separation ([contract and evidence](intersections.md)). Bounded incoming-edge-aware routing now enforces unconditional node-via OSM and authored `no` / `only` turns, with full restricted-map sensor replay ([evidence](turn-restrictions.md)). This does not complete M1.

Bounded acquisition-time LiDAR reprojection uses EKF history for delayed body-frame returns and map rays. Authored straight crossings vary road width/length, cruise speed, scan delivery cadence, delay and explicit acquisition failure. Motion prediction now advances acquired tracks to the current control clock while retaining original observations and an acquisition-anchored braking budget. An approach-speed envelope while uncommitted and `Proceeding` repairs the authored late second-crossing waiting-margin regression without changing its actors, map geometry or physical gates; `Waiting` keeps its existing bounded stopping prefix. This does not implement full delayed-sensor fusion, uncertainty growth or arbitrary late-threat guarantees. [Timing contract](sensor-replay.md) and [physical before/after evidence](intersections.md#repaired-late-conflict).

The same verified revision adds earlier braking after sustained GNSS innovation rejection and an empirical collision reserve for observed motion across the candidate direction, using estimated heading during stationary holds. Static/parallel objects retain the preceding reserve. The complete 276-run sweep retains the original localization, clearance, low-friction tracking and follower deadline gates, with two short-range follower failures still explicitly rejected. [Current evidence and recorded candidate failures](../assets/prediction-epoch-results.json).

Physical road query surfaces and optional measured local ground removal now extend native XYZ sensing. An explicit research body adds swept upright-box clearance and force-free native overlap witnesses. A bounded importer converts a pinned genuine OpenStreetMap extract into the ordinary ENU road graph, preserving source attribution and explicit simulation width calibration. These steps still retain planar driving and perception after projection. [Ground/body boundaries](ground-lidar.md); [map import and actual route limitations](osm-import.md). The tested native sharp branch now completes with explicit chassis-reference odometry and course-based steering, while retaining original widths/deadlines and default output bytes. [Repair and scope](chassis-reference.md).

Surveyed fixed-route speed intervals now anticipate lower limits and restore
cruise after passing them. Independent full-tick physical scoring and exact
sensor replay cover short, overlapping, stop-sign and rotated-route cases.
An opt-in lane-route exporter now compiles the selected lanes' original speed
limits into these intervals, retaining requested cruise outside lower limits.
A fixed lane-change connector now maps source, shared connector and target/tail
limits in actual arc length, including observed-gap admission. An early return
now carries an explicit lower-lane cap onto its alternate stopping route.
A bounded retry then preserves the connector cap and rebases the unchanged
original target tail, including short intervals and launch stopping prefixes.
[Retry contract](lane-resume-speeds.md).
Optional whole-edge road speed limits now survive stopped closure-driven route
handover, with physical scoring of each active policy and original-map replay.
This keeps map speed values fixed; updating them during driving remains future
work. [Navigation speed contract](navigation-speeds.md).
[Return contract](lane-return-speeds.md); [fixed-change trial and preserved audit error](lane-change-speeds.md).
This does not supply sign detection, live speed-map remapping or general road
rule understanding. [Contract and evidence](route-speed-zones.md).

Acquisition-time GNSS fusion now corrects up to 0.5 s of delayed fixes using bounded wheel/gyro history in the shared pipeline. Reference and native RNE accuracy and fault-recovery trials pass independent chronological reconstruction, with all preceding default outputs retained. Synchronous accepted map-pose/covariance events can be replayed atomically; an explicit option also retains GNSS events and admits map scans up to 0.2 s old in either arrival order. Delayed odometry, clock-offset estimation and broader external correction types remain unsupported. [GNSS improvement](gnss-latency.md), [combined synchronous map history](gnss-map-history.md), [delayed-map mode](delayed-map.md).

Optional [initial pose covariance](initial-pose-covariance.md) now records complete
posterior matrices and composes with temporal fusion. Physical heading offsets
also expose and repair incomplete side-path tracking in both plants. The supplied
±3-degree prior remains authored calibration; measured initialization uncertainty
and automatic sensor-only initialization remain open.

After measured stopped regional recovery, an opt-in
[bounded map-only window](relocalization-map-only.md) can support a new epoch
before fresh GNSS arrives. Fresh synchronous accepted maps must continue;
neither scans nor missing fixes can extend the five-second maximum. Absolute
expiry, interrupted motion and sensor failures retain braking. This improves
authored fixed-route recovery availability; it does not establish automotive
uncertainty or general GNSS-denied driving.

Optional [confirmed GNSS handoff](confirmed-gnss-handoff.md) now starts the
existing recovery policy at the measured reset. Returning proposals remain
withheld until the configured new-fix count; a reset-anchored deadline expires
without a delivery and stays latched through late healthy GNSS. Twenty guarded,
healthy and expiry cases pass, while four paired controls retain premature
acceptance failures. This adds failure handling within the same simulation scope.

Explicit [navigation preservation during measured recovery](navigation-relocalization.md)
now keeps original endpoints, accepted closure revisions, an already switched
active detour and its speed policy. Route admission and fresh stopped handover
confirmations are checked again. Twenty recovered missions reach their goals;
four recovered no-route controls stop, and four healthy/four disabled controls
pass. This extends recovery to authored directed live missions. Mapped traffic
controls, moving handover and global initialization remain open.

An explicit [braked handover confirmation](navigation-handover.md) now keeps a
healthy stopped vehicle braked while three new observations confirm a pending
route. Interruptions discard votes without authorizing acceleration; accepted
cancellation and confirmed switching release the hold. This also applies after
measured recovery and preserves the previous default mission behavior.
All 32 individual cases pass, with 0.20 s recovery handovers at standstill.
All twelve arrivals improve 1.90–2.85 s; five meet the frozen 2 s requirement,
so that comparison remains unmet. A separate viewed navigation-fault regression
now retains the hold through a valid still-blocked recovery snapshot.

Optional [fixed-stop preservation during measured recovery](stop-sign-relocalization.md)
now retains served stops, rearms duties ahead and requires a new full healthy
two-second standstill. Invalid admission keeps emergency braking through fresh
GNSS. Fixed stops also support native chassis/local-route calibration using the
original surveyed prefixes. All 32 follow-up cases pass; original coverage misses,
launch failures and invalid-epoch accuracy failures remain recorded. This extends
fixed-rule simulation recovery and keeps the subjective estimate at about 29%.

Optional [fresh signal permission after measured reset](signal-relocalization.md)
now retains committed lines behind the measured front and withdraws all old
permission. Remaining lines need a strictly newer complete acquisition; invalid
admission stays braked through fresh green/GNSS. All 28 fixed straight/curved
cases pass in both plants, including stop-sign compositions, permanent red and
disabled controls. This extends fixed-rule recovery while retaining full-interval
accuracy failures; subjective maturity stays about 29% and 30% remains unmet.

Fixed straight-route yield entries now have an explicit
[measured-reset preservation mode](yield-relocalization.md). It retains owned
entries beyond their line and restarts all uncommitted clear-scan confirmation;
invalid measured admission remains braked and blocks other mapped duties.
All 32 current inputs pass in two plants; the first 24/32 reclassification and
12 failed original/follow-up layouts remain retained. All 674 preceding and 44
current recordings retain exact replay. This extends authored recovery without completing independent
automotive sensing, measured uncertainty or the 30% waypoint.

Optional [obstruction stop settling](obstruction-stops.md) now retains a completed
collision stop instead of restarting on another partial blocked approach.
A complete clear candidate releases it, while health faults retain braking.
All 48 enabled policy regressions pass in both plants; eight moving-obstruction
and GNSS-loss cases hold actual 0 m/s, resume and reach their goals. Eight
disabled comparisons expose repeated restarts. The preceding blocked-road creep
is repaired, while four mission deadline failures, native launch-heading errors
and uncalibrated forecast coverage remain scored. This does not complete M1.

Remaining:

- Calibrated covariance/forecast uncertainty propagation, delayed odometry and general multi-sensor correction-event replay, per-point LiDAR deskew and trace migrations. Optional [declared motion error envelopes](forecast-envelope.md) propagate acquisition-origin error budgets and preserve a CV alternative, with physical coverage/liveness adoption gates still open. Bounded acquisition-time LiDAR reprojection and current-time motion extrapolation do not complete delayed-sensor fusion.
- Continuous moving-route handover and automatic live-position map matching, general external map/lane import, variable-width/lane topology, general turn rules, live speed-limit remapping and A* for larger maps. Explicit directed interior-edge origin/destination routing, bounded OSM road-graph import and fixed-route speed intervals are implemented. [Partial-route evidence and retained heading failure](mid-edge-routing.md).
- Continuous lateral offsets and controller-feasibility validation.
- Oriented-body planning, measured vehicle/actuator calibration, combined longitudinal/lateral friction feasibility and optimized speed profiles. An authored rectangular evaluation body and conservative planar envelope are implemented.
- General goal-area stopping/escape, constrained destinations and blocked lateral refuges; the authored GNSS-burst traffic regression is repaired.
- Reduce conservative terminal stop/hold behavior for a following vehicle in a narrow corridor; the authored 65-second deadline now passes, but repeated stops and short-range clearance failures remain.
- Add interaction-aware ego forecasts, richer traffic sensing/steering/priority, slowly varying localization biases and broader sensing-latency acceptance.
- Gate: independent collision/rule evaluators, documented failure cases and regression fixtures; no relaxation of constraints to mask failures.

## M2 — CARLA end-to-end integration

- Optional adapter, synchronous server stepping and Rust observation/control bridge.
- Calibration, ENU/Unreal conversion, synchronized LiDAR/GNSS/IMU and explicit actuation scaling.
- Independent CARLA collision callbacks and route-completion metrics on version-pinned towns.
- Verified 3D-to-planar baseline before claiming a 3D perception stack.
- Gate: cold-start reproducibility on a documented server/GPU configuration, no operational ground-truth perception/localization, multiple scenario outcomes and a real CARLA GIF.

## M3 — Maps and multi-sensor understanding

Bounded XYZ terrain/components, measured-data acquisition/evaluation and optional local fixed-map EKF corrections now execute. Reference/native five-second GNSS-denied runs and failure controls are replayed. This milestone remains incomplete: the frozen terrain baseline has held-out F1 0.2592 with six failed sites; apartment successes use imposed transforms and warm seeds, while natural alignment is rejected. Sparse native object acquisition also exposes ground false positives and missed actor clusters. [Measurements](datasets.md); [local matching](map-localization.md).

A density-aware classifier now raises the original calibration/regression scores, while one new thinned Autzen environment fails its frozen acceptance gates. Optional recorded RGB-D registration and CPU learned inference also execute, with explicit temporal-protocol and single-image limits. [Adaptive terrain](adaptive-ground.md); [recorded motion](recorded-rgbd.md); [offline inference](../integrations/onnx/README.md).

Recorded-depth odometry now completes eleven fresh consecutive pair estimates
and composes their natural 6DoF motion, with nine accepted fixed-first-cloud map
fits and two ambiguity rejections. The first missing-mocap trial is preserved;
full pair-plus-map acceptance remains unmet. Six independently annotated urban
photographs extend learned inference beyond the portrait, retaining low recall
and possible upstream COCO validation overlap. [Motion evidence](recorded-motion.md);
[camera evidence](road-camera-evaluation.md).

Measured keyframe localization now adds bounded reference replacement, explicit
root-frame chaining and accepted-pose expiry. It repairs a viewed short
fixed-map rejection case, but its first preregistered 35-update temporal trial
accepts 33 updates and scores only 13 root-accurate. The failed original and
exact source snapshot remain archived; chronological-malformed-input and
missing-reference audit corrections reproduce the same numerical result as
viewed regressions. [Implementation and retained drift](recorded-keyframes.md).

Fixed-root bounded measured submaps now fuse accepted scans with atomic voxel updates, but the first desk trial passes root accuracy on only 8/35 updates, while a separate office/camera trial scores 5/35 and latches loss. Eight separately frozen original BDD dashcam frames also expose low automotive recall (35/138 objects). [Submap evidence](recorded-submaps.md); [BDD evidence](bdd-road-evaluation.md).

An additional RGB-D visual path now associates original Rust image features
with registered depth before bounded robust rigid fitting. It improves the
viewed desk recording to 24/35 root-accurate updates, but the unchanged office
configuration still scores only 9/35. An independent pixel/descriptor/depth
reconstruction and separate SVD fitter verify both results. Repeated RGB images
cannot renew tracking, and accepted-pose expiry latches loss.
The separate first frozen sitting trial scores 31/35, with four retained
rejections; it also fails complete availability acceptance.
[Visual-motion protocol and full outcomes](recorded-visual-odometry.md).

Bounded pixel reprojection now repairs the viewed office result to 32/35
accurate updates without changing the original 3D support or tracking clocks.
The separate first Freiburg 2 attempt correctly rejects malformed full-source
labels; later viewed traces have no independently scorable accuracy.
[Refinement and retained input failure](recorded-reprojection.md).

Next: qualify full-source label timestamp ordering during metadata-only dataset
preparation, then freeze a new trial without selecting on accuracy. Diagnose
remaining measured-motion drift and validate camera/depth
calibration and timing; keep development on viewed recordings and freeze each
new accuracy trial before its first evaluation. Improve distant road-user
detection, set explicit per-class acceptance gates and evaluate a frozen model
on new road images. Validate uncertainty, measured extrinsics and
camera/geometry integration before claiming the 30% planning waypoint. Preserve
all observed failures as regression evidence.

- General 3D point processing and terrain classification, richer shape tracking and data association. Bounded local measured-ground removal is implemented for near-flat native query scenes.
- Map formats and map localization, inertial bias estimation and bounded GNSS-denied tests.
- Camera/radar fusion and optional learned models behind established contracts.
- Gate: versioned datasets, calibration validation, license review and measured accuracy/latency. Publish classical baselines as comparisons.

## M4 — Behavior and optimized control

- Traffic controls, priority/yield reasoning, intersections and lane topology.
- Interaction/multimodal prediction, optimizer-based planning and dynamic-model control.
- ROS 2 integration as an optional bridge; introduce mature transport only for concrete deployment requirements.
- Gate: rule/collision scoring, failure injection, resource/deadline measurements and deterministic replay of regressions.

## M5 — Deployment research

- Vehicle model adaptation, hardware-in-the-loop and controlled-track experiments only after a documented safety review.
- Redundant health monitoring, safe actuation architecture, sensor fault containment, system identification and operational domain constraints.
- Safety engineering and applicable standards with qualified reviewers. Passing simulator tests cannot establish road safety.

No real-vehicle actuator is part of the initial release. None of the planned milestones is represented as an empty implementation in the codebase.

The [first metadata-qualified room trial](qualified-recorded-motion.md) passes
35/35 updates in a fixed 1.1684-second interval with unchanged defaults. This
closes one short indoor evaluation, while broader scenes, longer durations,
measured vehicle calibration, uncertainty and camera-to-driving fusion remain
future work. The earlier failed protocols remain required regressions.

The [continuous same-room extension](temporal-recorded-motion.md) limits the
preceding short success: 52/179 updates meet root accuracy, then tracking is lost.
Improved measured-feature support, rotation accuracy and independently evaluated
relocalization are needed before sustained localization can be claimed. Origin
resets, dropped failures and relaxed accuracy gates do not resolve this result.

An opt-in [depth-supported descriptor domain](depth-supported-matching.md)
restores one extra fit in the viewed window, but lowers root accuracy from
52/179 to 49/179 and still loses tracking. It is not promoted to the original
path. Descriptor stability and accumulated rotation error remain priorities;
any recovery method must retain an independently evaluated continuous root.

The [pairwise pyramidal tracking comparison](pyramidal-recorded-tracking.md)
regresses to 6/179 accepted and accurate updates, losing tracking at 112.
Coarsest-level nonconvergence and insufficient coherent depth support dominate
this failure. The method remains opt-in. Better sensor-only image alignment and
independently evaluated recovery are needed; synthetic translation success and
forward/backward agreement do not establish correspondence uniqueness.
