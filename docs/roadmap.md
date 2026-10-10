# Development roadmap

RustDriving's long-term target is a practical independent autonomous driving OSS. Milestones are capability gates, not release dates or claims of parity with Autoware, Apollo or openpilot.

The near-term user goal is a **50% engineering maturity estimate**. [Capability waypoints and evidence requirements](maturity.md) define the development direction; the percentage is subjective and does not imply road safety or parity. The current estimate is about 20%; this remains a subjective simulation-prototype estimate. Bounded turn-rule routing and richer intersection recordings add reproducible behavior, while general driving, perception accuracy and planar/terrain limits remain.

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

Remaining:

- Covariance/forecast uncertainty propagation, delayed odometry/GNSS fusion, per-point LiDAR deskew and trace migrations. Bounded acquisition-time LiDAR reprojection and current-time motion extrapolation are narrower implemented steps, not complete delayed-sensor fusion.
- Continuous moving-route handover, routing from arbitrary mid-edge positions, general external map/lane import, variable-width/lane topology, turn/speed restrictions and A* for larger maps. Bounded OSM road-graph import is implemented.
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
