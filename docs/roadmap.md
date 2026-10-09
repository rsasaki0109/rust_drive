# Development roadmap

RustDrive's long-term target is a practical independent autonomous driving OSS. Milestones are capability gates, not release dates or claims of parity with Autoware, Apollo or openpilot.

The near-term user goal is a **50% engineering maturity estimate**. [Capability waypoints and evidence requirements](maturity.md) define the development direction; the percentage is subjective and does not imply road safety or parity. The current estimate is about 8%.

## M0 — Executable Rust baseline (implemented)

- Cargo workspace with independent algorithm crates and no mandatory ROS.
- Genuine sensor-processing / localization / tracking / prediction / planning / control loop.
- Repeatable curved-road mission, blocked-road stop and sensor freshness faults.
- Actual-run GIF, English design and capability documentation, lockfile and CI definition.
- Local build/test/Clippy and acceptance runs pass. Publication, remote CI and external platform results remain separate.

## M1 — Broader validated simulation (in progress)

Implemented: shared sensor-only pipeline; versioned JSONL input/output replay with truncation/mismatch detection; CPU-only RNE adapter using native kinematic/dynamic vehicles and Rapier ray queries; native friction limits/steering lag; explicit acquisition-failure braking and out-of-order freshness regressions. Both plants pass the supplied mission locally. Hazard fixtures, delayed actor motion and calibrated braking/curvature speed limits are implemented ([initial evidence](hazard-validation.md)). The planner now sweeps synchronized trajectory/forecast polylines, joins lagging maneuvers from the current estimate, and stops and waits for blocked candidates. Multiple blocked alternatives and opposing scheduled crossings extend the suite ([details](swept-planning.md)). Acceleration-aware longitudinal profiles, curvature-dependent local speed caps, retimed collision checks and acceleration feedforward are also implemented ([current evidence](speed-planning.md)). Smooth route interpolation, estimated-heading joins, braking headroom and interpolated pursuit targets reduce emergency fallback in the seeded low-friction fixture ([tracking evidence](tracking.md)). Directed road graphs, deterministic Dijkstra, destination selection and pre-departure closure detours now run in both backends, with minimum-clearance regression floors ([routing evidence](routing.md)). Stopped route handover after live closure snapshots, no-route holding and reopening are now exercised with preserved localization/tracking state and complete map-aware replay ([handover results](handover.md)). A center-return preference based on observed centerline occupancy discourages premature abandonment of unfinished lateral shifts and repairs the seeded 6 m/s detour deadlock ([evidence](avoidance-continuity.md)). Joint GNSS innovation gating, Joseph updates, accepted/observed diagnostics and bounded spike/burst/persistent-fault validation are implemented ([GNSS baseline](gnss-robustness.md)). Candidate-arc-length goal stopping, a terminal side preference for observed approaching traffic and post-arrival physical residence repair that baseline traffic failure ([terminal evidence](terminal-stopping.md)). Optional reactive traffic now follows scalar proximity measurements with bounded one-dimensional motion; stopped leads, resumption, follower braking and traffic queues run in both plants, with independent actor-pair acceptance ([evidence and deadline failures](reactive-traffic.md)). Sustained observed-braking prediction also repairs the unchanged short follower deadline ([current evidence](observed-braking.md)); repeated conservative stopping remains. Mapped stop-line control from timestamped infrastructure signal snapshots is also implemented, with red/yellow/unknown holding, green release, stale-feed recovery, independent physical crossing checks and complete replay ([evidence](traffic-signals.md)). Mapped stop signs now add continuous healthy standstill, brake retention, multiple signs, mixed controls and GNSS-fault recovery with independent physical checks ([evidence](stop-signs.md)). Fixed-route priority crossings additionally use observed predicted zone occupancy, fresh-scan clear dwell, brake retention and independent physical temporal separation ([contract and evidence](intersections.md)). This does not complete M1.

Remaining:

- Covariance and latency propagation, delayed-sensing motion compensation and trace migrations.
- Continuous moving-route handover, routing from arbitrary mid-edge positions, external map import, variable-width/lane topology, turn/speed restrictions and A* for larger maps.
- Continuous lateral offsets and controller-feasibility validation.
- Rectangular collision shapes, calibrated actuator/dynamics models, combined longitudinal/lateral friction feasibility and optimized speed profiles.
- General goal-area stopping/escape, constrained destinations and blocked lateral refuges; the authored GNSS-burst traffic regression is repaired.
- Reduce conservative terminal stop/hold behavior for a following vehicle in a narrow corridor; the authored 65-second deadline now passes, but repeated stops and short-range clearance failures remain.
- Add interaction-aware ego forecasts, richer traffic sensing/steering/priority, slowly varying localization biases and sensing latency.
- Gate: independent collision/rule evaluators, documented failure cases and regression fixtures; no relaxation of constraints to mask failures.

## M2 — CARLA end-to-end integration

- Optional adapter, synchronous server stepping and Rust observation/control bridge.
- Calibration, ENU/Unreal conversion, synchronized LiDAR/GNSS/IMU and explicit actuation scaling.
- Independent CARLA collision callbacks and route-completion metrics on version-pinned towns.
- Verified 3D-to-planar baseline before claiming a 3D perception stack.
- Gate: cold-start reproducibility on a documented server/GPU configuration, no operational ground-truth perception/localization, multiple scenario outcomes and a real CARLA GIF.

## M3 — Maps and multi-sensor understanding

- 3D point processing, ground removal, richer shape tracking and data association.
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
