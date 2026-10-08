# Development roadmap

RustDrive's long-term target is a practical independent autonomous driving OSS. Milestones are capability gates, not release dates or claims of parity with Autoware, Apollo or openpilot.

## M0 — Executable Rust baseline (implemented)

- Cargo workspace with independent algorithm crates and no mandatory ROS.
- Genuine sensor-processing / localization / tracking / prediction / planning / control loop.
- Repeatable curved-road mission, blocked-road stop and sensor freshness faults.
- Actual-run GIF, English design and capability documentation, lockfile and CI definition.
- Local build/test/Clippy and acceptance runs pass. Publication, remote CI and external platform results remain separate.

## M1 — Broader validated simulation

- Sensor-stream replay and versioned trace contracts; covariance and latency propagation.
- Parameterized road graphs, Dijkstra/A* route search, continuous lateral offsets, time-dependent longitudinal profiles and swept trajectory validation.
- Rectangular collision shapes, actuator delay, friction-limited dynamics and longitudinal/lateral feasibility constraints.
- Wider seeded scenario sweeps: occlusion, intersecting actors, stopped lead, localization outliers, delayed/out-of-order sensing and multiple blocked alternatives.
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
