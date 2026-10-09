# Reference project research

Research performed on 2026-10-08 using official repository README, license, module, and configuration sources. These are architectural observations and design tradeoffs, not independent performance or safety benchmarks. No source code or model weights from these projects were copied into RustDriving.

## Sources examined

| Project | Source revision | Official files examined |
|---|---|---|
| Autoware | `198c62fb50bd0b4d50d4295ba07735d4b64844b1` | [README](https://github.com/autowarefoundation/autoware/blob/198c62fb50bd0b4d50d4295ba07735d4b64844b1/README.md), [repository manifest](https://github.com/autowarefoundation/autoware/blob/198c62fb50bd0b4d50d4295ba07735d4b64844b1/repositories/autoware.repos), [LICENSE](https://github.com/autowarefoundation/autoware/blob/198c62fb50bd0b4d50d4295ba07735d4b64844b1/LICENSE) |
| Apollo | `d53aa3da47a06a08e6d0cd175d5623a34fa0d6aa` | [README](https://github.com/ApolloAuto/apollo/blob/d53aa3da47a06a08e6d0cd175d5623a34fa0d6aa/README.md), [Cyber RT](https://github.com/ApolloAuto/apollo/blob/d53aa3da47a06a08e6d0cd175d5623a34fa0d6aa/cyber/README.md), [prediction](https://github.com/ApolloAuto/apollo/blob/d53aa3da47a06a08e6d0cd175d5623a34fa0d6aa/modules/prediction/README.md), [localization](https://github.com/ApolloAuto/apollo/blob/d53aa3da47a06a08e6d0cd175d5623a34fa0d6aa/modules/localization/README.md), LICENSE |
| openpilot | `a742df61827f8b34eee1af99bbb881201163e472` | [README](https://github.com/commaai/openpilot/blob/a742df61827f8b34eee1af99bbb881201163e472/README.md), [cereal](https://github.com/commaai/openpilot/blob/a742df61827f8b34eee1af99bbb881201163e472/openpilot/cereal/README.md), [process configuration](https://github.com/commaai/openpilot/blob/a742df61827f8b34eee1af99bbb881201163e472/openpilot/system/manager/process_config.py), [service frequencies](https://github.com/commaai/openpilot/blob/a742df61827f8b34eee1af99bbb881201163e472/openpilot/cereal/services.py), LICENSE |

The repositories were inspected through shallow, filtered read-only reference clones outside the RustDriving checkout. An older Apollo architecture note describing ROS 1 was also examined, but is historical: the current design assessment uses Cyber RT. openpilot's architecture page at the inspected revision was just a heading, so the actual messaging and process definitions were used instead. GitHub API access was unavailable; native Git and official raw sources were usable.

## Autoware

Autoware targets a complete autonomous driving system. Its top-level repository assembles separately versioned components for messages, common utilities, core algorithms, a wider universe of algorithms, launch configuration, sensor drivers, localization dependencies, and simulators. The inspected manifest explicitly includes ROS-related message packages, ROS 2 CAN/transport drivers, Lanelet2 extensions, core and universe modules, and a simple planning simulator.

**Strengths:** clear subsystem boundaries, an established sensor/map ecosystem, many interchangeable algorithms, and explicit deployment configuration. Its core/universe organization offers a useful way to distinguish small maintained essentials from experimental alternatives.

**Tradeoffs for RustDriving:** bringing this ecosystem directly into a minimal Rust baseline would add ROS/build/dependency requirements before the driving loop works. Independent repositories and pinned deployment manifests improve modularity but introduce version compatibility and system integration work. This is a dependency/workflow assessment, not a claim that ROS 2 is inherently slow.

**Adopt:** perception/localization/mapping/prediction/planning/control boundaries, explicit sensor and vehicle adaptation, documented configurations and runnable simulator scenarios.

**Change:** use one Cargo workspace initially; no mandatory ROS 2 dependency; a small deterministic default demo. Add mature map formats and ROS interfaces when adapters can be validated.

## Apollo

Apollo supplies an extensive autonomous driving platform spanning drivers, perception, localization, prediction, planning, control, tools and simulation. Cyber RT is its specialized runtime with adaptive messaging, scheduling, well-defined tasks and developer tooling. Prediction separates containers, scenario reasoning, evaluators, and predictors, including classical and learned alternatives. Localization offers RTK, NDT and multisensor fusion components. The README documents progression from constrained waypoint driving toward richer urban scenarios.

**Strengths:** explicit module input/output contracts, scenario-based behavior organization, broad tooling and visualization, coexistence of learned and classical methods, and staged capability development.

**Tradeoffs for RustDriving:** a specialized runtime, package/configuration system, model artifacts and platform requirements create a large adoption and maintenance surface. The breadth cannot be reproduced merely by exposing similarly named APIs. A new project needs smaller independently validated increments.

**Adopt:** staged operating domains and acceptance criteria, separation of object prediction from ego planning, explicit health/configuration inputs, and recorded evaluation.

**Change:** ordinary Rust calls are enough for the first synchronous pipeline. Use a single constant-velocity predictor and small candidate planner as working baselines; defer an evaluator/plugin framework until multiple implementations justify it.

## openpilot

openpilot is principally a driver-assistance system rather than an interchangeable full urban autonomous-driving platform. Its process configuration separates sensor acquisition, model inference, localization, radar, planning, control, vehicle interaction, monitoring, logging, and UI. `cereal` uses msgq pub/sub and Cap'n Proto serialization. Messages carry monotonic timestamps and validity; its documented practices require SI units. Service configuration records frequency, logging and queue sizes (for example controls and vehicle state at 100 Hz, model and longitudinal planning at 20 Hz at the inspected revision).

**Strengths:** executable integration around a concrete driving function, model/classical/control separation, explicit process lifecycle, timestamped telemetry, and development through reproducible observations.

**Tradeoffs for RustDriving:** vehicle and device integration is intentionally specific, an AI pipeline requires additional artifacts and inference support, and driver assistance does not supply general routing, mapping or all autonomous traffic behavior. Its operational claims cannot be inherited by a new implementation.

**Adopt:** logs that connect sensing to decisions, freshness checks, human-readable SI metrics, and a demo that visibly exercises the complete loop.

**Change:** simulator-only outputs, transport-neutral Rust data, no hardware/device coupling or obligatory learned model.

## Systematic function inventory

| Area | Minimum baseline built here | Full-stack needs beyond v0.1 |
|---|---|---|
| Sensors / calibration | Synthetic planar LiDAR, GNSS, speed, gyro, configured spawn heading | Drivers, extrinsics, synchronization, uncertainty, camera/radar/3D LiDAR |
| Perception | Clustering, circle fitting, tracking | Semantic detection, free space, multimodal fusion, occlusion reasoning |
| Localization | Planar GNSS / odometry EKF | IMU biases, map matching, GNSS-denied operation, 3D pose |
| Mapping / routing | Supplied centerline, occupancy grid | SLAM, HD map formats, road graph, route search, map updates |
| Prediction | Constant velocity | Multimodal behavior, interactions, intent, calibrated uncertainty |
| Planning | Local offset candidates and stopping | Rules, intersections, lane changes, parking, optimization, feasibility |
| Control | Pure pursuit and PI bicycle control | Dynamic model, actuator latency, system identification, redundancy |
| System / validation | Freshness guard, seeded scenarios, CI and trace rendering | Fault containment, deadlines, replay datasets, adversarial scenarios, safety case |

## License strategy

At the inspected revisions, the top-level Autoware and Apollo licenses are Apache-2.0, and openpilot is MIT. RustDriving adopts Apache-2.0 for original code. Architectural ideas can be studied without copying implementation; this baseline contains no imported implementation from the reference stacks.

Apache-2.0 and MIT code can generally be combined in an Apache-2.0 distribution subject to preserving licenses, applicable notices and attribution. That does **not** establish that every nested component, dataset, map, trained weight, simulator asset, vendor SDK, or transitive dependency has the same terms. Before any future reuse, review the exact artifact and revision, preserve required notices, and assess patent, redistribution and model/data restrictions. Do not infer licensing from the parent repository.

The core's external Rust packages are limited to serialization and their macro dependencies. Their license expressions are recorded in [THIRD_PARTY.md](../THIRD_PARTY.md); there is no inference runtime or model weight in this release. CARLA licensing and separately supplied map assets must be reviewed when introducing that adapter. No reference project's name or operational maturity is used as evidence of RustDriving's safety or performance.

## Robot Native Engine as an optional CPU test backend

[RNE](https://github.com/rsasaki0109/RobotNativeEngine) is a Rust-native robotics engine, dual MIT/Apache-2.0. Rather than adding a new executor or making graphics dependencies mandatory, RustDriving embeds its existing native vehicle systems and backend-neutral sensor contracts. The adapter converts RNE Y-up poses/world LiDAR to the same ENU/body-frame boundary used by reference simulation and replay.

The initial integration exposed a stale Rapier query-geometry bug and the need for an acquisition API that distinguishes backend failure from a healthy empty scan. Both are fixed in the [pinned engine revision](https://github.com/rsasaki0109/RobotNativeEngine/commit/df6007aa40315e81d12ae00fc1f60369e393a178), with regressions and additive API behavior. This is direct, locally executed integration evidence, not a feature-completeness review of the entire engine. [Implementation and limits](../integrations/rne/README.md).
