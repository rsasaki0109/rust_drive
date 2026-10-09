# Path toward a broader Rust driving stack

The current engineering estimate is **about 15%**, compared with the scope, independent validation and operating experience of Autoware, Apollo and openpilot. Their operating domains differ. This is a subjective planning estimate, not a benchmark, safety score, percentage of source code or demonstrated parity.

The preceding estimate of about 8% reflected authored planar road behavior, observed-motion prediction, mapped infrastructure rules, fault recovery and optional inclined native LiDAR. The new increment adds operational measured-ground separation on physical native query surfaces, an explicit rectangular research body with independently reconstructed swept static-box clearance and actual native overlap rejection, and a bounded attributed external OSM importer. Those functions are integrated: genuine imported road coordinates drive native sensor acquisition, localization, planning, control and complete sensor-only replay in nine verified episodes. The new ground/body matrix has 60 healthy episodes and six separate confidence-stop cases. [Measured ground/body evidence](ground-lidar.md); [external-map evidence and retained sharp-turn failure](osm-import.md).

The estimate reflects these additional executable capabilities and independent checks. It does not increase for GIF quality or test counts. The research box is authored calibration, not a measured production vehicle; moving actors retain circular acceptance, native ego motion is planar, and Rapier overlap witnesses do not supply chassis contact response. Ground fitting assumes broad, near-flat support, can remove near-plane obstacle surfaces, and can falsely block on sparse/narrow ground. External OSM supplies historical geographic centerlines with explicit simulated widths, without lane topology or legal turn rules. A sharper imported branch passes only in the reference plant; native dynamics still deadlock safely and that negative is retained. Real datasets, camera perception, SLAM, general terrain, general intersection negotiation and real-vehicle interfaces remain absent. These limits prevent treating this as the broader 20% milestone or mature-stack parity.

## Evidence used for assessment

| Area | Current evidence | Evidence needed for a credible 50% planning milestone |
|---|---|---|
| Sensing / localization | Native XYZ LiDAR with measured local ground separation and confidence-failure braking; planar projection; noisy GNSS/wheel/gyro and bounded acquisition-time reprojection | General calibrated extrinsics and camera observations; volumetric perception; held-out real datasets; accuracy, synchronization, full delayed-sensor handling and degraded-sensing measurements |
| Maps / behavior | Bounded attributed OSM centerline import plus authored graphs/closures; infrastructure signal feed, stop-sign holds and basic fixed-route yielding | Lane topology, legal turn restrictions and more external layouts; general intersections/lane changes and interaction evaluation |
| Prediction / planning / control | Current-time observed-motion forecasts, lateral lattice, reachable speed profiles, uncommitted approach bounds and native ego dynamics | Calibrated footprints and actuator models; broader interaction forecasts; optimized planning/control; independent feasibility and difficult-world regressions |
| Simulation / validation | CPU reference/native RNE runs; physical road/obstacle query surfaces; independent static-box sweeps, actual force-free overlap negatives, circular traffic scoring and complete replay | Physical 3D sensor scenes, contact/collision callbacks, multiple environments, varied layouts/traffic, reproducible failures and scenario coverage |
| Engineering / integration | Locked Rust workspaces, three-platform CI, English docs, executable demos | Stable adapter contracts, trace migrations, resource/deadline benchmarks, failure containment, external reproducibility and contributor workflows |
| Deployment / safety evidence | Simulation only | Explicit operating domains and hazard analyses; qualified review before any hardware experiments; no simulator-only road-safety claim |

## Capability gates, not promised release dates

| Planning waypoint | Required work and acceptance |
|---|---|
| Around 10% | Broader authored road behavior: stop signs, signal faults, basic yielding/intersections; fixed rule/collision gates; varied maps, speeds and sensor timing; documented failures |
| Around 15% | Measured ground processing on physical native query surfaces, an explicit rectangular research body with independent swept clearance and native overlap rejection, attributed external-map import, and an integrated sensor-only native run/replay; preserve known failures and preceding acceptance floors |
| Around 20% | Broader sensor/support geometry and external road layouts; measured vehicle/extrinsics calibration; supported native sharp turns; independent contact/rule outcomes and reproducible failure coverage |
| Around 30% | Real-data perception and map-localization baselines; optional learned perception with licensed/versioned models; held-out accuracy and calibration checks; bounded GNSS-denied regressions |
| Around 40% | Lane topology/changes, priority reasoning and interaction-aware prediction; optimized longitudinal/lateral control; varied-traffic and actuation-fault acceptance |
| Around 50% | Those capabilities integrated into repeatable end-to-end runs in multiple environments; independently checked resource/latency and failure behavior; stable public interfaces, limitations and external reproduction evidence |

A waypoint is not reached simply by implementing one row or generating more scenarios. Reassess the whole stack against measured capability, validation depth and practical use. Keep unresolved failures in the report, preserve preceding acceptance gates, and publish the evidence that justifies an estimate change. Reaching the 50% planning milestone would still not establish mature real-vehicle deployment or safety certification.

The chronological implementation roadmap remains in [roadmap.md](roadmap.md); executable boundaries are in [capabilities.md](capabilities.md). Signal-control commands and evidence are in [traffic-signals.md](traffic-signals.md); stop-sign behavior is in [stop-signs.md](stop-signs.md); basic priority yielding is in [intersections.md](intersections.md).
