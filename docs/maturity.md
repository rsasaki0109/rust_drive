# Path toward a broader Rust driving stack

The current engineering estimate is **about 8%**, compared with the scope, independent validation and operating experience of mature driving OSS such as Autoware, Apollo and openpilot. Their operating domains differ; this is a subjective planning estimate, not a benchmark, safety score, percentage of source code or demonstrated parity. The preceding three-vehicle simulation baseline was estimated at about 5%; mapped infrastructure signals subsequently brought the estimate to about 6%.

The first functional extension is mapped signal control: red/yellow/unknown stop-line holding, fresh-green release, accepted-age handling, sensor-only replay and independent physical rule evaluation in both reference and native RNE plants. This broadens executable behavior while retaining the planar prototype's limits. Mapped stop signs now broaden actual authored road behavior with healthy continuous stopping, multiple lines, mixed signal/obstacle constraints and fault recovery. Stop-sign behavior motivated the preceding estimate of about 7%. Basic fixed-route priority yielding adds observed-motion forecast occupancy, healthy fresh-scan release and independent sampled physical priority scoring. This actual behavior extension motivated an estimate of about 8%. Bounded delayed-LiDAR reprojection and authored variations in straight-road geometry, speed, scan cadence and delivery/failure timing retained that estimate: a late second-crossing waiting-margin failure remained documented and broad physical 3D/external validation was absent. Current-time forecast extrapolation now corrects the acquired-origin assumption, and an uncommitted approach envelope repairs that authored late crossing without changing actors, map geometry or physical gates. [Actual evidence and limits](intersections.md#repaired-late-conflict). The estimate remains about 8%: physical 3D sensing, external maps/datasets and broader interaction remain absent. These narrower advances do not establish general intersection negotiation or complete the broader 10% gate. Visualization, documentation and additional test counts alone do not raise the estimate.

## Evidence used for assessment

| Area | Current evidence | Evidence needed for a credible 50% planning milestone |
|---|---|---|
| Sensing / localization | Synthetic planar LiDAR, noisy GNSS/wheel/gyro, bounded acquisition-time pose reprojection and fault regressions | Calibrated 3D and camera observations; held-out real datasets; accuracy, synchronization, full delayed-sensor handling and degraded-sensing measurements |
| Maps / behavior | Authored fixed road graphs, closures, scalar traffic actors, mapped infrastructure signal feed, measured stop-sign holds and basic mapped priority yielding | External map import and lane topology; signals/signs, intersections and lane changes; interaction and traffic-rule evaluation |
| Prediction / planning / control | Current-time observed-motion forecasts, lateral lattice, reachable speed profiles, uncommitted approach bounds and native ego dynamics | Calibrated footprints and actuator models; broader interaction forecasts; optimized planning/control; independent feasibility and difficult-world regressions |
| Simulation / validation | CPU reference and RNE runs, independent circular collision/rule scoring, complete replay | Physical 3D sensor scenes, contact/collision callbacks, multiple environments, varied layouts/traffic, reproducible failures and scenario coverage |
| Engineering / integration | Locked Rust workspaces, three-platform CI, English docs, executable demos | Stable adapter contracts, trace migrations, resource/deadline benchmarks, failure containment, external reproducibility and contributor workflows |
| Deployment / safety evidence | Simulation only | Explicit operating domains and hazard analyses; qualified review before any hardware experiments; no simulator-only road-safety claim |

## Capability gates, not promised release dates

| Planning waypoint | Required work and acceptance |
|---|---|
| Around 10% | Broader authored road behavior: stop signs, signal faults, basic yielding/intersections; fixed rule/collision gates; varied maps, speeds and sensor timing; documented failures |
| Around 20% | RNE physical scenery and 3D sensor acquisition; calibrated rectangular vehicle footprints; external map ingestion; independent contact/rule outcomes and reproducible CPU scenarios |
| Around 30% | Real-data perception and map-localization baselines; optional learned perception with licensed/versioned models; held-out accuracy and calibration checks; bounded GNSS-denied regressions |
| Around 40% | Lane topology/changes, priority reasoning and interaction-aware prediction; optimized longitudinal/lateral control; varied-traffic and actuation-fault acceptance |
| Around 50% | Those capabilities integrated into repeatable end-to-end runs in multiple environments; independently checked resource/latency and failure behavior; stable public interfaces, limitations and external reproduction evidence |

A waypoint is not reached simply by implementing one row or generating more scenarios. Reassess the whole stack against measured capability, validation depth and practical use. Keep unresolved failures in the report, preserve preceding acceptance gates, and publish the evidence that justifies an estimate change. Reaching the 50% planning milestone would still not establish mature real-vehicle deployment or safety certification.

The chronological implementation roadmap remains in [roadmap.md](roadmap.md); executable boundaries are in [capabilities.md](capabilities.md). Signal-control commands and evidence are in [traffic-signals.md](traffic-signals.md); stop-sign behavior is in [stop-signs.md](stop-signs.md); basic priority yielding is in [intersections.md](intersections.md).
