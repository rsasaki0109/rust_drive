# Path toward a broader Rust driving stack

The current engineering estimate is **about 20%**, compared with the scope, independent validation and operating experience of Autoware, Apollo and openpilot. Their operating domains differ. This is a subjective planning estimate, not a benchmark, safety score, percentage of source code or demonstrated parity.

The preceding estimate of about 8% reflected authored planar road behavior, observed-motion prediction, mapped infrastructure rules, fault recovery and optional inclined native LiDAR. The new increment adds operational measured-ground separation on physical native query surfaces, an explicit rectangular research body with independently reconstructed swept static-box clearance and actual native overlap rejection, and a bounded attributed external OSM importer. Those functions are integrated: genuine imported road coordinates drive native sensor acquisition, localization, planning, control and complete sensor-only replay in nine verified episodes. The new ground/body matrix has 60 healthy episodes and six separate confidence-stop cases. [Measured ground/body evidence](ground-lidar.md); [external-map evidence and repaired sharp-turn failure](osm-import.md).

The preceding 15% increment reflects those executable capabilities and independent checks. It did not increase for GIF quality or test counts. The research box is authored calibration, not a measured production vehicle; moving actors retain circular acceptance, native ego motion is planar, and Rapier overlap witnesses do not supply chassis contact response. Ground fitting assumes broad, near-flat support, can remove near-plane obstacle surfaces, and can falsely block on sparse/narrow ground. External OSM supplies historical geographic centerlines with explicit simulated widths, without lane topology; bounded unconditional node-via turn rules have since been implemented ([scope](turn-restrictions.md)). Explicit chassis-reference prediction and course-based steering repair the tested sharper native branch; its original failure remains historical evidence, and broader sharp-route feasibility remains unverified.

The new increment adds actual measured-data evaluation, bounded classical XYZ terrain/components, operational local scan-to-map EKF corrections and five-second GNSS-denied reference/native driving with independent expiry/failure controls. It **does not reach the requested 30%**. The frozen terrain baseline has calibration F1 0.7041 but held-out F1 0.2592, including six zero-F1 sites. Four apartment pose successes are warm-initialized imposed transforms of measured geometry; natural-pair alignment is rejected, and they do not establish real-motion pose accuracy or covariance calibration. An initial sparse native terrain/object acquisition conservatively blocks on residual ground and misses actor clusters. Existing dense acquisition and a declared research sensing height subsequently repair one native lead-stop case, with actual actor membership and complete replay checked independently. The failed sparse capture remains replayable. These failures remain evidence, rather than being removed or retuned against held-out labels. [Measured-data results](datasets.md); [bounded localization](map-localization.md); [experimental native objects](terrain-objects.md).

The latest increment adds density-aware terrain processing with a verified native lead-stop run, bounded local SE(3) matching against actual recorded depth/mocap, and optional Rust-native CPU learned image detection. The new environment terrain gate still fails (Autzen F1 0.3571); the original natural-pair protocol accepts 7/11 later temporal pairs in one room and rejects four, with markedly overconfident conditional covariance. All failed stages remain reproducible. These working but narrow additions support an estimate of about 20%; **the requested 30% remains unmet**. [Adaptive terrain](adaptive-ground.md); [recorded motion and covariance failures](recorded-rgbd.md); [optional inference](../integrations/onnx/README.md).

The subsequent [urban-camera diagnostic](road-camera-evaluation.md) actually
executes six real photographs with independent annotations: evaluator-held-out
precision 0.778 and recall 0.560. Its COCO validation images may overlap upstream
model selection, and traffic lights and small road users are missed. It supplies
reproducible diagnostics, without proving independent driving-domain accuracy.
Additional recorded-depth odometry and fixed-first-cloud localization preserve
sensor-only initialization, physical-reference gaps and rejected map fits. Their
engineering covariance allowance is explicitly provisional, rather than a
calibrated confidence guarantee. These additions do not increase the overall
estimate or close the remaining independent-environment and calibration gates.

Measured keyframe localization additionally replaces its reference with accepted
recorded clouds and latches loss on expired accepted poses, including malformed
acquisitions. It repairs the viewed short fixed-map rejection case, but the
preregistered longer temporal trial has only **13/35 root-accurate updates** and
two ambiguity rejections. The maximum accumulated error is 0.170314 m /
0.163528 rad. Its original sources and failure remain immutable; subsequent
guard fixes and repeats are explicitly viewed regressions. This does not raise
the estimate, establish automotive localization, or close the 30% waypoint.
[Keyframe protocol and evidence](recorded-keyframes.md).

General reliable terrain, measured vehicle/extrinsics, automotive camera perception, SLAM, lane topology, broader interaction/control and real-vehicle interfaces remain unresolved. The broader 20% calibration/contact goals also remain open. The estimate recognizes working local localization and reproducible measurement capability, without calling the calibration/contact waypoint or the 30% waypoint complete.

## Evidence used for assessment

| Area | Current evidence | Evidence needed for a credible 50% planning milestone |
|---|---|---|
| Sensing / localization | Native XYZ LiDAR, optional measured AABBs/terrain; failed held-out airborne PMF baseline; local fixed-map matching with bounded GNSS-denied native driving; noisy GNSS/wheel/gyro and acquisition-time reprojection | Reliable held-out automotive perception and real-motion map matching; measured extrinsics and camera observations; synchronization, covariance calibration and broader degraded-sensing measurements |
| Maps / behavior | Bounded attributed OSM centerline import with unconditional motorcar node-via no/only rules plus authored graphs/closures; infrastructure signal feed, stop-sign holds and basic fixed-route yielding | Lane topology, conditional/way-via restrictions and more external layouts; general intersections/lane changes and interaction evaluation |
| Prediction / planning / control | Current-time observed-motion forecasts, lateral lattice, reachable speed profiles, uncommitted approach bounds and native ego dynamics | Calibrated footprints and actuator models; broader interaction forecasts; optimized planning/control; independent feasibility and difficult-world regressions |
| Simulation / validation | CPU reference/native RNE runs; physical road/obstacle query surfaces; independent static-box sweeps, actual force-free overlap negatives, circular traffic scoring and complete replay | Physical 3D sensor scenes, contact/collision callbacks, multiple environments, varied layouts/traffic, reproducible failures and scenario coverage |
| Engineering / integration | Locked Rust workspaces, three-platform CI, English docs, executable demos | Stable adapter contracts, trace migrations, resource/deadline benchmarks, failure containment, external reproducibility and contributor workflows |
| Deployment / safety evidence | Simulation only | Explicit operating domains and hazard analyses; qualified review before any hardware experiments; no simulator-only road-safety claim |

## Capability gates, not promised release dates

| Planning waypoint | Required work and acceptance |
|---|---|
| Around 10% | Broader authored road behavior: stop signs, signal faults, basic yielding/intersections; fixed rule/collision gates; varied maps, speeds and sensor timing; documented failures |
| Around 15% | Measured ground processing on physical native query surfaces, an explicit rectangular research body with independent swept clearance and native overlap rejection, attributed external-map import, and an integrated sensor-only native run/replay; preserve known failures and preceding acceptance floors |
| Around 20% | Broader sensor/support geometry and external road layouts; measured vehicle/extrinsics calibration; broader native sharp turns; independent contact/rule outcomes and reproducible failure coverage |
| Around 30% | Real-data perception and map-localization baselines; optional learned perception with licensed/versioned models; held-out accuracy and calibration checks; bounded GNSS-denied regressions |
| Around 40% | Lane topology/changes, priority reasoning and interaction-aware prediction; optimized longitudinal/lateral control; varied-traffic and actuation-fault acceptance |
| Around 50% | Those capabilities integrated into repeatable end-to-end runs in multiple environments; independently checked resource/latency and failure behavior; stable public interfaces, limitations and external reproduction evidence |

A waypoint is not reached simply by implementing one row or generating more scenarios. Reassess the whole stack against measured capability, validation depth and practical use. Keep unresolved failures in the report, preserve preceding acceptance gates, and publish the evidence that justifies an estimate change. Reaching the 50% planning milestone would still not establish mature real-vehicle deployment or safety certification.

The chronological implementation roadmap remains in [roadmap.md](roadmap.md); executable boundaries are in [capabilities.md](capabilities.md). Signal-control commands and evidence are in [traffic-signals.md](traffic-signals.md); stop-sign behavior is in [stop-signs.md](stop-signs.md); basic priority yielding is in [intersections.md](intersections.md).
