# Capability matrix

“Implemented” means executable code exists and is exercised by the local tests/demo described in validation. It does not mean suitability for real vehicles.

| Capability | Status | Boundary |
|---|---|---|
| Closed-loop perception → prediction → planning → control | Implemented / tested | Reference and optional CPU RNE plants |
| Noisy GNSS + speed / gyro EKF | Implemented / tested | Configured initial heading; no bias state |
| Unlabeled LiDAR and occlusion | Implemented / tested | 720 rays, 45 m range, circular targets |
| Clustering / circle fitting / alpha-beta tracking | Implemented / tested | No semantic classes; nearest-neighbor association |
| Log-odds occupancy map | Implemented / tested | Exported diagnostic; not a planner input; no SLAM |
| Static obstacle avoidance | Implemented / tested | Wide supplied road, three candidate offsets |
| Moving lead / crossing object sensing | Implemented / tested in mission | CV forecast; no actor interaction model |
| Blocked road stop | Implemented / tested | Narrow-road scenario |
| LiDAR / GNSS dropout braking | Implemented / tested | Timestamp freshness and numeric checks |
| Swept collision and road boundary evaluation | Implemented / tested | Circular bodies and planar route corridor |
| Seeded repeatability and telemetry GIF | Implemented / tested | Same build/platform; Python optional |
| Sensor-only shared pipeline | Implemented / tested | Configured route/spawn; no operational truth inputs |
| Versioned sensor-log replay | Implemented / tested | Full recomputation; exact comparison on this build/platform; no physical acceptance inference |
| RNE kinematic/dynamic closed loop | Implemented / tested | CPU-only native plant, planar LiDAR in 3D query scene, friction/steering-lag dynamic model |
| RNE raycast acquisition failure | Implemented / tested | Explicit error causes braking; healthy empty scan is distinct |
| Initially occluded / late lateral crossing fixtures | Implemented / tested | Reference/RNE across 3 seeds; circular scheduled actors, no semantics |
| Low-friction avoidance and stopping | Implemented / tested | RNE dynamic mu=0.2, 0.15 s steering lag, separate longitudinal/lateral limits |
| Calibrated braking / curvature speed limits | Implemented / tested | Fixed known limits; sampled curvature; no combined-friction optimization |
| Linux build/test | Verified locally | Rust 1.90.0; optional RNE uses 1.95.0 |
| macOS / Windows | CI configured; not locally verified | No host-specific Rust dependencies expected |
| Camera, radar, 3D LiDAR, learned detection | Planned | No placeholder inference implementation |
| Learned prediction and training | Planned | No model/data/runtime packaged |
| 3D mapping / SLAM / map localization | Planned | No truth-as-localization substitution |
| Road graph / lane topology / global routing | Planned | v0.1 supplies a single route |
| Traffic lights / signs / right of way / parking | Planned | No traffic-rule claims |
| General dynamically feasible planning / MPC | Planned | Current lattice + pure pursuit baseline |
| CARLA bridge | Planned; not implemented or validated | No CARLA server/assets installed |
| ROS 2 bridge | Planned; optional | No ROS dependency in algorithms |
| Real vehicle / CAN actuation | Out of initial scope | Simulator commands only |
| Production safety / real-time performance | Not established | No certification or throughput benchmark |

There are no empty crates representing the planned capabilities. Future components should land with working behavior, datasets or simulator fixtures, and acceptance checks.
