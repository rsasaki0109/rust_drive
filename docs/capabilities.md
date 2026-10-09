# Capability matrix

“Implemented” means executable code exists and is exercised by the local tests/demo described in validation. It does not mean suitability for real vehicles.

| Capability | Status | Boundary |
|---|---|---|
| Closed-loop perception → prediction → planning → control | Implemented / tested | Reference and optional CPU RNE plants |
| Noisy GNSS + speed / gyro EKF | Implemented / tested | Full x/y innovation gate, Joseph covariance updates, observed/accepted diagnostics; configured initial heading, no bias state |
| Unlabeled LiDAR and occlusion | Implemented / tested | 720 rays, 45 m range, circular targets |
| Clustering / circle fitting / alpha-beta tracking | Implemented / tested | No semantic classes; nearest-neighbor association |
| Log-odds occupancy map | Implemented / tested | Exported diagnostic; not a planner input; no SLAM |
| Static obstacle avoidance | Implemented / tested | Wide supplied road, three candidate offsets |
| Moving lead / crossing object sensing | Implemented / tested in mission | CV / observed-braking ego forecast; optional simulator actors react to scalar proximity observations |
| Reactive traffic following / stopped lead / queue | Implemented / tested | 24 seeded positive runs; ideal route proximity sensing and bounded 1D actors, no steering, priority or interaction-aware ego forecast |
| Traffic-pair collision and endpoint acceptance | Implemented / tested | Reactive actor pairs swept independently; negative collisions/overruns fail; scheduled-only pairs retain earlier criteria |
| Narrow-corridor follower goal deadline | Authored regression repaired | Unchanged 65 s deadline / 8 s residence passes in both plants across 3 seeds; repeated conservative emergency stops remain |
| Observed-braking prediction | Implemented / tested | Sustained sensor-derived track deceleration, bounded one-second persistence then coasting; no intentions, multimodal forecast or guaranteed future braking |
| Short-range follower response | Known physical failure | Two RNE 5 m sensing cases violate the unchanged 1 m clearance floor; separately rejected |
| Blocked road stop | Implemented / tested | Narrow-road scenario |
| GNSS spike / burst / persistent-bias response | Implemented / tested | 42 physical fault runs with bounded error, accepted-age braking and recovery; scheduled traffic and post-arrival hold regressions |
| Terminal stopping-place preference / physical residence | Implemented / tested | Route-relative observed forward traffic, feasible lateral candidates; 16-second truth hold in two plants/three seeds, no general parking or escape planner |
| LiDAR / GNSS dropout braking | Implemented / tested | Timestamp freshness and numeric checks |
| Swept collision and road boundary evaluation | Implemented / tested | Circular bodies and planar route corridor |
| 3D visualization of recorded RNE driving | Implemented / tested | Blender Cycles CPU replay, pose/actor audit, original suburban assets and editable scene snapshots; planar physics, no sensor-camera feed |
| Seeded repeatability and telemetry GIF | Implemented / tested | Same build/platform; Python optional |
| Sensor-only shared pipeline | Implemented / tested | Configured route/spawn; no operational truth inputs |
| Versioned sensor-log replay | Implemented / tested | Full recomputation; exact comparison on this build/platform; no physical acceptance inference |
| RNE kinematic/dynamic closed loop | Implemented / tested | CPU-only native plant, planar LiDAR in 3D query scene, friction/steering-lag dynamic model |
| RNE raycast acquisition failure | Implemented / tested | Explicit error causes braking; healthy empty scan is distinct |
| Initially occluded / late lateral crossing fixtures | Implemented / tested | Reference/RNE across 3 seeds; circular scheduled actors, no semantics |
| Low-friction avoidance and stopping | Implemented / tested | RNE dynamic mu=0.2, 0.15 s steering lag, separate longitudinal/lateral limits |
| Calibrated braking / curvature speed limits | Implemented / tested | Fixed known limits; sampled curvature; no combined-friction optimization |
| Continuous candidate collision checks | Implemented / tested | Synchronized circular sweeps; accelerated-segment chord bound and retiming revalidation |
| Reachable longitudinal speed profiles | Implemented / tested | Forward/backward acceleration bounds, local curvature caps, finite arrival times; no joint tire-force or jerk optimization |
| Smooth route interpolation / heading join | Implemented / tested | C2 centerline segments and estimated-heading correction; sampled corridor containment, no joint geometry/control optimization |
| Low-friction tracking regression gate | Implemented / tested | Fixed RNE calibration, 3 seeds, at most 20 emergency ticks in the specified fixture; no universal bound |
| Acceleration feedforward control | Implemented / tested | First-segment acceleration with PI feedback; steering still pure pursuit |
| Stop and wait on blocked candidates | Implemented / tested | Feasible stopping profile and eight-second forecast hold, release on a clear candidate; no semantic priority rules |
| Multiple blocked alternatives / opposing crossings | Implemented / tested | Scheduled circular actors; independent physical acceptance and full replay |
| Linux build/test | Verified locally | Rust 1.90.0; optional RNE uses 1.95.0 |
| macOS / Windows | Remote checks observed on recorded revisions | Results and runner failures in [validation](validation.md); no local hosts |
| Camera, radar, 3D LiDAR, learned detection | Planned | No placeholder inference implementation |
| Learned prediction and training | Planned | No model/data/runtime packaged |
| 3D mapping / SLAM / map localization | Planned | No truth-as-localization substitution |
| Directed road graph / shortest-distance routing | Implemented / tested | Authored planar maps, deterministic Dijkstra, known pre-departure closure detours, 3 route fixtures × 2 plants × 3 seeds |
| Minimum-clearance regression conditions | Implemented / tested | Fixed swept-circle fixture floors; no real-driving clearance specification |
| Live closures / stopped route handover | Implemented / tested | Common-prefix policy, three healthy stopped estimates, no-route hold/reopening, 4 m/s fixtures with conservative curvature limits and a 6 m/s fixture without an optional curvature cap |
| Avoidance continuity | Implemented / tested | Observed centerline occupancy discourages premature center return; seeded 6 m/s detour regression, swept feasibility remains mandatory |
| Map-update fault handling and replay | Implemented / tested | Revision/stamp checks, latched malformed-update braking, full map-search and handover recomputation when configured |
| Lane topology / traffic-rule routing / continuous moving handover | Planned | No map importer, lane-change graph, turn penalties or intersection rules |
| Traffic lights / signs / right of way / parking | Planned | No traffic-rule claims |
| General dynamically feasible planning / MPC | Planned | Current lattice + pure pursuit baseline |
| CARLA bridge | Planned; not implemented or validated | No CARLA server/assets installed |
| ROS 2 bridge | Planned; optional | No ROS dependency in algorithms |
| Real vehicle / CAN actuation | Out of initial scope | Simulator commands only |
| Production safety / real-time performance | Not established | No certification or throughput benchmark |

There are no empty crates representing the planned capabilities. Future components should land with working behavior, datasets or simulator fixtures, and acceptance checks.
