# Capability matrix

“Implemented” means executable code exists and is exercised by the local tests/demo described in validation. It does not mean suitability for real vehicles.

| Capability | Status | Boundary |
|---|---|---|
| Closed-loop perception → prediction → planning → control | Implemented / tested | Reference and optional CPU RNE plants |
| Noisy GNSS + speed / gyro EKF | Implemented / tested | Full x/y innovation gate, Joseph covariance updates, observed/accepted diagnostics; hold after two new rejected innovations until acceptance; configured initial heading, no bias state |
| Optional local scan-to-fixed-map EKF corrections | Implemented / independently checked | Synchronous XY LiDAR, bounded local SE(2) registration/ambiguity checks; five-second GNSS-denied reference/native driving; ten-second maximum since genuine fix, no global localization/SLAM/6DOF or real-motion dataset pose accuracy |
| Measured-data terrain baseline | Implemented / failed generalization | Frozen PMF on 15 genuine airborne clouds: held-out F1 0.2592 and six zero-F1 sites; licensed/versioned acquisition, no automotive road-semantics claim |
| Measured XYZ components and AABBs | Implemented / tested experimentally | Bounded 3D Euclidean geometry, no semantic labels/hidden-object reconstruction; optional pipeline maps observed AABBs to planar envelopes; sparse native acquisition initially misses actor clusters and blocks on residual ground |
| Unlabeled LiDAR and occlusion | Implemented / tested | 720 rays, 45 m range, circular targets and opt-in native cuboids |
| Clustering / circle fitting / alpha-beta tracking | Implemented / tested | No semantic classes; nearest-neighbor association |
| Acquisition-time LiDAR reprojection | Implemented / tested | At most 0.35 s / 64 EKF estimates; world detections and map rays use historical pose; no retroactive GNSS smoothing, delayed odometry/GNSS fusion or covariance propagation |
| Current-time motion forecasts | Implemented / tested | Acquired tracks retain stamps; CV / observed braking advances to the control clock, consuming acquisition age from the one-second braking budget; unchanged low-speed deadband, no uncertainty growth or actor intent |
| Simulator LiDAR timing / explicit transient failure | Implemented / tested | Observation thinning, immutable acquisition stamps and queued delivery; failures flush queued scans; simulated time, no resource-latency benchmark |
| Log-odds occupancy map | Implemented / tested | Exported diagnostic; not a planner input; no SLAM |
| Static obstacle avoidance | Implemented / tested | Wide supplied road, three candidate offsets |
| Moving lead / crossing object sensing | Implemented / tested in mission | CV / observed-braking ego forecast; optional simulator actors react to scalar proximity observations |
| Reactive traffic following / stopped lead / queue | Implemented / tested | 30 seeded positive runs; ideal route proximity sensing and bounded 1D actors, no steering, priority or interaction-aware ego forecast |
| Traffic-pair collision and endpoint acceptance | Implemented / tested | Reactive actor pairs swept independently; negative collisions/overruns fail; scheduled-only pairs retain earlier criteria |
| Narrow-corridor follower goal deadline | Authored regression repaired | Unchanged 65 s deadline / 8 s residence passes in both plants across 3 seeds; repeated conservative emergency stops remain |
| Observed-braking prediction | Implemented / tested | Sustained sensor-derived track deceleration, bounded one-second persistence then coasting; no intentions, multimodal forecast or guaranteed future braking |
| Short-range follower response | Known physical failure | Two RNE 5 m sensing cases violate the unchanged 1 m clearance floor; separately rejected |
| Blocked road stop | Implemented / tested | Narrow-road scenario |
| GNSS spike / burst / persistent-bias response | Implemented / tested | 42 physical fault runs with bounded error, accepted-age braking and recovery; scheduled traffic and post-arrival hold regressions |
| Terminal stopping-place preference / physical residence | Implemented / tested | Route-relative observed forward traffic, feasible lateral candidates; 16-second truth hold in two plants/three seeds, no general parking or escape planner |
| LiDAR / GNSS dropout braking | Implemented / tested | Timestamp freshness and numeric checks |
| Swept collision and road boundary evaluation | Implemented / tested | Circular bodies and planar route corridor |
| 3D visualization of recorded RNE driving | Implemented / tested | Blender Cycles CPU replay, pose/actor audit, original suburban assets, four display vehicle types, a verified three-actor queue and editable scene snapshots; planar physics, no sensor-camera feed |
| Seeded repeatability and telemetry GIF | Implemented / tested | Same build/platform; Python optional |
| Sensor-only shared pipeline | Implemented / tested | Configured route/spawn; no operational truth inputs |
| Versioned sensor-log replay | Implemented / tested | Full recomputation; exact comparison on this build/platform; no physical acceptance inference |
| RNE kinematic/dynamic closed loop | Implemented / tested | CPU-only native plant, planar LiDAR in 3D query scene, friction/steering-lag dynamic model |
| RNE raycast acquisition failure | Implemented / tested | Explicit error causes braking; healthy empty scan is distinct |
| Opt-in native static 3D cuboids | Implemented / tested | Actual yaw-rotated Rapier query geometry; default 0.6 m operational scan, opt-in multi-height or inclined XYZ modes; no contact-response plant |
| Native scene capsule clearance | Implemented / independently checked | Recorded 200 Hz native positions and conservative speed-bound guard; fixed 1 m floor; low blind slabs rejected; separate simulator-only evidence |
| Operational multi-height LiDAR projection | Implemented / tested | Explicit mode; bounded synchronized body-XY planes, calibrated vehicle-height selection, 5 cm duplicate cells and historical EKF pose; sparse coverage, planar detection/tracking |
| Multi-height fault recovery and replay | Implemented / tested | Atomic timing/failure transport; malformed or partial bundles hold braking until a new complete post-fault acquisition; raw measured planes and calibration in sensor-only logs |
| Native inclined 3D LiDAR acquisition | Implemented / tested | Opt-in 720 × 16 actual Rapier beams, ±15° elevation, instantaneous measured XYZ and firing ordinal; yaw-only flat-road mounting; finite vertical coverage |
| Physical road query surfaces / measured ground removal | Implemented / tested | Separate opt-in 180 × 16 mode; bounded measured plane fitting, global angular and local residual support, confidence braking; broad near-flat physical support fixtures, no general terrain classification |
| Research rectangular body / native overlap witnesses | Implemented / tested | Explicit 4.2 × 1.8 × 1.5 m upright body, conservative swept translation/rotation and fixed 1 m floor; actual force-free Rapier sensor overlaps; circumscribed planar driver envelope, no contact response or measured real-vehicle calibration |
| Validated XYZ projection and replay | Implemented / tested | Bounded calibration, unique beam ordinals, range/direction validation before height gating and 5 cm XY deduplication; atomic timing and post-fault latch; downstream perception/map/planning remain planar |
| Initially occluded / late lateral crossing fixtures | Implemented / tested | Reference/RNE across 3 seeds; circular scheduled actors, no semantics |
| Low-friction avoidance and stopping | Implemented / tested | RNE dynamic mu=0.2, 0.15 s steering lag, separate longitudinal/lateral limits |
| Calibrated braking / curvature speed limits | Implemented / tested | Fixed known limits; sampled curvature; no combined-friction optimization |
| Continuous candidate collision checks | Implemented / tested | Synchronized circular sweeps; accelerated-segment chord bound, observed transverse-motion reserve and retiming revalidation; empirical margin, no certified error bound |
| Reachable longitudinal speed profiles | Implemented / tested | Forward/backward acceleration bounds, local curvature caps, finite arrival times; no joint tire-force or jerk optimization |
| Smooth route interpolation / heading join | Implemented / tested | C2 centerline segments and estimated-heading correction; sampled corridor containment, no joint geometry/control optimization |
| Local sparse-road corner geometry | Implemented / bounded evidence | Original-corridor local fillets; six reference and six native dynamic goals at 2 m/s; explicit chassis reference and noisy-odometry course; general bends/obstacles and mapped-rule coordinates remain unsupported |
| Low-friction tracking regression gate | Implemented / tested | Fixed RNE calibration, 3 seeds, at most 20 emergency ticks in the specified fixture; no universal bound |
| Acceleration feedforward control | Implemented / tested | First-segment acceleration with PI feedback; steering still pure pursuit |
| Stop and wait on blocked candidates | Implemented / tested | Feasible stopping profile and eight-second forecast hold, release on a clear candidate; no semantic priority rules |
| Multiple blocked alternatives / opposing crossings | Implemented / tested | Scheduled circular actors; independent physical acceptance and full replay |
| Linux build/test | Verified locally | Rust 1.90.0; optional RNE uses 1.95.0 |
| macOS / Windows | Remote checks observed on recorded revisions | Results and runner failures in [validation](validation.md); no local hosts |
| Offline learned image detection | Implemented / bounded real-image diagnostic | Optional locked tract-onnx CPU YOLOX; six urban COCO images, independent TP/FP/FN scoring, repeat inference and corruption checks; evaluator-held-out recall 0.56 with upstream model-validation overlap possible; no metric depth, sensor fusion or control integration; [evidence](road-camera-evaluation.md) |
| Adaptive terrain classifier | Implemented / failed new-environment acceptance | Original six calibration F1 0.8215 and nine viewed regression F1 0.8133; fresh Autzen reference F1 0.3571, confidence false; optional fail-closed pipeline mode, no automotive accuracy claim |
| Local 6DOF point registration / recorded RGB-D evaluator | Implemented / experimental | Bounded Horn/ICP with ambiguity checks; recorded natural-motion poses used only by evaluator; documented temporal protocols and rejected pairs, no global SLAM or vehicle fusion |
| Recorded-depth odometry / fixed first-cloud localization | Implemented / partial temporal protocol | Eleven accurate consecutive fresh pair fits and composed poses in one indoor room; 9/11 fixed-map fits accepted, two ambiguity rejections retained; missing-reference regression and provisional uncertainty audited; full pair-plus-map protocol fails; [evidence](recorded-motion.md) |
| Measured-cloud keyframe localization | Implemented / failed longer temporal protocol | Bounded sensor-only reference replacement, root-frame chaining and latched accepted-pose expiry; viewed short intervals pass 11/11, first frozen 35-update trial accepts 33 and scores only 13 root-accurate; accumulated drift and two ambiguity rejections retained; no vehicle fusion, SLAM or root covariance; [evidence](recorded-keyframes.md) |
| Radar / camera driving fusion / 3D object semantics | Planned | Offline inference and native XYZ acquisition do not establish metric camera observations or semantic 3D driving |
| Learned prediction and training | Planned | No model/data/runtime packaged |
| 3D mapping / SLAM / global localization | Planned | Optional local XY fixed-map matching is implemented separately; no truth-as-localization substitution |
| Directed road graph / shortest-distance routing | Implemented / tested | Authored planar maps, deterministic Dijkstra, known pre-departure closure detours, 3 route fixtures × 2 plants × 3 seeds |
| Bounded OpenStreetMap import | Implemented / tested | Versioned real ODbL extract, local WGS84/ENU, shared junctions and directed motor-road graph; explicit simulation widths, no lanes/elevation/turn-restriction interpretation |
| Minimum-clearance regression conditions | Implemented / tested | Fixed swept-circle fixture floors; no real-driving clearance specification |
| Live closures / stopped route handover | Implemented / tested | Common-prefix policy, three healthy stopped estimates, no-route hold/reopening, 4 m/s fixtures with conservative curvature limits and a 6 m/s fixture without an optional curvature cap |
| Avoidance continuity | Implemented / tested | Observed centerline occupancy discourages premature center return; seeded 6 m/s detour regression, swept feasibility remains mandatory |
| Map-update fault handling and replay | Implemented / tested | Revision/stamp checks, latched malformed-update braking, full map-search and handover recomputation when configured |
| Bounded legal-turn routing | Implemented / tested | Directed incoming-edge-aware search; unconditional motorcar node-via OSM no/only restrictions, closures and full restricted-map replay; 24 reference/native episodes, 24 corrupt-replay rejections; [limits](turn-restrictions.md) |
| Lane topology / general traffic-rule routing / continuous moving handover | Planned | No lane-change graph, turn penalties, conditional or via-way restrictions; restricted-map navigation and fixed-route signal/yield control are separate modes |
| Mapped traffic signals / stop-line holds | Implemented / tested | 36 dedicated physical/replay runs plus 6 stop-sign combinations in two plants; 5 Hz infrastructure snapshots, red/yellow/unknown stops, freshness and green release; no camera signal recognition |
| Mapped stop signs | Implemented / tested | 30 physical/replay episodes; continuous healthy two-second hold, brake retention, multiple signs, mixed signal/obstacle constraints and GNSS reset/recovery; no camera sign detector |
| Mapped priority crossing yield | Implemented / tested | 78 physical/replay episodes in two plants, varied authored widths/speeds/two zones and 24 timing episodes; fixed-route rectangles, eight-second current-time forecasts, healthy distinct-scan dwell and independent sampled separation |
| Uncommitted intersection approach envelope | Implemented / tested | Applies to `Proceeding`; `Waiting` retains its stopping prefix. Half calibrated braking authority, 0.25 s response allowance, nominal 2 m estimated reserve and 0.5 m/s creep floor; candidate cruise cap, no arbitrary-late-threat reserve guarantee |
| Late second-crossing waiting margin | Authored regression repaired | Original actors/map/1 m gates unchanged; original and 100 ms-delayed fixtures pass twelve episodes in both plants/three seeds, included in the verified 276-run sweep |
| General right of way / intersections / parking | Planned | Other actors do not obey controls; no all-way stop ordering, lane negotiation or parking behavior |
| General dynamically feasible planning / MPC | Planned | Current lattice + pure pursuit baseline |
| CARLA bridge | Planned; not implemented or validated | No CARLA server/assets installed |
| ROS 2 bridge | Planned; optional | No ROS dependency in algorithms |
| Real vehicle / CAN actuation | Out of initial scope | Simulator commands only |
| Production safety / real-time performance | Not established | No certification or throughput benchmark |

There are no empty crates representing the planned capabilities. Future components should land with working behavior, datasets or simulator fixtures, and acceptance checks.
