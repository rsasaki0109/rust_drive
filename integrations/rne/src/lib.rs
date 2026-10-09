//! CPU-only RNE plant and Rapier LiDAR adapter for the shared RustDriving pipeline.
pub mod body;
mod capsule_refinement;
#[cfg(test)]
mod grazing;
pub mod scene;
use body::{BodyCalibration, BodyCapture};
use rne_core::{KeyedRandom, SimDuration};
use rne_ecs::{Entity, World, spawn_named};
use rne_math::{Quat, Seconds, Vec3, yaw_rad};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsError, PhysicsWorldDesc, PhysicsWorldId,
    RaycastHit, RaycastQuery, RigidBody, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_robot::{AckermannDrive, VehicleDynamics, ackermann_kinematics, vehicle_dynamics};
use rne_sensor::{LidarRaycaster, LidarSpec, SensorNoiseKey, sample_lidar_checked};
use rne_world::Transform3;
use rustdriving_core::{
    ControlCommand, EgoState, Gnss, Lidar3dReturn, Lidar3dScan, LidarPlane, LidarScan,
    MultiHeightLidarScan, Odometry, Pose, Vec2, Vec3 as BodyPoint3,
};
use rustdriving_pipeline::{
    GroundConfig, Lidar3dConfig, MotionLimits, MultiHeightLidarConfig, Perception3dConfig,
    PipelineConfig, SensorFrame,
};
use rustdriving_sim::traffic::{TrafficTelemetry, TrafficWorld};
use rustdriving_sim::{
    Run, Scenario, SimulationBackend, WorldObject, pipeline_config, simulate_with_backend,
};
use scene::{MotionSample, Scene, SceneCapture, pose_json, scan_channel};
use serde_json::json;
use std::sync::{Arc, Mutex};

/// RNE plant selection. Both use native RNE systems, never RustDriving's reference integrator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Plant {
    Kinematic,
    Dynamic,
}
#[derive(Clone, Copy, PartialEq)]
enum GroundMode {
    Disabled,
    Segmentation,
    VehicleBody,
    TerrainObjects,
    AdaptiveTerrainObjects,
}
/// ENU planar coordinate to RNE Y-up world; north corresponds to negative Z.
pub fn to_rne(point: Vec2) -> Vec3 {
    Vec3::new(point.x, 0.6, -point.y)
}
/// World/sensor-local Y-up coordinates to planar ENU/body coordinates.
pub fn from_rne(point: Vec3) -> Vec2 {
    Vec2::new(point.x, -point.z)
}
struct EgoFilteredRaycaster<'a> {
    backend: &'a RapierBackend,
    ego: Entity,
    precise_geometry: Option<&'a [capsule_refinement::PhysicalCapsule]>,
}
impl LidarRaycaster for EgoFilteredRaycaster<'_> {
    fn lidar_raycast(
        &self,
        world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        let mut hits = self.backend.raycast(world, query)?;
        hits.retain(|h| h.entity != self.ego);
        if let Some(capsules) = self.precise_geometry {
            hits = capsule_refinement::merge_hits(capsules, query, hits)?;
        }
        Ok(hits)
    }
}
/// Owns RNE ECS/plant/physics state. Ground truth exists only on this simulator side.
pub struct RneBackend {
    scenario: Scenario,
    config: PipelineConfig,
    world: World,
    ego: Entity,
    objects: Vec<Entity>,
    physics: RapierBackend,
    physics_world: PhysicsWorldId,
    noise: KeyedRandom,
    seed: u64,
    plant: Plant,
    traffic: TrafficWorld,
    scene: Option<Arc<Mutex<SceneCapture>>>,
    multi_height: bool,
    lidar_3d: bool,
    vehicle_body: bool,
    precise_capsule_rays: bool,
    scene_obstacles: Vec<(Entity, String)>,
    /// Test/diagnostic injection; a checked raycast against an unknown world must brake.
    pub fail_lidar_from: Option<f64>,
    /// Test/diagnostic injection for an auxiliary operational plane only.
    pub fail_aux_lidar_from: Option<f64>,
    /// Test/diagnostic injection affecting the inclined operational cloud only.
    pub fail_lidar_3d_from: Option<f64>,
}
impl RneBackend {
    /// Creates a headless native vehicle with a CPU Rapier query scene.
    pub fn new(scenario: Scenario, seed: u64, plant: Plant) -> Result<Self, String> {
        Self::create(
            scenario,
            seed,
            plant,
            None,
            false,
            false,
            GroundMode::Disabled,
        )
    }
    /// Adds simulator-only native sensing geometry; operational calibration is unchanged.
    pub fn new_with_scene(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Scene,
    ) -> Result<Self, String> {
        if scenario.duration > 120.0 {
            return Err("native scene evidence requires duration <=120 seconds".into());
        }
        Self::create(
            scenario,
            seed,
            plant,
            Some(scene),
            false,
            false,
            GroundMode::Disabled,
        )
    }
    /// Enables synchronized measured height planes without exposing scene labels.
    pub fn new_with_scene_multi_height(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Scene,
    ) -> Result<Self, String> {
        if scenario.duration > 120.0 {
            return Err("native scene evidence requires duration <=120 seconds".into());
        }
        Self::create(
            scenario,
            seed,
            plant,
            Some(scene),
            true,
            false,
            GroundMode::Disabled,
        )
    }
    /// Enables an actual inclined native LiDAR cloud, preserving raw return ordinals.
    pub fn new_with_scene_lidar_3d(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Scene,
    ) -> Result<Self, String> {
        if scenario.duration > 120.0 {
            return Err("native scene evidence requires duration <=120 seconds".into());
        }
        Self::create(
            scenario,
            seed,
            plant,
            Some(scene),
            false,
            true,
            GroundMode::Disabled,
        )
    }
    /// Enables measured ground fitting over native road geometry without point-role labels.
    pub fn new_with_scene_ground(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Scene,
    ) -> Result<Self, String> {
        if scenario.duration > 120.0 {
            return Err("native scene evidence requires duration <=120 seconds".into());
        }
        Self::create(
            scenario,
            seed,
            plant,
            Some(scene),
            false,
            true,
            GroundMode::Segmentation,
        )
    }
    /// Opt-in authored research car envelope with native force-free sensor overlap evidence.
    pub fn new_with_scene_ground_body(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Scene,
    ) -> Result<Self, String> {
        if scenario.duration > 120.0 {
            return Err("native scene evidence requires duration <=120 seconds".into());
        }
        Self::create(
            scenario,
            seed,
            plant,
            Some(scene),
            false,
            true,
            GroundMode::VehicleBody,
        )
    }
    /// Measured terrain/object processing over an actual native inclined XYZ cloud.
    pub fn new_with_scene_terrain_objects(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Scene,
    ) -> Result<Self, String> {
        if scenario.duration > 120.0 {
            return Err("native scene evidence requires duration <=120 seconds".into());
        }
        Self::create(
            scenario,
            seed,
            plant,
            Some(scene),
            false,
            true,
            GroundMode::TerrainObjects,
        )
    }
    /// Optional density-adaptive terrain profile; no semantic simulator labels.
    pub fn new_with_scene_adaptive_terrain_objects(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Scene,
    ) -> Result<Self, String> {
        if scenario.duration > 120.0 {
            return Err("native scene evidence requires duration <=120 seconds".into());
        }
        Self::create(
            scenario,
            seed,
            plant,
            Some(scene),
            false,
            true,
            GroundMode::AdaptiveTerrainObjects,
        )
    }
    fn create(
        scenario: Scenario,
        seed: u64,
        plant: Plant,
        scene: Option<Scene>,
        multi_height: bool,
        lidar_3d: bool,
        ground_mode: GroundMode,
    ) -> Result<Self, String> {
        scenario.validate()?;
        if scenario.dynamics.is_some() && plant != Plant::Dynamic {
            return Err("friction/lag calibration requires --plant dynamic".into());
        }
        let mut config = pipeline_config(&scenario);
        // Dynamic RNE reports a chassis/COM pose, not a rear-axle pose. This
        // opt-in low-speed model uses declared plant geometry and noisy odometry.
        if scenario.local_route_geometry && plant == Plant::Dynamic {
            config.rear_axle_offset_m = Some(VehicleDynamics::default().rear_axle_m);
        }
        let ground_segmentation = ground_mode != GroundMode::Disabled;
        let vehicle_body = ground_mode == GroundMode::VehicleBody;
        let body_calibration = BodyCalibration::default();
        if vehicle_body {
            if config.route.half_width < 3.0 {
                return Err("authored vehicle-body mode requires road half width >=3 m".into());
            }
            config.vehicle.radius = body_calibration.corner_radius_m();
        }
        if multi_height {
            config.multi_height_lidar = Some(MultiHeightLidarConfig {
                heights_m: vec![0.6, 0.15, 3.7],
                collision_bottom_m: if vehicle_body {
                    body_calibration.bottom_m - 1.0
                } else {
                    0.1 - config.vehicle.radius
                },
                collision_top_m: if vehicle_body {
                    body_calibration.bottom_m + body_calibration.height_m + 1.0
                } else {
                    1.1 + config.vehicle.radius
                },
            });
        }
        if lidar_3d {
            config.lidar3d = Some(Lidar3dConfig {
                azimuth_columns: if ground_segmentation { 180 } else { 720 },
                elevation_rings: 16,
                min_elevation_rad: -std::f64::consts::PI / 12.0,
                max_elevation_rad: std::f64::consts::PI / 12.0,
                mount_height_m: 0.6,
                min_range_m: 0.2,
                max_range_m: 45.0,
                collision_bottom_m: if vehicle_body {
                    body_calibration.bottom_m - 1.0
                } else {
                    0.1 - config.vehicle.radius
                },
                collision_top_m: if vehicle_body {
                    body_calibration.bottom_m + body_calibration.height_m + 1.0
                } else {
                    1.1 + config.vehicle.radius
                },
                ground: if ground_segmentation {
                    Some(GroundConfig {
                        reference_height_m: 0.0,
                        max_slope: 0.05,
                        max_height_offset_m: 0.03,
                        residual_threshold_m: 0.02,
                        fit_radius_m: 8.0,
                        min_inliers: 200,
                        min_sector_inliers: 20,
                        min_cell_inliers: 6,
                    })
                } else {
                    None
                },
            });
        }
        if matches!(
            ground_mode,
            GroundMode::TerrainObjects | GroundMode::AdaptiveTerrainObjects
        ) {
            let acquisition = config.lidar3d.as_mut().unwrap();
            acquisition.ground = None;
            // Existing dense inclined acquisition and authored research height
            // envelope. This sensing calibration does not enable body physics.
            acquisition.azimuth_columns = 720;
            acquisition.collision_bottom_m = body_calibration.bottom_m;
            acquisition.collision_top_m = body_calibration.bottom_m + body_calibration.height_m;
            config.perception3d = Some(if ground_mode == GroundMode::AdaptiveTerrainObjects {
                Perception3dConfig::adaptive()
            } else {
                Perception3dConfig::default()
            });
        }
        config.validate()?;
        if plant == Plant::Dynamic {
            config.cruise_speed = scenario.cruise_speed.unwrap_or(6.0);
            if let Some(d) = scenario.dynamics {
                let conservative_acceleration = (0.6 * d.friction_coefficient * 9.81).min(2.5);
                let requested = config.motion_limits.unwrap_or(MotionLimits {
                    max_acceleration_m_s2: 2.0,
                    max_deceleration_m_s2: 2.5,
                    max_lateral_acceleration_m_s2: 2.5,
                });
                config.motion_limits = Some(MotionLimits {
                    max_acceleration_m_s2: conservative_acceleration
                        .min(requested.max_acceleration_m_s2),
                    max_deceleration_m_s2: conservative_acceleration
                        .min(requested.max_deceleration_m_s2),
                    max_lateral_acceleration_m_s2: conservative_acceleration
                        .min(requested.max_lateral_acceleration_m_s2),
                });
            }
        }
        let mut world = World::new();
        let ego = spawn_named(&mut world, "rustdriving_ego");
        let pose = config.initial_pose;
        let ego_collider = if vehicle_body {
            let mut collider = body_calibration.collider();
            collider.sensor = true;
            collider
        } else {
            Collider {
                shape: ColliderShape::Capsule {
                    half_height_m: 0.5,
                    radius_m: config.vehicle.radius,
                },
                ..Collider::default()
            }
        };
        world.entity_mut(ego).insert((
            Transform3::from_translation_rotation(
                to_rne(pose.position),
                Quat::from_rotation_y(pose.yaw),
            ),
            AckermannDrive {
                wheelbase_m: config.vehicle.wheelbase,
                max_speed_m_s: 12.0,
                max_steering_rad: config.vehicle.max_steer,
                max_acceleration_m_s2: 2.0,
                max_deceleration_m_s2: 6.0,
                max_steering_rate_rad_s: 0.7,
                ..AckermannDrive::default()
            },
            // Query geometry follows the separately integrated native RNE plant.
            // Rapier supplies sensing geometry; it does not integrate the vehicle a second time.
            RigidBody {
                body_type: if vehicle_body {
                    RigidBodyType::Dynamic
                } else {
                    RigidBodyType::Fixed
                },
                ..RigidBody::default()
            },
            ego_collider,
        ));
        if plant == Plant::Dynamic {
            world.entity_mut(ego).insert(VehicleDynamics {
                friction_coefficient: scenario.dynamics.map_or(0.9, |d| d.friction_coefficient),
                steering_lag_s: scenario.dynamics.map_or(0.08, |d| d.steering_lag_s),
                ..VehicleDynamics::default()
            });
        }
        let objects = scenario
            .objects
            .iter()
            .enumerate()
            .map(|(i, _)| spawn_named(&mut world, format!("obstacle_{i}")))
            .collect();
        // Fixed boxes take part in the same native Rapier queries as dynamic
        // actor capsules, while their labels remain entirely on the simulator side.
        let mut scene_obstacles = Vec::new();
        if let Some(scene) = &scene {
            for cuboid in scene.static_cuboids.iter().chain(&scene.ground_cuboids) {
                let entity = spawn_named(&mut world, format!("scene_{}", cuboid.id));
                if scene.static_cuboids.iter().any(|b| b.id == cuboid.id) {
                    scene_obstacles.push((entity, cuboid.id.clone()));
                }
                world.entity_mut(entity).insert((
                    Transform3::from_translation_rotation(
                        Vec3::new(cuboid.center_m[0], cuboid.center_m[2], -cuboid.center_m[1]),
                        Quat::from_rotation_y(cuboid.yaw_rad),
                    ),
                    RigidBody {
                        body_type: RigidBodyType::Fixed,
                        ..RigidBody::default()
                    },
                    Collider {
                        shape: ColliderShape::Cuboid {
                            half_extents_m: Vec3::new(
                                cuboid.half_extents_m[0],
                                cuboid.half_extents_m[2],
                                cuboid.half_extents_m[1],
                            ),
                        },
                        ..Collider::default()
                    },
                ));
            }
        }
        let scene = scene.map(|scene| {
            let mut capture = SceneCapture::new(scene, pose.position);
            if vehicle_body {
                capture.body = Some(
                    BodyCapture::new(body_calibration, pose)
                        .expect("validated body calibration and initial pose"),
                );
            }
            Arc::new(Mutex::new(capture))
        });
        let mut physics = RapierBackend::new();
        let physics_world = physics
            .create_world(if vehicle_body {
                PhysicsWorldDesc {
                    gravity_m_s2: Vec3::ZERO,
                    ..PhysicsWorldDesc::default()
                }
            } else {
                PhysicsWorldDesc::default()
            })
            .map_err(|e| e.to_string())?;
        let traffic = TrafficWorld::new(scenario.clone(), config.route.clone());
        let mut backend = Self {
            traffic,
            scenario,
            config,
            world,
            ego,
            objects,
            physics,
            physics_world,
            noise: KeyedRandom::new(seed, 0x5255535444524956),
            seed,
            plant,
            scene,
            multi_height,
            lidar_3d,
            vehicle_body,
            precise_capsule_rays: false,
            scene_obstacles,
            fail_lidar_from: None,
            fail_aux_lidar_from: None,
            fail_lidar_3d_from: None,
        };
        if vehicle_body {
            backend.capture_body_witness(0.0, SimDuration::from_seconds(Seconds::new(0.005)))?;
        }
        Ok(backend)
    }
    /// The exact stack configuration used by this plant, included in its replay header.
    pub fn config(&self) -> PipelineConfig {
        self.config.clone()
    }
    fn capture_body_witness(&mut self, time: f64, dt: SimDuration) -> Result<(), String> {
        self.sync_scene(time)?;
        let witness = body::sensor_witness(
            &mut self.physics,
            self.physics_world,
            &mut self.world,
            self.ego,
            &self.scene_obstacles,
            time,
            dt,
        )?;
        self.scene
            .as_ref()
            .unwrap()
            .lock()
            .map_err(|_| "native scene capture poisoned")?
            .body
            .as_mut()
            .unwrap()
            .witnesses
            .push(witness);
        Ok(())
    }
    fn sync_scene(&mut self, time: f64) -> Result<(), String> {
        for object in self.objects(time) {
            self.world
                .entity_mut(self.objects[object.id as usize])
                .insert((
                    Transform3::from_translation_rotation(to_rne(object.position), Quat::IDENTITY),
                    RigidBody {
                        body_type: RigidBodyType::Fixed,
                        ..RigidBody::default()
                    },
                    Collider {
                        shape: ColliderShape::Capsule {
                            half_height_m: 0.5,
                            radius_m: object.radius,
                        },
                        ..Collider::default()
                    },
                ));
        }
        self.physics
            .sync_from_ecs(&mut self.world, self.physics_world)
            .map_err(|e| e.to_string())
    }
    fn noisy(&self, tick: usize, slot: u64, amplitude: f64) -> f64 {
        self.noise
            .sample_f64(tick as u64, 0, slot, -amplitude, amplitude)
    }
}
impl SimulationBackend for RneBackend {
    fn state(&self) -> EgoState {
        let tf = self.world.get::<Transform3>(self.ego).unwrap();
        let drive = self.world.get::<AckermannDrive>(self.ego).unwrap();
        EgoState {
            pose: Pose {
                position: from_rne(tf.translation),
                yaw: yaw_rad(tf.rotation),
            },
            speed: drive.speed_m_s,
        }
    }
    fn objects(&self, time: f64) -> Vec<WorldObject> {
        self.traffic.objects(time)
    }
    fn traffic(&self) -> Vec<TrafficTelemetry> {
        self.traffic.telemetry()
    }
    fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String> {
        let truth = self.state();
        if let Some(scene) = &self.scene {
            let mut scene = scene.lock().map_err(|_| "native scene capture poisoned")?;
            scene.clock = time;
            scene
                .observations
                .push(json!({"time":time,"pose":pose_json(truth.pose)}));
        }
        let body = self.world.get::<RigidBody>(self.ego).unwrap();
        let odometry = Some(Odometry {
            stamp: time,
            speed: truth.speed + self.noisy(tick, 0, 0.015),
            yaw_rate: body.angular_velocity_rad_s.y + self.noisy(tick, 1, 0.001),
        });
        let gnss = if tick.is_multiple_of(4) && self.scenario.gnss_available(time) {
            Some(Gnss {
                stamp: time,
                position: truth.pose.position.plus(Vec2::new(
                    self.noisy(tick, 2, 0.14),
                    self.noisy(tick, 3, 0.14),
                )),
                variance: 0.02,
            })
        } else {
            None
        };
        let mut lidar = None;
        let mut multi_height_lidar = None;
        let mut lidar_3d = None;
        let mut lidar_failed = false;
        if tick.is_multiple_of(2) && self.scenario.lidar_dropout.is_none_or(|t| time < t) {
            self.sync_scene(time)?;
            let mount = *self.world.get::<Transform3>(self.ego).unwrap();
            let spec = LidarSpec {
                ray_count: 720,
                min_range_m: 0.2,
                max_range_m: 45.0,
                height_offset_m: 0.0,
                range_noise_stddev_m: 0.008,
                ..LidarSpec::default()
            };
            let capsule_snapshot = if self.precise_capsule_rays {
                Some(
                    capsule_refinement::snapshot(&self.world, self.ego)
                        .map_err(|e| e.to_string())?,
                )
            } else {
                None
            };
            let raycaster = EgoFilteredRaycaster {
                backend: &self.physics,
                ego: self.ego,
                precise_geometry: capsule_snapshot.as_deref(),
            };
            let world = if self.fail_lidar_from.is_some_and(|t| time >= t) {
                PhysicsWorldId(u32::MAX)
            } else {
                self.physics_world
            };
            match sample_lidar_checked(
                &raycaster,
                world,
                &mount,
                &spec,
                SensorNoiseKey::new(self.seed, 1, 1, tick as u64),
            ) {
                Ok(cloud) => {
                    let inverse = mount.rotation.conjugate();
                    let measured_plane = |points: &[Vec3], height_m| LidarPlane {
                        height_m,
                        points: points
                            .iter()
                            .map(|p| from_rne(inverse * (*p - mount.translation)))
                            .collect(),
                    };
                    let mut planes = if self.multi_height {
                        vec![measured_plane(&cloud.points_m, 0.6)]
                    } else {
                        vec![]
                    };
                    if let Some(scene) = &self.scene {
                        let mut channels = vec![scan_channel(
                            &cloud.points_m,
                            &cloud.ray_indices,
                            mount.translation,
                            0.6,
                        )?];
                        let mut failed_heights = Vec::new();
                        for (sensor, height) in [(2, 0.15), (3, 3.7)] {
                            let mut diagnostic_mount = mount;
                            diagnostic_mount.translation.y = height;
                            let auxiliary_world = if self.multi_height
                                && self.fail_aux_lidar_from.is_some_and(|t| time >= t)
                            {
                                PhysicsWorldId(u32::MAX)
                            } else {
                                world
                            };
                            let diagnostic = sample_lidar_checked(
                                &raycaster,
                                auxiliary_world,
                                &diagnostic_mount,
                                &spec,
                                SensorNoiseKey::new(self.seed, 1, sensor, tick as u64),
                            );
                            let diagnostic = match diagnostic {
                                Ok(cloud) => cloud,
                                Err(_) if self.multi_height || self.lidar_3d => {
                                    lidar_failed = true;
                                    failed_heights.push(height);
                                    continue;
                                }
                                Err(error) => {
                                    return Err(format!(
                                        "native scene diagnostic acquisition: {error:?}"
                                    ));
                                }
                            };
                            channels.push(scan_channel(
                                &diagnostic.points_m,
                                &diagnostic.ray_indices,
                                diagnostic_mount.translation,
                                height,
                            )?);
                            if self.multi_height {
                                planes.push(measured_plane(&diagnostic.points_m, height));
                            }
                        }
                        let mut acquisition = json!({"time":time,"pose":pose_json(truth.pose),"objects":self.objects(time),"channels":channels});
                        if !failed_heights.is_empty() {
                            acquisition["failed_heights_m"] = json!(failed_heights);
                        }
                        scene
                            .lock()
                            .map_err(|_| "native scene capture poisoned")?
                            .acquisitions
                            .push(acquisition);
                    }
                    // RNE emits world-frame points; calibrate into body x-forward/y-left.
                    if self.multi_height {
                        if !lidar_failed {
                            multi_height_lidar = Some(MultiHeightLidarScan {
                                stamp: time,
                                planes,
                            });
                        }
                    } else if !self.lidar_3d {
                        lidar = Some(LidarScan {
                            stamp: time,
                            points: cloud
                                .points_m
                                .into_iter()
                                .map(|p| from_rne(inverse * (p - mount.translation)))
                                .collect(),
                        });
                    }
                }
                Err(_) => lidar_failed = true,
            }
            if self.lidar_3d {
                if let Some(scene) = &self.scene {
                    let mut scene = scene.lock().map_err(|_| "native scene capture poisoned")?;
                    if scene
                        .acquisitions
                        .last()
                        .is_none_or(|a| a["time"].as_f64() != Some(time))
                    {
                        scene.acquisitions.push(json!({"time":time,"pose":pose_json(truth.pose),"objects":self.objects(time),"channels":[],"diagnostic_failed":true}));
                    }
                }
                let spec_3d = LidarSpec {
                    ray_count: u32::try_from(self.config.lidar3d.as_ref().unwrap().azimuth_columns)
                        .map_err(|_| "inclined LiDAR azimuth count exceeds native u32 bounds")?,
                    channel_count: 16,
                    min_elevation_rad: -std::f64::consts::PI / 12.0,
                    max_elevation_rad: std::f64::consts::PI / 12.0,
                    ..spec
                };
                let cloud_world = if self.fail_lidar_3d_from.is_some_and(|t| time >= t) {
                    PhysicsWorldId(u32::MAX)
                } else {
                    world
                };
                match sample_lidar_checked(
                    &raycaster,
                    cloud_world,
                    &mount,
                    &spec_3d,
                    SensorNoiseKey::new(self.seed, 1, 4, tick as u64),
                ) {
                    Ok(cloud) => {
                        let mut ranges: Vec<Option<f64>> =
                            vec![None; spec_3d.ray_count as usize * 16];
                        let mut returns = Vec::with_capacity(cloud.points_m.len());
                        let inverse = mount.rotation.conjugate();
                        for ((&point, &column), &ring) in cloud
                            .points_m
                            .iter()
                            .zip(&cloud.ray_indices)
                            .zip(&cloud.channel_indices)
                        {
                            let ray_index = column as usize * 16 + ring as usize;
                            let range = (point - mount.translation).length();
                            let slot = ranges
                                .get_mut(ray_index)
                                .ok_or("native 3D firing ordinal exceeds calibration")?;
                            if slot.is_some() || !range.is_finite() {
                                return Err(
                                    "native 3D cloud has invalid or duplicate return".into()
                                );
                            }
                            *slot = Some(range);
                            let local = inverse * (point - mount.translation);
                            returns.push(Lidar3dReturn {
                                ray_index,
                                point: BodyPoint3 {
                                    x: local.x,
                                    y: -local.z,
                                    z: 0.6 + local.y,
                                },
                            });
                        }
                        if cloud.points_m.len() != cloud.ray_indices.len()
                            || cloud.points_m.len() != cloud.channel_indices.len()
                        {
                            return Err("native 3D return attributes are not aligned".into());
                        }
                        if let Some(scene) = &self.scene {
                            let mut scene =
                                scene.lock().map_err(|_| "native scene capture poisoned")?;
                            if let Some(acquisition) = scene.acquisitions.last_mut() {
                                acquisition["cloud_3d"] =
                                    json!({"ranges_m":ranges,"returns":returns});
                            }
                        }
                        if !lidar_failed {
                            lidar_3d = Some(Lidar3dScan {
                                stamp: time,
                                returns,
                            });
                        }
                    }
                    Err(_) => {
                        lidar_failed = true;
                        if let Some(scene) = &self.scene {
                            let mut scene =
                                scene.lock().map_err(|_| "native scene capture poisoned")?;
                            if let Some(acquisition) = scene.acquisitions.last_mut() {
                                acquisition["cloud_3d_failed"] = json!(true);
                            }
                        }
                    }
                }
            }
        }
        Ok(SensorFrame {
            navigation_update: None,
            traffic_signal: None,
            time,
            odometry,
            gnss,
            lidar,
            multi_height_lidar,
            lidar3d: lidar_3d,
            lidar_failed,
        })
    }
    fn advance(&mut self, command: ControlCommand, dt: f64) -> Result<(), String> {
        if !command.finite() || !dt.is_finite() || dt <= 0.0 {
            return Err("invalid RNE actuation".into());
        }
        self.traffic
            .advance(self.state(), self.config.vehicle, dt)?;
        // RNE limits lateral tire force but shapes forward speed independently.
        // This adapter separately bounds longitudinal acceleration by mu*g.
        // A combined longitudinal/lateral friction ellipse is not modeled.
        let traction = self
            .world
            .get::<VehicleDynamics>(self.ego)
            .map_or(6.0, |d| d.friction_coefficient * 9.81);
        let acceleration = command
            .acceleration
            .clamp(-6.0_f64.min(traction), 2.0_f64.min(traction));
        let substeps = 10;
        let sub_dt = SimDuration::from_seconds(Seconds::new(dt / substeps as f64));
        {
            let mut drive = self.world.get_mut::<AckermannDrive>(self.ego).unwrap();
            drive.target_speed_m_s = (drive.speed_m_s + acceleration * dt).clamp(0.0, 12.0);
            drive.target_steering_rad = command
                .steering
                .clamp(-drive.max_steering_rad, drive.max_steering_rad);
            // Acceleration command maps to bounded target-speed ramp at this adapter boundary.
            drive.max_acceleration_m_s2 = acceleration.max(0.0);
            drive.max_deceleration_m_s2 = (-acceleration).max(0.0);
        }
        for _ in 0..substeps {
            match self.plant {
                Plant::Kinematic => ackermann_kinematics(&mut self.world, sub_dt),
                Plant::Dynamic => vehicle_dynamics(&mut self.world, sub_dt),
            }
            if let Some(scene) = self.scene.clone() {
                let mut scene = scene.lock().map_err(|_| "native scene capture poisoned")?;
                let pose = self.state().pose;
                let position = pose.position;
                let previous = scene.motion_samples.last().unwrap().position;
                let planar_speed = position.distance(previous) / (dt / substeps as f64);
                // Native forward speed and lateral slip are integrated separately.
                // Validate the total translation bound required by the scene guard.
                if !planar_speed.is_finite() || planar_speed > 12.0 + 1e-8 {
                    return Err(
                        "native scene planar substep exceeds 12 m/s clearance-bound contract"
                            .into(),
                    );
                }
                scene.clock += dt / substeps as f64;
                let time = scene.clock;
                scene.motion_samples.push(MotionSample { time, position });
                if let Some(body) = &mut scene.body {
                    body.record(time, pose)?;
                }
                drop(scene);
                if self.vehicle_body {
                    self.capture_body_witness(time, sub_dt)?;
                }
            }
        }
        Ok(())
    }
}
/// Execute and independently score the shared pipeline against the selected RNE plant.
pub fn run(scenario: Scenario, seed: u64, plant: Plant) -> Result<Run, String> {
    let backend = RneBackend::new(scenario.clone(), seed, plant)?;
    let config = backend.config();
    simulate_with_backend(
        scenario,
        seed,
        backend,
        config,
        match plant {
            Plant::Kinematic => "rne-kinematic-rapier-lidar",
            Plant::Dynamic => "rne-dynamic-rapier-lidar",
        },
    )
}
/// Execute with actual static cuboid sensing and a separate conservative capsule evaluator.
/// Scene geometry and auxiliary scans are never included in the operational replay header.
pub fn run_with_scene(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode(
        scenario,
        seed,
        plant,
        scene,
        false,
        false,
        GroundMode::Disabled,
    )
}
/// Execute the shared height-aware sensing path with simulator-only evidence.
pub fn run_with_scene_multi_height(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode(
        scenario,
        seed,
        plant,
        scene,
        true,
        false,
        GroundMode::Disabled,
    )
}
/// Execute with actual inclined native 3D sensing and simulator-only scene evidence.
pub fn run_with_scene_lidar_3d(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode(
        scenario,
        seed,
        plant,
        scene,
        false,
        true,
        GroundMode::Disabled,
    )
}
/// Execute native road/obstacle sensing with measured-ground segmentation enabled.
pub fn run_with_scene_ground(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode(
        scenario,
        seed,
        plant,
        scene,
        false,
        true,
        GroundMode::Segmentation,
    )
}
/// Execute the authored cuboid envelope, native overlap witness and independent body guard.
pub fn run_with_scene_ground_body(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode(
        scenario,
        seed,
        plant,
        scene,
        false,
        true,
        GroundMode::VehicleBody,
    )
}
/// Opt-in native body mode refining returned physical capsule candidates in
/// f64 and recovering missing physical capsule returns from the synchronized
/// ECS collider snapshot. Other query shapes remain native and unchanged.
pub fn run_with_scene_ground_body_precise(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode_precision(
        scenario,
        seed,
        plant,
        scene,
        false,
        true,
        GroundMode::VehicleBody,
        true,
    )
}
/// Execute measured terrain/object perception without simulator semantic labels.
pub fn run_with_scene_terrain_objects(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode(
        scenario,
        seed,
        plant,
        scene,
        false,
        true,
        GroundMode::TerrainObjects,
    )
}
/// Execute the bounded adaptive terrain profile over genuine native XYZ returns.
pub fn run_with_scene_adaptive_terrain_objects(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode(
        scenario,
        seed,
        plant,
        scene,
        false,
        true,
        GroundMode::AdaptiveTerrainObjects,
    )
}
fn run_scene_mode(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
    multi_height: bool,
    lidar_3d: bool,
    ground_mode: GroundMode,
) -> Result<(Run, serde_json::Value), String> {
    run_scene_mode_precision(
        scenario,
        seed,
        plant,
        scene,
        multi_height,
        lidar_3d,
        ground_mode,
        false,
    )
}
#[allow(clippy::too_many_arguments)]
fn run_scene_mode_precision(
    scenario: Scenario,
    seed: u64,
    plant: Plant,
    scene: Scene,
    multi_height: bool,
    lidar_3d: bool,
    ground_mode: GroundMode,
    precise_capsule_rays: bool,
) -> Result<(Run, serde_json::Value), String> {
    let ground_segmentation = ground_mode != GroundMode::Disabled;
    let vehicle_body = ground_mode == GroundMode::VehicleBody;
    let adaptive_terrain = ground_mode == GroundMode::AdaptiveTerrainObjects;
    let terrain_objects = matches!(
        ground_mode,
        GroundMode::TerrainObjects | GroundMode::AdaptiveTerrainObjects
    );
    let mut backend = if adaptive_terrain {
        RneBackend::new_with_scene_adaptive_terrain_objects(scenario.clone(), seed, plant, scene)?
    } else if terrain_objects {
        RneBackend::new_with_scene_terrain_objects(scenario.clone(), seed, plant, scene)?
    } else if vehicle_body {
        RneBackend::new_with_scene_ground_body(scenario.clone(), seed, plant, scene)?
    } else if ground_segmentation {
        RneBackend::new_with_scene_ground(scenario.clone(), seed, plant, scene)?
    } else if lidar_3d {
        RneBackend::new_with_scene_lidar_3d(scenario.clone(), seed, plant, scene)?
    } else if multi_height {
        RneBackend::new_with_scene_multi_height(scenario.clone(), seed, plant, scene)?
    } else {
        RneBackend::new_with_scene(scenario.clone(), seed, plant, scene)?
    };
    backend.precise_capsule_rays = precise_capsule_rays;
    let capture = backend.scene.as_ref().unwrap().clone();
    let config = backend.config();
    let calibration = config.multi_height_lidar.clone();
    let calibration_3d = config.lidar3d.clone();
    let terrain_calibration = config.perception3d.clone();
    let mut run = simulate_with_backend(
        scenario,
        seed,
        backend,
        config,
        match plant {
            Plant::Kinematic => "rne-kinematic-rapier-lidar",
            Plant::Dynamic => "rne-dynamic-rapier-lidar",
        },
    )?;
    let capture = capture
        .lock()
        .map_err(|_| "native scene capture poisoned")?;
    let mut evidence = capture.evidence(run.vehicle.radius);
    if let Some(body) = &capture.body {
        let body_evidence = body.evidence(&capture.scene);
        evidence["capsule_diagnostic"] = evidence["summary"].clone();
        evidence["summary"] = body_evidence["summary"].clone();
        evidence["body_guard"] = body_evidence;
        // Main-world collision geometry is the actual cuboid in this mode.
        // The legacy capsule calculation is a diagnostic, not acceptance.
        evidence.as_object_mut().unwrap().remove("ego_capsule");
    }
    drop(capture);
    evidence["backend"] = json!(run.backend);
    evidence["seed"] = json!(seed);
    evidence["scenario"] = json!(run.scenario.name);
    if precise_capsule_rays {
        evidence["precise_capsule_rays"] = json!(true);
        evidence["capsule_query_refinement"] = json!({
            "kind":"native_rapier_non_capsule_hits_f64_physical_capsule_recovery",
            "geometry":"synchronized physical ECS capsule and rigid local offset",
            "native_pin_unchanged":true,
            "shape_expansion_m":0.0,
            "recovers_native_false_negatives":true,
            "maximum_capsules_per_acquisition":1024,
            "other_shapes":"native hit distance and surface unchanged",
            "ordering":"physical capsule and native non-capsule distance then entity index; one return per capsule"
        });
    }
    if multi_height {
        evidence["operating_mode"] = json!("multi_height_lidar");
        evidence["multi_height_lidar"] = json!(calibration.unwrap());
    }
    if lidar_3d {
        evidence["operating_mode"] = json!(if adaptive_terrain {
            "lidar3d_adaptive_terrain_objects"
        } else if terrain_objects {
            "lidar3d_terrain_objects"
        } else if vehicle_body {
            "lidar3d_ground_body"
        } else if ground_segmentation {
            "lidar3d_ground"
        } else {
            "lidar3d"
        });
        evidence["lidar3d"] = json!(calibration_3d.unwrap());
    }
    if ground_segmentation {
        evidence["ground_observations"] = json!(
            run.sensor_log
                .as_ref()
                .unwrap()
                .ticks
                .iter()
                .filter_map(|tick| tick.expected.ground.as_ref())
                .collect::<Vec<_>>()
        );
    }
    if terrain_objects {
        evidence["perception3d"] = json!(terrain_calibration.unwrap());
        evidence["terrain_object_observations"] = json!(
            run.sensor_log
                .as_ref()
                .unwrap()
                .ticks
                .iter()
                .filter_map(|tick| tick.expected.perception3d.as_ref())
                .collect::<Vec<_>>()
        );
    }
    if evidence["summary"]["passed"] != json!(true) {
        run.summary.passed = false;
        for failure in evidence["summary"]["failures"].as_array().unwrap() {
            run.summary.failures.push(failure.as_str().unwrap().into());
        }
    }
    Ok((run, evidence))
}
#[cfg(test)]
mod tests {
    use super::*;
    fn static_scene(center: [f64; 3], half: [f64; 3]) -> Scene {
        Scene::from_json(&json!({"schema_version":1,"name":"native test geometry","static_cuboids":[{"id":"fixture","center_m":center,"half_extents_m":half,"yaw_rad":0.0}]}).to_string()).unwrap()
    }
    fn road_scene(obstacle: Option<([f64; 3], [f64; 3])>) -> Scene {
        let boxes = obstacle.map(|(center,half)|json!({"id":"barrier","center_m":center,"half_extents_m":half,"yaw_rad":0.0})).into_iter().collect::<Vec<_>>();
        Scene::from_json(&json!({"schema_version":2,"name":"measured native ground test","static_cuboids":boxes,"ground_cuboids":[{"id":"road","center_m":[40.0,0.0,-0.5],"half_extents_m":[60.0,25.0,0.5],"yaw_rad":0.0}]}).to_string()).unwrap()
    }
    fn scenario(name: &str) -> Scenario {
        serde_json::from_str(
            &std::fs::read_to_string(format!(
                "{}/../../scenarios/{name}.json",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap(),
        )
        .unwrap()
    }
    #[test]
    fn frames_preserve_heading_and_body_left() {
        let p = Vec2::new(3.0, 7.0);
        assert_eq!(from_rne(to_rne(p)), p);
        let rotation = Quat::from_rotation_y(std::f64::consts::FRAC_PI_2);
        assert!((yaw_rad(rotation) - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
        assert!(from_rne(rotation * Vec3::X).y > 0.99);
    }
    #[test]
    fn overhead_scene_preserves_operational_trace_and_default_pipeline_header() {
        let base = run(scenario("mission"), 7, Plant::Dynamic).unwrap();
        let (with_scene, evidence) = run_with_scene(
            scenario("mission"),
            7,
            Plant::Dynamic,
            static_scene([30.0, 0.0, 4.5], [1.0, 3.0, 1.0]),
        )
        .unwrap();
        assert!(with_scene.summary.passed, "{:?}", with_scene.summary);
        assert_eq!(
            serde_json::to_vec(&base).unwrap(),
            serde_json::to_vec(&with_scene).unwrap()
        );
        let mut base_log = vec![];
        let mut scene_log = vec![];
        base.sensor_log
            .as_ref()
            .unwrap()
            .write(&mut base_log)
            .unwrap();
        with_scene
            .sensor_log
            .as_ref()
            .unwrap()
            .write(&mut scene_log)
            .unwrap();
        assert_eq!(base_log, scene_log);
        assert_eq!(
            evidence["motion_samples"].as_array().unwrap().len(),
            (with_scene.summary.steps - 1) * 10 + 1
        );
        assert_eq!(
            evidence["observations"].as_array().unwrap().len(),
            with_scene.summary.steps
        );
        assert!(
            evidence["acquisitions"]
                .as_array()
                .unwrap()
                .iter()
                .any(|a| a["channels"][2]["ranges_m"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|r| r.is_number()))
        );
    }
    #[test]
    fn multi_height_lidar_stops_the_previously_blind_low_slab() {
        for plant in [Plant::Kinematic, Plant::Dynamic] {
            let (run, evidence) = run_with_scene_multi_height(
                scenario("native-scene-low-stop"),
                7,
                plant,
                static_scene([35.0, 0.0, 0.1], [0.5, 3.0, 0.1]),
            )
            .unwrap();
            assert!(run.summary.passed, "{plant:?}: {:?}", run.summary);
            assert!(!run.summary.reached_goal);
            assert!(run.summary.final_speed < 0.2);
            assert!(run.summary.max_tracks > 0);
            assert!(evidence["summary"]["min_clearance_m"].as_f64().unwrap() >= 1.0);
            assert_eq!(evidence["operating_mode"], json!("multi_height_lidar"));
            assert!(
                run.sensor_log
                    .as_ref()
                    .unwrap()
                    .ticks
                    .iter()
                    .all(|t| t.input.lidar.is_none())
            );
            assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
                t.input.multi_height_lidar.as_ref().is_some_and(|scan| {
                    scan.planes[1].height_m == 0.15 && !scan.planes[1].points.is_empty()
                })
            }));
        }
    }
    #[test]
    fn actual_native_ground_returns_are_removed_before_clear_road_driving() {
        let mut s = scenario("native-scene-raised-goal");
        s.half_width = 3.0;
        let (run, evidence) =
            run_with_scene_ground(s, 7, Plant::Dynamic, road_scene(None)).unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        assert!(run.summary.reached_goal);
        assert_eq!(evidence["lidar3d"]["azimuth_columns"], json!(180));
        assert_eq!(evidence["operating_mode"], json!("lidar3d_ground"));
        assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
            t.expected
                .ground
                .as_ref()
                .is_some_and(|ground| ground.confidence && ground.removed_points >= 200)
        }));
        assert!(
            run.sensor_log
                .as_ref()
                .unwrap()
                .ticks
                .iter()
                .filter_map(|t| t.input.lidar3d.as_ref())
                .flat_map(|scan| &scan.returns)
                .any(|r| r.point.z.abs() < 0.02)
        );
        let mut bytes = vec![];
        run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
        assert!(
            !String::from_utf8_lossy(&bytes)
                .lines()
                .next()
                .unwrap()
                .contains("ground_cuboids")
        );
        rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink()).unwrap();
    }
    #[test]
    fn native_ground_fitting_preserves_low_middle_and_raised_obstacles() {
        for (center, half, goal) in [
            ([35.0, 0.0, 0.1], [0.5, 3.0, 0.1], false),
            ([35.0, 0.0, 1.5], [0.5, 3.0, 0.1], false),
            ([35.0, 0.0, 4.5], [0.5, 3.0, 1.0], true),
        ] {
            let mut s = scenario(if goal {
                "native-scene-raised-goal"
            } else {
                "native-scene-low-stop"
            });
            s.half_width = 3.0;
            let (run, evidence) =
                run_with_scene_ground(s, 7, Plant::Dynamic, road_scene(Some((center, half))))
                    .unwrap();
            assert!(run.summary.passed, "height{}: {:?}", center[2], run.summary);
            assert_eq!(run.summary.reached_goal, goal);
            assert!(evidence["summary"]["min_clearance_m"].as_f64().unwrap() >= 1.0);
            assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
                t.expected
                    .ground
                    .as_ref()
                    .is_some_and(|g| g.confidence && g.removed_points > 0)
            }));
        }
    }
    #[test]
    fn native_missing_or_unsupported_ground_holds_the_driver_brake() {
        for scene in [
            static_scene([35.0, 0.0, 4.5], [0.5, 3.0, 1.0]),
            static_scene([40.0, 0.0, -0.44], [60.0, 25.0, 0.5]),
        ] {
            let mut backend = RneBackend::new_with_scene_ground(
                scenario("native-scene-raised-goal"),
                7,
                Plant::Dynamic,
                scene,
            )
            .unwrap();
            let mut pipeline =
                rustdriving_pipeline::DrivingPipeline::new(backend.config()).unwrap();
            for tick in 0..4 {
                let output = pipeline
                    .step(&backend.observe(tick as f64 * 0.05, tick).unwrap())
                    .unwrap();
                assert!(
                    output
                        .health
                        .contains(&rustdriving_pipeline::HealthIssue::InvalidLidar)
                );
                assert_eq!(output.command.acceleration, -6.0);
                if let Some(ground) = output.ground {
                    assert!(!ground.confidence);
                    assert_eq!(ground.removed_points, 0);
                }
            }
        }
    }
    #[test]
    fn native_ground_and_actual_actor_xyz_support_moving_lead_stopping() {
        let mut s = scenario("native-ground-traffic-stop");
        s.half_width = 3.0;
        let (run, evidence) =
            run_with_scene_ground(s, 7, Plant::Dynamic, road_scene(None)).unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        assert!(run.summary.max_tracks > 0);
        assert!(run.summary.min_clearance >= 1.0);
        assert!(run.frames.iter().any(|f| !f.traffic.is_empty()));
        assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
            t.input
                .lidar3d
                .as_ref()
                .is_some_and(|scan| scan.returns.iter().any(|r| r.point.z > 0.1))
                && t.expected
                    .ground
                    .as_ref()
                    .is_some_and(|ground| ground.confidence && ground.removed_points > 0)
        }));
        assert_eq!(
            evidence["summary"]["min_clearance_m"],
            serde_json::Value::Null
        );
    }
    #[test]
    fn inclined_lidar_3d_detects_and_stops_a_barrier_between_all_horizontal_planes() {
        let mut s = scenario("native-scene-ground-stop");
        s.name = "Native inclined LiDAR: mid-height thin barrier".into();
        let (run, evidence) = run_with_scene_lidar_3d(
            s,
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 1.5], [0.5, 3.0, 0.1]),
        )
        .unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        assert!(!run.summary.reached_goal);
        assert!(evidence["summary"]["min_clearance_m"].as_f64().unwrap() >= 1.0);
        assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
            t.input.lidar3d.as_ref().is_some_and(|scan| {
                scan.returns
                    .iter()
                    .any(|r| r.point.z > 1.35 && r.point.z < 1.65 && r.ray_index % 16 >= 8)
            })
        }));
        assert!(
            evidence["acquisitions"]
                .as_array()
                .unwrap()
                .iter()
                .all(|a| a["channels"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|c| c["ranges_m"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(serde_json::Value::is_null)))
        );
        let mut bytes = vec![];
        run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
        rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink()).unwrap();
    }
    #[test]
    fn inclined_lidar_3d_stops_the_slab_below_every_old_plane() {
        let (run, evidence) = run_with_scene_lidar_3d(
            scenario("native-scene-low-stop"),
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 0.05], [0.5, 3.0, 0.05]),
        )
        .unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        assert!(run.summary.max_tracks > 0);
        assert!(evidence["summary"]["min_clearance_m"].as_f64().unwrap() >= 1.0);
        assert_eq!(evidence["operating_mode"], json!("lidar3d"));
        assert!(
            run.sensor_log
                .as_ref()
                .unwrap()
                .ticks
                .iter()
                .all(|t| t.input.lidar.is_none() && t.input.multi_height_lidar.is_none())
        );
    }
    #[test]
    fn inclined_lidar_3d_preserves_overhead_clearance() {
        let (run, evidence) = run_with_scene_lidar_3d(
            scenario("native-scene-raised-goal"),
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 4.5], [0.5, 3.0, 1.0]),
        )
        .unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        assert!(run.summary.reached_goal);
        assert_eq!(run.summary.max_tracks, 0);
        assert!((evidence["summary"]["min_clearance_m"].as_f64().unwrap() - 1.15).abs() < 1e-10);
        assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
            t.input
                .lidar3d
                .as_ref()
                .is_some_and(|scan| !scan.returns.is_empty())
        }));
    }
    #[test]
    fn inclined_lidar_3d_keeps_close_objects_outside_vertical_fov_as_failures() {
        let mut backend = RneBackend::new_with_scene_lidar_3d(
            scenario("native-scene-raised-goal"),
            7,
            Plant::Dynamic,
            static_scene([5.0, 0.0, 2.25], [0.5, 2.0, 0.05]),
        )
        .unwrap();
        let first = backend.observe(0.0, 0).unwrap();
        assert!(first.lidar3d.unwrap().returns.is_empty());
        let (run, evidence) = run_with_scene_lidar_3d(
            scenario("native-scene-raised-goal"),
            7,
            Plant::Dynamic,
            static_scene([5.0, 0.0, 2.25], [0.5, 2.0, 0.05]),
        )
        .unwrap();
        assert!(!run.summary.passed);
        assert_eq!(evidence["summary"]["min_clearance_m"], json!(0.0));
    }
    #[test]
    fn inclined_lidar_3d_acquisition_failure_latches_until_complete_new_cloud() {
        let mut backend = RneBackend::new_with_scene_lidar_3d(
            scenario("native-scene-raised-goal"),
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 4.5], [0.5, 3.0, 1.0]),
        )
        .unwrap();
        let mut pipeline = rustdriving_pipeline::DrivingPipeline::new(backend.config()).unwrap();
        pipeline.step(&backend.observe(0.0, 0).unwrap()).unwrap();
        pipeline.step(&backend.observe(0.05, 1).unwrap()).unwrap();
        backend.fail_lidar_3d_from = Some(0.1);
        let failed = backend.observe(0.1, 2).unwrap();
        assert!(failed.lidar_failed && failed.lidar3d.is_none());
        let output = pipeline.step(&failed).unwrap();
        assert!(
            output
                .health
                .contains(&rustdriving_pipeline::HealthIssue::AcquisitionFailed)
        );
        assert_eq!(output.command.acceleration, -6.0);
        let held = pipeline.step(&backend.observe(0.15, 3).unwrap()).unwrap();
        assert_eq!(held.command.acceleration, -6.0);
        backend.fail_lidar_3d_from = None;
        let healthy = pipeline.step(&backend.observe(0.2, 4).unwrap()).unwrap();
        assert!(
            !healthy
                .health
                .contains(&rustdriving_pipeline::HealthIssue::AcquisitionFailed)
        );
        let mut s = scenario("lidar-fault");
        s.lidar_dropout = None;
        let mut backend = RneBackend::new_with_scene_lidar_3d(
            s.clone(),
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 4.5], [0.5, 3.0, 1.0]),
        )
        .unwrap();
        backend.fail_lidar_3d_from = Some(6.0);
        let config = backend.config();
        let run =
            simulate_with_backend(s, 7, backend, config, "rne-inclined-injected-failure").unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        let mut bytes = vec![];
        run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
        rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink()).unwrap();
    }
    #[test]
    fn multi_height_lidar_validates_overhead_returns_without_false_ground_obstacles() {
        let (run, evidence) = run_with_scene_multi_height(
            scenario("native-scene-raised-goal"),
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 4.5], [0.5, 3.0, 1.0]),
        )
        .unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        assert!(run.summary.reached_goal);
        assert_eq!(run.summary.max_tracks, 0);
        assert!((evidence["summary"]["min_clearance_m"].as_f64().unwrap() - 1.15).abs() < 1e-10);
        assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
            t.input
                .multi_height_lidar
                .as_ref()
                .is_some_and(|scan| !scan.planes[2].points.is_empty())
        }));
        let mut bytes = vec![];
        run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
        rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink()).unwrap();
    }
    #[test]
    fn multi_height_lidar_keeps_sub_plane_blind_spots_as_physical_failures() {
        let (run, evidence) = run_with_scene_multi_height(
            scenario("native-scene-blind-low"),
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 0.05], [0.5, 3.0, 0.05]),
        )
        .unwrap();
        assert!(run.summary.reached_goal);
        assert!(!run.summary.passed);
        assert_eq!(evidence["summary"]["min_clearance_m"], json!(0.0));
        assert_eq!(run.summary.max_tracks, 0);
    }
    #[test]
    fn multi_height_primary_or_auxiliary_acquisition_failure_reaches_driver_braking() {
        for auxiliary in [false, true] {
            let mut s = scenario("lidar-fault");
            s.lidar_dropout = None;
            let mut backend = RneBackend::new_with_scene_multi_height(
                s.clone(),
                7,
                Plant::Dynamic,
                static_scene([35.0, 0.0, 4.5], [0.5, 3.0, 1.0]),
            )
            .unwrap();
            if auxiliary {
                backend.fail_aux_lidar_from = Some(6.0);
            } else {
                backend.fail_lidar_from = Some(6.0);
            }
            let config = backend.config();
            let run =
                simulate_with_backend(s, 7, backend, config, "rne-multi-height-injected-failure")
                    .unwrap();
            assert!(
                run.summary.passed,
                "auxiliary={auxiliary}: {:?}",
                run.summary
            );
            assert!(run.sensor_log.as_ref().unwrap().ticks.iter().any(|t| {
                t.input.lidar_failed
                    && t.input.multi_height_lidar.is_none()
                    && t.input.lidar.is_none()
                    && t.expected
                        .health
                        .contains(&rustdriving_pipeline::HealthIssue::AcquisitionFailed)
                    && t.expected.command.acceleration == -6.0
            }));
            let mut bytes = vec![];
            run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
            rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap();
        }
    }
    #[test]
    fn low_blind_slab_is_diagnostic_only_and_rejected_by_capsule_evaluation() {
        let base = run(scenario("mission"), 7, Plant::Dynamic).unwrap();
        let (with_scene, evidence) = run_with_scene(
            scenario("mission"),
            7,
            Plant::Dynamic,
            static_scene([15.0, 0.0, 0.1], [1.0, 8.0, 0.1]),
        )
        .unwrap();
        // Its top is below the existing primary plane. The unchanged pipeline
        // reaches its goal, but the independent capsule evaluator must reject it.
        assert_eq!(
            serde_json::to_vec(&base.frames).unwrap(),
            serde_json::to_vec(&with_scene.frames).unwrap()
        );
        assert!(with_scene.summary.reached_goal);
        assert!(!with_scene.summary.passed);
        assert_eq!(evidence["summary"]["passed"], json!(false));
        assert_eq!(evidence["summary"]["min_clearance_m"], json!(0.0));
        assert!(
            evidence["summary"]["guard_overlap_intervals"]
                .as_u64()
                .unwrap()
                > 0
        );
        assert!(
            evidence["acquisitions"]
                .as_array()
                .unwrap()
                .iter()
                .any(|a| a["channels"][1]["ranges_m"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|r| r.is_number()))
        );
    }
    #[test]
    fn cuboid_occludes_actor_in_actual_native_scan_then_reveals_after_mount_moves() {
        let mut s = scenario("blocked");
        s.objects[0].s = 20.0;
        let mut backend = RneBackend::new_with_scene(
            s,
            7,
            Plant::Dynamic,
            static_scene([10.0, 0.0, 0.6], [0.3, 3.0, 0.6]),
        )
        .unwrap();
        let hidden = backend.observe(0.0, 0).unwrap().lidar.unwrap();
        assert!(
            hidden
                .points
                .iter()
                .any(|p| (p.x - 9.7).abs() < 0.03 && p.y.abs() < 0.1)
        );
        assert!(
            hidden
                .points
                .iter()
                .all(|p| p.distance(Vec2::new(20.0, 0.0)) > 1.05)
        );
        backend
            .world
            .get_mut::<Transform3>(backend.ego)
            .unwrap()
            .translation = to_rne(Vec2::new(15.0, 0.0));
        let visible = backend.observe(0.1, 2).unwrap().lidar.unwrap();
        assert!(
            visible
                .points
                .iter()
                .any(|p| (p.distance(Vec2::new(5.0, 0.0)) - 1.0).abs() < 0.03)
        );
    }
    #[test]
    fn ignored_sensing_cannot_pass_the_independent_native_capsule_guard() {
        let mut backend = RneBackend::new_with_scene(
            scenario("blocked"),
            7,
            Plant::Dynamic,
            static_scene([5.0, 0.0, 0.6], [0.3, 3.0, 0.6]),
        )
        .unwrap();
        for _ in 0..100 {
            backend
                .advance(
                    ControlCommand {
                        acceleration: 2.0,
                        steering: 0.0,
                    },
                    0.05,
                )
                .unwrap();
        }
        assert!(backend.state().pose.position.x > 6.0);
        let evidence = backend
            .scene
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .evidence(backend.config.vehicle.radius);
        assert_eq!(evidence["summary"]["passed"], json!(false));
        assert!(
            evidence["summary"]["guard_overlap_intervals"]
                .as_u64()
                .unwrap()
                > 0
        );
    }
    #[test]
    fn scene_guard_rejects_lateral_slip_exceeding_its_translation_bound() {
        let mut backend = RneBackend::new_with_scene(
            scenario("blocked"),
            7,
            Plant::Dynamic,
            static_scene([35.0, 0.0, 0.6], [1.0, 3.0, 0.6]),
        )
        .unwrap();
        backend
            .world
            .get_mut::<AckermannDrive>(backend.ego)
            .unwrap()
            .speed_m_s = 6.0;
        backend
            .world
            .get_mut::<VehicleDynamics>(backend.ego)
            .unwrap()
            .lateral_velocity_m_s = 20.0;
        let failure = backend
            .advance(
                ControlCommand {
                    acceleration: 0.0,
                    steering: 0.0,
                },
                0.05,
            )
            .unwrap_err();
        assert!(failure.contains("12 m/s clearance-bound contract"));
    }
    #[test]
    fn rne_mission_and_stop_run_without_a_renderer() {
        for name in ["mission", "blocked"] {
            let result = run(scenario(name), 7, Plant::Kinematic).unwrap();
            assert!(result.summary.passed, "{name}: {:?}", result.summary);
            assert!(result.summary.max_tracks > 0);
        }
    }
    #[test]
    fn dynamic_tire_limited_plant_completes_mission() {
        let result = run(scenario("mission"), 7, Plant::Dynamic).unwrap();
        assert!(result.summary.passed, "{:?}", result.summary);
    }
    #[test]
    fn mapped_destinations_and_detour_run_with_native_dynamics() {
        for (case, edge) in [
            ("route-direct", "main"),
            ("route-detour", "detour"),
            ("route-south", "south-branch"),
        ] {
            let result = run(scenario(case), 7, Plant::Dynamic).unwrap();
            assert!(result.summary.passed, "{case}: {:?}", result.summary);
            assert!(
                result
                    .navigation
                    .as_ref()
                    .unwrap()
                    .edge_ids
                    .iter()
                    .any(|id| id == edge)
            );
            assert!(result.summary.min_clearance >= 0.5);
        }
    }
    #[test]
    fn live_closure_handover_and_unreachable_hold_use_native_dynamics() {
        for case in ["route-handover", "route-no-path", "route-reopen"] {
            let result = run(scenario(case), 7, Plant::Dynamic).unwrap();
            assert!(result.summary.passed, "{case}: {:?}", result.summary);
            assert_eq!(result.summary.closure_violations, 0);
            if case != "route-no-path" {
                assert_eq!(result.summary.navigation_switches, 1);
                assert!(result.route_history[1].true_speed <= 0.1);
                assert!(result.route_history[1].estimated_speed.abs() <= 0.05);
            } else {
                assert_eq!(result.summary.navigation_switches, 0);
            }
        }
    }
    #[test]
    fn six_meter_per_second_handover_passes_without_a_curvature_cap() {
        let s = scenario("route-handover-fast");
        assert_eq!(s.cruise_speed, Some(6.0));
        assert!(s.motion_limits.is_none());
        let result = run(s, 7, Plant::Dynamic).unwrap();
        assert!(result.summary.passed, "{:?}", result.summary);
        assert!(result.summary.reached_goal);
        assert_eq!(result.summary.navigation_switches, 1);
        assert_eq!(result.summary.collisions, 0);
        assert_eq!(result.summary.road_violations, 0);
        assert_eq!(result.summary.closure_violations, 0);
        assert!(result.summary.min_clearance >= 0.5);
    }
    #[test]
    fn gnss_spike_burst_recovery_and_persistent_fault_use_native_dynamics() {
        for case in ["gnss-spike", "gnss-burst", "gnss-persistent-bias"] {
            let result = run(scenario(case), 7, Plant::Dynamic).unwrap();
            assert!(result.summary.passed, "{case}: {:?}", result.summary);
            assert!(result.summary.localization_max_error < 0.5);
            assert!(
                result
                    .sensor_log
                    .as_ref()
                    .unwrap()
                    .ticks
                    .last()
                    .unwrap()
                    .expected
                    .localization
                    .unwrap()
                    .rejected_fixes
                    > 0
            );
            if case == "gnss-burst" {
                assert!(
                    result
                        .frames
                        .iter()
                        .any(|f| (7.0..8.0).contains(&f.time) && f.truth.speed < 0.1)
                );
                assert!(result.summary.reached_goal);
            }
            if case == "gnss-persistent-bias" {
                assert!(
                    result
                        .sensor_log
                        .as_ref()
                        .unwrap()
                        .ticks
                        .last()
                        .unwrap()
                        .expected
                        .health
                        .contains(&rustdriving_pipeline::HealthIssue::StaleGnss)
                );
                assert_eq!(result.summary.final_speed, 0.0);
            }
        }
    }
    #[test]
    fn terminal_traffic_counterexample_and_physical_hold_reach_the_goal() {
        for name in ["gnss-burst-traffic", "gnss-burst-traffic-hold"] {
            let result = run(scenario(name), 7, Plant::Dynamic).unwrap();
            assert!(result.summary.passed, "{name}: {:?}", result.summary);
            assert_eq!(result.summary.collisions, 0);
            assert_eq!(result.summary.road_violations, 0);
            assert!(result.summary.reached_goal);
            assert!(result.summary.min_clearance >= 0.5);
            assert!(result.summary.localization_max_error < 0.5);
            if name.ends_with("-hold") {
                let end = result.frames.last().unwrap().time;
                assert!(
                    result
                        .frames
                        .iter()
                        .filter(|f| f.time >= end - result.scenario.goal_hold_seconds.unwrap())
                        .all(|f| f.truth.speed < 0.2 && f.progress >= result.route.length() - 2.0)
                );
                // The scripted lead has reached the original endpoint by now;
                // stopping the evaluation before it catches up cannot pass.
                let lead = result
                    .frames
                    .last()
                    .unwrap()
                    .objects
                    .iter()
                    .find(|o| o.id == 1)
                    .unwrap();
                assert!(lead.position.distance(*result.route.points.last().unwrap()) < 1e-8);
            }
        }
    }
    #[test]
    fn reactive_traffic_models_run_with_native_ego_dynamics() {
        for name in [
            "traffic-lead-stop",
            "traffic-follower-brake",
            "traffic-queue",
        ] {
            let r = run(scenario(name), 7, Plant::Dynamic).unwrap();
            assert!(r.summary.passed, "{name}: {:?}", r.summary);
            assert_eq!(
                r.summary.collisions
                    + r.summary.traffic_collisions
                    + r.summary.traffic_road_violations,
                0
            );
            assert!(r.summary.min_clearance >= 1.0);
            assert!(r.frames.iter().any(|f| !f.traffic.is_empty()));
        }
    }
    #[test]
    fn observed_braking_repairs_the_unchanged_follower_deadline() {
        for seed in [1, 7, 42] {
            let r = run(scenario("traffic-follower-deadline"), seed, Plant::Dynamic).unwrap();
            assert!(r.summary.passed, "{:?}", r.summary);
            assert!(r.summary.reached_goal);
            assert!(r.summary.simulated_seconds <= 65.0);
            assert!(r.summary.min_clearance >= 1.0);
            assert_eq!(r.summary.collisions + r.summary.traffic_collisions, 0);
            let end = r.frames.last().unwrap().time;
            for frame in r.frames.iter().filter(|f| f.time >= end - 8.0 - 1e-9) {
                assert!(frame.truth.speed < 0.2);
                assert!(frame.truth.pose.position.x >= 158.0);
            }
        }
    }
    #[test]
    fn insufficient_follower_sensing_still_fails_the_physical_clearance_floor() {
        for seed in [1, 42] {
            let r = run(
                scenario("traffic-follower-short-range"),
                seed,
                Plant::Dynamic,
            )
            .unwrap();
            assert!(!r.summary.passed);
            assert!(r.summary.min_clearance < 1.0);
            assert!(
                r.summary
                    .failures
                    .iter()
                    .any(|f| f.contains("minimum swept clearance"))
            );
        }
    }
    #[test]
    fn acquisition_error_reaches_braking_guard() {
        let mut s = scenario("lidar-fault");
        s.lidar_dropout = None;
        let mut backend = RneBackend::new(s.clone(), 7, Plant::Kinematic).unwrap();
        backend.fail_lidar_from = Some(6.0);
        let config = backend.config();
        let result =
            simulate_with_backend(s, 7, backend, config, "rne-injected-raycast-error").unwrap();
        assert!(result.summary.passed, "{:?}", result.summary);
        assert!(result.sensor_log.unwrap().ticks.iter().any(|t| {
            t.input.lidar_failed
                && t.expected
                    .health
                    .contains(&rustdriving_pipeline::HealthIssue::AcquisitionFailed)
                && t.expected.command.acceleration == -6.0
        }));
    }
    #[test]
    fn rne_sensor_log_replays_through_same_stack() {
        let result = run(scenario("blocked"), 42, Plant::Kinematic).unwrap();
        let mut bytes = vec![];
        result.sensor_log.unwrap().write(&mut bytes).unwrap();
        assert!(
            rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap()
                .ticks
                > 0
        );
    }
    #[test]
    fn hazard_scenarios_across_seeds() {
        for case in [
            "occluded-crossing",
            "cut-in",
            "low-friction",
            "low-friction-stop",
        ] {
            for seed in [1, 7, 42] {
                let result = run(scenario(case), seed, Plant::Dynamic).unwrap();
                assert!(
                    result.summary.passed,
                    "{case} seed {seed}: {:?}",
                    result.summary
                );
                assert!(result.summary.max_tracks > 0);
                if case == "low-friction" {
                    assert!(
                        result.summary.min_clearance >= 0.4,
                        "low-friction clearance regressed: {:?}",
                        result.summary
                    );
                    assert!(
                        result.summary.emergency_steps <= 20,
                        "low-friction tracking regressed for seed {seed}: {:?}",
                        result.summary
                    );
                }
            }
        }
    }
    #[test]
    fn lidar_reveals_a_geometrically_occluded_actor() {
        let s = scenario("occluded-crossing");
        let mut backend = RneBackend::new(s, 7, Plant::Dynamic).unwrap();
        let target = backend.objects(0.0)[1].clone();
        // Put both actors within the 45 m range to prove occlusion rather than range exclusion.
        backend
            .world
            .get_mut::<Transform3>(backend.ego)
            .unwrap()
            .translation = to_rne(Vec2::new(40.0, 0.0));
        let pose = backend.state().pose;
        let hidden = backend.observe(0.0, 0).unwrap().lidar.unwrap();
        assert!(!hidden.points.is_empty());
        assert!(
            hidden
                .points
                .iter()
                .all(|p| pose.to_world(*p).distance(target.position) > target.radius + 0.05)
        );
        // Sensor fixture: move the acquisition mount past the occluder while the
        // target remains stationary. Pose is used only for sensor verification.
        backend
            .world
            .get_mut::<Transform3>(backend.ego)
            .unwrap()
            .translation = to_rne(Vec2::new(70.0, 0.0));
        let pose = backend.state().pose;
        let visible = backend.observe(12.0, 240).unwrap().lidar.unwrap();
        assert!(
            visible
                .points
                .iter()
                .any(
                    |p| (pose.to_world(*p).distance(target.position) - target.radius).abs() < 0.05
                )
        );
    }
    #[test]
    fn low_friction_limits_actual_acceleration_and_braking() {
        let mut backend =
            RneBackend::new(scenario("low-friction-stop"), 7, Plant::Dynamic).unwrap();
        let dt = 0.05;
        backend
            .advance(
                ControlCommand {
                    acceleration: 2.0,
                    steering: 0.0,
                },
                dt,
            )
            .unwrap();
        assert!((backend.state().speed / dt - 0.2 * 9.81).abs() < 1e-9);
        backend
            .world
            .get_mut::<AckermannDrive>(backend.ego)
            .unwrap()
            .speed_m_s = 6.0;
        backend.advance(ControlCommand::emergency(), dt).unwrap();
        assert!(((6.0 - backend.state().speed) / dt - 0.2 * 9.81).abs() < 1e-9);
        assert!(
            backend
                .config()
                .motion_limits
                .unwrap()
                .max_deceleration_m_s2
                < 0.2 * 9.81
        );
        assert!(RneBackend::new(scenario("low-friction"), 7, Plant::Kinematic).is_err());
    }
}
