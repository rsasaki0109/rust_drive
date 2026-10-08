//! CPU-only RNE plant and Rapier LiDAR adapter for the shared RustDrive pipeline.
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
use rustdrive_core::{ControlCommand, EgoState, Gnss, LidarScan, Odometry, Pose, Vec2};
use rustdrive_pipeline::{MotionLimits, PipelineConfig, SensorFrame};
use rustdrive_sim::{
    Run, Scenario, SimulationBackend, WorldObject, pipeline_config, simulate_with_backend,
};

/// RNE plant selection. Both use native RNE systems, never RustDrive's reference integrator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Plant {
    Kinematic,
    Dynamic,
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
}
impl LidarRaycaster for EgoFilteredRaycaster<'_> {
    fn lidar_raycast(
        &self,
        world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        let mut hits = self.backend.raycast(world, query)?;
        hits.retain(|h| h.entity != self.ego);
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
    /// Test/diagnostic injection; a checked raycast against an unknown world must brake.
    pub fail_lidar_from: Option<f64>,
}
impl RneBackend {
    /// Creates a headless native vehicle with a CPU Rapier query scene.
    pub fn new(scenario: Scenario, seed: u64, plant: Plant) -> Result<Self, String> {
        scenario.validate()?;
        if scenario.dynamics.is_some() && plant != Plant::Dynamic {
            return Err("friction/lag calibration requires --plant dynamic".into());
        }
        let mut config = pipeline_config(&scenario);
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
        let ego = spawn_named(&mut world, "rustdrive_ego");
        let pose = config.initial_pose;
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
            // Fixed query geometry follows the separately integrated native RNE plant.
            // Rapier supplies sensing geometry; it does not integrate the vehicle a second time.
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Capsule {
                    half_height_m: 0.5,
                    radius_m: config.vehicle.radius,
                },
                ..Collider::default()
            },
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
        let mut physics = RapierBackend::new();
        let physics_world = physics
            .create_world(PhysicsWorldDesc::default())
            .map_err(|e| e.to_string())?;
        Ok(Self {
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
            fail_lidar_from: None,
        })
    }
    /// The exact stack configuration used by this plant, included in its replay header.
    pub fn config(&self) -> PipelineConfig {
        self.config.clone()
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
        self.scenario.world_objects(&self.config.route, time)
    }
    fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String> {
        let truth = self.state();
        let body = self.world.get::<RigidBody>(self.ego).unwrap();
        let odometry = Some(Odometry {
            stamp: time,
            speed: truth.speed + self.noisy(tick, 0, 0.015),
            yaw_rate: body.angular_velocity_rad_s.y + self.noisy(tick, 1, 0.001),
        });
        let gnss = if tick.is_multiple_of(4) && self.scenario.gnss_dropout.is_none_or(|t| time < t)
        {
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
            let raycaster = EgoFilteredRaycaster {
                backend: &self.physics,
                ego: self.ego,
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
                    // RNE emits world-frame points; calibrate into body x-forward/y-left.
                    let inverse = mount.rotation.conjugate();
                    lidar = Some(LidarScan {
                        stamp: time,
                        points: cloud
                            .points_m
                            .into_iter()
                            .map(|p| from_rne(inverse * (p - mount.translation)))
                            .collect(),
                    });
                }
                Err(_) => lidar_failed = true,
            }
        }
        Ok(SensorFrame {
            navigation_update: None,
            time,
            odometry,
            gnss,
            lidar,
            lidar_failed,
        })
    }
    fn advance(&mut self, command: ControlCommand, dt: f64) -> Result<(), String> {
        if !command.finite() || !dt.is_finite() || dt <= 0.0 {
            return Err("invalid RNE actuation".into());
        }
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
#[cfg(test)]
mod tests {
    use super::*;
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
    fn known_six_meter_per_second_obstacle_deadlock_remains_an_explicit_failed_mission() {
        let mut s = scenario("route-handover");
        s.cruise_speed = Some(6.0);
        s.motion_limits = None;
        let result = run(s, 7, Plant::Dynamic).unwrap();
        assert!(!result.summary.passed);
        assert!(!result.summary.reached_goal);
        assert_eq!(result.summary.navigation_switches, 1);
        assert_eq!(result.summary.collisions, 0);
        assert_eq!(result.summary.road_violations, 0);
        assert!(
            result
                .summary
                .failures
                .iter()
                .any(|f| f.contains("goal not reached"))
        );
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
                    .contains(&rustdrive_pipeline::HealthIssue::AcquisitionFailed)
                && t.expected.command.acceleration == -6.0
        }));
    }
    #[test]
    fn rne_sensor_log_replays_through_same_stack() {
        let result = run(scenario("blocked"), 42, Plant::Kinematic).unwrap();
        let mut bytes = vec![];
        result.sensor_log.unwrap().write(&mut bytes).unwrap();
        assert!(
            rustdrive_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
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
