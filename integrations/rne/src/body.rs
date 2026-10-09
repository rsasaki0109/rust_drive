//! Optional research car envelope and independent upright-cuboid motion guard.
//! The native plant reference pose stays planar; these dimensions are authored
//! research calibration, not measured specifications for a physical vehicle.
use crate::scene::{Scene, StaticCuboid};
use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use rustdriving_core::{Pose, Vec2};
use serde_json::{Value, json};

const SPEED_BOUND_M_S: f64 = 12.0;
const FLOOR_M: f64 = 1.0;

#[derive(Clone, Copy, Debug)]
pub struct BodyCalibration {
    pub length_m: f64,
    pub width_m: f64,
    pub height_m: f64,
    pub bottom_m: f64,
    /// Forward / left offset from the native plant's planar reference point.
    pub center_offset_body_m: Vec2,
}
impl Default for BodyCalibration {
    fn default() -> Self {
        Self {
            length_m: 4.2,
            width_m: 1.8,
            height_m: 1.5,
            bottom_m: 0.15,
            center_offset_body_m: Vec2::new(0.0, 0.0),
        }
    }
}
impl BodyCalibration {
    pub fn validate(self) -> Result<(), String> {
        if ![self.length_m, self.width_m, self.height_m, self.bottom_m]
            .into_iter()
            .all(f64::is_finite)
            || !(2.0..=8.0).contains(&self.length_m)
            || !(1.0..=3.0).contains(&self.width_m)
            || !(0.5..=3.0).contains(&self.height_m)
            || !(0.05..=0.5).contains(&self.bottom_m)
            || !self.center_offset_body_m.finite()
            || self
                .center_offset_body_m
                .x
                .hypot(self.center_offset_body_m.y)
                > 2.0
        {
            return Err("invalid authored body envelope calibration".into());
        }
        Ok(())
    }
    pub fn corner_radius_m(self) -> f64 {
        self.length_m.hypot(self.width_m) / 2.0
            + self
                .center_offset_body_m
                .x
                .hypot(self.center_offset_body_m.y)
    }
    /// Identical calibration for the actual main-scene Rapier cuboid. Its local
    /// vertical offset assumes the existing native plant Transform3 height 0.6 m.
    pub fn collider(self) -> Collider {
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(
                    self.length_m / 2.0,
                    self.height_m / 2.0,
                    self.width_m / 2.0,
                ),
            },
            local_offset: Transform3::from_translation_rotation(
                Vec3::new(
                    self.center_offset_body_m.x,
                    self.bottom_m + self.height_m / 2.0 - 0.6,
                    -self.center_offset_body_m.y,
                ),
                Quat::IDENTITY,
            ),
            ..Collider::default()
        }
    }
    pub fn to_json(self) -> Value {
        json!({"length_m":self.length_m,"width_m":self.width_m,"height_m":self.height_m,"bottom_m":self.bottom_m,"center_offset_body_m":[self.center_offset_body_m.x,self.center_offset_body_m.y],"pose_reference":"native_planar_plant_reference","speed_bound_m_s":SPEED_BOUND_M_S,"clearance_floor_m":FLOOR_M,"calibration_kind":"authored_research_dimensions"})
    }
}

#[derive(Clone, Copy)]
pub(crate) struct BodyMotionSample {
    pub time: f64,
    pub pose: Pose,
}
pub(crate) struct BodyCapture {
    pub calibration: BodyCalibration,
    pub motion_samples: Vec<BodyMotionSample>,
    pub witnesses: Vec<Value>,
}
impl BodyCapture {
    pub fn new(calibration: BodyCalibration, pose: Pose) -> Result<Self, String> {
        calibration.validate()?;
        if !pose.position.finite() || !pose.yaw.is_finite() {
            return Err("native body capture requires a finite initial pose".into());
        }
        Ok(Self {
            calibration,
            motion_samples: vec![BodyMotionSample { time: 0.0, pose }],
            witnesses: vec![],
        })
    }
    pub fn record(&mut self, time: f64, pose: Pose) -> Result<(), String> {
        let previous = self.motion_samples.last().unwrap();
        let dt = time - previous.time;
        let distance = pose.position.distance(previous.pose.position);
        let yaw_delta = angle_delta(previous.pose.yaw, pose.yaw).abs();
        if !time.is_finite()
            || !pose.position.finite()
            || !pose.yaw.is_finite()
            || dt <= 0.0
            || dt > 0.005 + 1e-9
            || distance > SPEED_BOUND_M_S * dt + 1e-8
            || yaw_delta > 0.1 + 1e-9
        {
            return Err(
                "native body motion exceeds checked 200 Hz translation/yaw contract".into(),
            );
        }
        self.motion_samples.push(BodyMotionSample { time, pose });
        Ok(())
    }
    pub fn evidence(&self, scene: &Scene) -> Value {
        let mut minimum: Option<f64> = None;
        let mut overlaps = 0;
        let mut checks = 0;
        for pair in self.motion_samples.windows(2) {
            // The pinned native plant translates linearly along its midpoint
            // velocity and rotates linearly by yaw_delta during each substep.
            // Any body point is within this displacement of one endpoint.
            let inflation = SPEED_BOUND_M_S * (pair[1].time - pair[0].time) / 2.0
                + self.calibration.corner_radius_m()
                    * angle_delta(pair[0].pose.yaw, pair[1].pose.yaw).abs()
                    / 2.0;
            let mut interval_overlap = false;
            for cuboid in &scene.static_cuboids {
                let endpoint_clearance = upright_clearance(self.calibration, pair[0].pose, cuboid)
                    .min(upright_clearance(self.calibration, pair[1].pose, cuboid));
                let clearance = (endpoint_clearance - inflation).max(0.0);
                minimum = Some(minimum.map_or(clearance, |old| old.min(clearance)));
                interval_overlap |= clearance <= 0.0;
                checks += 1;
            }
            overlaps += usize::from(interval_overlap);
        }
        for point in &self.motion_samples {
            for cuboid in &scene.static_cuboids {
                let clearance = upright_clearance(self.calibration, point.pose, cuboid);
                minimum = Some(minimum.map_or(clearance, |old| old.min(clearance)));
                checks += 1;
            }
        }
        let mut failures = Vec::new();
        if overlaps > 0 {
            failures.push(format!(
                "{overlaps} native car-body conservative guard overlaps"
            ));
        }
        if minimum.is_some_and(|clearance| clearance < FLOOR_M) {
            failures.push(format!(
                "native car-body clearance {:.6} m is below fixed 1 m floor",
                minimum.unwrap()
            ));
        }
        let native_overlap_samples = self
            .witnesses
            .iter()
            .filter(|w| !w["obstacle_ids"].as_array().unwrap().is_empty())
            .count();
        if native_overlap_samples > 0 {
            failures.push(format!(
                "{native_overlap_samples} native Rapier car-body sensor overlap samples"
            ));
        }
        json!({"schema_version":1,"calibration":self.calibration.to_json(),"motion_samples":self.motion_samples.iter().map(|s|json!({"time":s.time,"pose":crate::scene::pose_json(s.pose)})).collect::<Vec<_>>(),"rapier_sensor_witnesses":self.witnesses,"summary":{"passed":failures.is_empty(),"min_clearance_m":minimum,"guard_overlap_intervals":overlaps,"native_overlap_samples":native_overlap_samples,"checks":checks,"failures":failures}})
    }
}

/// Updates force-free sensor overlap events in the same gravity-zero Rapier
/// world. Native ECS pose/velocity remain authoritative. Temporarily zeroing the
/// carrier's velocities prevents Rapier from integrating a second trajectory;
/// restoring the native velocities preserves odometry. No sync_to_ecs is used.
pub(crate) fn sensor_witness(
    physics: &mut RapierBackend,
    world_id: PhysicsWorldId,
    world: &mut World,
    ego: Entity,
    obstacles: &[(Entity, String)],
    time: f64,
    dt: SimDuration,
) -> Result<Value, String> {
    let native_body = *world
        .get::<RigidBody>(ego)
        .ok_or("body witness requires native carrier")?;
    if native_body.body_type != RigidBodyType::Dynamic
        || !world
            .get::<Collider>(ego)
            .is_some_and(|c| c.sensor && matches!(c.shape, ColliderShape::Cuboid { .. }))
        || !time.is_finite()
        || dt.as_seconds().value() <= 0.0
        || dt.as_seconds().value() > 0.005 + 1e-9
    {
        return Err("body witness requires force-free sensor cuboid".into());
    }
    world.entity_mut(ego).insert(RigidBody {
        linear_velocity_m_s: Vec3::ZERO,
        angular_velocity_rad_s: Vec3::ZERO,
        ..native_body
    });
    let result = physics
        .sync_from_ecs(world, world_id)
        .and_then(|()| physics.step(world_id, dt));
    world.entity_mut(ego).insert(native_body);
    result.map_err(|e| e.to_string())?;
    let mut obstacle_ids = vec![];
    for event in physics.contacts(world_id).map_err(|e| e.to_string())? {
        let other = if event.entity_a == ego {
            event.entity_b
        } else if event.entity_b == ego {
            event.entity_a
        } else {
            continue;
        };
        if let Some((_, id)) = obstacles.iter().find(|(entity, _)| *entity == other) {
            if event.impulse != 0.0 || event.normal != Vec3::ZERO {
                return Err("car-body witness unexpectedly applied a contact impulse".into());
            }
            obstacle_ids.push(id.clone());
        }
    }
    obstacle_ids.sort();
    obstacle_ids.dedup();
    Ok(json!({"time":time,"obstacle_ids":obstacle_ids,"force_free":true}))
}

fn angle_delta(a: f64, b: f64) -> f64 {
    (b - a).sin().atan2((b - a).cos())
}
fn vertices(center: Vec2, half: Vec2, yaw: f64) -> [Vec2; 4] {
    [
        Vec2::new(-half.x, -half.y),
        Vec2::new(half.x, -half.y),
        Vec2::new(half.x, half.y),
        Vec2::new(-half.x, half.y),
    ]
    .map(|point| center.plus(point.rotated(yaw)))
}
fn point_segment(point: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b.minus(a);
    let q = point.minus(a);
    let t = ((q.x * d.x + q.y * d.y) / (d.x * d.x + d.y * d.y)).clamp(0.0, 1.0);
    point.distance(a.plus(d.scaled(t)))
}
fn polygon_distance(a: [Vec2; 4], b: [Vec2; 4]) -> f64 {
    // Separating-axis overlap is independent of the edge-distance calculation.
    let separated = [a, b].iter().any(|polygon| {
        (0..4).any(|i| {
            let edge = polygon[(i + 1) % 4].minus(polygon[i]);
            let axis = Vec2::new(-edge.y, edge.x);
            let range = |poly: &[Vec2; 4]| {
                poly.iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                        let dot = p.x * axis.x + p.y * axis.y;
                        (lo.min(dot), hi.max(dot))
                    })
            };
            let ar = range(&a);
            let br = range(&b);
            ar.1 < br.0 || br.1 < ar.0
        })
    });
    if !separated {
        return 0.0;
    }
    let mut distance = f64::INFINITY;
    for i in 0..4 {
        for j in 0..4 {
            distance = distance
                .min(point_segment(a[i], b[j], b[(j + 1) % 4]))
                .min(point_segment(b[i], a[j], a[(j + 1) % 4]));
        }
    }
    distance
}
/// Exact surface separation for two upright yaw-oriented boxes (no roll/pitch).
fn upright_clearance(calibration: BodyCalibration, pose: Pose, obstacle: &StaticCuboid) -> f64 {
    let center = pose
        .position
        .plus(calibration.center_offset_body_m.rotated(pose.yaw));
    let horizontal = polygon_distance(
        vertices(
            center,
            Vec2::new(calibration.length_m / 2.0, calibration.width_m / 2.0),
            pose.yaw,
        ),
        vertices(
            Vec2::new(obstacle.center_m[0], obstacle.center_m[1]),
            Vec2::new(obstacle.half_extents_m[0], obstacle.half_extents_m[1]),
            obstacle.yaw_rad,
        ),
    );
    let vertical = (obstacle.center_m[2]
        - obstacle.half_extents_m[2]
        - calibration.bottom_m
        - calibration.height_m)
        .max(calibration.bottom_m - obstacle.center_m[2] - obstacle.half_extents_m[2])
        .max(0.0);
    horizontal.hypot(vertical)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn pose(x: f64, y: f64, yaw: f64) -> Pose {
        Pose {
            position: Vec2::new(x, y),
            yaw,
        }
    }
    fn obstacle(center: [f64; 3], half: [f64; 3], yaw: f64) -> StaticCuboid {
        StaticCuboid {
            id: "fixture".into(),
            center_m: center,
            half_extents_m: half,
            yaw_rad: yaw,
        }
    }
    #[test]
    fn authored_calibration_maps_to_identical_native_cuboid() {
        let c = BodyCalibration::default();
        c.validate().unwrap();
        assert!((c.corner_radius_m() - 2.2847319317591723).abs() < 1e-12);
        let collider = c.collider();
        assert_eq!(
            collider.shape,
            ColliderShape::Cuboid {
                half_extents_m: Vec3::new(2.1, 0.75, 0.9)
            }
        );
        assert!((collider.local_offset.translation.y - 0.3).abs() < 1e-12);
        assert!(
            BodyCalibration {
                length_m: f64::NAN,
                ..c
            }
            .validate()
            .is_err()
        );
    }
    #[test]
    fn oriented_body_geometry_has_analytic_axis_and_rotated_golden_distances() {
        let c = BodyCalibration::default();
        let box_at = obstacle([5.0, 0.0, 0.9], [1.0, 1.0, 0.75], 0.0);
        assert!((upright_clearance(c, pose(0.0, 0.0, 0.0), &box_at) - 1.9).abs() < 1e-12);
        assert!(
            (upright_clearance(c, pose(0.0, 0.0, std::f64::consts::FRAC_PI_2), &box_at) - 3.1)
                .abs()
                < 1e-12
        );
        let a = std::f64::consts::FRAC_PI_4;
        let direction = Vec2::new(1.0, 0.0).rotated(a);
        let rotated_box = obstacle(
            [5.0 * direction.x, 5.0 * direction.y, 0.9],
            [1.0, 1.0, 0.75],
            a,
        );
        assert!((upright_clearance(c, pose(0.0, 0.0, a), &rotated_box) - 1.9).abs() < 1e-12);
    }
    #[test]
    fn overhead_bridge_clearance_uses_true_body_top_not_capsule_radius() {
        let c = BodyCalibration::default();
        let bridge = obstacle([0.0, 0.0, 3.2], [5.0, 4.0, 0.5], 0.0);
        assert!((upright_clearance(c, pose(0.0, 0.0, 0.0), &bridge) - 1.05).abs() < 1e-12);
        let low = obstacle([0.0, 0.0, 2.5], [5.0, 4.0, 0.5], 0.0);
        assert!((upright_clearance(c, pose(0.0, 0.0, 0.0), &low) - 0.35).abs() < 1e-12);
    }
    #[test]
    fn rotation_sweep_guard_rejects_intermediate_corner_contact() {
        let c = BodyCalibration::default();
        let scene = Scene::from_json(&json!({"schema_version":1,"name":"swept rotation","static_cuboids":[{"id":"corner","center_m":[2.1,0.9,0.9],"half_extents_m":[0.005,0.005,0.6],"yaw_rad":0}]}).to_string()).unwrap();
        assert!(upright_clearance(c, pose(0.0, 0.0, -0.05), &scene.static_cuboids[0]) > 0.0);
        assert!(upright_clearance(c, pose(0.0, 0.0, 0.05), &scene.static_cuboids[0]) > 0.0);
        assert_eq!(
            upright_clearance(c, pose(0.0, 0.0, 0.0), &scene.static_cuboids[0]),
            0.0
        );
        let mut capture = BodyCapture::new(c, pose(0.0, 0.0, -0.05)).unwrap();
        capture.record(0.005, pose(0.0, 0.0, 0.05)).unwrap();
        let report = capture.evidence(&scene);
        assert_eq!(report["summary"]["passed"], false);
        assert_eq!(report["summary"]["guard_overlap_intervals"], 1);
    }
    #[test]
    fn invalid_native_substep_cannot_enter_body_evidence() {
        let c = BodyCalibration::default();
        let mut capture = BodyCapture::new(c, pose(0.0, 0.0, 0.0)).unwrap();
        assert!(capture.record(0.01, pose(0.0, 0.0, 0.0)).is_err());
        assert!(capture.record(0.005, pose(0.2, 0.0, 0.0)).is_err());
        assert!(capture.record(0.005, pose(0.0, 0.0, 0.11)).is_err());
        assert_eq!(capture.motion_samples.len(), 1);
    }
    #[test]
    fn actual_rapier_sensor_witness_keeps_native_pose_and_velocity_unchanged() {
        use rne_ecs::spawn_named;
        use rne_math::Seconds;
        use rne_physics::{PhysicsWorldDesc, RaycastQuery, RigidBodyType};
        let mut world = World::new();
        let ego = spawn_named(&mut world, "native-carrier");
        let obstacle = spawn_named(&mut world, "native-box");
        let transform =
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.6, 0.0), Quat::IDENTITY);
        let body = RigidBody {
            body_type: RigidBodyType::Dynamic,
            linear_velocity_m_s: Vec3::new(4.0, 0.0, 0.0),
            angular_velocity_rad_s: Vec3::new(0.0, 1.2, 0.0),
            ..RigidBody::default()
        };
        let mut collider = BodyCalibration::default().collider();
        collider.sensor = true;
        world.entity_mut(ego).insert((transform, body, collider));
        world.entity_mut(obstacle).insert((
            Transform3::from_translation_rotation(Vec3::new(2.0, 0.9, 0.0), Quat::IDENTITY),
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(0.3, 0.5, 0.3),
                },
                ..Collider::default()
            },
        ));
        let mut physics = RapierBackend::new();
        let world_id = physics
            .create_world(PhysicsWorldDesc {
                gravity_m_s2: Vec3::ZERO,
                ..PhysicsWorldDesc::default()
            })
            .unwrap();
        let report = sensor_witness(
            &mut physics,
            world_id,
            &mut world,
            ego,
            &[(obstacle, "native-box".into())],
            0.0,
            SimDuration::from_seconds(Seconds::new(0.005)),
        )
        .unwrap();
        assert_eq!(report["obstacle_ids"], json!(["native-box"]));
        assert_eq!(report["force_free"], true);
        assert_eq!(*world.get::<Transform3>(ego).unwrap(), transform);
        assert_eq!(*world.get::<RigidBody>(ego).unwrap(), body);
        let hit = physics
            .raycast(
                world_id,
                RaycastQuery {
                    origin_m: Vec3::new(-10.0, 0.9, 0.0),
                    direction: Vec3::X,
                    max_distance_m: 20.0,
                },
            )
            .unwrap()
            .into_iter()
            .find(|h| h.entity == ego)
            .unwrap();
        assert!((hit.distance_m - 7.9).abs() < 1e-6);
        assert!((hit.point_m.x + 2.1).abs() < 1e-6);
        assert_eq!(physics.contacts(world_id).unwrap()[0].impulse, 0.0);
    }
    fn fixture(goal: bool) -> rustdriving_sim::Scenario {
        let input = if goal {
            include_str!("../../../scenarios/native-scene-raised-goal.json")
        } else {
            include_str!("../../../scenarios/native-scene-low-stop.json")
        };
        let mut scenario: rustdriving_sim::Scenario = serde_json::from_str(input).unwrap();
        scenario.half_width = 3.0;
        scenario
    }
    fn road_scene(obstacle: Option<([f64; 3], [f64; 3])>) -> Scene {
        let boxes = obstacle.map(|(center,half)|json!({"id":"barrier","center_m":center,"half_extents_m":half,"yaw_rad":0.0})).into_iter().collect::<Vec<_>>();
        Scene::from_json(&json!({"schema_version":2,"name":"authored body research road","static_cuboids":boxes,"ground_cuboids":[{"id":"road","center_m":[40.0,0.0,-0.5],"half_extents_m":[60.0,25.0,0.5],"yaw_rad":0.0}]}).to_string()).unwrap()
    }
    #[test]
    fn actual_body_mode_drives_clear_and_overhead_roads_and_stops_for_low_and_mid_boxes() {
        for (object, goal) in [
            (None, true),
            (Some(([35.0, 0.0, 4.5], [0.5, 3.0, 1.0])), true),
            (Some(([35.0, 0.0, 0.1], [0.5, 3.0, 0.1])), false),
            (Some(([35.0, 0.0, 1.5], [0.5, 3.0, 0.1])), false),
        ] {
            let (run, evidence) = crate::run_with_scene_ground_body(
                fixture(goal),
                7,
                crate::Plant::Dynamic,
                road_scene(object),
            )
            .unwrap();
            assert!(run.summary.passed, "{object:?}: {:?}", run.summary);
            assert_eq!(run.summary.reached_goal, goal);
            assert_eq!(evidence["operating_mode"], "lidar3d_ground_body");
            assert_eq!(evidence["lidar3d"]["collision_bottom_m"], -0.85);
            assert_eq!(evidence["lidar3d"]["collision_top_m"], 2.65);
            assert!(
                (run.vehicle.radius - BodyCalibration::default().corner_radius_m()).abs() < 1e-12
            );
            assert_eq!(
                evidence["body_guard"]["summary"]["native_overlap_samples"],
                0
            );
            assert_eq!(
                evidence["body_guard"]["motion_samples"]
                    .as_array()
                    .unwrap()
                    .len(),
                (run.summary.steps - 1) * 10 + 1
            );
            assert_eq!(
                evidence["body_guard"]["rapier_sensor_witnesses"]
                    .as_array()
                    .unwrap()
                    .len(),
                (run.summary.steps - 1) * 10 + 1
            );
            if object.is_some() {
                assert!(evidence["summary"]["min_clearance_m"].as_f64().unwrap() >= 1.0);
            } else {
                assert_eq!(evidence["summary"]["min_clearance_m"], Value::Null);
                assert_eq!(evidence["summary"]["checks"], 0);
            }
            let mut bytes = vec![];
            run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
            rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap();
        }
    }
    #[test]
    fn ignoring_native_braking_cannot_hide_actual_cuboid_overlap() {
        use crate::{Plant, RneBackend};
        use rustdriving_core::{ControlCommand, EgoState};
        use rustdriving_pipeline::SensorFrame;
        use rustdriving_sim::{SimulationBackend, WorldObject, simulate_with_backend};
        struct IgnoreControl(RneBackend);
        impl SimulationBackend for IgnoreControl {
            fn state(&self) -> EgoState {
                self.0.state()
            }
            fn objects(&self, time: f64) -> Vec<WorldObject> {
                self.0.objects(time)
            }
            fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String> {
                self.0.observe(time, tick)
            }
            fn advance(&mut self, _: ControlCommand, dt: f64) -> Result<(), String> {
                self.0.advance(
                    ControlCommand {
                        acceleration: 2.0,
                        steering: 0.0,
                    },
                    dt,
                )
            }
        }
        let scenario = fixture(false);
        let backend = RneBackend::new_with_scene_ground_body(
            scenario.clone(),
            7,
            Plant::Dynamic,
            road_scene(Some(([35.0, 0.0, 1.5], [0.5, 3.0, 0.5]))),
        )
        .unwrap();
        let capture = backend.scene.as_ref().unwrap().clone();
        let config = backend.config();
        let run = simulate_with_backend(
            scenario,
            7,
            IgnoreControl(backend),
            config,
            "negative-native-body",
        )
        .unwrap();
        assert!(run.summary.emergency_steps > 0);
        let capture = capture.lock().unwrap();
        let report = capture.body.as_ref().unwrap().evidence(&capture.scene);
        assert_eq!(report["summary"]["passed"], false);
        assert!(
            report["summary"]["guard_overlap_intervals"]
                .as_u64()
                .unwrap()
                > 0
        );
        assert!(
            report["summary"]["native_overlap_samples"]
                .as_u64()
                .unwrap()
                > 0
        );
        assert_eq!(report["summary"]["min_clearance_m"], 0.0);
    }
}
