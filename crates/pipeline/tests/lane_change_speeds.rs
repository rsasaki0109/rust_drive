use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Vec2, VehicleConfig};
use rustdriving_pipeline::lane_change::{LaneChangeConfig, LaneChangeReturnConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, MotionLimits, PipelineConfig, SensorFrame};
use rustdriving_routing::lanes::{LaneChangeRequest, LaneNetwork, LaneSide};

fn config() -> PipelineConfig {
    let network = LaneNetwork::new(
        serde_json::from_str(include_str!(
            "../../../maps/lanes/change-speeds-target-slow.json"
        ))
        .unwrap(),
    )
    .unwrap();
    let compiled = network
        .lane_change_route_with_speed_zones(
            "source",
            "target-exit",
            &[],
            LaneChangeRequest {
                side: LaneSide::Left,
                start_m: 25.,
                length_m: 30.,
            },
            VehicleConfig::default(),
        )
        .unwrap();
    let route = compiled.change.plan.route;
    let exit_s_m = route.project(Vec2::new(55., 3.5)).0;
    let mut config = PipelineConfig::new(route, Pose::default(), VehicleConfig::default());
    config.cruise_speed = 4.;
    config.local_route_geometry = true;
    config.motion_limits = Some(MotionLimits {
        max_acceleration_m_s2: 1.,
        max_deceleration_m_s2: 2.5,
        max_lateral_acceleration_m_s2: 1.,
    });
    config.speed_zones = compiled.speed_zones;
    config.lane_change = Some(LaneChangeConfig {
        entry_s_m: 25.,
        exit_s_m,
        target_start: Vec2::new(0., 3.5),
        target_end: Vec2::new(80., 3.5),
        target_width_m: 3.5,
        return_on_conflict: None,
    });
    config
}

#[test]
fn alternate_speed_policy_is_required_and_cannot_be_reused_for_a_mission_retry() {
    let mut config = config();
    assert!(config.validate().is_ok());
    config.lane_change.as_mut().unwrap().return_on_conflict = Some(LaneChangeReturnConfig {
        source_start: Vec2::default(),
        source_end: Vec2::new(80., 0.),
        source_width_m: 3.5,
        max_return_distance_m: 30.,
        speed_limit_m_s: None,
        resume_after_return: None,
    });
    assert!(config.validate().unwrap_err().contains("alternate-route"));
    config
        .lane_change
        .as_mut()
        .unwrap()
        .return_on_conflict
        .as_mut()
        .unwrap()
        .speed_limit_m_s = Some(2.);
    assert!(config.validate().is_ok());
    for speed in [0., 8.01, f64::NAN, f64::INFINITY] {
        let mut invalid = config.clone();
        invalid
            .lane_change
            .as_mut()
            .unwrap()
            .return_on_conflict
            .as_mut()
            .unwrap()
            .speed_limit_m_s = Some(speed);
        assert!(invalid.validate().is_err());
    }
    let mut retry = config.clone();
    retry
        .lane_change
        .as_mut()
        .unwrap()
        .return_on_conflict
        .as_mut()
        .unwrap()
        .resume_after_return = Some(rustdriving_pipeline::lane_change::LaneChangeResumeConfig {
        max_change_distance_m: 30.,
    });
    assert!(retry.validate().is_err());
    config.speed_zones.clear();
    config
        .lane_change
        .as_mut()
        .unwrap()
        .return_on_conflict
        .as_mut()
        .unwrap()
        .speed_limit_m_s = None;
    assert!(config.validate().is_ok());
}

#[test]
fn admission_prefix_and_release_replay_the_same_full_route_speed_policy() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("fixed-change-speeds", c);
    let mut waiting = false;
    let mut released = false;
    for i in 0..=28 {
        let time = i as f64 * 0.05;
        let input = SensorFrame {
            time,
            odometry: Some(Odometry {
                stamp: time,
                speed: 0.,
                yaw_rate: 0.,
            }),
            gnss: Some(Gnss {
                stamp: time,
                position: Vec2::default(),
                variance: 0.02,
            }),
            lidar: Some(LidarScan {
                stamp: time,
                points: vec![],
            }),
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        };
        let output = driver.step(&input).unwrap();
        assert!(!output.emergency);
        let far = output.trajectory.points.iter().any(|p| p.position.x > 30.);
        waiting |= !far;
        released |= far;
        for p in &output.trajectory.points {
            if (30.0..=70.0).contains(&p.position.x) {
                assert!(p.speed <= 2. + 1e-9);
            }
        }
        log.record(input, output);
    }
    assert!(waiting && released);
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), vec![])
            .unwrap()
            .verified
    );
    log.header.config.speed_zones.clear();
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(verify(std::io::Cursor::new(bytes), vec![]).is_err());
}
