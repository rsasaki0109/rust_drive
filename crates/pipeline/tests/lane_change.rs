use rustdriving_core::*;
use rustdriving_pipeline::lane_change::{LaneChangeConfig, LaneChangePhase};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, MotionLimits, PipelineConfig, SensorFrame};
use rustdriving_routing::lanes::{LaneChangeRequest, LaneMapSpec, LaneNetwork, LaneSide};

fn config() -> PipelineConfig {
    let map: LaneMapSpec =
        serde_json::from_str(include_str!("../../../maps/lanes/two-lane-change.json")).unwrap();
    let network = LaneNetwork::new(map).unwrap();
    let plan = network
        .lane_change_route(
            "source",
            "target-exit",
            &[],
            LaneChangeRequest {
                side: LaneSide::Left,
                start_m: 25.0,
                length_m: 30.0,
            },
            VehicleConfig::default(),
        )
        .unwrap()
        .plan;
    let exit_s_m = plan.route.project(Vec2::new(55.0, 3.5)).0;
    let mut c = PipelineConfig::new(plan.route, Pose::default(), VehicleConfig::default());
    c.cruise_speed = 2.0;
    c.local_route_geometry = true;
    c.motion_limits = Some(MotionLimits {
        max_acceleration_m_s2: 1.0,
        max_deceleration_m_s2: 2.5,
        max_lateral_acceleration_m_s2: 1.0,
    });
    c.lane_change = Some(LaneChangeConfig {
        entry_s_m: 25.0,
        exit_s_m,
        target_start: Vec2::new(0.0, 3.5),
        target_end: Vec2::new(80.0, 3.5),
        target_width_m: 3.5,
    });
    c
}
fn frame(time: f64) -> SensorFrame {
    SensorFrame {
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 0.0,
            yaw_rate: 0.0,
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
    }
}

#[test]
fn sensed_neighbor_revokes_ready_and_replay_recomputes_the_gap_gate() {
    let c = config();
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("observed-lane-gap", c);
    for i in 0..=28 {
        let mut input = frame(i as f64 * 0.05);
        if i % 2 != 0 {
            input.lidar = None;
        }
        if i >= 24 && i % 2 == 0 {
            input.lidar.as_mut().unwrap().points = (0..24)
                .map(|j| {
                    let a = j as f64 * std::f64::consts::TAU / 24.0;
                    Vec2::new(12.0 + a.cos(), 3.5 + a.sin())
                })
                .collect();
        }
        let output = pipeline.step(&input).unwrap();
        assert!(!output.emergency);
        let state = output.lane_change.as_ref().unwrap();
        if !(20..24).contains(&i) {
            assert_eq!(state.phase, LaneChangePhase::Waiting);
            assert!(output.trajectory.points.iter().all(|p| p.position.x < 25.0));
        } else {
            assert_eq!(state.phase, LaneChangePhase::Ready);
        }
        if i >= 24 {
            assert!(!state.blocking_tracks.is_empty());
            assert!(state.front_gap_m.is_some());
        }
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 29);
}

#[test]
fn cached_scans_faults_and_delayed_post_fault_acquisitions_cannot_earn_permission() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..=22 {
        let mut input = frame(i as f64 * 0.05);
        input.lidar.as_mut().unwrap().stamp = 0.0;
        let output = pipeline.step(&input).unwrap();
        assert_eq!(output.lane_change.unwrap().phase, LaneChangePhase::Waiting);
    }
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..=38 {
        let now = i as f64 * 0.05;
        let mut input = frame(now);
        if i == 10 {
            input.lidar = None;
            input.lidar_failed = true;
        }
        if (11..=14).contains(&i) {
            input.lidar.as_mut().unwrap().stamp = now - 0.2;
        }
        let output = pipeline.step(&input).unwrap();
        let status = output.lane_change.unwrap();
        if i < 35 {
            assert_eq!(status.phase, LaneChangePhase::Waiting, "tick {i}");
        }
        if i >= 35 {
            assert_eq!(status.phase, LaneChangePhase::Ready, "tick {i}");
        }
    }
}

#[test]
fn gate_rejects_mismatched_geometry_and_invalid_arc_metadata_without_panics() {
    for case in 0..10 {
        let mut c = config();
        match case {
            0 => c.route.lengths.clear(),
            1 => c.lane_change.as_mut().unwrap().target_end.x = f64::INFINITY,
            2 => c.lane_change.as_mut().unwrap().entry_s_m = 0.0,
            3 => c.lane_change.as_mut().unwrap().exit_s_m = 30.0,
            4 => c.lane_change.as_mut().unwrap().target_start.y = 10.0,
            5 => c.lane_change.as_mut().unwrap().target_width_m = 1.0,
            6 => c.local_route_geometry = false,
            7 => c.cruise_speed = 5.0,
            8 => c.initial_pose.position = Vec2::new(25.0, 0.0),
            _ => c.lane_change.as_mut().unwrap().target_end.x = -80.0,
        }
        assert!(DrivingPipeline::new(c).is_err(), "case {case}");
    }
}
