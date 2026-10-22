use rustdriving_core::{Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.fit_sparse_circles = true;
    c.enclose_track_measurements = true;
    c
}

fn frame(time: f64, count: usize) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":time,"odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":0.},"variance":0.02},
        "lidar":{"stamp":time,"points":(0..count).map(|i|{
            let a=std::f64::consts::PI-0.5+i as f64/(count-1) as f64;
            Vec2::new(40.+a.cos(),a.sin())
        }).collect::<Vec<_>>()}
    }))
    .unwrap()
}

#[test]
fn sparse_to_dense_acquisitions_preserve_the_stationary_center_and_velocity() {
    let c = config();
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("sparse-circular-acquisition", c);
    for (i, count) in [3, 4, 5, 8].into_iter().enumerate() {
        let input = frame(i as f64 * 0.1, count);
        let output = pipeline.step(&input).unwrap();
        let track = &output.tracks[0];
        assert!(track.position.distance(Vec2::new(40., 0.)) < 1e-8);
        assert!(track.velocity.x.hypot(track.velocity.y) < 1e-8);
        assert_eq!(track.id, 1);
        assert_eq!(track.last_seen, input.time);
        log.record(input, output);
    }
    let check = |log: &SensorLog| {
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        verify(std::io::Cursor::new(bytes), vec![]).is_ok_and(|r| r.verified)
    };
    assert!(check(&log));
    log.header.config.fit_sparse_circles = false;
    assert!(!check(&log));
    log.header.config.fit_sparse_circles = true;
    log.ticks[1].expected.track_measurements.as_mut().unwrap()[0]
        .center
        .x -= 0.5;
    assert!(!check(&log));
}

#[test]
fn omitted_and_explicitly_disabled_policies_preserve_serialized_configuration() {
    let mut c = config();
    c.fit_sparse_circles = false;
    let value = serde_json::to_value(&c).unwrap();
    assert!(value.get("fit_sparse_circles").is_none());
    let decoded: PipelineConfig = serde_json::from_value(value).unwrap();
    assert!(!decoded.fit_sparse_circles);
}

#[test]
fn a_delayed_sparse_scan_uses_its_acquisition_pose_while_the_ego_turns() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    let center = Vec2::new(40., 3.);
    for i in 0..=6 {
        let mut input = frame(i as f64 * 0.05, 3);
        input.lidar = None;
        let motion = input.odometry.as_mut().unwrap();
        motion.speed = 4.;
        motion.yaw_rate = 0.8;
        if i > 0 {
            input.gnss = None;
        }
        if i >= 2 && i % 2 == 0 {
            let acquired_tick = i - 2;
            let mut acquired_position = Vec2::default();
            for j in 0..acquired_tick {
                let yaw = j as f64 * 0.04;
                acquired_position =
                    acquired_position.plus(Vec2::new(yaw.cos(), yaw.sin()).scaled(0.2));
            }
            input.lidar = Some(rustdriving_core::LidarScan {
                stamp: acquired_tick as f64 * 0.05,
                sweep: None,
                points: (0..3)
                    .map(|j| {
                        let a = std::f64::consts::PI - 0.5 + j as f64 * 0.5;
                        center
                            .plus(Vec2::new(a.cos(), a.sin()))
                            .minus(acquired_position)
                            .rotated(-(acquired_tick as f64 * 0.04))
                    })
                    .collect(),
            });
        }
        let out = driver.step(&input).unwrap();
        if i >= 2 {
            assert!(out.health.is_empty());
            assert!(out.tracks[0].position.distance(center) < 1e-8);
            assert!(out.tracks[0].velocity.x.hypot(out.tracks[0].velocity.y) < 1e-8);
            assert!(out.tracks[0].last_seen <= input.time - 0.1 + 1e-9);
        }
    }
}
