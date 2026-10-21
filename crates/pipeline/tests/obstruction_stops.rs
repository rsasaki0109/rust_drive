use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}
fn input(i: usize) -> SensorFrame {
    let time = i as f64 * 0.05;
    let failed = (10..13).contains(&i);
    SensorFrame {
        absolute_heading: None,
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: if i <= 1 { 8. } else { 0. },
            yaw_rate: 0.,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::new(if i == 0 { 0. } else { 0.4 }, 0.),
            variance: 0.02,
        }),
        lidar: (!failed).then(|| LidarScan {
            stamp: time,
            points: if i >= 30 {
                vec![]
            } else {
                (0..32)
                    .map(|j| {
                        let a = j as f64 * std::f64::consts::TAU / 32.;
                        Vec2::new(12. - if i == 0 { 0. } else { 0.4 } + a.cos(), a.sin())
                    })
                    .collect()
            },
        }),
        lidar_failed: failed,
        multi_height_lidar: None,
        lidar3d: None,
        navigation_update: None,
        traffic_signal: None,
    }
}

#[test]
fn measured_collision_stop_keeps_braking_through_sensor_fault_and_releases_after_the_path_clears() {
    let mut c = config();
    c.settle_obstruction_stops = true;
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("obstruction-stop-measurement-fixture", c);
    let mut released = false;
    for i in 0..=60 {
        let inp = input(i);
        let out = driver.step(&inp).unwrap();
        if i == 1 {
            assert!(out.emergency);
        }
        if (1..30).contains(&i) {
            assert!(out.command.acceleration <= -0.5);
        }
        if (10..13).contains(&i) {
            assert_eq!(out.command.acceleration, -6.);
        }
        if i >= 45 {
            released |= out.command.acceleration > 0.;
        }
        log.record(inp, out);
    }
    assert!(released);
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), vec![])
            .unwrap()
            .verified
    );
    log.header.config.settle_obstruction_stops = false;
    let mut changed = vec![];
    log.write(&mut changed).unwrap();
    assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
}

#[test]
fn omitted_policy_preserves_the_legacy_header_and_rejects_a_nonboolean_value() {
    let mut value = serde_json::to_value(config()).unwrap();
    assert!(value.get("settle_obstruction_stops").is_none());
    value["settle_obstruction_stops"] = serde_json::json!(1);
    assert!(serde_json::from_value::<PipelineConfig>(value).is_err());
}

#[test]
fn stop_search_requires_retention_and_preserves_the_default_header() {
    let mut c = config();
    assert!(
        serde_json::to_value(&c)
            .unwrap()
            .get("search_obstruction_stops")
            .is_none()
    );
    c.search_obstruction_stops = true;
    assert!(c.validate().is_err());
    c.settle_obstruction_stops = true;
    assert!(c.validate().is_ok());
    let mut value = serde_json::to_value(c).unwrap();
    assert_eq!(value["search_obstruction_stops"], true);
    value["search_obstruction_stops"] = serde_json::json!(1);
    assert!(serde_json::from_value::<PipelineConfig>(value).is_err());
}
