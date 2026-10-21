use rustdriving_core::{
    Gnss, LidarScan, Odometry, Pose, Route, RouteSpeedZone, Vec2, VehicleConfig,
};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.speed_zones = vec![RouteSpeedZone {
        start_s_m: 20.,
        end_s_m: 30.,
        speed_m_s: 2.,
    }];
    c
}
#[test]
fn mapped_speed_zones_are_recorded_and_recomputed_from_sensor_only_logs() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("fixed-route-speed-zone", c);
    for i in 0..=4 {
        let time = i as f64 * 0.05;
        let input = SensorFrame {
            absolute_heading: None,
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
                sweep: None,
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
        assert!(
            output
                .trajectory
                .points
                .iter()
                .filter(|p| (20.0..=30.0).contains(&p.position.x))
                .all(|p| p.speed <= 2. + 1e-10)
        );
        log.record(input, output);
    }
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
#[test]
fn invalid_and_out_of_route_limits_are_rejected_before_driving() {
    for bad in [
        RouteSpeedZone {
            start_s_m: 90.,
            end_s_m: 101.,
            speed_m_s: 2.,
        },
        RouteSpeedZone {
            start_s_m: 10.,
            end_s_m: 10.,
            speed_m_s: 2.,
        },
        RouteSpeedZone {
            start_s_m: 10.,
            end_s_m: 20.,
            speed_m_s: f64::INFINITY,
        },
    ] {
        let mut c = config();
        c.speed_zones = vec![bad];
        assert!(DrivingPipeline::new(c).is_err());
    }
    let mut c = config();
    c.speed_zones = vec![c.speed_zones[0]; 129];
    assert!(DrivingPipeline::new(c).is_err());
}
