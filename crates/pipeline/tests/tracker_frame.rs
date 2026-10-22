use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, PipelineOutput, SensorFrame};

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(150., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}

// An independently specified straight 2 m/s sensor trajectory. The stack only
// receives these wheel/gyro, position-fix and body-point observations.
fn frame(time: f64, stamp: f64, biased: bool, moving: bool) -> SensorFrame {
    let center = Vec2::new(
        40. + if moving { stamp } else { 0. } - 2. * stamp,
        3. + if moving { 0.3 * stamp } else { 0. },
    );
    SensorFrame {
        absolute_heading: None,
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 2.,
            yaw_rate: 0.,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::new(
                2. * time,
                if biased && time >= 0.5 {
                    if ((time * 10.).round() as usize).is_multiple_of(2) {
                        0.25
                    } else {
                        -0.25
                    }
                } else {
                    0.
                },
            ),
            variance: 0.02,
        }),
        lidar: Some(LidarScan {
            sweep: None,
            stamp,
            points: (0..32)
                .map(|i| {
                    let a = i as f64 * std::f64::consts::TAU / 32.;
                    center.plus(Vec2::new(0.5 * a.cos(), 0.5 * a.sin()))
                })
                .collect(),
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}

fn close(a: Vec2, b: Vec2) {
    assert!(a.distance(b) < 1e-7, "{a:?} differs from {b:?}");
}

fn in_body(output: &PipelineOutput, p: Vec2) -> Vec2 {
    p.minus(output.estimate.pose.position)
        .rotated(-output.estimate.pose.yaw)
}

#[test]
fn position_fix_corrections_cannot_turn_static_returns_into_object_velocity() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let mut maximum_heading_correction = 0.0_f64;
    for i in 0..=40 {
        let time = i as f64 * 0.1;
        let output = pipeline.step(&frame(time, time, true, false)).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(output.tracks.len(), 1);
        let track = &output.tracks[0];
        assert_eq!((track.id, track.last_seen), (1, time));
        maximum_heading_correction = maximum_heading_correction.max(output.estimate.pose.yaw.abs());
        let transform = output.tracking_frame.unwrap();
        close(
            transform.to_world(Vec2::new(2. * time, 0.)),
            output.estimate.pose.position,
        );
        close(track.velocity, Vec2::default());
        close(
            in_body(&output, track.position),
            Vec2::new(40. - 2. * time, 3.),
        );
    }
    assert!(maximum_heading_correction > 0.01);
}

#[test]
fn delayed_moving_returns_and_forecasts_are_invariant_to_position_fix_corrections() {
    let config = config();
    let mut clean = DrivingPipeline::new(config.clone()).unwrap();
    let mut corrected = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("tracking-frame-delayed-moving-circle", config);
    for i in 0..=40 {
        let time = i as f64 * 0.1;
        // A delayed, off-grid acquisition needs pose-history interpolation.
        let stamp = if i == 0 { 0. } else { time - 0.075 };
        let a = clean.step(&frame(time, stamp, false, true)).unwrap();
        let input = frame(time, stamp, true, true);
        let b = corrected.step(&input).unwrap();
        assert!(a.health.is_empty() && b.health.is_empty());
        assert_eq!((a.tracks.len(), b.tracks.len()), (1, 1));
        assert_eq!((a.tracks[0].id, b.tracks[0].id), (1, 1));
        assert_eq!(b.tracks[0].last_seen, stamp);
        close(
            in_body(&a, a.tracks[0].position),
            in_body(&b, b.tracks[0].position),
        );
        close(
            a.tracks[0].velocity.rotated(-a.estimate.pose.yaw),
            b.tracks[0].velocity.rotated(-b.estimate.pose.yaw),
        );
        for (pa, pb) in a.predictions[0]
            .positions
            .iter()
            .zip(&b.predictions[0].positions)
        {
            close(in_body(&a, *pa), in_body(&b, *pb));
        }
        log.record(input, b);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 41);
}

#[test]
fn missing_or_rejected_scans_reproject_cached_tracks_without_renewing_observations() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..=10 {
        let time = i as f64 * 0.1;
        let mut input = frame(time, time, true, false);
        if i == 5 {
            input.lidar = None;
        } else if i == 6 {
            input.lidar.as_mut().unwrap().points[0].x = f64::NAN;
        }
        let output = pipeline.step(&input).unwrap();
        assert_eq!(output.tracks.len(), 1);
        let track = &output.tracks[0];
        assert_eq!(track.id, 1);
        close(track.velocity, Vec2::default());
        if i == 5 || i == 6 {
            assert_eq!(track.last_seen, 0.4);
        } else {
            assert_eq!(track.last_seen, time);
        }
        close(
            in_body(&output, track.position),
            Vec2::new(40. - 2. * time, 3.),
        );
        if i == 6 {
            assert!(output.emergency);
            assert_eq!(output.command.acceleration, -6.);
        }
    }
}

#[test]
fn real_turns_and_chassis_lateral_motion_survive_localization_heading_wraps() {
    for offset in [None, Some(1.)] {
        let mut config = config();
        config.initial_pose.yaw = 3.05;
        config.rear_axle_offset_m = offset;
        config.local_route_geometry = offset.is_some();
        config.motion_limits = Some(rustdriving_pipeline::MotionLimits {
            max_acceleration_m_s2: 1.,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: 1.,
        });
        let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
        let mut log = SensorLog::new("turning-tracking-frame", config.clone());
        let mut measured_pose = config.initial_pose;
        let target = Vec2::new(-40., 2.);
        for i in 0..=20 {
            let time = i as f64 * 0.1;
            if i > 0 {
                let yaw = measured_pose.yaw + if offset.is_some() { 0.0075 } else { 0. };
                measured_pose.position = measured_pose.position.plus(
                    Vec2::new(2., offset.unwrap_or(0.) * 0.15)
                        .rotated(yaw)
                        .scaled(0.1),
                );
                measured_pose.yaw = rustdriving_core::wrap_angle(measured_pose.yaw + 0.015);
            }
            let mut input = frame(time, time, true, false);
            input.odometry.as_mut().unwrap().yaw_rate = 0.15;
            let bias = input.gnss.unwrap().position.y;
            input.gnss.as_mut().unwrap().position =
                measured_pose.position.plus(Vec2::new(0., bias));
            input.lidar.as_mut().unwrap().points = (0..32)
                .map(|j| {
                    let angle = j as f64 * std::f64::consts::TAU / 32.;
                    target
                        .plus(Vec2::new(0.5 * angle.cos(), 0.5 * angle.sin()))
                        .minus(measured_pose.position)
                        .rotated(-measured_pose.yaw)
                })
                .collect();
            let output = pipeline.step(&input).unwrap();
            assert_eq!(output.tracks.len(), 1);
            close(output.tracks[0].velocity, Vec2::default());
            close(
                in_body(&output, output.tracks[0].position),
                target
                    .minus(measured_pose.position)
                    .rotated(-measured_pose.yaw),
            );
            close(
                output
                    .tracking_frame
                    .unwrap()
                    .to_world(measured_pose.position),
                output.estimate.pose.position,
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
    }
}

#[test]
fn sustained_braking_history_remains_in_one_frame_across_gnss_corrections() {
    let mut clean = DrivingPipeline::new(config()).unwrap();
    let mut corrected = DrivingPipeline::new(config()).unwrap();
    let mut braking_forecasts = 0;
    for i in 0..=40 {
        let time = i as f64 * 0.1;
        let mut input = frame(time, time, false, true);
        let extra = Vec2::new(
            3. * time - 0.5 * time * time,
            0.3 * (3. * time - 0.5 * time * time),
        );
        for point in &mut input.lidar.as_mut().unwrap().points {
            *point = point.plus(extra);
        }
        let a = clean.step(&input).unwrap();
        input.gnss = frame(time, time, true, true).gnss;
        let b = corrected.step(&input).unwrap();
        assert_eq!((a.tracks.len(), b.tracks.len()), (1, 1));
        for (pa, pb) in a.predictions[0]
            .positions
            .iter()
            .zip(&b.predictions[0].positions)
        {
            close(in_body(&a, *pa), in_body(&b, *pb));
        }
        let speed = a.tracks[0].velocity.x.hypot(a.tracks[0].velocity.y);
        if speed >= 0.7
            && a.predictions[0].positions[5].distance(a.tracks[0].position) < speed - 0.05
        {
            braking_forecasts += 1;
        }
    }
    assert!(braking_forecasts >= 3);
}
