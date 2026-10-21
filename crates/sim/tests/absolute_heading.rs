use rustdriving_sim::{
    Scenario,
    heading::{HeadingBiasWindow, HeadingSensor},
    pipeline_config,
    sensor_timing::SensorFailureWindow,
};

fn scenario() -> Scenario {
    serde_json::from_str(include_str!("../../../scenarios/blocked.json")).unwrap()
}
fn sensor() -> HeadingSensor {
    HeadingSensor {
        period_ticks: 4,
        variance_rad2: 1e-4,
        delay_ticks: 0,
        dropout_windows: vec![],
        bias_windows: vec![],
    }
}

#[test]
fn physical_sensor_parameters_are_independent_of_operational_calibration() {
    let mut s = scenario();
    assert!(s.heading_observation(7, 0, 0., 0.).is_none());
    s.heading_sensor = Some(sensor());
    assert!(!pipeline_config(&s).absolute_heading_fusion);
    assert!(
        serde_json::to_value(pipeline_config(&s))
            .unwrap()
            .get("heading_sensor")
            .is_none()
    );
    s.absolute_heading_fusion = true;
    s.validate().unwrap();
    let a = s.heading_observation(7, 4, 0.2, 0.3).unwrap();
    s.objects.clear();
    s.gnss_dropout = Some(0.);
    let b = s.heading_observation(7, 4, 0.2, 0.3).unwrap();
    assert_eq!(a.yaw_rad, b.yaw_rad);
    assert_eq!(a.stamp, 0.2);
    assert_eq!(a.variance_rad2, 1e-4);
    assert_ne!(
        a.yaw_rad,
        s.heading_observation(42, 4, 0.2, 0.3).unwrap().yaw_rad
    );
    assert!(s.heading_observation(7, 3, 0.15, 0.3).is_none());
}

#[test]
fn declared_uniform_variance_and_half_open_fault_windows_match_acquisitions() {
    let mut s = scenario();
    s.heading_sensor = Some(sensor());
    let mut mean = 0.;
    let mut moment = 0.;
    let count = 100_000;
    for i in 0..count {
        let yaw = s
            .heading_observation(313, i * 4, i as f64 * 0.2, 0.)
            .unwrap()
            .yaw_rad;
        mean += yaw / count as f64;
        moment += yaw * yaw / count as f64;
    }
    assert!(mean.abs() < 2e-4);
    assert!((moment - 1e-4).abs() < 3e-6);
    s.heading_sensor.as_mut().unwrap().dropout_windows = vec![SensorFailureWindow {
        from: 1.,
        until: 2.,
    }];
    s.heading_sensor.as_mut().unwrap().bias_windows = vec![HeadingBiasWindow {
        from: 2.,
        until: 3.,
        offset_rad: 0.8,
    }];
    s.validate().unwrap();
    assert!(s.heading_observation(7, 20, 1., 0.).is_none());
    assert!(s.heading_observation(7, 40, 2., 0.).unwrap().yaw_rad > 0.78);
    assert!(s.heading_observation(7, 60, 3., 0.).unwrap().yaw_rad.abs() < 0.02);
}

#[test]
fn invalid_sensor_period_noise_and_fault_windows_are_rejected_before_driving() {
    let mut delayed = scenario();
    delayed.heading_sensor = Some(HeadingSensor {
        delay_ticks: 21,
        ..sensor()
    });
    assert!(delayed.validate().is_err());
    assert!(
        serde_json::to_value(sensor())
            .unwrap()
            .get("delay_ticks")
            .is_none()
    );
    for period in [0, 8] {
        let mut s = scenario();
        s.heading_sensor = Some(HeadingSensor {
            period_ticks: period,
            ..sensor()
        });
        assert!(s.validate().is_err());
    }
    for variance in [0., -1., f64::NAN, f64::INFINITY, 10.] {
        let mut s = scenario();
        s.heading_sensor = Some(HeadingSensor {
            variance_rad2: variance,
            ..sensor()
        });
        assert!(s.validate().is_err());
    }
    let mut s = scenario();
    s.heading_sensor = Some(sensor());
    s.heading_sensor.as_mut().unwrap().dropout_windows = vec![
        SensorFailureWindow {
            from: 1.,
            until: 3.,
        },
        SensorFailureWindow {
            from: 2.,
            until: 4.,
        },
    ];
    assert!(s.validate().is_err());
    s.heading_sensor.as_mut().unwrap().dropout_windows.clear();
    s.heading_sensor.as_mut().unwrap().bias_windows = vec![HeadingBiasWindow {
        from: 0.,
        until: 1.,
        offset_rad: 4.,
    }];
    assert!(s.validate().is_err());
}

#[test]
fn delayed_transport_uses_acquired_body_heading_and_keeps_schedule_out_of_runtime() {
    let mut s = scenario();
    s.duration = 1.;
    s.full_rate_recording = true;
    s.absolute_heading_fusion = true;
    s.gnss_latency_compensation = true;
    s.heading_latency_compensation = true;
    s.heading_sensor = Some(HeadingSensor {
        delay_ticks: 4,
        ..sensor()
    });
    let run = rustdriving_sim::simulate(s.clone(), 7).unwrap();
    let log = run.sensor_log.unwrap();
    assert!(log.header.config.heading_latency_compensation);
    assert!(
        !serde_json::to_string(&log.header.config)
            .unwrap()
            .contains("delay_ticks")
    );
    for (i, tick) in log.ticks.iter().enumerate() {
        if i >= 4 && (i - 4) % 4 == 0 {
            let acquisition = &run.frames[i - 4];
            let expected = s
                .heading_observation(7, i - 4, acquisition.time, acquisition.truth.pose.yaw)
                .unwrap();
            let sample = tick.input.absolute_heading.unwrap();
            assert_eq!(sample.stamp, expected.stamp);
            assert_eq!(sample.yaw_rad, expected.yaw_rad);
        } else {
            assert!(tick.input.absolute_heading.is_none());
        }
    }
}
