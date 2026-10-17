use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame,
    replay::{LogRecord, SensorLog, verify},
};
use std::io::Cursor;
fn recorded_log() -> SensorLog {
    let config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("sensor-only-test", config);
    for i in 0..6 {
        let time = i as f64 * 0.05;
        let input = SensorFrame {
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
        };
        let output = pipeline.step(&input).unwrap();
        log.record(input, output);
    }
    log
}
fn log() -> Vec<u8> {
    let mut bytes = vec![];
    recorded_log().write(&mut bytes).unwrap();
    bytes
}
fn recompute(log: &mut SensorLog) {
    let mut pipeline = DrivingPipeline::new(log.header.config.clone()).unwrap();
    for tick in &mut log.ticks {
        tick.expected = pipeline.step(&tick.input).unwrap();
    }
}
#[test]
fn replay_recomputes_outputs_from_sensor_only_log() {
    let mut outputs = vec![];
    let result = verify(Cursor::new(log()), &mut outputs).unwrap();
    assert_eq!(result.ticks, 6);
    assert_eq!(
        outputs
            .split(|b| *b == b'\n')
            .filter(|l| !l.is_empty())
            .count(),
        6
    );
}
#[test]
fn changed_command_is_detected() {
    let lines = String::from_utf8(log()).unwrap();
    let mut records: Vec<serde_json::Value> = lines
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    records[2]["tick"]["expected"]["command"]["acceleration"] = serde_json::json!(-5.0);
    let edited = records
        .iter()
        .map(|r| serde_json::to_string(r).unwrap() + "\n")
        .collect::<String>();
    assert!(
        verify(Cursor::new(edited), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
#[test]
fn gnss_input_and_correction_diagnostics_are_both_recomputed() {
    for edit_input in [true, false] {
        let lines = String::from_utf8(log()).unwrap();
        let mut records: Vec<serde_json::Value> = lines
            .lines()
            .map(|s| serde_json::from_str(s).unwrap())
            .collect();
        if edit_input {
            records[2]["tick"]["input"]["gnss"]["position"]["x"] = serde_json::json!(30.0);
        } else {
            records[2]["tick"]["expected"]["localization"]["accepted_fixes"] =
                serde_json::json!(999);
        }
        let edited = records
            .iter()
            .map(|r| serde_json::to_string(r).unwrap() + "\n")
            .collect::<String>();
        assert!(
            verify(Cursor::new(edited), std::io::sink())
                .unwrap_err()
                .contains("mismatch")
        );
    }
}

#[test]
fn changing_a_map_snapshot_is_detected_by_full_recomputation() {
    let scenario: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/route-handover.json")).unwrap();
    let nav: rustdriving_pipeline::navigation::NavigationConfig =
        serde_json::from_value(scenario["navigation"].clone()).unwrap();
    let mut config = PipelineConfig::new(
        nav.initial_plan().unwrap().route,
        Pose::default(),
        VehicleConfig::default(),
    );
    config.navigation = Some(nav);
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("navigation-replay", config);
    for i in 0..8 {
        let time = i as f64 * 0.05;
        let mut input = SensorFrame {
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
        };
        if i == 1 {
            input.navigation_update = Some(rustdriving_pipeline::navigation::NavigationUpdate {
                stamp: time,
                revision: 1,
                closed_edges: vec!["main".into()],
            });
        }
        let output = pipeline.step(&input).unwrap();
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(Cursor::new(bytes), std::io::sink()).unwrap().ticks,
        8
    );
    log.ticks[1]
        .input
        .navigation_update
        .as_mut()
        .unwrap()
        .closed_edges
        .clear();
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
#[test]
fn empty_and_truncated_logs_cannot_claim_verified() {
    assert!(verify(Cursor::new(vec![]), std::io::sink()).is_err());
    let text = String::from_utf8(log()).unwrap();
    let lines: Vec<_> = text.lines().collect();
    let truncated = lines[..lines.len() - 1].join("\n");
    assert!(
        verify(Cursor::new(truncated), std::io::sink())
            .unwrap_err()
            .contains("footer")
    );
    let removed = lines
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != 3)
        .map(|(_, s)| *s)
        .collect::<Vec<_>>()
        .join("\n");
    assert!(verify(Cursor::new(removed), std::io::sink()).is_err());
}
#[test]
fn wrong_schema_and_hidden_truth_fields_are_rejected() {
    let original = String::from_utf8(log()).unwrap();
    assert!(
        verify(
            Cursor::new(original.replace("\"schema_version\":1", "\"schema_version\":999")),
            std::io::sink()
        )
        .is_err()
    );
    assert!(
        verify(
            Cursor::new(original.replace("\"odometry\":", "\"truth\":{},\"odometry\":")),
            std::io::sink()
        )
        .is_err()
    );
}
#[test]
fn writer_failures_are_reported() {
    struct Broken;
    impl std::io::Write for Broken {
        fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
            Err(std::io::Error::other("disk unavailable"))
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    assert!(
        verify(Cursor::new(log()), Broken)
            .unwrap_err()
            .contains("disk unavailable")
    );
}

#[test]
fn oversized_header_is_rejected_without_partial_output() {
    let mut log = recorded_log();
    log.header.source = "x".repeat(8 * 1024 * 1024);
    let mut written = Vec::new();
    assert!(log.write(&mut written).unwrap_err().contains("size limit"));
    assert!(written.is_empty());
}

#[test]
fn oversized_tick_retains_only_preceding_complete_records() {
    let mut log = recorded_log();
    let point = log.ticks[0].expected.trajectory.points[0];
    log.ticks[0]
        .expected
        .trajectory
        .points
        .resize(200_000, point);
    let mut written = Vec::new();
    assert!(log.write(&mut written).unwrap_err().contains("size limit"));
    assert_eq!(written.iter().filter(|&&b| b == b'\n').count(), 1);
    assert!(matches!(
        serde_json::from_slice::<LogRecord>(&written).unwrap(),
        LogRecord::Header { .. }
    ));
    assert!(verify(Cursor::new(written), std::io::sink()).is_err());
}

#[test]
fn encoded_line_limit_includes_newline_and_json_escaping() {
    const LIMIT: usize = 8 * 1024 * 1024;
    let mut log = recorded_log();
    log.header.source.clear();
    let fixed_bytes = serde_json::to_vec(&LogRecord::Header {
        header: Box::new(log.header.clone()),
    })
    .unwrap()
    .len()
        + 1; // Count the final newline, as the reader does.
    log.header.source = "x".repeat(LIMIT - fixed_bytes);
    let mut written = Vec::new();
    log.write(&mut written).unwrap();
    assert_eq!(written.iter().position(|&b| b == b'\n').unwrap() + 1, LIMIT);
    let report = verify(Cursor::new(written), std::io::sink()).unwrap();
    assert_eq!(report.source.len(), LIMIT - fixed_bytes);
    assert_eq!(report.ticks, 6);

    log.header.source.push('x');
    let mut written = Vec::new();
    assert!(log.write(&mut written).is_err());
    assert!(written.is_empty());

    // This short source expands to more than 8 MiB of JSON escape sequences.
    log.header.source = "\0".repeat(LIMIT / 6 + 1);
    assert!(log.header.source.len() < LIMIT);
    assert!(log.write(&mut written).is_err());
    assert!(written.is_empty());
}

#[test]
fn recording_propagates_write_and_flush_failures() {
    struct Broken {
        fail_write: bool,
    }
    impl std::io::Write for Broken {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            if self.fail_write {
                Err(std::io::Error::other("record write unavailable"))
            } else {
                Ok(bytes.len())
            }
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Err(std::io::Error::other("record flush unavailable"))
        }
    }
    let log = recorded_log();
    for fail_write in [false, true] {
        let error = log.write(Broken { fail_write }).unwrap_err();
        assert!(error.contains(if fail_write {
            "record write unavailable"
        } else {
            "record flush unavailable"
        }));
    }
}

#[test]
fn nonfinite_header_numbers_cannot_become_null_or_disappear() {
    for optional in [true, false] {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut log = recorded_log();
            if optional {
                log.header.config.rear_axle_offset_m = Some(bad);
            } else {
                log.header.config.vehicle.wheelbase = bad;
            }
            let mut written = Vec::new();
            let result = log.write(&mut written);
            assert!(
                result.is_err(),
                "nonfinite header was saved; replay result: {:?}",
                verify(Cursor::new(&written), std::io::sink())
            );
            assert!(result.unwrap_err().contains("non-finite"));
            assert!(written.is_empty());
        }
    }
}

#[test]
fn nonfinite_sensor_sample_is_rejected_even_when_the_driver_brakes_correctly() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut log = recorded_log();
        log.ticks[2].input.odometry.as_mut().unwrap().speed = bad;
        recompute(&mut log);
        assert!(log.ticks[2].expected.emergency);
        assert!(log.ticks[2].expected.command.finite());
        let mut written = Vec::new();
        let error = log.write(&mut written).unwrap_err();
        assert!(error.starts_with("tick 2:"));
        assert!(error.contains("non-finite"));
        assert_eq!(written.iter().filter(|&&b| b == b'\n').count(), 3);
        assert!(verify(Cursor::new(written), std::io::sink()).is_err());
    }
}

#[test]
fn nonfinite_expected_numbers_in_sequences_and_optional_fields_are_rejected() {
    for field in 0..3 {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut log = recorded_log();
            let expected = &mut log.ticks[0].expected;
            match field {
                0 => expected.command.acceleration = bad,
                1 => expected.trajectory.points[0].position.y = bad,
                _ => expected.localization.as_mut().unwrap().last_nis = Some(bad),
            }
            let mut written = Vec::new();
            let error = log.write(&mut written).unwrap_err();
            assert!(error.starts_with("tick 0:"));
            assert!(error.contains("non-finite"));
            assert_eq!(written.iter().filter(|&&b| b == b'\n').count(), 1);
        }
    }
}

#[test]
fn nulls_strings_and_finite_extremes_keep_their_recorded_meaning() {
    let mut log = recorded_log();
    log.header.source = "NaN Infinity -Infinity null".into();
    log.ticks[1].input.odometry.as_mut().unwrap().yaw_rate = -0.0;
    log.ticks[2].input.odometry.as_mut().unwrap().yaw_rate = f64::from_bits(1);
    log.ticks[3].input.gnss.as_mut().unwrap().variance = f64::MAX;
    log.ticks[4].input.gnss = None;
    recompute(&mut log);
    let mut written = Vec::new();
    log.write(&mut written).unwrap();
    let lines: Vec<_> = written.split(|&b| b == b'\n').collect();
    for index in 1..=3 {
        let LogRecord::Tick { tick } = serde_json::from_slice(lines[index + 1]).unwrap() else {
            panic!("expected tick");
        };
        if index == 3 {
            assert_eq!(
                tick.input.gnss.unwrap().variance.to_bits(),
                f64::MAX.to_bits()
            );
        } else {
            assert_eq!(
                tick.input.odometry.unwrap().yaw_rate.to_bits(),
                log.ticks[index].input.odometry.unwrap().yaw_rate.to_bits()
            );
        }
    }
    let LogRecord::Tick { tick } = serde_json::from_slice(lines[5]).unwrap() else {
        panic!("expected tick");
    };
    assert!(tick.input.gnss.is_none());
    let report = verify(Cursor::new(written), std::io::sink()).unwrap();
    assert_eq!(report.source, log.header.source);
    assert_eq!(report.ticks, 6);
}

#[test]
fn invalid_header_metadata_and_configuration_cannot_be_saved() {
    for case in 0..5 {
        let mut log = recorded_log();
        match case {
            0 => log.header.source.clear(),
            1 => log.header.schema_version += 1,
            2 => log.header.config.vehicle.wheelbase = 0.0,
            3 => log.header.config.nominal_dt = 0.0,
            _ => log.header.config.route.lengths[1] += 1.0,
        }
        let mut written = Vec::new();
        let result = log.write(&mut written);
        assert!(
            result.is_err(),
            "invalid header was saved; replay result: {:?}",
            verify(Cursor::new(&written), std::io::sink())
        );
        assert!(result.unwrap_err().starts_with("log header:"));
        assert!(written.is_empty());
    }
}

#[test]
fn invalid_frame_clock_is_rejected_before_any_record_is_written() {
    for case in 0..6 {
        let mut log = recorded_log();
        let index = if case == 0 { 0 } else { 4 };
        let time = match case {
            0 => -0.1,
            1 => log.ticks[3].input.time,
            2 => log.ticks[3].input.time - 0.01,
            3 => f64::NAN,
            4 => f64::INFINITY,
            _ => f64::NEG_INFINITY,
        };
        log.ticks[index].input.time = time;
        log.ticks[index].expected.time = time;
        let mut written = Vec::new();
        let error = log.write(&mut written).unwrap_err();
        assert!(error.starts_with(&format!("tick {index}:")));
        assert!(written.is_empty());
    }
}

#[test]
fn output_clock_must_match_input_including_the_sign_of_zero() {
    for case in 0..4 {
        let mut log = recorded_log();
        let index = if case == 1 { 0 } else { 2 };
        log.ticks[index].expected.time = match case {
            0 => log.ticks[index].input.time + 0.01,
            1 => -0.0,
            2 => f64::NAN,
            _ => f64::INFINITY,
        };
        let mut written = Vec::new();
        let error = log.write(&mut written).unwrap_err();
        assert!(error.starts_with(&format!("tick {index}:")));
        assert!(error.contains("output time"));
        assert!(written.is_empty());
    }
}

#[test]
fn nonuniform_clocks_gaps_and_invalid_sensor_stamps_remain_recordable() {
    for start in [-0.0, 5.0] {
        let mut log = recorded_log();
        for (index, offset) in [0.0, 1e-8, 0.05, 0.31, 0.4, 0.5].into_iter().enumerate() {
            let time = if index == 0 { start } else { start + offset };
            let input = &mut log.ticks[index].input;
            input.time = time;
            input.odometry.as_mut().unwrap().stamp = time;
            input.gnss.as_mut().unwrap().stamp = time;
            input.lidar.as_mut().unwrap().stamp = time;
        }
        log.ticks[4].input.gnss.as_mut().unwrap().stamp = -1.0;
        recompute(&mut log);
        assert!(
            log.ticks[3]
                .expected
                .health
                .contains(&HealthIssue::ClockGap)
        );
        assert!(
            log.ticks[4]
                .expected
                .health
                .contains(&HealthIssue::InvalidGnss)
        );
        let mut written = Vec::new();
        log.write(&mut written).unwrap();
        let first = written.split(|&b| b == b'\n').nth(1).unwrap();
        let LogRecord::Tick { tick } = serde_json::from_slice(first).unwrap() else {
            panic!("expected tick");
        };
        assert_eq!(tick.input.time.to_bits(), start.to_bits());
        assert_eq!(tick.expected.time.to_bits(), start.to_bits());
        assert_eq!(
            verify(Cursor::new(written), std::io::sink()).unwrap().ticks,
            6
        );
    }
}
