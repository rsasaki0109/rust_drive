use rustdrive_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdrive_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};
use std::io::Cursor;
fn log() -> Vec<u8> {
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
            lidar_failed: false,
            navigation_update: None,
        };
        let output = pipeline.step(&input).unwrap();
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    bytes
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
    let nav: rustdrive_pipeline::navigation::NavigationConfig =
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
            lidar_failed: false,
            navigation_update: None,
        };
        if i == 1 {
            input.navigation_update = Some(rustdrive_pipeline::navigation::NavigationUpdate {
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
