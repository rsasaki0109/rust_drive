use std::{
    fs,
    path::PathBuf,
    process::Command,
    sync::atomic::{AtomicU64, Ordering},
};

struct Workspace(PathBuf);
impl Workspace {
    fn new() -> Self {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let p = std::env::temp_dir().join(format!(
            "rustdriving-lane-speeds-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&p).unwrap();
        Self(p)
    }
    fn map(&self) -> PathBuf {
        let p = self.0.join("map.json");
        fs::write(&p, include_str!("../../../maps/lanes/speed-zones.json")).unwrap();
        p
    }
}
impl Drop for Workspace {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
fn cli() -> Command {
    Command::new(env!("CARGO_BIN_EXE_rustdriving"))
}

#[test]
fn local_speed_export_is_opt_in_preserves_cruise_and_matches_the_preview() {
    let w = Workspace::new();
    let map = w.map();
    let mut local = None;
    for enabled in [false, true] {
        let target = w.0.join(format!("{enabled}.json"));
        let mut command = cli();
        command
            .arg("lane-route")
            .arg("--map")
            .arg(&map)
            .args([
                "--start",
                "approach",
                "--goal",
                "exit",
                "--cruise-speed",
                "6",
            ])
            .arg("--scenario-output")
            .arg(&target);
        if enabled {
            command.arg("--local-speed-limits");
        }
        let result = command.output().unwrap();
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        let report: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
        let scenario: serde_json::Value =
            serde_json::from_slice(&fs::read(target).unwrap()).unwrap();
        assert_eq!(report["plan"]["speed_limit_m_s"], 2.);
        assert_eq!(scenario["cruise_speed"], if enabled { 6. } else { 2. });
        if enabled {
            assert_eq!(scenario["speed_zones"], report["speed_zones"]);
            assert_eq!(
                scenario["speed_zones"],
                serde_json::json!([
                    {"start_s_m":0.,"end_s_m":30.,"speed_m_s":6.},
                    {"start_s_m":30.,"end_s_m":60.,"speed_m_s":2.},
                    {"start_s_m":60.,"end_s_m":120.,"speed_m_s":6.}
                ])
            );
            local = Some(report);
        } else {
            assert!(scenario.get("speed_zones").is_none());
            assert!(report.get("speed_zones").is_none());
        }
    }
    let preview = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(map)
        .args([
            "--start",
            "approach",
            "--goal",
            "exit",
            "--local-speed-limits",
        ])
        .output()
        .unwrap();
    assert!(preview.status.success());
    let preview: serde_json::Value = serde_json::from_slice(&preview.stdout).unwrap();
    assert_eq!(preview, local.unwrap());
}

#[test]
fn unsupported_changes_and_existing_outputs_are_rejected_without_writing() {
    let w = Workspace::new();
    let change_map = w.0.join("change.json");
    fs::write(
        &change_map,
        include_str!("../../../maps/lanes/two-lane-change.json"),
    )
    .unwrap();
    let destination = w.0.join("absent/scenario.json");
    let result = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(change_map)
        .args([
            "--start",
            "source",
            "--goal",
            "target-exit",
            "--change-side",
            "left",
            "--change-start",
            "25",
            "--change-length",
            "30",
            "--local-speed-limits",
            "--wait-for-gap",
            "--return-on-conflict",
            "--resume-after-return",
        ])
        .arg("--scenario-output")
        .arg(&destination)
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&result.stderr).contains("mission retries"));
    assert!(!destination.parent().unwrap().exists());
    let map = w.map();
    let original = fs::read(&map).unwrap();
    let result = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(&map)
        .args([
            "--start",
            "approach",
            "--goal",
            "exit",
            "--local-speed-limits",
        ])
        .arg("--scenario-output")
        .arg(&map)
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    assert_eq!(fs::read(map).unwrap(), original);
}

#[test]
fn exported_map_limits_drive_physically_and_replay_from_sensors() {
    let w = Workspace::new();
    let scenario_path = w.0.join("scenario.json");
    let result = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(w.map())
        .args([
            "--start",
            "approach",
            "--goal",
            "exit",
            "--local-speed-limits",
            "--cruise-speed",
            "6",
        ])
        .arg("--scenario-output")
        .arg(&scenario_path)
        .output()
        .unwrap();
    assert!(result.status.success());
    let scenario = serde_json::from_slice(&fs::read(scenario_path).unwrap()).unwrap();
    let run = rustdriving_sim::simulate(scenario, 7).unwrap();
    assert!(run.summary.passed, "{:?}", run.summary.failures);
    assert_eq!(run.frames.len(), run.summary.steps);
    assert!(
        run.frames
            .iter()
            .filter(|f| (28.75..=61.25).contains(&f.truth.pose.position.x))
            .all(|f| f.truth.speed <= 2.15)
    );
    assert!(
        run.frames
            .iter()
            .any(|f| f.truth.pose.position.x > 70. && f.truth.speed > 5.)
    );
    let mut bytes = vec![];
    run.sensor_log.unwrap().write(&mut bytes).unwrap();
    let replay = rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(replay.verified);
    assert_eq!(replay.ticks, run.summary.steps);
}

#[test]
fn fixed_lane_change_speed_export_keeps_gap_admission_and_records_the_policy() {
    let w = Workspace::new();
    let map = w.0.join("change-speeds.json");
    fs::write(
        &map,
        include_str!("../../../maps/lanes/change-speeds-target-slow.json"),
    )
    .unwrap();
    let path = w.0.join("scenario.json");
    let output = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(map)
        .args([
            "--start",
            "source",
            "--goal",
            "target-exit",
            "--change-side",
            "left",
            "--change-start",
            "25",
            "--change-length",
            "30",
            "--wait-for-gap",
            "--local-speed-limits",
            "--cruise-speed",
            "4",
        ])
        .arg("--scenario-output")
        .arg(&path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let scenario: serde_json::Value = serde_json::from_slice(&fs::read(path).unwrap()).unwrap();
    let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(scenario["cruise_speed"], 4.);
    assert_eq!(scenario["speed_zones"], report["speed_zones"]);
    assert_eq!(scenario["lane_change"], report["lane_change_gate"]);
    assert!(scenario["lane_change"]["return_on_conflict"].is_null());
    assert_eq!(report["lane_change"]["to_lane"], "target");
    assert!(scenario["speed_zones"][1]["end_s_m"].as_f64().unwrap() > 80.2);
}

#[test]
fn returning_lane_change_exports_a_separate_map_derived_speed_cap() {
    let w = Workspace::new();
    let map = w.0.join("change-speeds.json");
    fs::write(
        &map,
        include_str!("../../../maps/lanes/change-speeds-target-slow.json"),
    )
    .unwrap();
    let path = w.0.join("scenario.json");
    let output = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(map)
        .args([
            "--start",
            "source",
            "--goal",
            "target-exit",
            "--change-side",
            "left",
            "--change-start",
            "25",
            "--change-length",
            "30",
            "--wait-for-gap",
            "--local-speed-limits",
            "--return-on-conflict",
            "--cruise-speed",
            "4",
        ])
        .arg("--scenario-output")
        .arg(&path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let scenario: serde_json::Value = serde_json::from_slice(&fs::read(path).unwrap()).unwrap();
    let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(scenario["cruise_speed"], 4.);
    assert_eq!(scenario["speed_zones"], report["speed_zones"]);
    assert_eq!(scenario["lane_change"], report["lane_change_gate"]);
    assert_eq!(
        scenario["lane_change"]["return_on_conflict"]["speed_limit_m_s"],
        2.
    );
    assert!(scenario["lane_change"]["return_on_conflict"]["resume_after_return"].is_null());
    assert_eq!(report["lane_change"]["to_lane"], "target");
    assert!(scenario["speed_zones"][1]["end_s_m"].as_f64().unwrap() > 80.2);
}
