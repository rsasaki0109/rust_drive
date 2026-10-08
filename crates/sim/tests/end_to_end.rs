use rustdrive_sim::{Scenario, simulate};
fn scenario(name: &str) -> Scenario {
    serde_json::from_str(
        &std::fs::read_to_string(format!(
            "{}/../../scenarios/{name}.json",
            env!("CARGO_MANIFEST_DIR")
        ))
        .unwrap(),
    )
    .unwrap()
}
#[test]
fn closed_loop_mission_across_seeds() {
    for seed in [1, 7, 42] {
        let run = simulate(scenario("mission"), seed).unwrap();
        assert!(run.summary.passed, "seed {seed}: {:?}", run.summary);
        assert!(run.summary.avoidance_steps > 10);
        assert!(run.summary.max_tracks >= 2);
        assert!(run.summary.localization_rmse < 0.3);
    }
}
#[test]
fn blocked_road_stops_without_collision() {
    let run = simulate(scenario("blocked"), 7).unwrap();
    assert!(run.summary.passed, "{:?}", run.summary);
    assert!(run.summary.min_clearance > 0.5);
}
#[test]
fn stale_lidar_and_gnss_trigger_braking() {
    for name in ["lidar-fault", "gnss-fault"] {
        let run = simulate(scenario(name), 7).unwrap();
        assert!(run.summary.passed, "{name}: {:?}", run.summary);
        assert!(run.summary.emergency_steps > 20);
    }
}
#[test]
fn identical_seed_replays_exactly() {
    let a = simulate(scenario("blocked"), 42).unwrap();
    let b = simulate(scenario("blocked"), 42).unwrap();
    assert_eq!(
        serde_json::to_vec(&a).unwrap(),
        serde_json::to_vec(&b).unwrap()
    );
}
#[test]
fn malformed_scenario_rejected() {
    let mut s = scenario("mission");
    s.duration = -1.0;
    assert!(simulate(s, 1).is_err());
}

#[test]
fn actual_collision_fails_acceptance() {
    let mut s = scenario("blocked");
    s.objects[0].s = 0.0;
    let run = simulate(s, 7).unwrap();
    assert!(!run.summary.passed);
    assert!(run.summary.collisions > 0);
    assert!(run.summary.min_clearance < 0.0);
}

#[test]
fn cli_reports_failure_and_writes_evidence() {
    use std::{fs, process::Command};
    let directory = std::env::temp_dir().join(format!("rustdrive-cli-{}", std::process::id()));
    fs::create_dir_all(&directory).unwrap();
    let mut s = scenario("mission");
    s.duration = 1.0;
    let input = directory.join("scenario.json");
    fs::write(&input, serde_json::to_vec(&s).unwrap()).unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_rustdrive"))
        .args(["run", "--scenario"])
        .arg(&input)
        .arg("--output")
        .arg(directory.join("run"))
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let evidence: serde_json::Value =
        serde_json::from_slice(&fs::read(directory.join("run/summary.json")).unwrap()).unwrap();
    assert_eq!(evidence["passed"], false);
    let result = Command::new(env!("CARGO_BIN_EXE_rustdrive"))
        .args(["run", "--unknown", "x"])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    fs::remove_dir_all(directory).unwrap();
}

#[test]
fn cli_replay_recomputes_and_rejects_corruption_without_stale_success() {
    use std::{fs, io::BufWriter, process::Command};
    let directory =
        std::env::temp_dir().join(format!("rustdrive-replay-cli-{}", std::process::id()));
    fs::create_dir_all(&directory).unwrap();
    let mut s = scenario("mission");
    s.duration = 1.0;
    let result = simulate(s, 7).unwrap();
    // Replay validates computation, even when the short physical mission fails.
    assert!(!result.summary.passed);
    let log = directory.join("sensors.jsonl");
    result
        .sensor_log
        .as_ref()
        .unwrap()
        .write(BufWriter::new(fs::File::create(&log).unwrap()))
        .unwrap();
    let invoke = || {
        Command::new(env!("CARGO_BIN_EXE_rustdrive"))
            .args(["replay", "--log"])
            .arg(&log)
            .arg("--output")
            .arg(directory.join("replay"))
            .output()
            .unwrap()
    };
    assert_eq!(invoke().status.code(), Some(0));
    let summary = directory.join("replay/replay.json");
    let report: serde_json::Value = serde_json::from_slice(&fs::read(&summary).unwrap()).unwrap();
    assert_eq!(report["verified"], true);
    let text = fs::read_to_string(&log).unwrap();
    let mut lines: Vec<serde_json::Value> = text
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    lines[1]["tick"]["expected"]["command"]["acceleration"] = serde_json::json!(123.0);
    fs::write(
        &log,
        lines
            .iter()
            .map(|v| serde_json::to_string(v).unwrap() + "\n")
            .collect::<String>(),
    )
    .unwrap();
    assert_eq!(invoke().status.code(), Some(2));
    assert!(
        !summary.exists(),
        "failed replay must remove the previous success report"
    );
    fs::write(&summary, "previous success").unwrap();
    fs::remove_file(&log).unwrap();
    assert_eq!(invoke().status.code(), Some(2));
    assert!(!summary.exists());
    fs::remove_dir_all(directory).unwrap();
}
