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
        let path = std::env::temp_dir().join(format!(
            "rustdriving-lanes-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&path).unwrap();
        Self(path)
    }
    fn map(&self) -> PathBuf {
        let path = self.0.join("map.json");
        fs::write(
            &path,
            include_str!("../../../maps/lanes/two-lane-merge.json"),
        )
        .unwrap();
        path
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
fn lane_following_merge_and_closure_detour_drive_and_replay_across_seeds() {
    let workspace = Workspace::new();
    let map = workspace.map();
    for (case, start, closed, expected, limit) in [
        (
            "right",
            "east-right-in",
            None,
            ["east-right-in", "east-right-merge", "east-exit"],
            3.0,
        ),
        (
            "merge",
            "east-left-in",
            None,
            ["east-left-in", "east-left-merge", "east-exit"],
            3.0,
        ),
        (
            "detour",
            "east-right-in",
            Some("east-right-merge"),
            ["east-right-in", "east-bypass", "east-exit"],
            2.0,
        ),
    ] {
        let scenario = workspace.0.join(format!("{case}.json"));
        let mut command = cli();
        command
            .arg("lane-route")
            .arg("--map")
            .arg(&map)
            .args([
                "--start",
                start,
                "--goal",
                "east-exit",
                "--cruise-speed",
                "4",
            ])
            .arg("--scenario-output")
            .arg(&scenario);
        if let Some(closed) = closed {
            command.args(["--closed-lane", closed]);
        }
        let output = command.output().unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(report["plan"]["lane_ids"], serde_json::json!(expected));
        let declared: serde_json::Value =
            serde_json::from_slice(&fs::read(&scenario).unwrap()).unwrap();
        assert_eq!(declared["cruise_speed"], limit);
        assert_eq!(declared["local_route_geometry"], true);
        assert_eq!(report["plan"]["speed_limit_m_s"], limit);
        for seed in [1, 7, 42] {
            let run_dir = workspace.0.join(format!("{case}-{seed}"));
            let output = cli()
                .arg("run")
                .arg("--scenario")
                .arg(&scenario)
                .args(["--seed", &seed.to_string()])
                .arg("--output")
                .arg(&run_dir)
                .output()
                .unwrap();
            assert!(
                output.status.success(),
                "case {case} seed {seed}: {} {}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            let summary: serde_json::Value =
                serde_json::from_slice(&fs::read(run_dir.join("summary.json")).unwrap()).unwrap();
            assert_eq!(summary["reached_goal"], true);
            assert_eq!(summary["collisions"], 0);
            assert_eq!(summary["road_violations"], 0);
            let run: serde_json::Value =
                serde_json::from_slice(&fs::read(run_dir.join("run.json")).unwrap()).unwrap();
            assert_eq!(run["route"], report["plan"]["route"]);
            let replay_dir = run_dir.join("replay");
            let output = cli()
                .arg("replay")
                .arg("--log")
                .arg(run_dir.join("sensors.jsonl"))
                .arg("--output")
                .arg(&replay_dir)
                .output()
                .unwrap();
            assert!(
                output.status.success(),
                "{}",
                String::from_utf8_lossy(&output.stderr)
            );
            let replay: serde_json::Value =
                serde_json::from_slice(&fs::read(replay_dir.join("replay.json")).unwrap()).unwrap();
            assert_eq!(replay["verified"], true);
            assert_eq!(replay["ticks"], summary["steps"]);
        }
    }
}

#[test]
fn malformed_lane_maps_and_unreachable_neighbor_requests_do_not_export_scenarios() {
    let workspace = Workspace::new();
    let input = workspace.0.join("invalid.json");
    let scenario = workspace.0.join("scenario.json");
    let original: serde_json::Value =
        serde_json::from_str(include_str!("../../../maps/lanes/two-lane-merge.json")).unwrap();
    for case in 0..7 {
        let mut map = original.clone();
        match case {
            0 => map["schema_version"] = 2.into(),
            1 => map["unknown"] = true.into(),
            2 => map["lanes"][0]["direction"] = "reverse".into(),
            3 => map["lanes"][0]["left"]["allowed"] = true.into(),
            4 => map["lanes"][0]["successors"] = serde_json::json!(["east-left-in"]),
            5 => map["lanes"][1]["right"] = serde_json::Value::Null,
            _ => map["lanes"][0]["width_m"] = "NaN".into(),
        }
        fs::write(&input, serde_json::to_vec(&map).unwrap()).unwrap();
        let output = cli()
            .arg("lane-route")
            .arg("--map")
            .arg(&input)
            .args(["--start", "east-right-in", "--goal", "east-exit"])
            .arg("--scenario-output")
            .arg(&scenario)
            .output()
            .unwrap();
        assert_eq!(output.status.code(), Some(2), "case {case}");
        assert!(!scenario.exists());
    }
    let input = workspace.map();
    for options in [
        vec!["--goal", "east-left-in"],
        vec!["--goal", "east-exit", "--closed-lane", "east-exit"],
        vec!["--goal", "east-exit", "--closed-lane", "unknown"],
    ] {
        let output = cli()
            .arg("lane-route")
            .arg("--map")
            .arg(&input)
            .args(["--start", "east-right-in"])
            .args(options)
            .arg("--scenario-output")
            .arg(&scenario)
            .output()
            .unwrap();
        assert_eq!(output.status.code(), Some(2));
        assert!(!scenario.exists());
    }
}

#[test]
fn oversized_inputs_and_existing_output_paths_are_rejected_without_clobbering() {
    let workspace = Workspace::new();
    let input = workspace.map();
    let original = fs::read(&input).unwrap();
    let output = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(&input)
        .args(["--start", "east-right-in", "--goal", "east-exit"])
        .arg("--scenario-output")
        .arg(&input)
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert_eq!(fs::read(&input).unwrap(), original);
    let large = workspace.0.join("large.json");
    fs::File::create(&large)
        .unwrap()
        .set_len(16 * 1024 * 1024 + 1)
        .unwrap();
    let scenario = workspace.0.join("scenario.json");
    let output = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(&large)
        .args(["--start", "east-right-in", "--goal", "east-exit"])
        .arg("--scenario-output")
        .arg(&scenario)
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("exceeds 16 MiB"));
    assert!(!scenario.exists());
}

#[test]
fn invalid_runtime_requests_fail_before_scenario_creation() {
    let workspace = Workspace::new();
    let input = workspace.map();
    let scenario = workspace.0.join("scenario.json");
    for options in [
        ["--cruise-speed", "NaN"],
        ["--cruise-speed", "-1"],
        ["--cruise-speed", "9"],
        ["--duration", "NaN"],
        ["--duration", "0"],
        ["--duration", "301"],
    ] {
        let output = cli()
            .arg("lane-route")
            .arg("--map")
            .arg(&input)
            .args(["--start", "east-right-in", "--goal", "east-exit"])
            .args(options)
            .arg("--scenario-output")
            .arg(&scenario)
            .output()
            .unwrap();
        assert_eq!(output.status.code(), Some(2));
        assert!(!scenario.exists());
    }
}

#[test]
fn exporting_a_geometric_route_checks_the_existing_vehicle_corner_feasibility() {
    let workspace = Workspace::new();
    let input = workspace.map();
    let mut map: serde_json::Value = serde_json::from_slice(&fs::read(&input).unwrap()).unwrap();
    map["lanes"][2]["centerline"] = serde_json::json!([
        {"x":40.0,"y":0.0}, {"x":40.0,"y":60.0}, {"x":100.0,"y":0.0}
    ]);
    fs::write(&input, serde_json::to_vec(&map).unwrap()).unwrap();
    let mut route = cli();
    route.arg("lane-route").arg("--map").arg(&input).args([
        "--start",
        "east-right-in",
        "--goal",
        "east-exit",
        "--closed-lane",
        "east-bypass",
    ]);
    assert!(route.output().unwrap().status.success());
    let scenario = workspace.0.join("scenario.json");
    let output = route
        .arg("--scenario-output")
        .arg(&scenario)
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert!(!scenario.exists());
    assert!(String::from_utf8_lossy(&output.stderr).contains("corner"));
}
