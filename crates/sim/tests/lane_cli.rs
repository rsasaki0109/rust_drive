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
fn permitted_left_right_and_rotated_lane_changes_drive_into_the_target_and_replay() {
    let workspace = Workspace::new();
    let original: serde_json::Value =
        serde_json::from_str(include_str!("../../../maps/lanes/two-lane-merge.json")).unwrap();
    for (case, side, mirror, angle) in [
        ("left", "left", 1.0, 0.0_f64),
        ("right", "right", -1.0, 0.0_f64),
        ("north", "left", 1.0, std::f64::consts::FRAC_PI_2),
    ] {
        let mut map = original.clone();
        for lane in map["lanes"].as_array_mut().unwrap() {
            if mirror < 0.0 {
                let left = lane.as_object_mut().unwrap().remove("left");
                let right = lane.as_object_mut().unwrap().remove("right");
                if let Some(left) = left {
                    lane["right"] = left;
                }
                if let Some(right) = right {
                    lane["left"] = right;
                }
            }
            for point in lane["centerline"].as_array_mut().unwrap() {
                let x = point["x"].as_f64().unwrap();
                let y = point["y"].as_f64().unwrap() * mirror;
                point["x"] = (x * angle.cos() - y * angle.sin()).into();
                point["y"] = (x * angle.sin() + y * angle.cos()).into();
            }
        }
        let map_path = workspace.0.join(format!("{case}-map.json"));
        fs::write(&map_path, serde_json::to_vec(&map).unwrap()).unwrap();
        let scenario = workspace.0.join(format!("{case}-scenario.json"));
        let output = cli()
            .arg("lane-route")
            .arg("--map")
            .arg(&map_path)
            .args([
                "--start",
                "east-right-in",
                "--goal",
                "east-exit",
                "--change-side",
                side,
                "--change-start",
                "5",
                "--change-length",
                "30",
                "--cruise-speed",
                "4",
            ])
            .arg("--scenario-output")
            .arg(&scenario)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(
            report["plan"]["lane_ids"],
            serde_json::json!([
                "east-right-in",
                "east-left-in",
                "east-left-merge",
                "east-exit"
            ])
        );
        assert_eq!(report["lane_change"]["request"]["side"], side);
        assert_eq!(report["lane_change"]["to_lane"], "east-left-in");
        let exported: serde_json::Value =
            serde_json::from_slice(&fs::read(&scenario).unwrap()).unwrap();
        assert_eq!(exported["cruise_speed"], 3.0);
        assert_eq!(exported["duration"], 120.0);
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
            let run: serde_json::Value =
                serde_json::from_slice(&fs::read(run_dir.join("run.json")).unwrap()).unwrap();
            assert_eq!(run["summary"]["reached_goal"], true);
            assert_eq!(run["summary"]["collisions"], 0);
            assert_eq!(run["summary"]["road_violations"], 0);
            assert_eq!(run["route"], report["plan"]["route"]);
            let radius = run["vehicle"]["radius"].as_f64().unwrap();
            let mut entered_target = false;
            for frame in run["frames"].as_array().unwrap() {
                let p = &frame["truth"]["pose"]["position"];
                let x =
                    p["x"].as_f64().unwrap() * angle.cos() + p["y"].as_f64().unwrap() * angle.sin();
                let y = (-p["x"].as_f64().unwrap() * angle.sin()
                    + p["y"].as_f64().unwrap() * angle.cos())
                    * mirror;
                if (0.0..=40.0).contains(&x) {
                    assert!(
                        y - radius >= -1.75 - 1e-8 && y + radius <= 5.25 + 1e-8,
                        "case {case} seed {seed}, x {x} y {y}"
                    );
                    entered_target |= x >= 35.0 && (y - 3.5).abs() + radius <= 1.75;
                }
            }
            assert!(
                entered_target,
                "case {case} seed {seed} never fully entered target before its end"
            );
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
            assert_eq!(replay["ticks"], run["summary"]["steps"]);
        }
    }
}

#[test]
fn invalid_forbidden_closed_or_incomplete_lane_changes_do_not_export() {
    let workspace = Workspace::new();
    let map = workspace.map();
    let scenario = workspace.0.join("scenario.json");
    for options in [
        vec!["--change-side", "left"],
        vec!["--change-start", "5", "--change-length", "30"],
        vec![
            "--change-side",
            "up",
            "--change-start",
            "5",
            "--change-length",
            "30",
        ],
        vec![
            "--change-side",
            "right",
            "--change-start",
            "5",
            "--change-length",
            "30",
        ],
        vec![
            "--change-side",
            "left",
            "--change-start",
            "NaN",
            "--change-length",
            "30",
        ],
        vec![
            "--change-side",
            "left",
            "--change-start",
            "-1",
            "--change-length",
            "30",
        ],
        vec![
            "--change-side",
            "left",
            "--change-start",
            "5",
            "--change-length",
            "inf",
        ],
        vec![
            "--change-side",
            "left",
            "--change-start",
            "5",
            "--change-length",
            "2",
        ],
        vec![
            "--change-side",
            "left",
            "--change-start",
            "20",
            "--change-length",
            "30",
        ],
        vec![
            "--change-side",
            "left",
            "--change-start",
            "5",
            "--change-length",
            "30",
            "--closed-lane",
            "east-left-in",
        ],
        vec![
            "--change-side",
            "left",
            "--change-start",
            "5",
            "--change-length",
            "30",
            "--closed-lane",
            "east-left-merge",
        ],
    ] {
        let output = cli()
            .arg("lane-route")
            .arg("--map")
            .arg(&map)
            .args(["--start", "east-right-in", "--goal", "east-exit"])
            .args(&options)
            .arg("--scenario-output")
            .arg(&scenario)
            .output()
            .unwrap();
        assert_eq!(output.status.code(), Some(2), "options {options:?}");
        assert!(!scenario.exists());
    }
    let output = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(&map)
        .args([
            "--start",
            "east-left-in",
            "--goal",
            "east-exit",
            "--change-side",
            "right",
            "--change-start",
            "5",
            "--change-length",
            "30",
        ])
        .arg("--scenario-output")
        .arg(&scenario)
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert!(!scenario.exists());
    // A valid change can be planned without exporting or touching the input map.
    let original = fs::read(&map).unwrap();
    let output = cli()
        .arg("lane-route")
        .arg("--map")
        .arg(&map)
        .args([
            "--start",
            "east-right-in",
            "--goal",
            "east-left-in",
            "--change-side",
            "left",
            "--change-start",
            "5",
            "--change-length",
            "30",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_eq!(fs::read(&map).unwrap(), original);
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
