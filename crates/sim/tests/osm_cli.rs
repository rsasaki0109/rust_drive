//! Public map import must produce a runnable sensor-only scenario, not just a graph.
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
            "rustdrive-osm-cli-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}
impl Drop for Workspace {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
fn cli() -> Command {
    Command::new(env!("CARGO_BIN_EXE_rustdrive"))
}
#[test]
fn external_map_import_route_physical_goal_and_complete_sensor_replay() {
    let dir = Workspace::new();
    let input = dir.0.join("source.json");
    let map = dir.0.join("map.json");
    let scenario = dir.0.join("scenario.json");
    fs::write(
        &input,
        include_str!("../../../maps/osm/german-road-extract.json"),
    )
    .unwrap();
    let imported = cli()
        .arg("import-osm")
        .arg("--input")
        .arg(&input)
        .arg("--output")
        .arg(&map)
        .args([
            "--origin-lat",
            "48.136",
            "--origin-lon",
            "10.0695",
            "--default-half-width",
            "3.0",
        ])
        .arg("--scenario-output")
        .arg(&scenario)
        .args([
            "--start",
            "osm-node-7119017425",
            "--goal",
            "osm-node-274969423",
            "--duration",
            "180",
            "--cruise-speed",
            "2",
        ])
        .output()
        .unwrap();
    assert!(
        imported.status.success(),
        "{}",
        String::from_utf8_lossy(&imported.stderr)
    );
    let report: serde_json::Value =
        serde_json::from_slice(&fs::read(map.with_extension("import.json")).unwrap()).unwrap();
    assert_eq!(report["imported_ways"], 4);
    assert_eq!(report["directed_edges"], 10);
    assert_eq!(report["clipped_short_ways"], 2);
    let run_dir = dir.0.join("run");
    let run = cli()
        .arg("run")
        .arg("--scenario")
        .arg(&scenario)
        .args(["--seed", "7"])
        .arg("--output")
        .arg(&run_dir)
        .output()
        .unwrap();
    assert!(
        run.status.success(),
        "{}",
        String::from_utf8_lossy(&run.stdout)
    );
    let summary: serde_json::Value =
        serde_json::from_slice(&fs::read(run_dir.join("summary.json")).unwrap()).unwrap();
    assert_eq!(summary["reached_goal"], true);
    assert_eq!(summary["road_violations"], 0);
    assert_eq!(summary["collisions"], 0);
    let replay_dir = dir.0.join("replay");
    let replay = cli()
        .arg("replay")
        .arg("--log")
        .arg(run_dir.join("sensors.jsonl"))
        .arg("--output")
        .arg(&replay_dir)
        .output()
        .unwrap();
    assert!(
        replay.status.success(),
        "{}",
        String::from_utf8_lossy(&replay.stderr)
    );
    let replay: serde_json::Value =
        serde_json::from_slice(&fs::read(replay_dir.join("replay.json")).unwrap()).unwrap();
    assert_eq!(replay["verified"], true);
    assert_eq!(replay["ticks"], summary["steps"]);
}
#[test]
fn unreachable_route_and_oversized_document_fail_without_generated_map() {
    let dir = Workspace::new();
    let input = dir.0.join("source.json");
    let map = dir.0.join("map.json");
    fs::write(
        &input,
        include_str!("../../../maps/osm/german-road-extract.json"),
    )
    .unwrap();
    let result = cli()
        .arg("import-osm")
        .arg("--input")
        .arg(&input)
        .arg("--output")
        .arg(&map)
        .args(["--origin-lat", "48.136", "--origin-lon", "10.0695"])
        .arg("--scenario-output")
        .arg(dir.0.join("scenario.json"))
        .args([
            "--start",
            "osm-node-7119017425",
            "--goal",
            "osm-node-274969423",
            "--closed-edge",
            "osm-way-25216931-0-forward",
        ])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    assert!(!map.exists());
    let large = dir.0.join("large.json");
    fs::File::create(&large)
        .unwrap()
        .set_len(16 * 1024 * 1024 + 1)
        .unwrap();
    let result = cli()
        .arg("import-osm")
        .arg("--input")
        .arg(&large)
        .arg("--output")
        .arg(&map)
        .args(["--origin-lat", "48.136", "--origin-lon", "10.0695"])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    assert!(!map.exists());
    assert!(String::from_utf8_lossy(&result.stderr).contains("exceeds 16 MiB"));
}
