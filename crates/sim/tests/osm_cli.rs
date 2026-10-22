//! Public map import must produce a runnable sensor-only scenario, not just a graph.
use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
    process::Command,
    sync::atomic::{AtomicU64, Ordering},
};

struct Workspace(PathBuf);
impl Workspace {
    fn new() -> Self {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let path = std::env::temp_dir().join(format!(
            "rustdriving-osm-cli-{}-{}",
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
    Command::new(env!("CARGO_BIN_EXE_rustdriving"))
}

fn import_command(input: &Path, map: &Path, scenario: Option<&Path>) -> Command {
    let mut command = cli();
    command
        .arg("import-osm")
        .arg("--input")
        .arg(input)
        .arg("--output")
        .arg(map)
        .args(["--origin-lat", "48.136", "--origin-lon", "10.0695"]);
    if let Some(scenario) = scenario {
        command.arg("--scenario-output").arg(scenario).args([
            "--start",
            "osm-node-7119017425",
            "--goal",
            "osm-node-274969423",
        ]);
    }
    command
}

fn source(path: &Path) {
    fs::write(
        path,
        include_bytes!("../../../maps/osm/german-road-extract.json"),
    )
    .unwrap();
}

// Observe entries independently of exporter internals; never follow directory links.
fn snapshot(root: &Path) -> BTreeMap<PathBuf, Vec<u8>> {
    fn walk(root: &Path, directory: &Path, entries: &mut BTreeMap<PathBuf, Vec<u8>>) {
        for entry in fs::read_dir(directory).unwrap() {
            let path = entry.unwrap().path();
            let metadata = fs::symlink_metadata(&path).unwrap();
            let contents = if metadata.is_symlink() {
                format!("link:{}", fs::read_link(&path).unwrap().display()).into_bytes()
            } else if metadata.is_dir() {
                walk(root, &path, entries);
                b"directory".to_vec()
            } else {
                fs::read(&path).unwrap()
            };
            entries.insert(path.strip_prefix(root).unwrap().to_owned(), contents);
        }
    }
    let mut entries = BTreeMap::new();
    walk(root, root, &mut entries);
    entries
}

#[test]
fn input_used_as_each_output_is_preserved_without_other_exports() {
    for index in 0..3 {
        let dir = Workspace::new();
        let map = dir.0.join("map.json");
        let report = map.with_extension("import.json");
        let scenario = dir.0.join("scenario.json");
        let input = [&map, &report, &scenario][index];
        source(input);
        let before = snapshot(&dir.0);
        let result = import_command(input, &map, Some(&scenario))
            .output()
            .unwrap();
        assert_eq!(result.status.code(), Some(2));
        assert_eq!(snapshot(&dir.0), before);
    }
}

#[cfg(unix)]
#[test]
fn linked_input_at_each_output_preserves_source_link_and_all_other_entries() {
    use std::os::unix::fs::{MetadataExt, symlink};
    for symbolic in [false, true] {
        for index in 0..3 {
            let dir = Workspace::new();
            let input = dir.0.join("source.json");
            source(&input);
            let map = dir.0.join("map.json");
            let report = map.with_extension("import.json");
            let scenario = dir.0.join("scenario.json");
            let alias = [&map, &report, &scenario][index];
            if symbolic {
                symlink(&input, alias).unwrap();
            } else {
                fs::hard_link(&input, alias).unwrap();
            }
            let inode = fs::symlink_metadata(alias).unwrap().ino();
            let before = snapshot(&dir.0);
            let result = import_command(&input, &map, Some(&scenario))
                .output()
                .unwrap();
            assert_eq!(result.status.code(), Some(2));
            assert_eq!(snapshot(&dir.0), before);
            assert_eq!(fs::symlink_metadata(alias).unwrap().ino(), inode);
        }
    }
}

#[test]
fn existing_files_and_directories_at_each_output_are_preserved_without_partial_exports() {
    for directory in [false, true] {
        for index in 0..3 {
            let dir = Workspace::new();
            let input = dir.0.join("source.json");
            source(&input);
            let map = dir.0.join("map.json");
            let report = map.with_extension("import.json");
            let scenario = dir.0.join("scenario.json");
            let occupied = [&map, &report, &scenario][index];
            if directory {
                fs::create_dir(occupied).unwrap();
                fs::write(occupied.join("keep"), b"existing directory contents").unwrap();
            } else {
                fs::write(occupied, b"existing file contents").unwrap();
            }
            let before = snapshot(&dir.0);
            let result = import_command(&input, &map, Some(&scenario))
                .output()
                .unwrap();
            assert_eq!(result.status.code(), Some(2));
            assert_eq!(snapshot(&dir.0), before);
        }
    }
}

#[cfg(unix)]
#[test]
fn foreign_and_dangling_links_at_each_output_are_preserved_without_following_them() {
    use std::os::unix::fs::symlink;
    for dangling in [false, true] {
        for index in 0..3 {
            let dir = Workspace::new();
            let input = dir.0.join("source.json");
            source(&input);
            let map = dir.0.join("map.json");
            let report = map.with_extension("import.json");
            let scenario = dir.0.join("scenario.json");
            let target = dir.0.join("foreign.json");
            if !dangling {
                fs::write(&target, b"foreign contents").unwrap();
            }
            symlink(&target, [&map, &report, &scenario][index]).unwrap();
            let before = snapshot(&dir.0);
            let result = import_command(&input, &map, Some(&scenario))
                .output()
                .unwrap();
            assert_eq!(result.status.code(), Some(2));
            assert_eq!(snapshot(&dir.0), before);
        }
    }
}

#[test]
fn equivalent_destinations_with_parent_components_are_rejected_before_export() {
    for report_alias in [false, true] {
        for parent_components in [false, true] {
            let dir = Workspace::new();
            let input = dir.0.join("source.json");
            source(&input);
            let map = dir.0.join("map.json");
            let name = if report_alias {
                "map.import.json"
            } else {
                "map.json"
            };
            fs::create_dir(dir.0.join("child")).unwrap();
            let scenario = if parent_components {
                dir.0.join("child").join("..").join(name)
            } else {
                dir.0.join(name)
            };
            let before = snapshot(&dir.0);
            let result = import_command(&input, &map, Some(&scenario))
                .output()
                .unwrap();
            assert_eq!(result.status.code(), Some(2));
            assert!(String::from_utf8_lossy(&result.stderr).contains("paths must differ"));
            assert_eq!(snapshot(&dir.0), before);
        }
    }
}

#[cfg(unix)]
#[test]
fn equivalent_destinations_through_a_parent_link_are_rejected_before_export() {
    use std::os::unix::fs::symlink;
    for name in ["map.json", "map.import.json"] {
        let dir = Workspace::new();
        let input = dir.0.join("source.json");
        source(&input);
        let map = dir.0.join("map.json");
        symlink(&dir.0, dir.0.join("alias")).unwrap();
        let scenario = dir.0.join("alias").join(name);
        let before = snapshot(&dir.0);
        let result = import_command(&input, &map, Some(&scenario))
            .output()
            .unwrap();
        assert_eq!(result.status.code(), Some(2));
        assert!(String::from_utf8_lossy(&result.stderr).contains("paths must differ"));
        assert_eq!(snapshot(&dir.0), before);
    }
}

#[test]
fn invalid_scenario_parent_leaves_no_map_or_report() {
    let dir = Workspace::new();
    let input = dir.0.join("source.json");
    source(&input);
    let map = dir.0.join("map.json");
    let parent = dir.0.join("blocked-parent");
    fs::write(&parent, b"keep this file").unwrap();
    let before = snapshot(&dir.0);
    let result = import_command(&input, &map, Some(&parent.join("scenario.json")))
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    assert_eq!(snapshot(&dir.0), before);
}

#[test]
fn fresh_exports_support_distinct_directories_and_map_only_with_matching_report_stdout() {
    for with_scenario in [false, true] {
        let dir = Workspace::new();
        let input = dir.0.join("source.json");
        source(&input);
        let map = dir.0.join("maps with spaces/map.json");
        let scenario = dir.0.join("scenarios with spaces/scenario.json");
        let result = import_command(&input, &map, with_scenario.then_some(scenario.as_path()))
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        let network: serde_json::Value = serde_json::from_slice(&fs::read(&map).unwrap()).unwrap();
        assert_eq!(network["nodes"].as_array().unwrap().len(), 6);
        assert_eq!(network["edges"].as_array().unwrap().len(), 10);
        let mut report = fs::read(map.with_extension("import.json")).unwrap();
        report.push(b'\n');
        assert_eq!(result.stdout, report);
        if with_scenario {
            let generated: serde_json::Value =
                serde_json::from_slice(&fs::read(&scenario).unwrap()).unwrap();
            assert_eq!(generated["navigation"]["network"], network);
            assert!(
                cli()
                    .arg("validate-scenario")
                    .arg("--scenario")
                    .arg(&scenario)
                    .output()
                    .unwrap()
                    .status
                    .success()
            );
        } else {
            assert!(!scenario.exists());
        }
        assert!(
            !snapshot(&dir.0)
                .keys()
                .any(|path| path.to_string_lossy().contains(".osm-stage-"))
        );
    }
}

#[test]
fn repeating_a_successful_import_preserves_all_previous_exports() {
    let dir = Workspace::new();
    let input = dir.0.join("source.json");
    source(&input);
    let map = dir.0.join("map.json");
    let scenario = dir.0.join("scenario.json");
    let result = import_command(&input, &map, Some(&scenario))
        .output()
        .unwrap();
    assert!(result.status.success());
    let before = snapshot(&dir.0);
    let result = import_command(&input, &map, Some(&scenario))
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    assert_eq!(snapshot(&dir.0), before);
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
