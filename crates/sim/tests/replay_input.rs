use rustdriving_sim::{Scenario, simulate};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

struct TestDirectory(PathBuf);
impl TestDirectory {
    fn new(label: &str) -> Self {
        let path = std::env::temp_dir().join(format!(
            "rustdriving-replay-input-{}-{label}",
            std::process::id()
        ));
        fs::create_dir(&path).unwrap();
        Self(path)
    }
}
impl Drop for TestDirectory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn sensor_log() -> Vec<u8> {
    let mut scenario: Scenario =
        serde_json::from_str(include_str!("../../../scenarios/mission.json")).unwrap();
    scenario.duration = 1.0;
    let run = simulate(scenario, 7).unwrap();
    let mut bytes = Vec::new();
    run.sensor_log.unwrap().write(&mut bytes).unwrap();
    bytes
}

fn replay(input: &Path, output: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rustdriving"))
        .args(["replay", "--log"])
        .arg(input)
        .arg("--output")
        .arg(output)
        .output()
        .unwrap()
}

fn assert_collision_rejected(result: Output) {
    assert_eq!(result.status.code(), Some(2));
    assert!(
        String::from_utf8_lossy(&result.stderr).contains("aliases input log"),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
}

#[test]
fn replay_outputs_cannot_overwrite_or_delete_the_input_log() {
    let bytes = sensor_log();
    for name in ["outputs.jsonl", "replay.json"] {
        let directory = TestDirectory::new(name);
        let input = directory.0.join(name);
        let other = directory.0.join(if name == "outputs.jsonl" {
            "replay.json"
        } else {
            "outputs.jsonl"
        });
        fs::write(&input, &bytes).unwrap();
        fs::write(&other, b"previous artifact").unwrap();
        let result = replay(&input, &directory.0);
        assert!(fs::read(&input).unwrap() == bytes, "input log was modified");
        assert_eq!(fs::read(&other).unwrap(), b"previous artifact");
        assert_collision_rejected(result);
    }
}

#[cfg(unix)]
#[test]
fn replay_outputs_cannot_modify_input_through_symbolic_or_hard_links() {
    let bytes = sensor_log();
    for symbolic in [false, true] {
        for name in ["outputs.jsonl", "replay.json"] {
            let directory = TestDirectory::new(&format!("linked-{symbolic}-{name}"));
            let input = directory.0.join("sensors.jsonl");
            let alias = directory.0.join(name);
            fs::write(&input, &bytes).unwrap();
            if symbolic {
                std::os::unix::fs::symlink(&input, &alias).unwrap();
            } else {
                fs::hard_link(&input, &alias).unwrap();
            }
            let result = replay(&input, &directory.0);
            assert!(fs::read(&input).unwrap() == bytes, "input log was modified");
            assert!(fs::read(&alias).unwrap() == bytes, "alias was replaced");
            assert_collision_rejected(result);
        }
    }
}

#[test]
fn distinct_input_in_the_output_directory_replays_and_replaces_old_results() {
    let directory = TestDirectory::new("distinct");
    let input = directory.0.join("sensors.jsonl");
    let bytes = sensor_log();
    fs::write(&input, &bytes).unwrap();
    fs::write(directory.0.join("outputs.jsonl"), b"previous output").unwrap();
    fs::write(directory.0.join("replay.json"), b"previous report").unwrap();
    let result = replay(&input, &directory.0);
    assert_eq!(
        result.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert!(fs::read(&input).unwrap() == bytes);
    let report: serde_json::Value =
        serde_json::from_slice(&fs::read(directory.0.join("replay.json")).unwrap()).unwrap();
    assert_eq!(report["verified"], true);
    assert!(report["ticks"].as_u64().unwrap() > 0);
    let output = fs::read_to_string(directory.0.join("outputs.jsonl")).unwrap();
    assert_eq!(output.lines().count() as u64, report["ticks"]);
}
