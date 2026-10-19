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

fn recomputed_outputs(input: &[u8]) -> Vec<u8> {
    let mut outputs = Vec::new();
    rustdriving_pipeline::replay::verify(input, &mut outputs).unwrap();
    outputs
}

fn assert_verified_outputs(result: Output, directory: &Path, expected: &[u8]) {
    assert_eq!(
        result.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let summary = directory.join("replay.json");
    let outputs = directory.join("outputs.jsonl");
    let report: serde_json::Value = serde_json::from_slice(&fs::read(&summary).unwrap()).unwrap();
    assert_eq!(report["verified"], true);
    assert!(
        fs::read(&outputs).unwrap() == expected,
        "recomputed outputs were replaced"
    );
    let ticks = std::str::from_utf8(expected).unwrap().lines().count() as u64;
    assert!(ticks > 0);
    assert_eq!(report["ticks"], ticks);
    for artifact in [&summary, &outputs] {
        assert!(fs::symlink_metadata(artifact).unwrap().is_file());
    }
    assert_no_staging(directory);
}

fn assert_no_staging(directory: &Path) {
    assert!(fs::read_dir(directory).unwrap().all(|entry| {
        !entry
            .unwrap()
            .file_name()
            .to_string_lossy()
            .starts_with(".replay-stage-")
    }));
}

#[test]
fn late_verification_failures_publish_no_command_prefix_or_success_report() {
    let valid = sensor_log();
    let lines: Vec<_> = std::str::from_utf8(&valid).unwrap().lines().collect();
    let tick_index = lines.len() - 2;
    let mut changed: serde_json::Value = serde_json::from_str(lines[tick_index]).unwrap();
    let steering = changed["tick"]["expected"]["command"]["steering"]
        .as_f64()
        .unwrap();
    changed["tick"]["expected"]["command"]["steering"] = (steering + 0.125).into();
    let mut mismatched = lines
        .iter()
        .map(|line| (*line).to_owned())
        .collect::<Vec<_>>();
    mismatched[tick_index] = serde_json::to_string(&changed).unwrap();
    let truncated = lines[..lines.len() - 1].join("\n") + "\n";
    for (label, bytes) in [
        ("mismatched-last-tick", mismatched.join("\n") + "\n"),
        ("missing-footer", truncated.clone()),
        ("malformed-tail", truncated + "invalid JSON\n"),
        (
            "after-footer",
            String::from_utf8(valid.clone()).unwrap() + "invalid JSON\n",
        ),
    ] {
        let directory = TestDirectory::new(label);
        let input = directory.0.join("sensors.jsonl");
        fs::write(&input, bytes.as_bytes()).unwrap();
        fs::write(directory.0.join("outputs.jsonl"), b"previous commands").unwrap();
        fs::write(directory.0.join("replay.json"), br#"{"verified":true}"#).unwrap();
        let result = replay(&input, &directory.0);
        assert_eq!(result.status.code(), Some(2), "{label}");
        assert_eq!(fs::read(&input).unwrap(), bytes.as_bytes());
        assert!(!directory.0.join("outputs.jsonl").exists(), "{label}");
        assert!(!directory.0.join("replay.json").exists(), "{label}");
        assert_no_staging(&directory.0);
    }
}

#[test]
fn missing_input_invalidates_both_previous_replay_artifacts() {
    let directory = TestDirectory::new("missing-input-old-stream");
    fs::write(directory.0.join("outputs.jsonl"), b"previous commands").unwrap();
    fs::write(directory.0.join("replay.json"), br#"{"verified":true}"#).unwrap();
    let result = replay(&directory.0.join("missing.jsonl"), &directory.0);
    assert_eq!(result.status.code(), Some(2));
    assert!(!directory.0.join("outputs.jsonl").exists());
    assert!(!directory.0.join("replay.json").exists());
    assert_no_staging(&directory.0);
}

#[test]
fn artifact_directories_and_their_contents_are_preserved_on_failure() {
    let bytes = sensor_log();
    for name in ["outputs.jsonl", "replay.json"] {
        let directory = TestDirectory::new(&format!("directory-{name}"));
        let input = directory.0.join("sensors.jsonl");
        fs::write(&input, &bytes).unwrap();
        let artifact = directory.0.join(name);
        fs::create_dir(&artifact).unwrap();
        fs::write(artifact.join("keep.txt"), b"unrelated contents").unwrap();
        let result = replay(&input, &directory.0);
        assert_eq!(result.status.code(), Some(2));
        assert_eq!(fs::read(&input).unwrap(), bytes);
        assert_eq!(
            fs::read(artifact.join("keep.txt")).unwrap(),
            b"unrelated contents"
        );
        assert_no_staging(&directory.0);
    }
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
    assert!(fs::read(&input).unwrap() == bytes);
    assert_verified_outputs(result, &directory.0, &recomputed_outputs(&bytes));
}

#[cfg(unix)]
#[test]
fn replay_replaces_cross_linked_artifacts_with_distinct_verified_files() {
    let bytes = sensor_log();
    let expected = recomputed_outputs(&bytes);
    for name in ["outputs.jsonl", "replay.json"] {
        for target_exists in [false, true] {
            let directory = TestDirectory::new(&format!("cross-{name}-{target_exists}"));
            let input = directory.0.join("sensors.jsonl");
            fs::write(&input, &bytes).unwrap();
            let target = if name == "outputs.jsonl" {
                "replay.json"
            } else {
                "outputs.jsonl"
            };
            if target_exists {
                fs::write(directory.0.join(target), b"previous artifact").unwrap();
            }
            std::os::unix::fs::symlink(target, directory.0.join(name)).unwrap();
            let result = replay(&input, &directory.0);
            assert!(fs::read(&input).unwrap() == bytes);
            assert_verified_outputs(result, &directory.0, &expected);
        }
    }
}

#[cfg(unix)]
#[test]
fn replay_replaces_output_links_without_modifying_their_external_targets() {
    let bytes = sensor_log();
    let expected = recomputed_outputs(&bytes);
    for symbolic in [false, true] {
        for target_exists in [false, true] {
            if !symbolic && !target_exists {
                continue;
            }
            for name in ["outputs.jsonl", "replay.json"] {
                let directory =
                    TestDirectory::new(&format!("external-{symbolic}-{target_exists}-{name}"));
                let input = directory.0.join("sensors.jsonl");
                let target = directory.0.join("external.json");
                let output = directory.0.join("replay");
                fs::create_dir(&output).unwrap();
                fs::write(&input, &bytes).unwrap();
                if target_exists {
                    fs::write(&target, b"unrelated file").unwrap();
                }
                if symbolic {
                    std::os::unix::fs::symlink("../external.json", output.join(name)).unwrap();
                } else {
                    fs::hard_link(&target, output.join(name)).unwrap();
                }
                let result = replay(&input, &output);
                if target_exists {
                    assert!(
                        fs::read(&target).unwrap() == b"unrelated file",
                        "external file was modified through {name}"
                    );
                } else {
                    assert!(!target.exists(), "external file was created through {name}");
                }
                assert!(fs::read(&input).unwrap() == bytes);
                assert_verified_outputs(result, &output, &expected);
            }
        }
    }
}

#[cfg(unix)]
#[test]
fn missing_input_clears_a_dangling_success_report_link() {
    let directory = TestDirectory::new("missing-dangling-report");
    let summary = directory.0.join("replay.json");
    std::os::unix::fs::symlink("outputs.jsonl", &summary).unwrap();
    let result = replay(&directory.0.join("missing.jsonl"), &directory.0);
    assert_eq!(result.status.code(), Some(2));
    assert_eq!(
        fs::symlink_metadata(&summary).unwrap_err().kind(),
        std::io::ErrorKind::NotFound
    );
    assert!(!directory.0.join("outputs.jsonl").exists());
}
