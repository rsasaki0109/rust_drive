//! Shared subprocess acceptance checks for both actual run CLIs.
use super::{BINARY, NATIVE};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

pub struct Directory(pub PathBuf);
impl Directory {
    pub fn new(label: &str) -> Self {
        let path = std::env::temp_dir().join(format!(
            "rustdriving-run-cli-{}-{label}",
            std::process::id()
        ));
        fs::create_dir(&path).unwrap();
        Self(path)
    }
}
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
pub fn names() -> Vec<&'static str> {
    let mut result = vec!["run.json", "sensors.jsonl", "summary.json"];
    if NATIVE {
        result.push("scene.json");
    }
    result
}
pub fn scenario() -> Vec<u8> {
    let mut value: serde_json::Value =
        serde_json::from_str(include_str!("../../../../scenarios/mission.json")).unwrap();
    value["duration"] = 1.0.into();
    serde_json::to_vec_pretty(&value).unwrap()
}
pub fn command() -> Command {
    let mut cmd = Command::new(BINARY);
    if NATIVE {
        cmd.args(["--plant", "dynamic"]);
    } else {
        cmd.arg("run");
    }
    cmd
}
pub fn invoke(input: &Path, output: &Path, scene: Option<&Path>) -> Output {
    let mut cmd = command();
    cmd.arg("--scenario").arg(input).arg("--output").arg(output);
    if let Some(scene) = scene {
        cmd.arg("--scene").arg(scene);
    }
    cmd.output().unwrap()
}
pub fn no_stage(path: &Path) {
    assert!(fs::read_dir(path).unwrap().all(|entry| {
        !entry
            .unwrap()
            .file_name()
            .to_string_lossy()
            .starts_with(".run-stage-")
    }));
}
pub fn preserved_alias(result: Output) {
    assert_eq!(result.status.code(), Some(2));
    assert!(
        String::from_utf8_lossy(&result.stderr).contains("aliases input"),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
}
pub fn completed(output: &Path, result: Output) {
    assert_eq!(
        result.status.code(),
        Some(1),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let summary: serde_json::Value =
        serde_json::from_slice(&fs::read(output.join("summary.json")).unwrap()).unwrap();
    assert_eq!(summary["passed"], false);
    let run: serde_json::Value =
        serde_json::from_slice(&fs::read(output.join("run.json")).unwrap()).unwrap();
    assert_eq!(run["summary"], summary);
    let sensor_bytes = fs::read(output.join("sensors.jsonl")).unwrap();
    let report =
        rustdriving_pipeline::replay::verify(sensor_bytes.as_slice(), std::io::sink()).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks as u64, summary["steps"].as_u64().unwrap());
    for name in ["run.json", "sensors.jsonl", "summary.json"] {
        assert!(fs::symlink_metadata(output.join(name)).unwrap().is_file());
    }
    no_stage(output);
}

#[test]
fn direct_input_aliases_preserve_the_entire_previous_artifact_set() {
    let original = scenario();
    for name in names() {
        let root = Directory::new(&format!("direct-{name}"));
        for artifact in names() {
            fs::write(root.0.join(artifact), b"previous artifact").unwrap();
        }
        let input = root.0.join(name);
        fs::write(&input, &original).unwrap();
        preserved_alias(invoke(&input, &root.0, None));
        for artifact in names() {
            assert_eq!(
                fs::read(root.0.join(artifact)).unwrap(),
                if artifact == name {
                    original.as_slice()
                } else {
                    b"previous artifact"
                }
            );
        }
        no_stage(&root.0);
    }
}

#[cfg(unix)]
#[test]
fn symbolic_and_hard_link_input_aliases_are_rejected_before_invalidation() {
    let original = scenario();
    for symbolic in [false, true] {
        for name in names() {
            let root = Directory::new(&format!("input-link-{symbolic}-{name}"));
            let input = root.0.join("input.json");
            fs::write(&input, &original).unwrap();
            let output = root.0.join("out");
            fs::create_dir(&output).unwrap();
            for artifact in names() {
                if artifact != name {
                    fs::write(output.join(artifact), b"previous artifact").unwrap();
                }
            }
            if symbolic {
                std::os::unix::fs::symlink(&input, output.join(name)).unwrap();
            } else {
                fs::hard_link(&input, output.join(name)).unwrap();
            }
            preserved_alias(invoke(&input, &output, None));
            assert_eq!(fs::read(&input).unwrap(), original);
            assert_eq!(fs::read(output.join(name)).unwrap(), original);
            for artifact in names() {
                if artifact != name {
                    assert_eq!(
                        fs::read(output.join(artifact)).unwrap(),
                        b"previous artifact"
                    );
                }
            }
            no_stage(&output);
        }
    }
}

#[test]
fn malformed_and_missing_inputs_remove_previous_completion_and_data_files() {
    for missing in [false, true] {
        let root = Directory::new(&format!("bad-input-{missing}"));
        let input = root.0.join("input.json");
        if !missing {
            fs::write(&input, b"malformed scenario").unwrap();
        }
        for artifact in names() {
            fs::write(root.0.join(artifact), b"previous artifact").unwrap();
        }
        assert_eq!(invoke(&input, &root.0, None).status.code(), Some(2));
        for artifact in names() {
            assert!(!root.0.join(artifact).exists());
        }
        if !missing {
            assert_eq!(fs::read(input).unwrap(), b"malformed scenario");
        }
        no_stage(&root.0);
    }
}

#[cfg(unix)]
#[test]
fn output_links_are_replaced_without_writing_to_foreign_targets() {
    for symbolic in [false, true] {
        for exists in [false, true] {
            if !symbolic && !exists {
                continue;
            }
            let root = Directory::new(&format!("foreign-{symbolic}-{exists}"));
            let input = root.0.join("input.json");
            let original = scenario();
            fs::write(&input, &original).unwrap();
            let target = root.0.join("unrelated.json");
            if exists {
                fs::write(&target, b"unrelated bytes").unwrap();
            }
            let output = root.0.join("out");
            fs::create_dir(&output).unwrap();
            for name in names() {
                if symbolic {
                    std::os::unix::fs::symlink(&target, output.join(name)).unwrap();
                } else {
                    fs::hard_link(&target, output.join(name)).unwrap();
                }
            }
            completed(&output, invoke(&input, &output, None));
            assert_eq!(fs::read(&input).unwrap(), original);
            if exists {
                assert_eq!(fs::read(&target).unwrap(), b"unrelated bytes");
            } else {
                assert!(!target.exists());
            }
            if NATIVE {
                assert!(!output.join("scene.json").exists());
            }
        }
    }
}

#[test]
fn artifact_directories_are_preserved_and_cannot_retain_a_previous_success_file() {
    for name in names() {
        let root = Directory::new(&format!("directory-{name}"));
        let input = root.0.join("input.json");
        let original = scenario();
        fs::write(&input, &original).unwrap();
        for artifact in names() {
            if artifact != name {
                fs::write(root.0.join(artifact), b"previous artifact").unwrap();
            }
        }
        fs::create_dir(root.0.join(name)).unwrap();
        fs::write(root.0.join(name).join("keep"), b"unrelated contents").unwrap();
        assert_eq!(invoke(&input, &root.0, None).status.code(), Some(2));
        assert_eq!(
            fs::read(root.0.join(name).join("keep")).unwrap(),
            b"unrelated contents"
        );
        if name != "summary.json" {
            assert!(!root.0.join("summary.json").exists());
        }
        assert_eq!(fs::read(input).unwrap(), original);
        no_stage(&root.0);
    }
}

#[test]
fn complete_failed_runs_are_retained_and_a_distinct_input_in_output_is_supported() {
    let root = Directory::new("completed-failed");
    let input = root.0.join("input.json");
    let original = scenario();
    fs::write(&input, &original).unwrap();
    for _ in 0..2 {
        completed(&root.0, invoke(&input, &root.0, None));
        assert_eq!(fs::read(&input).unwrap(), original);
    }
}

#[test]
fn absent_required_input_also_invalidates_old_completion() {
    let root = Directory::new("absent-option");
    for name in names() {
        fs::write(root.0.join(name), b"previous artifact").unwrap();
    }
    let result = command().arg("--output").arg(&root.0).output().unwrap();
    assert_eq!(result.status.code(), Some(2));
    for name in names() {
        assert!(!root.0.join(name).exists());
    }
    no_stage(&root.0);
}
