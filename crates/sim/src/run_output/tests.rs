use super::*;
use crate::{Scenario, simulate};

struct Directory(PathBuf);
impl Directory {
    fn new(label: &str) -> Self {
        let path = std::env::temp_dir().join(format!(
            "rustdriving-run-publication-{}-{label}",
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
fn run() -> Run {
    let mut scenario: Scenario =
        serde_json::from_str(include_str!("../../../../scenarios/mission.json")).unwrap();
    scenario.duration = 1.0;
    simulate(scenario, 7).unwrap()
}

#[test]
fn late_invalid_sensor_record_leaves_no_final_artifacts_or_staging() {
    let root = Directory::new("invalid-log");
    for name in ["run.json", "sensors.jsonl", "summary.json"] {
        fs::write(root.0.join(name), b"previous result").unwrap();
    }
    let mut run = run();
    run.sensor_log
        .as_mut()
        .unwrap()
        .ticks
        .last_mut()
        .unwrap()
        .expected
        .command
        .steering = f64::NAN;
    let output = RunOutput::reference(&root.0, &[]).unwrap();
    assert!(output.finish(&run, None::<&()>).is_err());
    assert_eq!(fs::read_dir(&root.0).unwrap().count(), 0);
}

#[test]
fn late_scene_serialization_failure_removes_all_private_prefixes() {
    struct Failing;
    impl Serialize for Failing {
        fn serialize<S: serde::Serializer>(
            &self,
            serializer: S,
        ) -> std::result::Result<S::Ok, S::Error> {
            use serde::ser::SerializeSeq;
            let mut sequence = serializer.serialize_seq(None)?;
            sequence.serialize_element("partial scene")?;
            Err(serde::ser::Error::custom("injected scene write failure"))
        }
    }
    let root = Directory::new("invalid-scene");
    let output = RunOutput::native(&root.0, &[]).unwrap();
    assert!(output.finish(&run(), Some(&Failing)).is_err());
    assert_eq!(fs::read_dir(&root.0).unwrap().count(), 0);
}

#[test]
fn publication_collisions_preserve_new_entries_and_rollback_only_published_files() {
    for scene in [false, true] {
        for name in DATA
            .into_iter()
            .filter(|name| *name != DATA[2] || scene)
            .chain(["summary.json"])
        {
            let root = Directory::new(&format!("collision-{scene}-{name}"));
            {
                let output = RunOutput::native(&root.0, &[]).unwrap();
                for file in DATA
                    .into_iter()
                    .filter(|name| *name != DATA[2] || scene)
                    .chain(["summary.json"])
                {
                    fs::write(output.stage.join(file), b"complete staged file").unwrap();
                }
                fs::write(root.0.join(name), b"new unrelated entry").unwrap();
                assert!(output.publish(scene).is_err());
                assert_eq!(fs::read(root.0.join(name)).unwrap(), b"new unrelated entry");
                for other in DATA
                    .into_iter()
                    .chain(["summary.json"])
                    .filter(|other| *other != name)
                {
                    assert!(!root.0.join(other).exists());
                }
            }
            assert_eq!(fs::read_dir(&root.0).unwrap().count(), 1);
        }
    }
}

#[test]
fn complete_failed_acceptance_preserves_the_preceding_encodings() {
    let root = Directory::new("encoded-failed-run");
    let run = run();
    assert!(!run.summary.passed);
    let scene = serde_json::json!({"diagnostic": [1, 2, 3]});
    let mut sensor_bytes = Vec::new();
    run.sensor_log
        .as_ref()
        .unwrap()
        .write(&mut sensor_bytes)
        .unwrap();
    // The preceding CLI used these encoders. Compare bytes, independent of the
    // new staging/publication implementation and its file operation ordering.
    let expected = [
        ("run.json", serde_json::to_vec(&run).unwrap()),
        ("sensors.jsonl", sensor_bytes),
        ("scene.json", serde_json::to_vec(&scene).unwrap()),
        (
            "summary.json",
            serde_json::to_vec_pretty(&run.summary).unwrap(),
        ),
    ];
    let output = RunOutput::native(&root.0, &[]).unwrap();
    let printed = output.finish(&run, Some(&scene)).unwrap();
    for (name, bytes) in expected {
        assert_eq!(fs::read(root.0.join(name)).unwrap(), bytes, "{name}");
    }
    assert_eq!(
        printed.as_bytes(),
        fs::read(root.0.join("summary.json")).unwrap()
    );
    assert_eq!(fs::read_dir(&root.0).unwrap().count(), 4);
}
