use super::*;
use serde::{Serializer, ser::SerializeSeq};

struct Workspace(PathBuf);
impl Workspace {
    fn new() -> Self {
        let id = NEXT_STAGE.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "rustdriving-osm-output-{}-{id}",
            std::process::id()
        ));
        fs::create_dir(&path).unwrap();
        Self(path)
    }
}
impl Drop for Workspace {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

struct Interrupted;
impl Serialize for Interrupted {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut sequence = serializer.serialize_seq(Some(2))?;
        sequence.serialize_element("written prefix")?;
        Err(serde::ser::Error::custom("injected serialization failure"))
    }
}

#[test]
fn serialization_failure_at_each_position_publishes_nothing_and_cleans_staging() {
    for failed in 0..3 {
        let root = Workspace::new();
        let paths: Vec<_> = ["map.json", "scenario.json", "map.import.json"]
            .map(|name| root.0.join(name))
            .into();
        {
            let output =
                ImportOutput::new(&paths.iter().map(PathBuf::as_path).collect::<Vec<_>>()).unwrap();
            for index in 0..failed {
                output
                    .json(index, &serde_json::json!({"complete": index}))
                    .unwrap();
            }
            assert!(output.json(failed, &Interrupted).is_err());
            assert!(paths.iter().all(|path| !path.exists()));
        }
        assert_eq!(fs::read_dir(&root.0).unwrap().count(), 0);
    }
}

#[test]
fn late_collision_at_each_position_preserves_foreign_entry_and_rolls_back_other_directories() {
    for occupied in 0..3 {
        for directory in [false, true] {
            let root = Workspace::new();
            let paths: Vec<_> = [
                "maps/map.json",
                "scenarios/scenario.json",
                "reports/import.json",
            ]
            .map(|name| root.0.join(name))
            .into();
            let output =
                ImportOutput::new(&paths.iter().map(PathBuf::as_path).collect::<Vec<_>>()).unwrap();
            for index in 0..3 {
                output
                    .json(index, &serde_json::json!({"complete": index}))
                    .unwrap();
            }
            let foreign = if directory {
                fs::create_dir(&paths[occupied]).unwrap();
                paths[occupied].join("keep")
            } else {
                paths[occupied].clone()
            };
            fs::write(&foreign, b"foreign bytes").unwrap();
            assert!(output.publish().is_err());
            assert_eq!(fs::read(&foreign).unwrap(), b"foreign bytes");
            for (index, path) in paths.iter().enumerate() {
                if index != occupied {
                    assert!(!path.exists());
                }
                assert_eq!(
                    fs::read_dir(path.parent().unwrap()).unwrap().count(),
                    usize::from(index == occupied)
                );
            }
        }
    }
}

#[test]
fn missing_last_staged_payload_rolls_back_complete_earlier_files() {
    let root = Workspace::new();
    let paths = [root.0.join("map.json"), root.0.join("map.import.json")];
    let output = ImportOutput::new(&[&paths[0], &paths[1]]).unwrap();
    output
        .json(0, &serde_json::json!({"complete": true}))
        .unwrap();
    assert!(output.publish().is_err());
    assert_eq!(fs::read_dir(&root.0).unwrap().count(), 0);
}
