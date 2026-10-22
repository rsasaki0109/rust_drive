const BINARY: &str = env!("CARGO_BIN_EXE_rustdriving-rne");
const NATIVE: bool = true;
#[path = "../../../crates/sim/tests/fixtures/run_output_cases.rs"]
mod cases;

use std::fs;
const SCENE: &[u8] = br#"{"schema_version":1,"name":"publication scene","static_cuboids":[{"id":"far-box","center_m":[100.0,10.0,1.0],"half_extents_m":[1.0,1.0,1.0],"yaw_rad":0.0}]}"#;

#[test]
fn scene_inputs_are_protected_from_every_native_artifact_destination() {
    for name in cases::names() {
        let root = cases::Directory::new(&format!("scene-alias-{name}"));
        let input = root.0.join("input.json");
        let original = cases::scenario();
        fs::write(&input, &original).unwrap();
        for artifact in cases::names() {
            fs::write(root.0.join(artifact), b"previous artifact").unwrap();
        }
        let scene = root.0.join(name);
        fs::write(&scene, SCENE).unwrap();
        cases::preserved_alias(cases::invoke(&input, &root.0, Some(&scene)));
        assert_eq!(fs::read(&input).unwrap(), original);
        assert_eq!(fs::read(scene).unwrap(), SCENE);
        for artifact in cases::names() {
            if artifact != name {
                assert_eq!(
                    fs::read(root.0.join(artifact)).unwrap(),
                    b"previous artifact"
                );
            }
        }
        cases::no_stage(&root.0);
    }
}

#[test]
fn a_complete_scene_is_published_and_removed_by_a_later_sceneless_run() {
    let root = cases::Directory::new("scene-reuse");
    let input = root.0.join("input.json");
    fs::write(&input, cases::scenario()).unwrap();
    let scene = root.0.join("input-scene.json");
    fs::write(&scene, SCENE).unwrap();
    cases::completed(&root.0, cases::invoke(&input, &root.0, Some(&scene)));
    let evidence = root.0.join("scene.json");
    assert!(fs::symlink_metadata(&evidence).unwrap().is_file());
    assert!(
        serde_json::from_slice::<serde_json::Value>(&fs::read(evidence).unwrap())
            .unwrap()
            .is_object()
    );
    cases::completed(&root.0, cases::invoke(&input, &root.0, None));
    assert!(!root.0.join("scene.json").exists());
    assert_eq!(fs::read(scene).unwrap(), SCENE);
}

#[test]
fn missing_or_invalid_scene_does_not_leave_prior_acceptance_or_scene_evidence() {
    for missing in [false, true] {
        let root = cases::Directory::new(&format!("bad-scene-{missing}"));
        let input = root.0.join("input.json");
        fs::write(&input, cases::scenario()).unwrap();
        let scene = root.0.join("input-scene.json");
        if !missing {
            fs::write(&scene, b"malformed scene").unwrap();
        }
        for name in cases::names() {
            fs::write(root.0.join(name), b"previous artifact").unwrap();
        }
        assert_eq!(
            cases::invoke(&input, &root.0, Some(&scene)).status.code(),
            Some(2)
        );
        for name in cases::names() {
            assert!(!root.0.join(name).exists());
        }
        cases::no_stage(&root.0);
    }
}
