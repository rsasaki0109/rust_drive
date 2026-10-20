use std::{fs, process::Command};

#[test]
fn scenario_preflight_rejects_invalid_geometry_without_creating_driving_outputs() {
    let root = std::env::temp_dir().join(format!("rustdriving-validation-{}", std::process::id()));
    fs::create_dir(&root).unwrap();
    let input = root.join("scenario.json");
    let mut value: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/blocked.json")).unwrap();
    let command = || {
        Command::new(env!("CARGO_BIN_EXE_rustdriving"))
            .current_dir(&root)
            .arg("validate-scenario")
            .arg("--scenario")
            .arg(&input)
            .output()
            .unwrap()
    };
    value["objects"][0]["radius"] = serde_json::json!(3.3);
    fs::write(&input, serde_json::to_vec(&value).unwrap()).unwrap();
    let invalid = command();
    assert_eq!(invalid.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&invalid.stderr).contains("invalid object parameters"));
    value["objects"][0]["radius"] = serde_json::json!(2.0);
    value["lateral_candidate_margin_m"] = serde_json::json!(0.35);
    let bytes = serde_json::to_vec(&value).unwrap();
    fs::write(&input, &bytes).unwrap();
    let valid = command();
    assert!(valid.status.success());
    assert_eq!(
        serde_json::from_slice::<serde_json::Value>(&valid.stdout).unwrap()["valid"],
        true
    );
    assert_eq!(fs::read(&input).unwrap(), bytes);
    assert_eq!(fs::read_dir(&root).unwrap().count(), 1);
    let missing = Command::new(env!("CARGO_BIN_EXE_rustdriving"))
        .arg("validate-scenario")
        .output()
        .unwrap();
    assert_eq!(missing.status.code(), Some(2));
    fs::remove_dir_all(root).unwrap();
}
