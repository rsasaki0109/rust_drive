use std::{fs, process::Command};
fn command(args: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_rustdriving"))
        .args(args)
        .output()
        .unwrap()
}
#[test]
fn cli_compiles_positions_preserves_original_map_and_never_clobbers_inputs() {
    let root = std::env::temp_dir().join(format!("rustdriving-road-route-{}", std::process::id()));
    fs::create_dir(&root).unwrap();
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/mid-edge-clipped.json")).unwrap();
    let input = root.join("map.json");
    let output = root.join("scenario.json");
    let bytes = serde_json::to_vec(&fixture["navigation"]["network"]).unwrap();
    fs::write(&input, &bytes).unwrap();
    let args = [
        "road-route",
        "--map",
        input.to_str().unwrap(),
        "--start-edge",
        "approach",
        "--start-s",
        "12.5",
        "--goal-edge",
        "east-exit",
        "--goal-s",
        "25.25",
        "--scenario-output",
        output.to_str().unwrap(),
    ];
    let result = command(&args);
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(report["plan"]["distance_m"], 172.75);
    let scenario: rustdriving_sim::Scenario =
        serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
    scenario.validate().unwrap();
    let config = rustdriving_sim::pipeline_config(&scenario);
    let nav = config.navigation.unwrap();
    let original: rustdriving_routing::RoadNetworkSpec =
        serde_json::from_value(fixture["navigation"]["network"].clone()).unwrap();
    assert_eq!(
        serde_json::to_value(&nav.network).unwrap(),
        serde_json::to_value(&original).unwrap()
    );
    assert_eq!(config.initial_pose.position.x, 12.5);
    let saved = fs::read(&output).unwrap();
    assert!(!command(&args).status.success());
    assert_eq!(fs::read(&output).unwrap(), saved);
    let bad = command(&[
        "road-route",
        "--map",
        input.to_str().unwrap(),
        "--start",
        "start",
        "--start-edge",
        "approach",
        "--start-s",
        "12.5",
        "--goal",
        "east",
        "--scenario-output",
        input.to_str().unwrap(),
    ]);
    assert!(!bad.status.success());
    assert_eq!(fs::read(&input).unwrap(), bytes);
    fs::remove_dir_all(root).unwrap();
}
