//! Restricted map search must participate in driving and sensor-only replay.
use rustdriving_pipeline::replay::verify;
use rustdriving_sim::{Scenario, pipeline_config, simulate};
use std::io::Cursor;

fn scenario(name: &str) -> Scenario {
    serde_json::from_str(
        &std::fs::read_to_string(format!(
            "{}/../../scenarios/{name}.json",
            env!("CARGO_MANIFEST_DIR")
        ))
        .unwrap(),
    )
    .unwrap()
}

#[test]
fn prohibited_and_only_turns_select_and_drive_the_legal_detour() {
    for name in ["turn-no-main", "turn-only-detour", "turn-arrival-memory"] {
        let run = simulate(scenario(name), 7).unwrap();
        assert!(run.summary.passed, "{name}: {:?}", run.summary);
        assert_eq!(
            run.navigation.as_ref().unwrap().edge_ids,
            ["approach", "detour", "east-exit"]
        );
        let log = run.sensor_log.as_ref().unwrap();
        assert!(log.header.config.navigation.is_some());
        for tick in &log.ticks {
            assert_eq!(
                tick.expected.navigation.as_ref().unwrap().active_edges,
                ["approach", "detour", "east-exit"]
            );
        }
        let mut bytes = Vec::new();
        log.write(&mut bytes).unwrap();
        let replay = verify(Cursor::new(bytes), std::io::sink()).unwrap();
        assert!(replay.verified);
        assert_eq!(replay.ticks, run.summary.steps);
    }
}

#[test]
fn replay_rejects_removed_turn_rule_instead_of_trusting_resolved_geometry() {
    let run = simulate(scenario("turn-no-main"), 7).unwrap();
    let mut log = run.sensor_log.unwrap();
    log.header
        .config
        .navigation
        .as_mut()
        .unwrap()
        .network
        .turn_restrictions
        .clear();
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    let error = verify(Cursor::new(bytes), std::io::sink()).unwrap_err();
    assert!(error.contains("initial route"), "{error}");
}

#[test]
fn closing_the_only_allowed_turn_is_unreachable_without_a_forbidden_fallback() {
    let mut spec = scenario("turn-only-detour");
    spec.navigation.as_mut().unwrap().closed_edges = vec!["detour".into()];
    assert!(spec.validate().is_err());
    assert!(simulate(spec, 7).is_err());
}

#[test]
fn legacy_unrestricted_static_map_still_uses_resolved_route_replay() {
    assert!(
        pipeline_config(&scenario("route-direct"))
            .navigation
            .is_none()
    );
}
