use rustdriving_sim::{Scenario, pipeline_config};

#[test]
fn scenario_validation_and_operational_header_retain_the_stop_search_policy() {
    let mut s: Scenario =
        serde_json::from_str(include_str!("../../../scenarios/mission.json")).unwrap();
    assert!(!s.search_obstruction_stops);
    assert!(
        serde_json::to_value(&s)
            .unwrap()
            .get("search_obstruction_stops")
            .is_none()
    );
    s.search_obstruction_stops = true;
    assert!(s.validate().is_err());
    s.settle_obstruction_stops = true;
    assert!(s.validate().is_ok());
    assert!(pipeline_config(&s).search_obstruction_stops);
}
