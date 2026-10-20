use rustdriving_sim::{Scenario, pipeline_config, simulate};

#[test]
fn full_rate_evaluation_records_every_tick_without_changing_sensor_inputs_or_driver_outputs() {
    let mut scenario: Scenario =
        serde_json::from_str(include_str!("../../../scenarios/mission.json")).unwrap();
    scenario.duration = 1.;
    scenario.objects.clear();
    let sparse = simulate(scenario.clone(), 7).unwrap();
    scenario.full_rate_recording = true;
    assert!(
        serde_json::to_value(pipeline_config(&scenario))
            .unwrap()
            .get("full_rate_recording")
            .is_none()
    );
    let dense = simulate(scenario, 7).unwrap();
    assert_eq!(dense.frames.len(), dense.summary.steps);
    assert!(sparse.frames.len() < dense.frames.len());
    let mut a = vec![];
    let mut b = vec![];
    sparse.sensor_log.as_ref().unwrap().write(&mut a).unwrap();
    dense.sensor_log.as_ref().unwrap().write(&mut b).unwrap();
    assert_eq!(a, b);
}
