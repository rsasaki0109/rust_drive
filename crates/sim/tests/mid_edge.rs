use rustdriving_pipeline::replay::verify;
use rustdriving_sim::{Scenario, simulate};
use std::io::Cursor;

#[test]
fn partial_destinations_restricted_turns_and_stopped_handover_drive_and_replay() {
    for name in [
        "same-edge",
        "obstacle",
        "clipped",
        "restricted",
        "handover",
        "reopen",
        "blocked-goal",
        "osm",
    ] {
        let scenario: Scenario = serde_json::from_str(
            &std::fs::read_to_string(format!(
                "{}/../../scenarios/mid-edge-{name}.json",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap(),
        )
        .unwrap();
        let run = simulate(scenario, 7).unwrap();
        assert!(run.summary.passed, "{name}: {:?}", run.summary);
        assert_eq!(run.summary.reached_goal, name != "blocked-goal");
        assert_eq!(run.frames.len(), run.summary.steps);
        assert!(run.summary.min_clearance >= 1.);
        assert_eq!(
            run.summary.navigation_switches,
            usize::from(matches!(name, "handover" | "reopen"))
        );
        let log = run.sensor_log.as_ref().unwrap();
        assert!(log.header.config.navigation.is_some());
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        assert!(verify(Cursor::new(&bytes), vec![]).unwrap().verified);
        if name == "clipped" {
            // Modifying only the recorded origin must be detected by initial
            // map/route consistency before any saved expected output is trusted.
            let raw = String::from_utf8(bytes).unwrap();
            let (header, tail) = raw.split_once('\n').unwrap();
            let mut header: serde_json::Value = serde_json::from_str(header).unwrap();
            header["header"]["config"]["navigation"]["start"]["s_m"] = 13.0.into();
            let changed = format!("{}\n{tail}", serde_json::to_string(&header).unwrap());
            assert!(verify(Cursor::new(changed), vec![]).is_err());
        }
    }
}
