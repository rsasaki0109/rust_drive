use rustdriving_pipeline::replay::verify;
use rustdriving_sim::{Scenario, simulate};

#[test]
fn temporary_navigation_stops_preserve_the_detour_obstacle_clearance() {
    for source in [
        include_str!("../../../scenarios/route-handover.json"),
        include_str!("../../../scenarios/route-reopen.json"),
    ] {
        let run = simulate(serde_json::from_str(source).unwrap(), 7).unwrap();
        assert!(run.summary.passed, "{:?}", run.summary);
        assert_eq!(run.summary.collisions, 0);
        assert_eq!(run.summary.road_violations, 0);
        // An intermediate terminal tolerance changed the temporary stop and
        // left only 0.633/0.654 m here. Preserve the preceding >1 m clearance.
        assert!(run.summary.min_clearance > 1.0, "{:?}", run.summary);
        let mut bytes = vec![];
        run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
        assert!(
            verify(std::io::Cursor::new(bytes), vec![])
                .unwrap()
                .verified
        );
    }
}

#[test]
fn the_unchanged_follower_deadline_includes_a_complete_stationary_residence() {
    for confirm_slow_motion in [false, true] {
        for seed in [1, 7, 42] {
            let mut scenario: Scenario = serde_json::from_str(include_str!(
                "../../../scenarios/traffic-follower-deadline.json"
            ))
            .unwrap();
            scenario.confirm_slow_motion = confirm_slow_motion;
            let run = simulate(scenario, seed).unwrap();
            assert!(
                run.summary.passed,
                "seed {seed}, slow {confirm_slow_motion}: {:?}",
                run.summary
            );
            assert!(run.summary.reached_goal);
            assert!(run.summary.simulated_seconds <= 65.0);
            assert_eq!(run.summary.collisions, 0);
            assert_eq!(run.summary.road_violations, 0);
            assert!(run.summary.min_clearance >= 1.0);

            // Independently inspect the physical residence, rather than trusting
            // the simulator's reached_goal flag or its resettable residence timer.
            let end = run.frames.last().unwrap().time;
            let residence: Vec<_> = run
                .frames
                .iter()
                .filter(|f| f.time >= end - 8.0 - 1e-9)
                .collect();
            assert!(residence.len() >= 161);
            assert!(end - residence[0].time >= 8.0 - 1e-9);
            assert!(residence.iter().all(|f| {
                f.truth.pose.position.x >= 158.0
                    && f.truth.pose.position.x <= 160.0
                    && f.truth.speed < 0.2
                    && f.command.acceleration <= 1e-9
            }));
            let displacement = residence[0]
                .truth
                .pose
                .position
                .distance(residence.last().unwrap().truth.pose.position);
            assert!(displacement < 0.02, "residence moved {displacement} m");

            let mut bytes = vec![];
            run.sensor_log.as_ref().unwrap().write(&mut bytes).unwrap();
            let replay = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
            assert!(replay.verified);
            assert_eq!(replay.ticks, run.summary.steps);
        }
    }
}
