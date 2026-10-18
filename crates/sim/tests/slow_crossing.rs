use rustdriving_pipeline::replay::verify;
use rustdriving_sim::{Scenario, simulate};

#[test]
fn observed_slow_crossing_avoids_collision_and_replays_without_future_actor_state() {
    for seed in [1, 7, 42] {
        let scenario: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/slow-crossing.json")).unwrap();
        let run = simulate(scenario, seed).unwrap();
        let log = run.sensor_log.as_ref().unwrap();
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        assert!(
            verify(std::io::Cursor::new(bytes), vec![])
                .unwrap()
                .verified
        );
        assert!(run.summary.passed, "seed {seed}: {:?}", run.summary);
        assert_eq!(run.summary.collisions, 0);
        assert_eq!(run.summary.road_violations, 0);
        assert!(run.summary.min_clearance >= 1.);
        assert!(run.summary.reached_goal);
        // Require that the repaired behavior actually consumes a slow-motion
        // forecast, rather than merely obtaining a fortunate physical result.
        assert!(run.frames.iter().any(|frame| {
            frame.tracks.iter().any(|track| {
                let speed = track.velocity.x.hypot(track.velocity.y);
                (0.1..0.7).contains(&speed)
                    && frame.predictions.iter().any(|p| {
                        p.id == track.id
                            && p.positions.len() == 41
                            && p.positions[40].distance(p.positions[0]) > 1.
                    })
            })
        }));
    }
}
