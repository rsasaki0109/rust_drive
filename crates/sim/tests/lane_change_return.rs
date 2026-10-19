use rustdriving_core::{Vec2, VehicleConfig};
use rustdriving_pipeline::lane_change::{
    LaneChangeConfig, LaneChangePhase, LaneChangeReturnConfig, ReturnDecision,
};
use rustdriving_pipeline::replay::verify;
use rustdriving_routing::lanes::{LaneChangeRequest, LaneMapSpec, LaneNetwork, LaneSide};
use rustdriving_sim::{Scenario, simulate};

fn scenario(case: &str) -> Scenario {
    let map: LaneMapSpec =
        serde_json::from_str(include_str!("../../../maps/lanes/two-lane-change.json")).unwrap();
    let network = LaneNetwork::new(map).unwrap();
    let p = network
        .lane_change_route(
            "source",
            "target-exit",
            &[],
            LaneChangeRequest {
                side: LaneSide::Left,
                start_m: 25.,
                length_m: 30.,
            },
            VehicleConfig::default(),
        )
        .unwrap()
        .plan;
    let gate = LaneChangeConfig {
        entry_s_m: 25.,
        exit_s_m: p.route.project(Vec2::new(55., 3.5)).0,
        target_start: Vec2::new(0., 3.5),
        target_end: Vec2::new(80., 3.5),
        target_width_m: 3.5,
        return_on_conflict: Some(LaneChangeReturnConfig {
            source_start: Vec2::default(),
            source_end: Vec2::new(80., 0.),
            source_width_m: 3.5,
            max_return_distance_m: 30.,
            speed_limit_m_s: None,
            resume_after_return: None,
        }),
    };
    let activation = if case == "late" { 24. } else { 20. };
    let actor = |x, y| serde_json::json!({"s":0.,"lateral":0.,"radius":1.1,"active_from":activation,"world_motion":{"position":{"x":x,"y":y},"velocity":{"x":0.,"y":0.}}});
    let mut actors = vec![actor(52., 3.5)];
    if case == "source-blocked" {
        actors.push(actor(40., 0.));
    }
    serde_json::from_value(serde_json::json!({"name":format!("Early return {case}"),"duration":70.,"road_length":p.distance_m,"half_width":1.75,"local_route_geometry":true,
        "expected":"stop","cruise_speed":2.,"min_clearance_m":1.,
        "motion_limits":{"max_acceleration_m_s2":1.,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.},
        "navigation":{"network":p.network,"start":p.start_node,"goal":p.goal_node,"closed_edges":[]},"lane_change":gate,"objects":actors})).unwrap()
}

#[test]
fn early_return_source_blockage_and_late_conflict_drive_and_replay_across_seeds() {
    for case in ["early", "source-blocked", "late"] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(
                result.summary.passed,
                "case {case}, seed {seed}: {:?}",
                result.summary.failures
            );
            assert_eq!(result.summary.collisions, 0);
            assert_eq!(result.summary.road_violations, 0);
            assert!(result.summary.min_clearance >= 1.0);
            assert!(!result.summary.reached_goal);
            let mut returned = false;
            let mut clear_rejection = false;
            let mut peak = 0.0_f64;
            let mut hold_start = None;
            let mut hold_s = 0.0_f64;
            for f in &result.frames {
                let state = f.lane_change.as_ref().unwrap();
                let status = state.return_state.as_ref().unwrap();
                peak = peak.max(f.truth.pose.position.y);
                returned |= state.phase == LaneChangePhase::Aborting;
                clear_rejection |= status.decision
                    == if case == "source-blocked" {
                        ReturnDecision::SourceBlocked
                    } else {
                        ReturnDecision::OutsideSource
                    };
                if state.phase == LaneChangePhase::Aborted {
                    assert_eq!(case, "early");
                    assert!(f.truth.pose.position.y.abs() + result.vehicle.radius <= 1.75);
                    if f.truth.speed <= 0.05 {
                        let start = *hold_start.get_or_insert(f.time);
                        hold_s = hold_s.max(f.time - start);
                    } else {
                        hold_start = None;
                    }
                }
            }
            assert!(
                peak > 0.1,
                "case {case}, seed {seed} never physically began the change"
            );
            assert_eq!(returned, case == "early", "case {case} seed {seed}");
            let last = result.frames.last().unwrap();
            if case == "early" {
                assert_eq!(
                    last.lane_change.as_ref().unwrap().phase,
                    LaneChangePhase::Aborted
                );
                assert!(last.truth.pose.position.y.abs() <= 0.25);
                assert!(hold_s >= 2.0);
                let status = last
                    .lane_change
                    .as_ref()
                    .unwrap()
                    .return_state
                    .as_ref()
                    .unwrap();
                assert!(status.request.unwrap().distance_m < 30.0);
                assert!(status.observed_clearance_m.unwrap() >= 1.3);
            } else {
                assert!(
                    clear_rejection,
                    "case {case} seed {seed}: no reason for rejecting return"
                );
            }
            let mut bytes = vec![];
            result
                .sensor_log
                .as_ref()
                .unwrap()
                .write(&mut bytes)
                .unwrap();
            let replay = verify(std::io::Cursor::new(bytes), std::io::sink()).unwrap();
            assert!(replay.verified);
            assert_eq!(replay.ticks, result.summary.steps);
        }
    }
}

#[test]
fn return_configuration_requires_exact_source_geometry_width_and_stopping_room() {
    for case in 0..7 {
        let mut c = scenario("early");
        let r = c
            .lane_change
            .as_mut()
            .unwrap()
            .return_on_conflict
            .as_mut()
            .unwrap();
        match case {
            0 => r.source_end.y = 1.,
            1 => r.source_start.x = 1.,
            2 => r.source_width_m = 4.,
            3 => r.max_return_distance_m = f64::NAN,
            4 => r.max_return_distance_m = 60.,
            5 => r.source_width_m = 1.,
            _ => r.source_end.x = f64::INFINITY,
        }
        assert!(c.validate().is_err(), "case {case}");
    }
}
