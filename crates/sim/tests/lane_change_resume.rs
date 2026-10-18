use rustdriving_core::{Vec2, VehicleConfig};
use rustdriving_pipeline::lane_change::{
    LaneChangeConfig, LaneChangePhase, LaneChangeResumeConfig, LaneChangeReturnConfig,
    ResumeDecision,
};
use rustdriving_pipeline::replay::verify;
use rustdriving_routing::lanes::{LaneChangeRequest, LaneMapSpec, LaneNetwork, LaneSide};
use rustdriving_sim::{Scenario, simulate};

fn scenario(case: &str) -> Scenario {
    let network = LaneNetwork::new(
        serde_json::from_str::<LaneMapSpec>(include_str!(
            "../../../maps/lanes/two-lane-change.json"
        ))
        .unwrap(),
    )
    .unwrap();
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
            resume_after_return: Some(LaneChangeResumeConfig {
                max_change_distance_m: 30.,
            }),
        }),
    };
    let mut actors = vec![
        serde_json::json!({"s":0.,"lateral":0.,"radius":1.1,"active_from":20.,"moving_from":40.,
        "world_motion":{"position":{"x":52.,"y":3.5},"velocity":{"x":if case=="blocked" {0.} else {4.},"y":0.}}}),
    ];
    if case == "reblocked" {
        actors.push(serde_json::json!({"s":0.,"lateral":0.,"radius":1.1,"active_from":54.,"world_motion":{"position":{"x":66.,"y":3.5},"velocity":{"x":0.,"y":0.}}}));
    }
    serde_json::from_value(serde_json::json!({"name":format!("Mission retry {case}"),"duration":120.,"expected":if case=="release" {"goal"}else{"stop"},"road_length":p.distance_m,"half_width":1.75,"local_route_geometry":true,
        "lane_change":gate,"min_clearance_m":1.,"cruise_speed":2.,"motion_limits":{"max_acceleration_m_s2":1.,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.},
        "navigation":{"network":p.network,"start":p.start_node,"goal":p.goal_node,"closed_edges":[]},"objects":actors})).unwrap()
}

#[test]
fn returned_missions_wait_for_observed_release_reach_the_original_goal_or_stop_on_new_conflict() {
    for case in ["release", "blocked", "reblocked"] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(
                result.summary.passed,
                "case {case} seed {seed}: {:?}",
                result.summary.failures
            );
            assert_eq!(result.summary.collisions, 0);
            assert_eq!(result.summary.road_violations, 0);
            assert!(result.summary.min_clearance >= 1.);
            assert_eq!(result.summary.reached_goal, case == "release");
            let mut held_at = None;
            let mut resumed_at = None;
            let mut request = None;
            let mut stopped_point = None;
            let mut count = 0;
            let mut previous_phase = None;
            for f in &result.frames {
                let status = f.lane_change.as_ref().unwrap();
                if status.phase == LaneChangePhase::Aborted {
                    let p = f.truth.pose.position;
                    assert!(p.y.abs() + result.vehicle.radius <= 1.75 && f.truth.speed <= 0.05);
                    held_at.get_or_insert(f.time);
                    let stopped = *stopped_point.get_or_insert(p);
                    assert!(p.distance(stopped) < 0.05, "crept during source hold");
                }
                let retry = status
                    .return_state
                    .as_ref()
                    .unwrap()
                    .resume_state
                    .as_ref()
                    .unwrap();
                if let Some(r) = retry.request {
                    if request.is_none() {
                        request = Some(r);
                    }
                    assert_eq!(
                        serde_json::to_value(r).unwrap(),
                        serde_json::to_value(request.unwrap()).unwrap()
                    );
                    assert_eq!(retry.started_at, resumed_at.or(Some(f.time)));
                    resumed_at = retry.started_at;
                }
                if status.phase == LaneChangePhase::Resuming {
                    assert!(retry.observed_clearance_m.unwrap() >= 1.3);
                    if previous_phase != Some(LaneChangePhase::Resuming) {
                        count += 1;
                    }
                }
                previous_phase = Some(status.phase);
            }
            let hold = held_at.expect("return stop missing");
            let last = result.frames.last().unwrap();
            if case == "blocked" {
                assert!(request.is_none());
                assert_eq!(
                    last.lane_change.as_ref().unwrap().phase,
                    LaneChangePhase::Aborted
                );
                let retry = last
                    .lane_change
                    .as_ref()
                    .unwrap()
                    .return_state
                    .as_ref()
                    .unwrap()
                    .resume_state
                    .as_ref()
                    .unwrap();
                assert_eq!(retry.decision, ResumeDecision::TrafficBlocked);
                assert!(last.truth.pose.position.distance(stopped_point.unwrap()) < 0.05);
            } else {
                assert_eq!(count, 1);
                let start = resumed_at.unwrap();
                assert!(start >= 41. && start - hold >= 1.);
                let r = request.unwrap();
                assert!(r.start.position.distance(stopped_point.unwrap()) < 0.5);
                assert!(r.start.position.x > 25. && r.start.position.y.abs() < 0.25);
                let final_state = last.lane_change.as_ref().unwrap();
                if case == "release" {
                    assert_eq!(final_state.phase, LaneChangePhase::Completed);
                    assert!(
                        last.truth.pose.position.x >= 157.
                            && (last.truth.pose.position.y - 3.5).abs() < 0.25
                    );
                } else {
                    assert!(last.truth.speed <= 0.05 && last.truth.pose.position.x < 64.);
                    assert_eq!(
                        final_state
                            .return_state
                            .as_ref()
                            .unwrap()
                            .resume_state
                            .as_ref()
                            .unwrap()
                            .started_at,
                        Some(start)
                    );
                }
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
fn invalid_resume_distances_are_rejected_before_running() {
    for distance in [f64::NAN, f64::INFINITY, 0., 9.99, 60.01] {
        let mut s = scenario("release");
        s.lane_change
            .as_mut()
            .unwrap()
            .return_on_conflict
            .as_mut()
            .unwrap()
            .resume_after_return
            .as_mut()
            .unwrap()
            .max_change_distance_m = distance;
        assert!(s.validate().is_err());
    }
}
