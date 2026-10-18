use rustdriving_core::{Vec2, VehicleConfig};
use rustdriving_pipeline::lane_change::{
    LaneChangeConfig, LaneChangePhase, LaneChangeResumeConfig, LaneChangeReturnConfig,
    LaunchHoldPhase, ResumeDecision,
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
    if matches!(
        case,
        "rear"
            | "rear-slow"
            | "launch-blocked"
            | "launch-recovery"
            | "second-rear"
            | "second-blocked"
            | "second-fault"
    ) {
        let (x, v) = match case {
            "rear-slow" => (35., 4.),
            "launch-blocked" => (54., 0.),
            _ => (30., 6.),
        };
        actors.push(serde_json::json!({"s":0.,"lateral":0.,"radius":1.1,"active_from":49.,"world_motion":{"position":{"x":x,"y":3.5},"velocity":{"x":v,"y":0.}}}));
    }
    if matches!(case, "second-rear" | "second-blocked") {
        let (x, v) = if case == "second-rear" {
            (36., 15.)
        } else {
            (54., 0.)
        };
        actors.push(serde_json::json!({"s":0.,"lateral":0.,"radius":1.1,"active_from":56.,"world_motion":{"position":{"x":x,"y":3.5},"velocity":{"x":v,"y":0.}}}));
    }
    let mut scenario = serde_json::json!({"name":format!("Mission retry {case}"),"duration":120.,"expected":if matches!(case,"release"|"rear"|"rear-slow"|"launch-recovery"|"second-rear"|"second-fault") {"goal"}else{"stop"},"road_length":p.distance_m,"half_width":1.75,"local_route_geometry":true,
        "lane_change":gate,"min_clearance_m":1.,"cruise_speed":2.,"motion_limits":{"max_acceleration_m_s2":1.,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.},
        "navigation":{"network":p.network,"start":p.start_node,"goal":p.goal_node,"closed_edges":[]},"objects":actors});
    if case == "launch-recovery" {
        scenario["sensor_timing"] = serde_json::json!({"lidar_period_ticks":2,"lidar_delay_ticks":0,"lidar_failure_windows":[{"from":50.,"until":51.}]});
    }
    if case == "second-fault" {
        scenario["sensor_timing"] = serde_json::json!({"lidar_period_ticks":2,"lidar_delay_ticks":0,"lidar_failure_windows":[{"from":56.,"until":57.}]});
    }
    serde_json::from_value(scenario).unwrap()
}

#[test]
fn repeated_events_hold_with_room_and_report_committed_crossings_across_seeds() {
    let mut held_cases = 0;
    let mut committed_cases = 0;
    for case in ["second-rear", "second-blocked", "second-fault"] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(
                result.summary.passed,
                "{case} seed {seed}: {:?}",
                result.summary.failures
            );
            assert_eq!(result.summary.collisions, 0);
            assert_eq!(result.summary.road_violations, 0);
            assert!(result.summary.min_clearance >= 1.);
            assert_eq!(result.summary.reached_goal, case != "second-blocked");
            let mut first_release = None;
            let mut selected = None;
            let mut previous_record = None;
            let mut second_seen = false;
            let mut stop_position = None;
            let mut stopped_since = None;
            let mut hold_duration = 0.0_f64;
            let mut last_speed = f64::INFINITY;
            let mut latest = None;
            for f in &result.frames {
                let retry = f
                    .lane_change
                    .as_ref()
                    .unwrap()
                    .return_state
                    .as_ref()
                    .unwrap()
                    .resume_state
                    .as_ref()
                    .unwrap();
                if let Some(request) = retry.request {
                    let selected = selected.get_or_insert(serde_json::to_value(request).unwrap());
                    assert_eq!(*selected, serde_json::to_value(request).unwrap());
                }
                let Some(hold) = &retry.launch_hold else {
                    continue;
                };
                if hold.cycle == 1 && hold.phase == LaunchHoldPhase::Released {
                    let first = *first_release.get_or_insert(f.time);
                    assert_eq!(hold.released_at, Some(first));
                }
                assert!((1..=2).contains(&hold.cycle));
                if hold.cycle != 2 {
                    continue;
                }
                second_seen = true;
                assert!((56.0..56.5).contains(&hold.initiated_at));
                assert_eq!(hold.previous_holds.len(), 1);
                let record = &hold.previous_holds[0];
                assert_eq!(record.cycle, 1);
                assert_eq!(Some(record.released_at), first_release);
                assert!(record.released_at < hold.initiated_at);
                let previous = previous_record.get_or_insert(serde_json::to_value(record).unwrap());
                assert_eq!(*previous, serde_json::to_value(record).unwrap());
                if hold.phase != LaunchHoldPhase::Released {
                    assert!(f.command.acceleration <= -hold.deceleration_m_s2 + 1e-9);
                    assert!(f.truth.pose.position.y.abs() + result.vehicle.radius <= 1.75);
                    assert!(f.truth.speed <= last_speed + 1e-9);
                    last_speed = f.truth.speed;
                    if f.truth.speed <= 0.05 {
                        let p = *stop_position.get_or_insert(f.truth.pose.position);
                        assert!(p.distance(f.truth.pose.position) < 0.05);
                        let start = *stopped_since.get_or_insert(f.time);
                        hold_duration = hold_duration.max(f.time - start);
                    }
                } else {
                    assert!(hold.stopped_at.unwrap() >= 56.);
                    assert!(hold.clear_since.unwrap() > hold.after_fault_at);
                    assert!(hold.released_at.unwrap() - hold.clear_since.unwrap() >= 1. - 1e-9);
                    assert!(hold.crossing_preview_clearance_m.unwrap() >= 1.3);
                }
                latest = Some(hold.clone());
            }
            let last = result.frames.last().unwrap();
            if !second_seen {
                committed_cases += 1;
                let hold = last
                    .lane_change
                    .as_ref()
                    .unwrap()
                    .return_state
                    .as_ref()
                    .unwrap()
                    .resume_state
                    .as_ref()
                    .unwrap()
                    .launch_hold
                    .as_ref()
                    .unwrap();
                let event = hold
                    .limit_event
                    .as_ref()
                    .expect("missing independent stopping-room witness");
                assert_eq!(hold.cycle, 1);
                assert!(hold.previous_holds.is_empty());
                assert!(
                    event.committed_at <= event.observed_at
                        && (56.0..56.5).contains(&event.observed_at)
                );
                assert!(
                    event.required_endpoint_s_m > hold.source_limit_s_m
                        || !event.source_fits_at_commit
                );
                if case == "second-fault" {
                    assert!(event.invalid_evidence);
                }
            } else {
                held_cases += 1;
                assert!(
                    hold_duration >= 1.,
                    "{case} seed {seed}: missing second physical hold"
                );
                let hold = latest.unwrap();
                if case == "second-blocked" {
                    assert!(hold.released_at.is_none() && !hold.target_blocking_tracks.is_empty());
                    assert!(
                        last.truth.speed <= 0.05
                            && last.truth.pose.position.y.abs() + result.vehicle.radius <= 1.75
                    );
                } else {
                    assert_eq!(hold.phase, LaunchHoldPhase::Released);
                    assert_eq!(
                        last.lane_change.as_ref().unwrap().phase,
                        LaneChangePhase::Completed
                    );
                    assert!(
                        last.truth.pose.position.x >= 157.
                            && (last.truth.pose.position.y - 3.5).abs() < 0.25
                    );
                    if case == "second-fault" {
                        assert!(hold.after_fault_at >= 57. && hold.clear_since.unwrap() > 57.);
                    }
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
    assert_eq!((held_cases, committed_cases), (6, 3));
}

#[test]
fn early_resume_conflicts_hold_source_then_reconfirm_the_same_crossing_across_seeds() {
    for case in ["rear", "rear-slow", "launch-blocked", "launch-recovery"] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(
                result.summary.passed,
                "{case} seed {seed}: {:?}",
                result.summary.failures
            );
            assert_eq!(result.summary.collisions, 0);
            assert_eq!(result.summary.road_violations, 0);
            assert!(result.summary.min_clearance >= 1.);
            assert_eq!(result.summary.reached_goal, case != "launch-blocked");
            let mut stopped_position = None;
            let mut held_since = None;
            let mut latest_hold = None;
            let mut selected = None;
            let mut release_at = None;
            let mut initiation_at = None;
            let mut initial_speed = None;
            let mut stopped_duration = 0.0_f64;
            let mut recovery_seen = false;
            for f in &result.frames {
                let retry = f
                    .lane_change
                    .as_ref()
                    .unwrap()
                    .return_state
                    .as_ref()
                    .unwrap()
                    .resume_state
                    .as_ref()
                    .unwrap();
                if let Some(request) = retry.request {
                    let selected = selected.get_or_insert(serde_json::to_value(request).unwrap());
                    assert_eq!(
                        *selected,
                        serde_json::to_value(request).unwrap(),
                        "connector was replaced"
                    );
                }
                if let Some(hold) = &retry.launch_hold {
                    let initiated = *initiation_at.get_or_insert(hold.initiated_at);
                    assert_eq!(initiated, hold.initiated_at);
                    if hold.phase != LaunchHoldPhase::Released {
                        assert!(f.command.acceleration <= -hold.deceleration_m_s2 + 1e-9);
                        assert!(f.truth.pose.position.y.abs() + result.vehicle.radius <= 1.75);
                        let speed = *initial_speed.get_or_insert(f.truth.speed);
                        assert!(f.truth.speed <= speed + 1e-9, "accelerated during hold");
                        initial_speed = Some(f.truth.speed);
                        if f.truth.speed <= 0.05 {
                            let p = *stopped_position.get_or_insert(f.truth.pose.position);
                            assert!(p.distance(f.truth.pose.position) < 0.05, "hold creep");
                            let start = *held_since.get_or_insert(f.time);
                            stopped_duration = stopped_duration.max(f.time - start);
                        }
                    } else {
                        let released = *release_at.get_or_insert(f.time);
                        assert_eq!(hold.released_at, Some(released));
                        assert!(hold.clear_since.unwrap() > hold.after_fault_at);
                        assert!(released - hold.clear_since.unwrap() >= 1. - 1e-9);
                        assert!(hold.crossing_preview_clearance_m.unwrap() >= 1.3);
                    }
                    recovery_seen |= case == "launch-recovery" && hold.after_fault_at >= 51.;
                    latest_hold = Some(hold.clone());
                }
            }
            assert!(
                initiation_at.is_some_and(|t| (49.0..50.0).contains(&t)),
                "{case} seed {seed} did not pause"
            );
            assert!(
                stopped_duration >= 1.,
                "{case} seed {seed}: no physical source hold"
            );
            let hold = latest_hold.unwrap();
            assert!(hold.planning_endpoint_s_m <= hold.source_limit_s_m);
            if case == "launch-blocked" {
                assert!(release_at.is_none());
                assert!(!hold.target_blocking_tracks.is_empty());
                assert!(result.frames.last().unwrap().truth.speed <= 0.05);
            } else {
                assert!(release_at.is_some());
                assert_eq!(hold.phase, LaunchHoldPhase::Released);
                let last = result.frames.last().unwrap();
                assert_eq!(
                    last.lane_change.as_ref().unwrap().phase,
                    LaneChangePhase::Completed
                );
                assert!(
                    last.truth.pose.position.x >= 157.
                        && (last.truth.pose.position.y - 3.5).abs() < 0.25
                );
                if case == "launch-recovery" {
                    assert!(recovery_seen);
                    assert!(hold.clear_since.unwrap() > 51.);
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
