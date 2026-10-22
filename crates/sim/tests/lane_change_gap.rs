use rustdriving_core::{Vec2, VehicleConfig};
use rustdriving_pipeline::lane_change::{LaneChangeConfig, LaneChangePhase};
use rustdriving_pipeline::replay::verify;
use rustdriving_routing::lanes::{LaneChangeRequest, LaneMapSpec, LaneNetwork, LaneSide};
use rustdriving_sim::{Scenario, simulate};

fn scenario(case: &str) -> Scenario {
    let map: LaneMapSpec =
        serde_json::from_str(include_str!("../../../maps/lanes/two-lane-change.json")).unwrap();
    let network = LaneNetwork::new(map).unwrap();
    let change = network
        .lane_change_route(
            "source",
            "target-exit",
            &[],
            LaneChangeRequest {
                side: LaneSide::Left,
                start_m: 25.0,
                length_m: 30.0,
            },
            VehicleConfig::default(),
        )
        .unwrap();
    let p = change.plan;
    let gate = LaneChangeConfig {
        entry_s_m: 25.0,
        exit_s_m: p.route.project(Vec2::new(55.0, 3.5)).0,
        target_start: Vec2::new(0.0, 3.5),
        target_end: Vec2::new(80.0, 3.5),
        target_width_m: 3.5,
        return_on_conflict: None,
    };
    let (x, vx, moving_from) = match case {
        "rear" => (-40., 3., 0.),
        "release" => (28., 4., 15.),
        _ => (28., 0., 0.),
    };
    serde_json::from_value(serde_json::json!({
        "name":format!("Observed lane-change gap {case}"), "duration":if case=="blocked" {35.} else {120.},
        "road_length":p.distance_m, "half_width":1.75, "local_route_geometry":true,
        "expected":if case=="blocked" {"stop"} else {"goal"}, "cruise_speed":2., "min_clearance_m":1.,
        "motion_limits":{"max_acceleration_m_s2":1., "max_deceleration_m_s2":2.5, "max_lateral_acceleration_m_s2":1.},
        "navigation":{"network":p.network,"start":p.start_node,"goal":p.goal_node,"closed_edges":[]},
        "lane_change":gate,
        "objects":[{"s":0.,"lateral":0.,"radius":1.1,"moving_from":moving_from,
                    "world_motion":{"position":{"x":x,"y":3.5},"velocity":{"x":vx,"y":0.}}}]
    })).unwrap()
}

#[test]
fn measured_rear_front_and_opening_gaps_wait_on_source_then_drive_or_remain_stopped() {
    for case in ["rear", "release", "blocked"] {
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
            let mut first_hold = None;
            let mut longest_hold = 0.0_f64;
            let mut observed_block = false;
            let mut completed = false;
            for frame in &result.frames {
                let state = frame.lane_change.as_ref().unwrap();
                let p = frame.truth.pose.position;
                if state.phase == LaneChangePhase::Waiting {
                    assert!(
                        p.x + result.vehicle.radius <= 25.0 + 1e-8,
                        "case {case}, seed {seed}, time {} crossed waiting line",
                        frame.time
                    );
                    assert!(p.y.abs() + result.vehicle.radius <= 1.75 + 1e-8);
                    if p.x > 15.0 && frame.truth.speed <= 0.05 {
                        let start = *first_hold.get_or_insert(frame.time);
                        longest_hold = longest_hold.max(frame.time - start);
                    } else {
                        first_hold = None;
                    }
                    let object = &frame.objects[0];
                    observed_block |= !state.blocking_tracks.is_empty()
                        && if case == "rear" {
                            object.position.x < p.x && state.rear_gap_m.is_some()
                        } else {
                            object.position.x >= p.x && state.front_gap_m.is_some()
                        };
                } else {
                    first_hold = None;
                }
                if state.phase == LaneChangePhase::Changing {
                    assert_ne!(case, "blocked");
                    assert!(
                        frame.objects[0].position.x
                            - p.x
                            - result.vehicle.radius
                            - frame.objects[0].radius
                            >= 6.0,
                        "case {case} seed {seed}: insufficient actual front gap at change"
                    );
                }
                completed |= state.phase == LaneChangePhase::Completed;
            }
            assert!(
                observed_block,
                "case {case} seed {seed}: did not block on measured relevant traffic"
            );
            assert!(
                longest_hold >= 1.0,
                "case {case} seed {seed}: only held {longest_hold}s"
            );
            assert_eq!(completed, case != "blocked");
            assert_eq!(result.summary.reached_goal, case != "blocked");
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
fn independent_world_motion_has_no_route_clamp_and_rejects_ambiguous_or_invalid_inputs() {
    let c = scenario("rear");
    let route = c.route();
    let world = rustdriving_sim::traffic::TrafficWorld::new(c.clone(), route);
    let at_start = world.objects(0.0)[0].position;
    let at_end = world.objects(100.0)[0].position;
    assert_eq!(at_start, Vec2::new(-40.0, 3.5));
    assert_eq!(at_end, Vec2::new(260.0, 3.5));
    for case in 0..6 {
        let mut c = c.clone();
        let actor = &mut c.objects[0];
        match case {
            0 => actor.world_motion.as_mut().unwrap().velocity.x = f64::NAN,
            1 => actor.world_motion.as_mut().unwrap().velocity.x = 16.0,
            2 => actor.world_motion.as_mut().unwrap().position.y = f64::INFINITY,
            3 => actor.speed = 1.0,
            4 => actor.lateral_speed = 1.0,
            _ => actor.s = 1.0,
        }
        assert!(c.validate().is_err(), "case {case}");
    }
    let mut delayed = c;
    delayed.objects[0].moving_from = 10.0;
    delayed.objects[0].moving_until = Some(20.0);
    let world = rustdriving_sim::traffic::TrafficWorld::new(delayed.clone(), delayed.route());
    assert_eq!(world.objects(5.0)[0].position, Vec2::new(-40.0, 3.5));
    assert_eq!(world.objects(15.0)[0].position, Vec2::new(-25.0, 3.5));
    assert_eq!(world.objects(30.0)[0].position, Vec2::new(-10.0, 3.5));
}
