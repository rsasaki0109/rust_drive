use rustdrive_sim::{Scenario, simulate};
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
fn closed_loop_mission_across_seeds() {
    for seed in [1, 7, 42] {
        let run = simulate(scenario("mission"), seed).unwrap();
        assert!(run.summary.passed, "seed {seed}: {:?}", run.summary);
        assert!(run.summary.avoidance_steps > 10);
        assert!(run.summary.max_tracks >= 2);
        assert!(run.summary.localization_rmse < 0.3);
    }
}
#[test]
fn blocked_road_stops_without_collision() {
    let run = simulate(scenario("blocked"), 7).unwrap();
    assert!(run.summary.passed, "{:?}", run.summary);
    assert!(run.summary.min_clearance > 0.5);
}
#[test]
fn stale_lidar_and_gnss_trigger_braking() {
    for name in ["lidar-fault", "gnss-fault"] {
        let run = simulate(scenario(name), 7).unwrap();
        assert!(run.summary.passed, "{name}: {:?}", run.summary);
        assert!(run.summary.emergency_steps > 20);
    }
}
#[test]
fn identical_seed_replays_exactly() {
    let a = simulate(scenario("blocked"), 42).unwrap();
    let b = simulate(scenario("blocked"), 42).unwrap();
    assert_eq!(
        serde_json::to_vec(&a).unwrap(),
        serde_json::to_vec(&b).unwrap()
    );
}
#[test]
fn malformed_scenario_rejected() {
    let mut s = scenario("mission");
    s.duration = -1.0;
    assert!(simulate(s, 1).is_err());
}
#[test]
fn invalid_or_overlapping_gnss_bias_windows_are_rejected() {
    let s = scenario("gnss-burst");
    for (from, until, x) in [
        (-1.0, 8.0, 30.0),
        (5.0, 5.0, 30.0),
        (65.0, 66.0, 30.0),
        (5.0, 301.0, 30.0),
        (5.0, 8.0, f64::NAN),
    ] {
        let mut invalid = s.clone();
        invalid.gnss_bias_windows[0].from = from;
        invalid.gnss_bias_windows[0].until = until;
        invalid.gnss_bias_windows[0].offset.x = x;
        assert!(invalid.validate().is_err());
    }
    let mut overlapping = s.clone();
    overlapping.gnss_bias_windows.push(s.gnss_bias_windows[0]);
    assert!(overlapping.validate().is_err());
    assert!(scenario("gnss-persistent-bias").validate().is_ok());
}
#[test]
fn gnss_outliers_stop_and_recover_without_resetting_localization() {
    for case in ["gnss-spike", "gnss-burst", "gnss-persistent-bias"] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(
                result.summary.passed,
                "{case} seed {seed}: {:?}",
                result.summary
            );
            assert!(result.summary.localization_max_error < 0.5);
            let log = result.sensor_log.as_ref().unwrap();
            for tick in &log.ticks {
                let diagnostic = tick.expected.localization.unwrap();
                if tick.input.gnss.is_some()
                    && (5.0..if case == "gnss-spike" {
                        5.2
                    } else if case == "gnss-burst" {
                        8.0
                    } else {
                        30.0
                    })
                        .contains(&tick.input.time)
                {
                    assert_eq!(
                        diagnostic.last_decision,
                        Some(rustdrive_core::GnssDecision::RejectedInnovation)
                    );
                    assert!(diagnostic.last_accepted_stamp.unwrap() < 5.0);
                }
            }
            if case == "gnss-burst" {
                assert!(
                    result
                        .frames
                        .iter()
                        .any(|f| (7.0..8.0).contains(&f.time) && f.truth.speed < 0.1)
                );
                assert!(log.ticks.iter().any(|t| {
                    t.input.time >= 8.0
                        && t.expected
                            .localization
                            .unwrap()
                            .last_accepted_stamp
                            .is_some_and(|s| s >= 8.0)
                }));
            }
            if case == "gnss-persistent-bias" {
                assert!(
                    log.ticks
                        .last()
                        .unwrap()
                        .expected
                        .health
                        .contains(&rustdrive_pipeline::HealthIssue::StaleGnss)
                );
            }
            let mut bytes = Vec::new();
            log.write(&mut bytes).unwrap();
            assert_eq!(
                rustdrive_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                    .unwrap()
                    .ticks,
                result.summary.steps
            );
        }
    }
}

#[test]
fn actual_collision_fails_acceptance() {
    let mut s = scenario("blocked");
    s.objects[0].s = 0.0;
    let run = simulate(s, 7).unwrap();
    assert!(!run.summary.passed);
    assert!(run.summary.collisions > 0);
    assert!(run.summary.min_clearance < 0.0);
}

#[test]
fn cli_reports_failure_and_writes_evidence() {
    use std::{fs, process::Command};
    let directory = std::env::temp_dir().join(format!("rustdrive-cli-{}", std::process::id()));
    fs::create_dir_all(&directory).unwrap();
    let mut s = scenario("mission");
    s.duration = 1.0;
    let input = directory.join("scenario.json");
    fs::write(&input, serde_json::to_vec(&s).unwrap()).unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_rustdrive"))
        .args(["run", "--scenario"])
        .arg(&input)
        .arg("--output")
        .arg(directory.join("run"))
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let evidence: serde_json::Value =
        serde_json::from_slice(&fs::read(directory.join("run/summary.json")).unwrap()).unwrap();
    assert_eq!(evidence["passed"], false);
    let result = Command::new(env!("CARGO_BIN_EXE_rustdrive"))
        .args(["run", "--unknown", "x"])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(2));
    fs::remove_dir_all(directory).unwrap();
}

#[test]
fn cli_replay_recomputes_and_rejects_corruption_without_stale_success() {
    use std::{fs, io::BufWriter, process::Command};
    let directory =
        std::env::temp_dir().join(format!("rustdrive-replay-cli-{}", std::process::id()));
    fs::create_dir_all(&directory).unwrap();
    let mut s = scenario("mission");
    s.duration = 1.0;
    let result = simulate(s, 7).unwrap();
    // Replay validates computation, even when the short physical mission fails.
    assert!(!result.summary.passed);
    let log = directory.join("sensors.jsonl");
    result
        .sensor_log
        .as_ref()
        .unwrap()
        .write(BufWriter::new(fs::File::create(&log).unwrap()))
        .unwrap();
    let invoke = || {
        Command::new(env!("CARGO_BIN_EXE_rustdrive"))
            .args(["replay", "--log"])
            .arg(&log)
            .arg("--output")
            .arg(directory.join("replay"))
            .output()
            .unwrap()
    };
    assert_eq!(invoke().status.code(), Some(0));
    let summary = directory.join("replay/replay.json");
    let report: serde_json::Value = serde_json::from_slice(&fs::read(&summary).unwrap()).unwrap();
    assert_eq!(report["verified"], true);
    let text = fs::read_to_string(&log).unwrap();
    let mut lines: Vec<serde_json::Value> = text
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    lines[1]["tick"]["expected"]["command"]["acceleration"] = serde_json::json!(123.0);
    fs::write(
        &log,
        lines
            .iter()
            .map(|v| serde_json::to_string(v).unwrap() + "\n")
            .collect::<String>(),
    )
    .unwrap();
    assert_eq!(invoke().status.code(), Some(2));
    assert!(
        !summary.exists(),
        "failed replay must remove the previous success report"
    );
    fs::write(&summary, "previous success").unwrap();
    fs::remove_file(&log).unwrap();
    assert_eq!(invoke().status.code(), Some(2));
    assert!(!summary.exists());
    fs::remove_dir_all(directory).unwrap();
}

#[test]
fn crossing_hazards_across_seeds() {
    for case in ["occluded-crossing", "cut-in", "opposing-crossings"] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(
                result.summary.passed,
                "{case} seed {seed}: {:?}",
                result.summary
            );
            assert!(result.summary.max_tracks > 0);
        }
    }
}
#[test]
fn stops_when_multiple_obstacles_block_all_lateral_alternatives() {
    for seed in [1, 7, 42] {
        let result = simulate(scenario("multiple-blocked"), seed).unwrap();
        assert!(result.summary.passed, "seed {seed}: {:?}", result.summary);
        assert!(result.summary.max_tracks >= 3);
        assert!(result.summary.progress < 40.0);
        assert!(result.summary.min_clearance > 0.5);
    }
}
#[test]
fn newly_appearing_and_terminal_overlaps_fail_acceptance() {
    let mut s = scenario("blocked");
    s.duration = 1.0;
    s.objects[0].s = 0.0;
    s.objects[0].active_from = 0.05;
    let result = simulate(s.clone(), 7).unwrap();
    assert!(!result.summary.passed);
    assert!(result.summary.collisions > 0);
    assert!(result.summary.collisions <= result.summary.steps);
    // An actor appearing only at the final evaluated tick must still count.
    s.objects[0].s = 1.0;
    s.objects[0].active_from = s.duration;
    let result = simulate(s, 7).unwrap();
    assert!(result.summary.collisions > 0);
    assert!(!result.summary.passed);
}
#[test]
fn unsupported_friction_is_not_silently_ignored() {
    assert!(
        simulate(scenario("low-friction"), 7)
            .unwrap_err()
            .contains("RNE")
    );
    let mut s = scenario("low-friction");
    s.dynamics.as_mut().unwrap().friction_coefficient = 0.0;
    assert!(s.validate().is_err());
    let mut s = scenario("cut-in");
    s.objects[0].moving_from = f64::INFINITY;
    assert!(s.validate().is_err());
}
#[test]
fn actors_exist_before_motion_begins() {
    let s = scenario("cut-in");
    let road = s.route();
    assert_eq!(
        s.scheduled_objects(&road, 0.0)[0].position,
        s.scheduled_objects(&road, 4.9)[0].position
    );
    assert!(
        s.scheduled_objects(&road, 5.1)[0]
            .position
            .distance(s.scheduled_objects(&road, 0.0)[0].position)
            > 0.2
    );
}

#[test]
fn map_destinations_and_closure_detours_drive_and_replay_across_seeds() {
    for (case, edges, goal) in [
        (
            "route-direct",
            vec!["approach", "main", "east-exit"],
            "east",
        ),
        (
            "route-detour",
            vec!["approach", "detour", "east-exit"],
            "east",
        ),
        ("route-south", vec!["approach", "south-branch"], "south"),
    ] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(
                result.summary.passed,
                "{case} seed {seed}: {:?}",
                result.summary
            );
            let plan = result.navigation.as_ref().unwrap();
            assert_eq!(plan.edge_ids, edges);
            assert_eq!(plan.node_ids.last().unwrap(), goal);
            assert!(result.summary.min_clearance >= 0.5);
            let mut bytes = vec![];
            result
                .sensor_log
                .as_ref()
                .unwrap()
                .write(&mut bytes)
                .unwrap();
            assert_eq!(
                rustdrive_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                    .unwrap()
                    .ticks,
                result.summary.steps
            );
        }
    }
}

#[test]
fn closed_or_invalid_map_rejected_before_driving() {
    let mut s = scenario("route-detour");
    s.navigation
        .as_mut()
        .unwrap()
        .closed_edges
        .push("detour".into());
    assert!(simulate(s, 7).unwrap_err().contains("unreachable"));
    let mut s = scenario("route-direct");
    s.navigation.as_mut().unwrap().goal = "unknown".into();
    assert!(simulate(s, 7).is_err());
    let mut s = scenario("route-direct");
    s.navigation.as_mut().unwrap().network.edges[0].points[0].x = 3.0;
    assert!(simulate(s, 7).is_err());
}

#[test]
fn clearance_acceptance_rejects_a_collision_free_but_too_close_run() {
    let s = scenario("route-direct");
    let baseline = simulate(s.clone(), 7).unwrap();
    assert!(baseline.summary.passed);
    let mut stricter = s;
    stricter.min_clearance_m = Some(baseline.summary.min_clearance + 0.1);
    let result = simulate(stricter, 7).unwrap();
    assert_eq!(result.summary.collisions, 0);
    assert!(!result.summary.passed);
    assert!(
        result
            .summary
            .failures
            .iter()
            .any(|f| f.contains("minimum swept clearance"))
    );
    let mut invalid = scenario("mission");
    invalid.min_clearance_m = Some(-1.0);
    assert!(simulate(invalid, 7).is_err());
}

#[test]
fn live_map_closures_stop_then_handover_without_resetting_estimation() {
    for case in ["route-handover", "route-handover-fast", "route-reopen"] {
        for seed in [1, 7, 42] {
            let result = simulate(scenario(case), seed).unwrap();
            assert!(result.summary.passed, "seed {seed}: {:?}", result.summary);
            assert_eq!(result.summary.navigation_switches, 1);
            assert_eq!(result.summary.closure_violations, 0);
            assert_eq!(result.route_history.len(), 2);
            let switch = &result.route_history[1];
            assert!(switch.estimated_speed.abs() <= 0.05);
            assert!(switch.true_speed <= 0.1);
            assert!(switch.time > 3.0);
            assert!(result.frames.iter().any(|f| f.time >= 3.0
                && f.navigation.as_ref().unwrap().phase
                    == rustdrive_pipeline::navigation::NavigationPhase::Braking));
            // Objects stay on the original world road, rather than teleporting when a route changes.
            assert!(
                result
                    .frames
                    .iter()
                    .all(|f| f.objects[0].position == result.frames[0].objects[0].position)
            );
            let mut bytes = Vec::new();
            result
                .sensor_log
                .as_ref()
                .unwrap()
                .write(&mut bytes)
                .unwrap();
            assert_eq!(
                rustdrive_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                    .unwrap()
                    .ticks,
                result.summary.steps
            );
        }
    }
}

#[test]
fn no_route_holds_before_the_closed_branch_across_seeds() {
    for seed in [1, 7, 42] {
        let result = simulate(scenario("route-no-path"), seed).unwrap();
        assert!(result.summary.passed, "seed {seed}: {:?}", result.summary);
        assert_eq!(result.summary.navigation_switches, 0);
        assert_eq!(result.summary.closure_violations, 0);
        assert!(result.summary.progress + result.vehicle.radius < 40.0);
        assert_eq!(
            result
                .frames
                .last()
                .unwrap()
                .navigation
                .as_ref()
                .unwrap()
                .phase,
            rustdrive_pipeline::navigation::NavigationPhase::Blocked
        );
    }
}

#[test]
fn late_closure_is_scored_as_failure_and_never_redirects_a_moving_vehicle() {
    let mut s = scenario("route-handover");
    s.navigation_updates[0].stamp = 18.0;
    s.duration = 25.0;
    let result = simulate(s, 7).unwrap();
    assert!(!result.summary.passed);
    assert!(result.summary.closure_violations > 0);
    assert_eq!(result.summary.navigation_switches, 0);
    assert!(result.summary.final_speed < 0.2);
}

#[test]
fn terminal_traffic_is_avoided_through_an_extended_physical_hold() {
    for name in ["gnss-burst-traffic", "gnss-burst-traffic-hold"] {
        let result = simulate(scenario(name), 7).unwrap();
        assert!(result.summary.passed, "{name}: {:?}", result.summary);
        assert_eq!(result.summary.collisions, 0);
        assert!(result.summary.reached_goal);
        assert!(result.summary.min_clearance >= 0.5);
        let mut bytes = Vec::new();
        result
            .sensor_log
            .as_ref()
            .unwrap()
            .write(&mut bytes)
            .unwrap();
        assert_eq!(
            rustdrive_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap()
                .ticks,
            result.summary.steps
        );
    }
}

#[test]
fn goal_hold_does_not_hide_a_collision_after_first_arrival() {
    let mut s = scenario("gnss-spike");
    s.goal_hold_seconds = Some(8.0);
    s.objects.push(
        serde_json::from_value(serde_json::json!({
            "s": 219.0, "lateral": 0.0, "radius": 3.0, "active_from": 34.0
        }))
        .unwrap(),
    );
    let mut no_hold = s.clone();
    no_hold.goal_hold_seconds = None;
    let early = simulate(no_hold, 7).unwrap();
    assert!(early.summary.passed);
    assert!(early.summary.simulated_seconds < 34.0);
    let result = simulate(s, 7).unwrap();
    assert!(result.summary.simulated_seconds >= 34.0);
    assert!(!result.summary.passed);
    assert!(result.summary.collisions > 0);
}

#[test]
fn invalid_goal_residence_and_insufficient_episode_time_are_rejected() {
    let s = scenario("gnss-spike");
    for hold in [-1.0, f64::NAN, f64::INFINITY, s.duration + 1.0] {
        let mut invalid = s.clone();
        invalid.goal_hold_seconds = Some(hold);
        assert!(invalid.validate().is_err());
    }
    let mut short = s;
    short.goal_hold_seconds = Some(35.0);
    let result = simulate(short, 7).unwrap();
    assert!(!result.summary.reached_goal);
    assert!(!result.summary.passed);
}

#[test]
fn reactive_traffic_stops_resumes_and_replays_from_ego_observations() {
    for name in [
        "traffic-lead-stop",
        "traffic-follower-brake",
        "traffic-queue",
    ] {
        let result = simulate(scenario(name), 7).unwrap();
        assert!(result.summary.passed, "{name}: {:?}", result.summary);
        assert_eq!(result.summary.traffic_collisions, 0);
        assert_eq!(result.summary.traffic_road_violations, 0);
        assert!(result.summary.min_clearance >= 1.0);
        assert!(result.frames.iter().any(|f| !f.traffic.is_empty()));
        let mut bytes = Vec::new();
        result
            .sensor_log
            .as_ref()
            .unwrap()
            .write(&mut bytes)
            .unwrap();
        assert_eq!(
            rustdrive_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap()
                .ticks,
            result.summary.steps
        );
    }
}
#[test]
fn traffic_collisions_and_endpoint_overrun_fail_physical_acceptance() {
    let mut s = scenario("traffic-queue");
    s.objects = serde_json::from_value(serde_json::json!([
        {"s":10,"lateral":0,"radius":1,"speed":10,"following":{"initial_speed_m_s":10,"sensor_range_m":5}},
        {"s":30,"lateral":0,"radius":1}
    ])).unwrap();
    let r = simulate(s.clone(), 7).unwrap();
    assert!(!r.summary.passed);
    assert!(r.summary.traffic_collisions > 0);
    s.objects.truncate(1);
    s.objects[0].s = 137.0;
    let r = simulate(s, 7).unwrap();
    assert!(!r.summary.passed);
    assert!(r.summary.traffic_road_violations > 0);
    assert!(
        r.frames
            .iter()
            .any(|f| f.objects[0].position.x > r.route.length())
    );
}
#[test]
fn malformed_following_parameters_and_stop_windows_are_rejected() {
    let s = scenario("traffic-lead-stop");
    for key in [
        "minimum_gap_m",
        "time_headway_s",
        "max_acceleration_m_s2",
        "comfortable_deceleration_m_s2",
        "max_deceleration_m_s2",
        "sensor_range_m",
    ] {
        let mut value = serde_json::to_value(&s).unwrap();
        value["objects"][0]["following"][key] = serde_json::json!(-1.0);
        assert!(
            serde_json::from_value::<Scenario>(value)
                .unwrap()
                .validate()
                .is_err()
        );
    }
    let mut s = s;
    s.objects[0].following.as_mut().unwrap().stop_windows.push(
        rustdrive_sim::traffic::StopWindow {
            from: 9.0,
            until: 12.0,
        },
    );
    assert!(s.validate().is_err());
    s.objects[0]
        .following
        .as_mut()
        .unwrap()
        .stop_windows
        .clear();
    s.objects[0].lateral_speed = 1.0;
    assert!(s.validate().is_err());
    s.objects[0].lateral_speed = 0.0;
    s.objects[0].moving_from = 2.0;
    assert!(s.validate().is_err());
    s.objects[0].moving_from = 0.0;
    s.objects[0].speed = 0.001;
    assert!(s.validate().is_err());
    s.objects[0].speed = 4.0;
    s.objects[0].following.as_mut().unwrap().initial_speed_m_s = f64::NAN;
    assert!(s.validate().is_err());
}
