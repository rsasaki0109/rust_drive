use super::*;
use rustdriving_core::Pose;

fn config() -> LaneChangeConfig {
    LaneChangeConfig {
        entry_s_m: 25.0,
        exit_s_m: 56.0,
        target_start: Vec2::new(0.0, 3.5),
        target_end: Vec2::new(80.0, 3.5),
        target_width_m: 3.5,
        return_on_conflict: None,
    }
}

#[test]
fn an_envelope_containing_a_rear_forecast_cannot_admit_its_closing_gap() {
    let continued = forecast(7.25, 3.5, 0.6, 0.);
    assert!(blocks(&config(), &continued, 20., 2.5, 1.25));
    let mut uncertain = forecast(7.25, 3.5, 0.3, 0.);
    uncertain.additional_radius_m = (0..uncertain.positions.len())
        .map(|i| 0.3 * i as f64 * uncertain.dt)
        .collect();
    // The enclosing circle covers both stopping here and continuing at 0.6 m/s.
    for (i, center) in uncertain.positions.iter().enumerate() {
        assert!(
            center.distance(continued.positions[i]) <= uncertain.additional_radius_m[i] + 1e-10
        );
        assert!(
            center.distance(continued.positions[0]) <= uncertain.additional_radius_m[i] + 1e-10
        );
    }
    assert!(blocks(&config(), &uncertain, 20., 2.5, 1.25));
}
fn forecast(x: f64, y: f64, vx: f64, vy: f64) -> Prediction {
    Prediction {
        additional_radius_m: vec![],
        id: 7,
        positions: (0..=30)
            .map(|i| Vec2::new(x + vx * i as f64 * 0.2, y + vy * i as f64 * 0.2))
            .collect(),
        radius: 1.0,
        dt: 0.2,
    }
}

#[test]
fn rear_closing_and_front_traffic_block_but_departing_and_other_lane_traffic_do_not() {
    let c = config();
    assert!(blocks(&c, &forecast(-30.0, 3.5, 6.0, 0.0), 20.0, 2.5, 1.25));
    assert!(blocks(&c, &forecast(35.0, 3.5, 0.0, 0.0), 20.0, 2.5, 1.25));
    assert!(!blocks(&c, &forecast(60.0, 3.5, 6.0, 0.0), 20.0, 2.5, 1.25));
    assert!(!blocks(&c, &forecast(35.0, 0.0, 0.0, 0.0), 20.0, 2.5, 1.25));
    // Rotate the map and the forecast 90 degrees: classification is unchanged.
    let mut rotated = config();
    rotated.target_start = Vec2::new(-3.5, 0.0);
    rotated.target_end = Vec2::new(-3.5, 80.0);
    let mut p = forecast(-30.0, 3.5, 6.0, 0.0);
    for point in &mut p.positions {
        *point = Vec2::new(-point.y, point.x);
    }
    assert!(blocks(&rotated, &p, 20.0, 2.5, 1.25));
}

#[test]
fn continuous_segment_clipping_catches_a_crossing_with_both_samples_outside_the_lane() {
    let p = Prediction {
        additional_radius_m: vec![],
        id: 1,
        positions: vec![Vec2::new(35.0, -100.0), Vec2::new(35.0, 100.0)],
        radius: 1.0,
        dt: 6.0,
    };
    assert!(blocks(&config(), &p, 20.0, 2.5, 1.25));
    let mut outside = p.clone();
    for point in &mut outside.positions {
        point.x = 100.0;
    }
    assert!(!blocks(&config(), &outside, 20.0, 2.5, 1.25));
    // Independent dense time sampling agrees on a grid of finite crossing motions.
    for x in [-30.0, 0.0, 20.0, 35.0, 65.0] {
        for vx in [-4.0, 0.0, 4.0] {
            let p = forecast(x, -8.0, vx, 4.0);
            let dense = (0..=60_000).any(|i| {
                let t = i as f64 / 10_000.0;
                let px = x + vx * t;
                let py = -8.0 + 4.0 * t;
                (py - 3.5).abs() <= 3.25
                    && px >= 20.0 - (1.25 + 1.0 + 6.0 + 2.0 * vx.max(0.0))
                    && px <= 20.0 + 2.5 * t + (1.25 + 1.0 + 6.0 + 2.0 * 2.5)
            });
            assert_eq!(
                blocks(&config(), &p, 20.0, 2.5, 1.25),
                dense,
                "x {x}, vx {vx}"
            );
        }
    }
}

#[test]
fn incomplete_or_invalid_forecasts_hold_admission_instead_of_inventing_a_clear_gap() {
    for case in 0..8 {
        let mut p = forecast(100.0, 3.5, 0.0, 0.0);
        match case {
            0 => p.positions.clear(),
            1 => p.positions.truncate(5),
            2 => p.dt = f64::NAN,
            3 => p.radius = f64::INFINITY,
            4 => p.positions[2].x = f64::NAN,
            5 => p.positions[2].y = f64::MAX,
            6 => p.positions = vec![Vec2::default(); 1025],
            _ => p.radius = -1.0,
        }
        assert!(blocks(&config(), &p, 20.0, 2.5, 1.25), "case {case}");
    }
}

#[test]
fn ready_is_revoked_before_entry_but_a_started_change_does_not_restore_an_old_prefix() {
    let route = Route::new(
        vec![
            Vec2::default(),
            Vec2::new(25.0, 0.0),
            Vec2::new(55.0, 3.5),
            Vec2::new(160.0, 3.5),
        ],
        1.75,
    )
    .unwrap();
    let mut gate = LaneChangeGate::new(config(), VehicleConfig::default(), 2.0);
    let mut ego = EgoState {
        pose: Pose {
            position: Vec2::new(20.0, 0.0),
            yaw: 0.0,
        },
        speed: 0.0,
    };
    for i in 0..=10 {
        let t = i as f64 * 0.1;
        gate.step(&route, ego, &[], t, Some(t), true);
    }
    assert_eq!(gate.status().phase, LaneChangePhase::Ready);
    gate.step(
        &route,
        ego,
        &[forecast(30.0, 3.5, 0.0, 0.0)],
        1.1,
        Some(1.1),
        true,
    );
    assert_eq!(gate.status().phase, LaneChangePhase::Waiting);
    assert!(gate.planning_route(&route).unwrap().length() < 25.0);
    for i in 12..=22 {
        let t = i as f64 * 0.1;
        gate.step(&route, ego, &[], t, Some(t), true);
    }
    assert_eq!(gate.status().phase, LaneChangePhase::Ready);
    ego.pose.position = Vec2::new(25.0, 0.0);
    gate.step(&route, ego, &[], 2.3, Some(2.3), true);
    assert_eq!(gate.status().phase, LaneChangePhase::Changing);
    gate.step(
        &route,
        ego,
        &[forecast(30.0, 3.5, 0.0, 0.0)],
        2.4,
        Some(2.4),
        false,
    );
    assert_eq!(gate.status().phase, LaneChangePhase::Changing);
    assert!(gate.planning_route(&route).is_none());
    ego.pose.position = Vec2::new(60.0, 3.5);
    gate.step(&route, ego, &[], 2.5, Some(2.5), true);
    assert_eq!(gate.status().phase, LaneChangePhase::Completed);
}

#[test]
fn missed_acquisition_gaps_with_fresh_deliveries_restart_the_full_clear_interval() {
    let route = Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 1.75).unwrap();
    let ego = EgoState::default();
    let mut gate = LaneChangeGate::new(config(), VehicleConfig::default(), 2.0);
    for t in [
        0.0, 0.1, 0.2, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4,
    ] {
        gate.step(&route, ego, &[], t, Some(t), true);
        assert_eq!(gate.status().phase, LaneChangePhase::Waiting);
    }
    gate.step(&route, ego, &[], 1.5, Some(1.5), true);
    assert_eq!(gate.status().phase, LaneChangePhase::Ready);
    assert_eq!(gate.status().clear_since, Some(0.5));
}

#[test]
fn uncertain_occupancy_blocks_lane_entry_and_malformed_shapes_fail_closed() {
    let mut p = forecast(35., 0., 0., 0.);
    assert!(!blocks(&config(), &p, 20., 2.5, 1.25));
    p.additional_radius_m = (0..p.positions.len()).map(|i| i as f64 * 0.1).collect();
    assert!(blocks(&config(), &p, 20., 2.5, 1.25));
    p.additional_radius_m = vec![0.];
    assert!(blocks(&config(), &p, 20., 2.5, 1.25));
}

#[test]
fn stopping_continuation_envelopes_retain_blocked_rear_gaps_across_rotated_axes() {
    let mut blocked_comparisons = 0;
    for velocity in [0.1, 0.2, 0.5, 0.69] {
        for excess_gap in [-1., -0.5, -0.1, 0.1, 0.5, 1.] {
            let x = 20. - (1.25 + 1. + 6. + velocity * 8.) - excess_gap;
            let continued = forecast(x, 3.5, velocity, 0.);
            let mut uncertain = forecast(x, 3.5, velocity / 2., 0.);
            uncertain.additional_radius_m = (0..uncertain.positions.len())
                .map(|i| velocity / 2. * i as f64 * uncertain.dt)
                .collect();
            for angle in [0., std::f64::consts::FRAC_PI_2, -0.7] {
                let transform = |p: Vec2| p.rotated(angle).plus(Vec2::new(-13., 7.));
                let mut c = config();
                c.target_start = transform(c.target_start);
                c.target_end = transform(c.target_end);
                let mut a = continued.clone();
                let mut b = uncertain.clone();
                for p in &mut a.positions {
                    *p = transform(*p);
                }
                for p in &mut b.positions {
                    *p = transform(*p);
                }
                if blocks(&c, &a, 20., 2.5, 1.25) {
                    blocked_comparisons += 1;
                    assert!(
                        blocks(&c, &b, 20., 2.5, 1.25),
                        "velocity={velocity}, gap={excess_gap}, angle={angle}"
                    );
                }
            }
        }
    }
    assert_eq!(blocked_comparisons, 36);
}

#[test]
fn an_envelope_growing_around_receding_motion_can_still_leave_a_clear_gap() {
    let mut receding = forecast(11.5, 3.5, -0.3, 0.);
    receding.additional_radius_m = (0..receding.positions.len())
        .map(|i| 0.3 * i as f64 * receding.dt)
        .collect();
    // The forward extent stays at x=11.5; growth does not turn receding motion
    // into an approaching vehicle. A stationary body at x=11.5 is also clear.
    assert!(!blocks(
        &config(),
        &forecast(11.5, 3.5, 0., 0.),
        20.,
        2.5,
        1.25
    ));
    assert!(!blocks(&config(), &receding, 20., 2.5, 1.25));
}

#[test]
fn expanding_rear_occupancy_revokes_ready_and_restarts_clear_acquisitions() {
    let route = Route::new(vec![Vec2::default(), Vec2::new(80., 0.)], 1.75).unwrap();
    let mut gate = LaneChangeGate::new(config(), VehicleConfig::default(), 2.);
    let ego = EgoState {
        pose: Pose {
            position: Vec2::new(20., 0.),
            yaw: 0.,
        },
        speed: 0.,
    };
    for i in 0..=10 {
        let t = i as f64 * 0.1;
        gate.step(&route, ego, &[], t, Some(t), true);
    }
    assert_eq!(gate.status().phase, LaneChangePhase::Ready);
    let mut uncertain = forecast(7.25, 3.5, 0.3, 0.);
    uncertain.additional_radius_m = (0..uncertain.positions.len())
        .map(|i| 0.3 * i as f64 * uncertain.dt)
        .collect();
    gate.step(&route, ego, &[uncertain], 1.1, Some(1.1), true);
    assert_eq!(gate.status().phase, LaneChangePhase::Waiting);
    assert_eq!(gate.status().blocking_tracks, vec![7]);
    assert!(gate.status().clear_since.is_none());
    for i in 12..=22 {
        let t = i as f64 * 0.1;
        gate.step(&route, ego, &[], t, Some(t), true);
        assert_eq!(
            gate.status().phase,
            if i == 22 {
                LaneChangePhase::Ready
            } else {
                LaneChangePhase::Waiting
            }
        );
    }
}
