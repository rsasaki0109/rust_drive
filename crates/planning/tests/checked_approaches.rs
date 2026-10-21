use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn at(x: f64, y: f64, yaw: f64) -> Vec2 {
    Vec2::new(x, y).rotated(yaw).plus(Vec2::new(12., -7.))
}
fn road(width: f64, yaw: f64) -> Route {
    Route::new(vec![at(0., 0., yaw), at(100., 0., yaw)], width).unwrap()
}
fn ego(x: f64, speed: f64, yaw: f64) -> EgoState {
    EgoState {
        pose: Pose {
            position: at(x, 0., yaw),
            yaw,
        },
        speed,
    }
}
fn object(x: f64, yaw: f64, growing: bool) -> Prediction {
    Prediction {
        id: 7,
        positions: vec![at(x, 0., yaw); 41],
        radius: 1.,
        dt: 0.2,
        additional_radius_m: if growing {
            (0..=40)
                .map(|i| {
                    let t = i as f64 * 0.2;
                    0.2 + 0.3 * t + 0.05 * t * t
                })
                .collect()
        } else {
            vec![]
        },
    }
}
fn armed(yaw: f64) -> LatticePlanner {
    let mut p = LatticePlanner::default();
    p.settle_obstruction_stops = true;
    p.search_obstruction_stops = true;
    p.replan_checked_approaches = true;
    p.lateral_candidate_margin_m = Some(0.35);
    assert_eq!(
        p.plan(
            ego(0., 8., yaw),
            &road(2.1, yaw),
            &[object(18., yaw, false)]
        )
        .mode,
        DrivingMode::Emergency
    );
    p.plan(
        ego(10., 0., yaw),
        &road(2.1, yaw),
        &[object(18., yaw, false)],
    );
    assert!(p.holding_obstruction_stop());
    p
}

#[test]
fn new_acquisitions_can_replace_a_stationary_hold_with_a_complete_checked_stop() {
    for yaw in [0., 0.7, -1.9] {
        let mut p = armed(yaw);
        let r = road(5.5, yaw);
        let o = object(50., yaw, true);
        let e = ego(10., 0., yaw);
        for (now, stamp, count) in [(0., 0., 1), (0.05, 0., 1), (0.1, 0.1, 2), (0.15, 0.1, 2)] {
            p.observe_checked_approach_scene(now, Some(stamp));
            let path = p.plan(e, &r, std::slice::from_ref(&o));
            assert_eq!(
                p.checked_approach_diagnostics().supporting_acquisitions,
                count
            );
            assert!(!p.checked_approach_diagnostics().adopted);
            assert!(path.points.iter().all(|q| q.speed == 0.));
        }
        p.observe_checked_approach_scene(0.2, Some(0.2));
        let path = p.plan(e, &r, std::slice::from_ref(&o));
        let status = p.checked_approach_diagnostics();
        assert!(status.adopted);
        assert_eq!(status.supporting_acquisitions, 3);
        assert_eq!(path.mode, DrivingMode::Yield);
        assert!(path.points.iter().any(|q| q.speed > 1.));
        let last = path.points.last().unwrap();
        let before = path.points[path.points.len() - 2];
        assert_eq!(last.position, before.position);
        assert_eq!(last.speed, 0.);
        assert!(last.time - before.time >= 8. - 1e-9);
        assert_eq!(status.proposal_endpoint, Some(last.position));
        assert!(status.proposal_end_progress_m.unwrap() >= 12.);
        // Independently sample accelerated motion and the entire stationary tail.
        for pair in path.points.windows(2) {
            let a = pair[0];
            let b = pair[1];
            let duration = b.time - a.time;
            for i in 0..=100 {
                let dt = duration * i as f64 / 100.;
                let t = a.time + dt;
                let arc = a.speed * dt + (b.speed - a.speed) * dt * dt / (2. * duration);
                let length = a.position.distance(b.position);
                let position = if length > 0. {
                    a.position
                        .plus(b.position.minus(a.position).scaled(arc / length))
                } else {
                    a.position
                };
                let knot = (t / 0.2).clamp(0., 40.);
                let k = knot.floor() as usize;
                let next = (k + 1).min(40);
                let radius = 1.
                    + o.additional_radius_m[k]
                    + (o.additional_radius_m[next] - o.additional_radius_m[k]) * (knot - k as f64);
                assert!(position.distance(at(50., 0., yaw)) > p.vehicle.radius + radius);
                assert!(r.project(position).1.abs() + p.vehicle.radius <= r.half_width + 1e-9);
            }
        }
    }
}

#[test]
fn a_fault_or_route_reset_cannot_keep_confirmation_and_cached_acquisitions_cannot_restart_it() {
    for reset in [false, true] {
        let mut p = armed(0.);
        let r = road(5.5, 0.);
        let o = [object(50., 0., true)];
        let e = ego(10., 0., 0.);
        for now in [0., 0.1] {
            p.observe_checked_approach_scene(now, Some(now));
            p.plan(e, &r, &o);
        }
        if reset {
            p.reset_route_state();
        } else {
            p.observe_checked_approach_scene(0.15, None);
            p.plan(e, &r, &o);
        }
        p.observe_checked_approach_scene(0.2, Some(0.1));
        p.plan(e, &r, &o);
        assert!(!p.checked_approach_diagnostics().adopted);
        assert_eq!(p.checked_approach_diagnostics().supporting_acquisitions, 0);
    }
}

#[test]
fn an_unsafe_prediction_or_moving_ego_cannot_replace_the_stopping_commitment() {
    let mut p = armed(0.);
    let r = road(5.5, 0.);
    let mut o = object(50., 0., true);
    o.additional_radius_m[0] = 100.;
    for now in [0., 0.1, 0.2] {
        p.observe_checked_approach_scene(now, Some(now));
        let path = p.plan(ego(10., 0., 0.), &r, std::slice::from_ref(&o));
        assert_eq!(path.mode, DrivingMode::Emergency);
        assert!(!p.checked_approach_diagnostics().adopted);
        assert!(p.holding_obstruction_stop());
    }
    let o = [object(50., 0., true)];
    for now in [0.3, 0.4, 0.5] {
        p.observe_checked_approach_scene(now, Some(now));
        p.plan(ego(10., 0.4, 0.), &r, &o);
        assert_eq!(p.checked_approach_diagnostics().supporting_acquisitions, 0);
    }
}

#[test]
fn standalone_policy_requires_stop_search_retention_and_declared_width() {
    for which in 0..3 {
        let mut p = armed(0.);
        match which {
            0 => p.search_obstruction_stops = false,
            1 => p.settle_obstruction_stops = false,
            _ => p.lateral_candidate_margin_m = None,
        }
        p.observe_checked_approach_scene(0., Some(0.));
        assert_eq!(
            p.plan(ego(10., 0., 0.), &road(5.5, 0.), &[object(50., 0., true)])
                .mode,
            DrivingMode::Emergency
        );
        assert!(!p.checked_approach_diagnostics().adopted);
    }
}
