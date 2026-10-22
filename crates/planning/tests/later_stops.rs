use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn ego(x: f64, speed: f64) -> EgoState {
    EgoState {
        pose: Pose {
            position: Vec2::new(x, 0.),
            yaw: 0.,
        },
        speed,
    }
}
fn road() -> Route {
    Route::new(vec![Vec2::new(0., 0.), Vec2::new(100., 0.)], 2.1).unwrap()
}
fn planner(later: bool) -> LatticePlanner {
    let mut p = LatticePlanner::default();
    p.settle_obstruction_stops = true;
    p.search_obstruction_stops = true;
    p.replan_checked_approaches = true;
    p.lateral_candidate_margin_m = Some(0.35);
    p.search_later_obstruction_stops = later;
    p.set_diagnostics_enabled(true);
    p
}
fn crossing() -> Prediction {
    Prediction {
        id: 7,
        positions: (0..=40)
            .map(|i| Vec2::new(20., -5. + 0.4 * i as f64))
            .collect(),
        radius: 1.,
        dt: 0.2,
        additional_radius_m: vec![],
    }
}
fn growing() -> Prediction {
    Prediction {
        id: 7,
        positions: vec![Vec2::new(30., 0.); 41],
        radius: 1.,
        dt: 0.2,
        additional_radius_m: (0..=40)
            .map(|i| {
                let t = i as f64 * 0.2;
                0.2 + 0.3 * t + 0.05 * t * t
            })
            .collect(),
    }
}

#[test]
fn a_retimed_later_stop_extends_a_crossing_approach_without_arming_earlier_stop_retention() {
    let object = crossing();
    let old = planner(false).plan(ego(0., 8.), &road(), std::slice::from_ref(&object));
    let mut p = planner(true);
    let path = p.plan(ego(0., 8.), &road(), &[object]);
    assert_eq!(path.mode, DrivingMode::Yield);
    assert_eq!(path.points[0].speed, 8.);
    assert!(path.points.last().unwrap().position.x > 12.8);
    assert!(path.points.last().unwrap().position.x > old.points.last().unwrap().position.x + 1.);
    let diagnostics = p.diagnostics().unwrap();
    assert!(
        diagnostics.candidates[diagnostics.selected_candidate.unwrap() as usize]
            .profile
            .as_ref()
            .unwrap()
            .searched_stop
    );
    assert!(!p.braking_obstruction_stop());
    assert!(!p.holding_obstruction_stop());
    let clear = p.plan(ego(1., 8.), &road(), &[]);
    assert_eq!(clear.mode, DrivingMode::Cruise);
    assert!(!p.braking_obstruction_stop());
}

#[test]
fn enabling_later_search_does_not_withdraw_an_existing_braking_or_stationary_commitment() {
    let mut old = planner(false);
    let object = growing();
    assert_eq!(
        old.plan(ego(0., 8.), &road(), std::slice::from_ref(&object))
            .mode,
        DrivingMode::Yield
    );
    assert!(old.braking_obstruction_stop());
    let mut new = old.clone();
    new.search_later_obstruction_stops = true;
    for (x, speed) in [(4., 6.), (8., 4.), (20., 0.)] {
        let a = old.plan(ego(x, speed), &road(), std::slice::from_ref(&object));
        let b = new.plan(ego(x, speed), &road(), std::slice::from_ref(&object));
        assert_eq!(a.mode, b.mode);
        assert_eq!(a.lateral_target, b.lateral_target);
        assert_eq!(a.points.len(), b.points.len());
        for (left, right) in a.points.iter().zip(&b.points) {
            assert_eq!(left.position, right.position);
            assert_eq!(left.speed, right.speed);
            assert_eq!(left.time, right.time);
        }
        assert!(new.braking_obstruction_stop());
    }
    assert!(new.holding_obstruction_stop());
    let clear = new.plan(ego(20., 0.), &road(), &[]);
    assert_eq!(clear.mode, DrivingMode::Cruise);
    assert!(!new.holding_obstruction_stop());
}

#[test]
fn standalone_later_search_requires_checked_approach_ownership() {
    let mut p = planner(true);
    p.replan_checked_approaches = false;
    assert_eq!(
        p.plan(ego(0., 0.), &road(), &[]).mode,
        DrivingMode::Emergency
    );
}
