use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn point(x: f64, y: f64, yaw: f64) -> Vec2 {
    Vec2::new(x, y).rotated(yaw).plus(Vec2::new(12., -7.))
}
fn road(width: f64, yaw: f64) -> Route {
    Route::new(vec![point(0., 0., yaw), point(100., 0., yaw)], width).unwrap()
}
fn ego(x: f64, speed: f64, yaw: f64) -> EgoState {
    EgoState {
        pose: Pose {
            position: point(x, 0., yaw),
            yaw,
        },
        speed,
    }
}
fn object(x: f64, yaw: f64) -> Prediction {
    Prediction {
        id: 1,
        positions: vec![point(x, 0., yaw); 41],
        radius: 1.,
        dt: 0.2,
        additional_radius_m: vec![],
    }
}
fn enabled() -> LatticePlanner {
    let mut planner = LatticePlanner::default();
    planner.settle_obstruction_stops = true;
    planner
}
fn arm(planner: &mut LatticePlanner, yaw: f64) {
    let p = planner.plan(ego(0., 8., yaw), &road(2.1, yaw), &[object(18., yaw)]);
    assert_eq!(p.mode, DrivingMode::Emergency);
    assert!(p.points.is_empty());
    assert!(!planner.holding_obstruction_stop());
    let stopped = planner.plan(ego(10., 0., yaw), &road(2.1, yaw), &[object(18., yaw)]);
    assert_eq!(stopped.mode, DrivingMode::Yield);
    assert!(planner.holding_obstruction_stop());
}

#[test]
fn collision_braking_does_not_restart_on_a_retimed_approach_and_releases_on_a_clear_path() {
    for yaw in [0., 0.7, -1.9] {
        let mut planner = enabled();
        arm(&mut planner, yaw);
        for x in [10., 10.04, 9.98, 10.1] {
            let state = ego(x, 0., yaw);
            let p = planner.plan(state, &road(2.1, yaw), &[object(18., yaw)]);
            assert_eq!(p.mode, DrivingMode::Yield);
            assert!(planner.holding_obstruction_stop());
            assert_eq!(p.points.len(), 2);
            assert_eq!(p.points.last().unwrap().time, 8.);
            assert!(
                p.points
                    .iter()
                    .all(|p| p.speed == 0. && p.position.distance(state.pose.position) < 1e-10)
            );
        }
        let clear = planner.plan(ego(10., 0., yaw), &road(2.1, yaw), &[]);
        assert_eq!(clear.mode, DrivingMode::Cruise);
        assert!(clear.points[1].speed > 0.);
        assert!(!planner.holding_obstruction_stop());
    }
}

#[test]
fn moving_recovery_uses_reachable_monotonic_braking_instead_of_an_instantaneous_speed_change() {
    for yaw in [0., 0.7, -1.9] {
        let mut planner = enabled();
        arm(&mut planner, yaw);
        let state = ego(4., 2., yaw);
        let path = planner.plan(state, &road(2.1, yaw), &[object(18., yaw)]);
        assert_eq!(path.mode, DrivingMode::Yield);
        assert_eq!(path.points[0].speed, 2.);
        assert!(
            path.points
                .last()
                .unwrap()
                .position
                .distance(point(4.8, 0., yaw))
                < 1e-9
        );
        for p in path.points.windows(2) {
            let dt = p[1].time - p[0].time;
            let a = (p[1].speed - p[0].speed) / dt;
            assert!((-2.5 - 1e-9..=1e-9).contains(&a));
            assert!(
                (p[1].position.distance(p[0].position) - (p[0].speed + p[1].speed) * dt / 2.).abs()
                    < 1e-9
            );
        }
    }
}

#[test]
fn an_unsafe_full_hold_is_rejected_and_the_collision_stop_remains_requested() {
    let mut planner = enabled();
    arm(&mut planner, 0.);
    let crossing = Prediction {
        id: 2,
        positions: (0..=40)
            .map(|i| point(10., 10. - i as f64 * 0.4, 0.))
            .collect(),
        ..object(10., 0.)
    };
    let path = planner.plan(
        ego(10., 0., 0.),
        &road(2.1, 0.),
        &[object(18., 0.), crossing],
    );
    assert_eq!(path.mode, DrivingMode::Emergency);
    assert!(path.points.is_empty());
    assert!(planner.holding_obstruction_stop());
}

#[test]
fn a_completely_clear_lateral_escape_releases_even_when_its_speed_score_is_worse() {
    let mut planner = enabled();
    arm(&mut planner, 0.);
    let path = planner.plan(ego(10., 0., 0.), &road(5.5, 0.), &[object(40., 0.)]);
    assert_eq!(path.mode, DrivingMode::Avoid);
    assert!(path.lateral_target.abs() > 3.);
    assert!(path.points[1].speed > 0.);
    assert!(!planner.holding_obstruction_stop());
}

#[test]
fn invalid_inputs_cannot_arm_a_stop_or_withdraw_an_existing_collision_brake() {
    let mut planner = enabled();
    let mut invalid = object(18., 0.);
    invalid.positions.clear();
    assert_eq!(
        planner
            .plan(ego(0., 8., 0.), &road(2.1, 0.), &[invalid.clone()])
            .mode,
        DrivingMode::Emergency
    );
    assert!(!planner.holding_obstruction_stop());
    arm(&mut planner, 0.);
    planner.plan(ego(10., 0., 0.), &road(2.1, 0.), &[invalid]);
    assert!(planner.holding_obstruction_stop());
    planner.reset_route_state();
    let path = planner.plan(ego(10., 0., 0.), &road(2.1, 0.), &[object(18., 0.)]);
    assert!(path.points.iter().all(|p| p.speed == 0.));
    assert!(planner.holding_obstruction_stop());
}

#[test]
fn disabled_policy_retains_the_previous_restart_after_a_collision_emergency() {
    let mut planner = LatticePlanner::default();
    assert_eq!(
        planner
            .plan(ego(0., 8., 0.), &road(2.1, 0.), &[object(18., 0.)])
            .mode,
        DrivingMode::Emergency
    );
    let path = planner.plan(ego(10., 0., 0.), &road(2.1, 0.), &[object(18., 0.)]);
    assert_eq!(path.mode, DrivingMode::Yield);
    assert!(path.points[1].speed > 0.);
    assert!(!planner.holding_obstruction_stop());
}

#[test]
fn tiny_positive_speeds_still_include_a_complete_representable_stationary_hold() {
    for speed in [1e-3, 1e-5, 1e-7] {
        let mut planner = enabled();
        arm(&mut planner, 0.);
        let path = planner.plan(ego(4., speed, 0.), &road(2.1, 0.), &[object(18., 0.)]);
        assert_eq!(path.mode, DrivingMode::Yield);
        assert!(path.points.len() >= 3);
        assert_eq!(path.points[0].speed, speed);
        let last = path.points.last().unwrap();
        let before = &path.points[path.points.len() - 2];
        assert_eq!(last.speed, 0.);
        assert_eq!(before.speed, 0.);
        assert!((last.time - before.time - 8.).abs() < 1e-9);
        assert_eq!(last.position, before.position);
        assert!(path.points.windows(2).all(|p| p[1].speed <= p[0].speed));
    }
}
