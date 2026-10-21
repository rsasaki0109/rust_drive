use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn point(x: f64, yaw: f64) -> Vec2 {
    Vec2::new(x, 0.).rotated(yaw).plus(Vec2::new(12., -7.))
}
fn state(x: f64, speed: f64, yaw: f64) -> EgoState {
    EgoState {
        pose: Pose {
            position: point(x, yaw),
            yaw,
        },
        speed,
    }
}
fn road(yaw: f64) -> Route {
    Route::new(vec![point(0., yaw), point(100., yaw)], 2.1).unwrap()
}
fn growing(x: f64, yaw: f64) -> Prediction {
    Prediction {
        id: 1,
        positions: vec![point(x, yaw); 41],
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
fn planner(search: bool) -> LatticePlanner {
    let mut p = LatticePlanner::default();
    p.settle_obstruction_stops = true;
    p.search_obstruction_stops = search;
    p
}

#[test]
fn an_earlier_stop_repairs_retiming_without_changing_the_forecast_or_braking_authority() {
    for yaw in [0., 0.7, -1.9] {
        let object = growing(30., yaw);
        let mut old = planner(false);
        assert_eq!(
            old.plan(
                state(0., 8., yaw),
                &road(yaw),
                std::slice::from_ref(&object)
            )
            .mode,
            DrivingMode::Emergency
        );
        let mut new = planner(true);
        let p = new.plan(
            state(0., 8., yaw),
            &road(yaw),
            std::slice::from_ref(&object),
        );
        assert_eq!(p.mode, DrivingMode::Yield);
        assert_eq!(p.points[0].speed, 8.);
        let tail = &p.points[p.points.len() - 2..];
        assert_eq!(tail[0].speed, 0.);
        assert_eq!(tail[0].position, tail[1].position);
        assert!((tail[1].time - tail[0].time - 8.).abs() < 1e-12);
        // Independent 1 ms integration of constant acceleration, including the
        // entire hold and every growing-radius interval, rather than chord tests.
        for pair in p.points.windows(2) {
            let a = pair[0];
            let b = pair[1];
            let dt = b.time - a.time;
            let acceleration = (b.speed - a.speed) / dt;
            assert!((-2.5 - 1e-9..=2.0 + 1e-9).contains(&acceleration));
            let length = a.position.distance(b.position);
            assert!((length - (a.speed + b.speed) * dt * 0.5).abs() < 1e-9);
            for i in 0..=(dt * 1000.).ceil() as usize {
                let elapsed = (i as f64 * 0.001).min(dt);
                let time = a.time + elapsed;
                let fraction = if length > 1e-10 {
                    (a.speed * elapsed + 0.5 * acceleration * elapsed * elapsed) / length
                } else {
                    0.
                };
                let position = a
                    .position
                    .plus(b.position.minus(a.position).scaled(fraction));
                let step = (time / 0.2).min(40.);
                let index = step.floor() as usize;
                let next = (index + 1).min(40);
                let extra = object.additional_radius_m[index]
                    + (object.additional_radius_m[next] - object.additional_radius_m[index])
                        * (step - index as f64);
                assert!(
                    position.distance(object.positions[0])
                        > new.vehicle.radius + object.radius + extra + 0.3 + 0.06 * time.min(5.)
                );
            }
        }
    }
}

#[test]
fn a_planned_stop_arms_retention_before_any_emergency_and_only_a_clear_candidate_releases_it() {
    let mut p = planner(true);
    let obj = growing(30., 0.);
    assert_eq!(
        p.plan(state(0., 8., 0.), &road(0.), std::slice::from_ref(&obj))
            .mode,
        DrivingMode::Yield
    );
    assert!(!p.holding_obstruction_stop());
    let anchor = p.plan(state(4., 6., 0.), &road(0.), std::slice::from_ref(&obj));
    assert_eq!(anchor.mode, DrivingMode::Yield);
    assert!(p.braking_obstruction_stop());
    let moving = p.plan(state(8., 4., 0.), &road(0.), std::slice::from_ref(&obj));
    assert_eq!(moving.mode, DrivingMode::Yield);
    assert_eq!(moving.points[0].speed, 4.);
    assert!(
        moving
            .points
            .windows(2)
            .all(|pair| pair[1].speed <= pair[0].speed)
    );
    assert!(
        moving.points.last().unwrap().position.x <= anchor.points.last().unwrap().position.x + 1e-8
    );
    let stopped = p.plan(state(20., 0., 0.), &road(0.), std::slice::from_ref(&obj));
    assert_eq!(stopped.mode, DrivingMode::Yield);
    assert!(p.holding_obstruction_stop());
    assert!(
        stopped
            .points
            .iter()
            .all(|q| q.speed == 0. && q.position == point(20., 0.))
    );
    p.reset_route_state();
    assert!(p.holding_obstruction_stop());
    let mut invalid = obj.clone();
    invalid.additional_radius_m[1] = f64::NAN;
    assert_eq!(
        p.plan(state(20., 0., 0.), &road(0.), &[invalid]).mode,
        DrivingMode::Emergency
    );
    assert!(p.holding_obstruction_stop());
    let clear = p.plan(state(20., 0., 0.), &road(0.), &[]);
    assert_eq!(clear.mode, DrivingMode::Cruise);
    assert!(clear.points[1].speed > 0.);
    assert!(!p.holding_obstruction_stop());
}

#[test]
fn an_ordinary_safe_approach_from_rest_does_not_arm_a_searched_stop() {
    let mut p = planner(true);
    let obj = growing(35., 0.);
    for _ in 0..3 {
        let approach = p.plan(state(0., 0., 0.), &road(0.), std::slice::from_ref(&obj));
        assert_eq!(approach.mode, DrivingMode::Yield);
        assert!(approach.points[1].speed > 0.);
        assert!(!p.holding_obstruction_stop());
    }
}

#[test]
fn unreachable_braking_and_an_interior_hold_conflict_remain_emergencies() {
    let mut p = planner(true);
    assert_eq!(
        p.plan(state(0., 8., 0.), &road(0.), &[growing(14., 0.)])
            .mode,
        DrivingMode::Emergency
    );
    let mut conflict = growing(30., 0.);
    // Clear at both forecast endpoints, but no stopping point survives the peak.
    conflict.additional_radius_m.fill(0.);
    conflict.additional_radius_m[30] = 50.;
    assert_eq!(
        planner(true)
            .plan(state(0., 0., 0.), &road(0.), &[conflict])
            .mode,
        DrivingMode::Emergency
    );
}

#[test]
fn search_without_stop_retention_is_inadmissible() {
    let mut p = planner(true);
    p.settle_obstruction_stops = false;
    assert_eq!(
        p.plan(state(0., 0., 0.), &road(0.), &[]).mode,
        DrivingMode::Emergency
    );
}

#[test]
fn a_complete_clear_side_candidate_can_release_a_searched_stop() {
    let mut p = planner(true);
    p.plan(state(0., 8., 0.), &road(0.), &[growing(30., 0.)]);
    p.plan(state(20., 0., 0.), &road(0.), &[growing(30., 0.)]);
    assert!(p.holding_obstruction_stop());
    let wide = Route::new(vec![point(0., 0.), point(100., 0.)], 5.5).unwrap();
    let mut obj = growing(30., 0.);
    obj.additional_radius_m.clear();
    let escape = p.plan(state(20., 0., 0.), &wide, &[obj]);
    assert_ne!(escape.mode, DrivingMode::Emergency);
    assert!(escape.lateral_target.abs() > 0.1);
    assert!(escape.points.last().unwrap().speed > 0.);
    assert!(!p.holding_obstruction_stop());
}
