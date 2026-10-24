use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn world(point: Vec2, yaw: f64) -> Vec2 {
    Vec2::new(
        12. + point.x * yaw.cos() - point.y * yaw.sin(),
        -7. + point.x * yaw.sin() + point.y * yaw.cos(),
    )
}

fn route(yaw: f64) -> Route {
    Route::new(
        vec![world(Vec2::default(), yaw), world(Vec2::new(100., 0.), yaw)],
        2.,
    )
    .unwrap()
}

fn lead(x: f64, yaw: f64) -> Prediction {
    Prediction {
        id: 1,
        positions: vec![world(Vec2::new(x, 0.), yaw); 41],
        radius: 1.,
        additional_radius_m: vec![],
        dt: 0.2,
    }
}

fn settled(yaw: f64) -> (LatticePlanner, EgoState) {
    let mut planner = LatticePlanner::default();
    planner.cruise_speed = 4.;
    let approach = EgoState {
        pose: Pose {
            position: world(Vec2::default(), yaw),
            yaw,
        },
        speed: 4.,
    };
    let braking = planner.plan(approach, &route(yaw), &[lead(24., yaw)]);
    assert_eq!(braking.mode, DrivingMode::Yield);
    let stopped = EgoState {
        pose: Pose {
            position: braking.points.last().unwrap().position,
            yaw,
        },
        speed: 0.,
    };
    let hold = planner.plan(stopped, &route(yaw), &[lead(24., yaw)]);
    assert_eq!(hold.mode, DrivingMode::Yield);
    assert!(hold.points.iter().all(|p| p.speed == 0.));
    (planner, stopped)
}

fn crossing(stopped: EgoState, yaw: f64) -> Prediction {
    let left = Vec2::new(-yaw.sin(), yaw.cos());
    Prediction {
        id: 2,
        positions: (0..=40)
            .map(|i| stopped.pose.position.plus(left.scaled(6. - i as f64 * 0.2)))
            .collect(),
        radius: 0.5,
        additional_radius_m: vec![],
        dt: 0.2,
    }
}

#[test]
fn a_temporary_infeasible_hold_does_not_restart_the_same_blocked_queue() {
    for yaw in [0., 0.7, -1.9] {
        for measured_speed in [0., 0.02, 0.12] {
            let (mut planner, stopped) = settled(yaw);
            let traffic = [lead(24., yaw), crossing(stopped, yaw)];
            // Retaining a stop must never accept an unsafe eight-second hold.
            for _ in 0..2 {
                let emergency = planner.plan(stopped, &route(yaw), &traffic);
                assert_eq!(emergency.mode, DrivingMode::Emergency);
                assert!(emergency.points.is_empty());
            }
            let mut observed = stopped;
            observed.speed = measured_speed;
            // A refreshed stationary lead estimate offers a farther stopping
            // point. It does not authorize restarting a completed queue stop.
            let held = planner.plan(observed, &route(yaw), &[lead(26., yaw)]);
            assert_eq!(held.mode, DrivingMode::Yield);
            assert_eq!(held.points[0].speed, measured_speed);
            let distance = held
                .points
                .last()
                .unwrap()
                .position
                .distance(stopped.pose.position);
            assert!(
                distance <= measured_speed * measured_speed / 5. + 1e-9,
                "blocked queue restarted after a temporary emergency"
            );
            for pair in held.points.windows(2) {
                assert!(pair[1].speed <= pair[0].speed);
                let duration = pair[1].time - pair[0].time;
                let acceleration = (pair[1].speed - pair[0].speed) / duration;
                assert!((-2.5 - 1e-9..=1e-9).contains(&acceleration));
                assert!(
                    (pair[0].position.distance(pair[1].position)
                        - 0.5 * (pair[0].speed + pair[1].speed) * duration)
                        .abs()
                        < 1e-9
                );
            }
            let last = held.points.last().unwrap();
            let before = &held.points[held.points.len() - 2];
            assert_eq!(last.speed, 0.);
            assert!((last.time - before.time - 8.).abs() < 1e-9);
            // A completely clear candidate can release the retained stop.
            let clear = planner.plan(stopped, &route(yaw), &[]);
            assert_ne!(clear.mode, DrivingMode::Emergency);
            assert!(clear.points.iter().any(|p| p.speed > 0.1));
        }
    }
}

#[test]
fn a_route_reset_withdraws_the_previous_yield_endpoint() {
    let (mut planner, stopped) = settled(0.);
    let emergency = planner.plan(stopped, &route(0.), &[lead(24., 0.), crossing(stopped, 0.)]);
    assert_eq!(emergency.mode, DrivingMode::Emergency);
    planner.reset_route_state();
    let renewed = planner.plan(stopped, &route(0.), &[lead(26., 0.)]);
    assert_ne!(renewed.mode, DrivingMode::Emergency);
    assert!(renewed.points.iter().any(|p| p.speed > 0.1));
}

#[test]
fn legacy_recordings_can_reproduce_the_original_restart() {
    let (mut planner, stopped) = settled(0.);
    planner.retain_yield_stop_on_emergency = false;
    let emergency = planner.plan(stopped, &route(0.), &[lead(24., 0.), crossing(stopped, 0.)]);
    assert_eq!(emergency.mode, DrivingMode::Emergency);
    let renewed = planner.plan(stopped, &route(0.), &[lead(26., 0.)]);
    assert_ne!(renewed.mode, DrivingMode::Emergency);
    assert!(renewed.points.iter().any(|p| p.speed > 0.1));
}
