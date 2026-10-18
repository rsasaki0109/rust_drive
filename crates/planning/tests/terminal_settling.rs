use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn transformed(point: Vec2, yaw: f64) -> Vec2 {
    Vec2::new(
        12.0 + point.x * yaw.cos() - point.y * yaw.sin(),
        -7.0 + point.x * yaw.sin() + point.y * yaw.cos(),
    )
}

fn road(yaw: f64) -> Route {
    Route::new(
        vec![
            transformed(Vec2::default(), yaw),
            transformed(Vec2::new(100.0, 0.0), yaw),
        ],
        2.1,
    )
    .unwrap()
}

fn ego(x: f64, speed: f64, yaw: f64) -> EgoState {
    EgoState {
        pose: Pose {
            position: transformed(Vec2::new(x, 0.0), yaw),
            yaw,
        },
        speed,
    }
}

#[test]
fn a_settled_goal_does_not_restart_for_small_position_changes() {
    for yaw in [0.0, 0.7, -1.9] {
        let route = road(yaw);
        let mut planner = LatticePlanner::default();
        for x in [98.55, 98.8, 99.1, 98.7, 99.45] {
            let state = ego(x, 0.0, yaw);
            let path = planner.plan(state, &route, &[]);
            assert_eq!(path.mode, DrivingMode::Goal);
            assert_eq!(path.points.len(), 2);
            assert!(
                path.points.iter().all(|p| {
                    p.position.distance(state.pose.position) < 1e-10 && p.speed == 0.0
                })
            );
            assert_eq!(path.points[0].time, 0.0);
            assert_eq!(path.points[1].time, 8.0);
        }
    }
}

#[test]
fn settling_a_slow_vehicle_preserves_reachable_monotonic_braking() {
    for yaw in [0.0, 0.7, -1.9] {
        for speed in [0.02, 0.1, 0.19] {
            let state = ego(98.7, speed, yaw);
            let path = LatticePlanner::default().plan(state, &road(yaw), &[]);
            assert_eq!(path.mode, DrivingMode::Goal);
            assert_eq!(path.points[0].speed, speed);
            let last = path.points.last().unwrap();
            assert_eq!(last.speed, 0.0);
            assert!(last.position.distance(state.pose.position) < 0.02);
            let stop = &path.points[path.points.len() - 2];
            assert_eq!(stop.position, last.position);
            assert_eq!(last.time - stop.time, 8.0);
            for pair in path.points.windows(2) {
                let duration = pair[1].time - pair[0].time;
                let acceleration = (pair[1].speed - pair[0].speed) / duration;
                assert!((-2.5 - 1e-9..=1e-9).contains(&acceleration));
                let expected = 0.5 * (pair[0].speed + pair[1].speed) * duration;
                assert!((pair[0].position.distance(pair[1].position) - expected).abs() < 1e-9);
            }
        }
    }
}

#[test]
fn the_full_hold_still_rejects_a_future_crossing_and_does_not_restart_after_it_clears() {
    let route = road(0.0);
    let state = ego(98.7, 0.0, 0.0);
    let mut planner = LatticePlanner::default();
    let crossing = Prediction {
        id: 1,
        radius: 0.7,
        dt: 0.2,
        positions: (0..=40)
            .map(|i| transformed(Vec2::new(98.7, 10.0 - i as f64 * 0.2), 0.0))
            .collect(),
    };
    let path = planner.plan(state, &route, &[crossing]);
    assert_eq!(path.mode, DrivingMode::Emergency);
    assert!(path.points.is_empty());
    let clear = planner.plan(state, &route, &[]);
    assert_eq!(clear.mode, DrivingMode::Goal);
    assert!(clear.points.iter().all(|p| p.speed == 0.0));
    assert_eq!(clear.points.last().unwrap().position, state.pose.position);
}

#[test]
fn approaching_or_unaligned_vehicles_still_complete_their_goal_maneuver() {
    let route = road(0.0);
    let mut unaligned = ego(98.7, 0.0, 0.0);
    unaligned.pose.yaw = 0.2;
    let mut lateral = ego(98.7, 0.0, 0.0);
    lateral.pose.position.y += 0.3;
    for state in [ego(98.4, 0.0, 0.0), ego(98.7, 0.3, 0.0), unaligned, lateral] {
        let path = LatticePlanner::default().plan(state, &route, &[]);
        assert_ne!(path.mode, DrivingMode::Emergency);
        assert!(path.points.iter().any(|p| p.speed > state.speed));
        assert!(
            path.points
                .last()
                .unwrap()
                .position
                .distance(state.pose.position)
                > 0.09
        );
    }
    let mut exact = LatticePlanner::default();
    exact.terminal_settling = false;
    let path = exact.plan(ego(98.7, 0.0, 0.0), &route, &[]);
    assert!(path.points.iter().any(|p| p.speed > 0.0));
    assert!(
        path.points
            .last()
            .unwrap()
            .position
            .distance(transformed(Vec2::new(99.0, 0.0), 0.0))
            < 1e-8
    );
}

#[test]
fn a_feasible_settled_refuge_remains_preferred_on_a_wide_road() {
    let route = Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap();
    let mut planner = LatticePlanner::default();
    let traffic = Prediction {
        id: 1,
        radius: 1.0,
        dt: 0.2,
        positions: (0..=40)
            .map(|i| Vec2::new(62.0 + 0.5 * i as f64, 0.0))
            .collect(),
    };
    let approaching = EgoState {
        pose: Pose {
            position: Vec2::new(70.0, 0.0),
            yaw: 0.0,
        },
        speed: 6.0,
    };
    let side = planner.plan(approaching, &route, &[traffic]).lateral_target;
    assert!(side.abs() > 3.0);
    let parked = EgoState {
        pose: Pose {
            position: Vec2::new(98.7, side),
            yaw: 0.0,
        },
        speed: 0.0,
    };
    let hold = planner.plan(parked, &route, &[]);
    assert_eq!(hold.mode, DrivingMode::Goal);
    assert_eq!(hold.lateral_target, side);
    assert!(hold.points.iter().all(|p| p.speed == 0.0));
    assert_eq!(hold.points.last().unwrap().position, parked.pose.position);
}
