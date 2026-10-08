use rustdrive_core::{DrivingMode, EgoState, Planner, Prediction, Route, Vec2};
use rustdrive_planning::LatticePlanner;

#[test]
fn oncoming_obstacle_between_samples_requires_braking() {
    // A supported small circular footprint and a 12 m/s oncoming forecast.
    // At ego speed 3 m/s the first path interval is 0.5 m / 3 m/s.
    // Both interval endpoints are clear; the bodies meet inside that interval.
    let mut planner = LatticePlanner::default();
    planner.vehicle.radius = 0.2;
    let route = Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap();
    let object = Prediction {
        id: 1,
        positions: vec![Vec2::new(1.25, 0.0), Vec2::new(-1.15, 0.0)],
        radius: 0.2,
        dt: 0.2,
    };
    let ego = EgoState {
        speed: 3.0,
        ..EgoState::default()
    };
    let trajectory = planner.plan(ego, &route, &[object]);
    assert_eq!(trajectory.mode, DrivingMode::Yield);
    assert_eq!(trajectory.points[0].speed, 0.0);
}

#[test]
fn malformed_forecasts_cannot_be_treated_as_a_clear_road() {
    let route = Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap();
    let valid = Prediction {
        id: 1,
        positions: vec![Vec2::new(20.0, 0.0)],
        radius: 1.0,
        dt: 0.2,
    };
    for invalid in [
        Prediction {
            positions: vec![],
            ..valid.clone()
        },
        Prediction {
            dt: f64::NAN,
            ..valid.clone()
        },
        Prediction {
            dt: 0.0,
            ..valid.clone()
        },
        Prediction {
            radius: -1.0,
            ..valid.clone()
        },
        Prediction {
            positions: vec![Vec2::new(f64::NAN, 0.0)],
            ..valid
        },
    ] {
        let trajectory = LatticePlanner::default().plan(EgoState::default(), &route, &[invalid]);
        assert_eq!(trajectory.mode, DrivingMode::Emergency);
        assert!(trajectory.points.is_empty());
    }
}
