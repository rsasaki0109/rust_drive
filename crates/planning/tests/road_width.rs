use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn point(x: f64, y: f64, yaw: f64) -> Vec2 {
    Vec2::new(x, y).rotated(yaw).plus(Vec2::new(10., -8.))
}
fn road(width: f64, yaw: f64) -> Route {
    Route::new(vec![point(0., 0., yaw), point(100., 0., yaw)], width).unwrap()
}
fn ego(yaw: f64) -> EgoState {
    EgoState {
        pose: Pose {
            position: point(0., 0., yaw),
            yaw,
        },
        speed: 4.,
    }
}
fn object(x: f64, y: f64, yaw: f64) -> Prediction {
    Prediction {
        id: 1,
        positions: vec![point(x, y, yaw); 41],
        radius: 0.5,
        dt: 0.2,
        additional_radius_m: vec![],
    }
}
fn planner(margin: Option<f64>) -> LatticePlanner {
    let mut p = LatticePlanner::default();
    p.cruise_speed = 4.;
    p.lateral_candidate_margin_m = margin;
    p
}

#[test]
fn a_narrower_road_gets_complete_contained_side_paths_in_every_orientation() {
    for yaw in [0., 0.7, -1.9] {
        let route = road(4.5, yaw);
        let obstacles = [object(25., 0., yaw)];
        let old = planner(None).plan(ego(yaw), &route, &obstacles);
        assert_ne!(old.mode, DrivingMode::Avoid);
        let fitted = planner(Some(0.35)).plan(ego(yaw), &route, &obstacles);
        assert_eq!(fitted.mode, DrivingMode::Avoid);
        assert!((fitted.lateral_target.abs() - 2.9).abs() < 1e-12);
        assert!(route.project(fitted.points.last().unwrap().position).0 > 39.);
        for p in &fitted.points {
            assert!(route.project(p.position).1.abs() + 1.25 <= 4.5 + 1e-10);
        }
    }
}

#[test]
fn an_occupied_left_refuge_uses_the_right_instead_of_clamping_an_unsafe_path() {
    for yaw in [0., 0.7, -1.9] {
        let path = planner(Some(0.35)).plan(
            ego(yaw),
            &road(4.5, yaw),
            &[object(25., 0., yaw), object(25., 2.9, yaw)],
        );
        assert_eq!(path.mode, DrivingMode::Avoid);
        assert!((path.lateral_target + 2.9).abs() < 1e-12);
    }
}

#[test]
fn a_road_without_a_useful_side_corridor_does_not_invent_one() {
    let path = planner(Some(0.35)).plan(ego(0.), &road(1.5, 0.), &[object(25., 0., 0.)]);
    assert_ne!(path.mode, DrivingMode::Avoid);
    assert_eq!(path.lateral_target, 0.);
    assert!(
        path.points
            .iter()
            .all(|p| road(1.5, 0.).project(p.position).1.abs() <= 0.25)
    );
}

#[test]
fn fitting_a_wide_road_preserves_every_original_trajectory_value() {
    let mut old = planner(None);
    let mut fitted = planner(Some(0.35));
    for obstacles in [
        vec![],
        vec![object(25., 0., 0.)],
        vec![object(25., 3.5, 0.)],
        vec![],
    ] {
        let a = old.plan(ego(0.), &road(5.5, 0.), &obstacles);
        let b = fitted.plan(ego(0.), &road(5.5, 0.), &obstacles);
        assert_eq!(a.mode, b.mode);
        assert_eq!(a.lateral_target, b.lateral_target);
        assert_eq!(a.points.len(), b.points.len());
        for (a, b) in a.points.iter().zip(&b.points) {
            assert_eq!(a.position, b.position);
            assert_eq!(a.time, b.time);
            assert_eq!(a.speed, b.speed);
        }
    }
}

#[test]
fn unshortened_width_candidates_preserve_the_chassis_course_join() {
    let mut old = planner(None);
    let mut fitted = planner(Some(0.35));
    for p in [&mut old, &mut fitted] {
        p.local_route_geometry = true;
        p.point_reference_recovery = false;
        p.max_lateral_acceleration_m_s2 = Some(2.5);
    }
    for (x, y) in [(0., 0.), (4., 0.6), (8., 2.5), (27., 3.5)] {
        let measured = EgoState {
            pose: Pose {
                position: point(x, y, 0.),
                yaw: 0.,
            },
            speed: 2.,
        };
        let objects = [object(25., 0., 0.)];
        let a = old.plan(measured, &road(5.5, 0.), &objects);
        let b = fitted.plan(measured, &road(5.5, 0.), &objects);
        assert_ne!(a.mode, DrivingMode::Emergency);
        assert_eq!(a.mode, b.mode);
        assert_eq!(a.lateral_target, b.lateral_target);
        assert_eq!(a.points.len(), b.points.len());
        for (a, b) in a.points.iter().zip(&b.points) {
            assert_eq!(a.position, b.position);
            assert_eq!(a.time, b.time);
            assert_eq!(a.speed, b.speed);
        }
    }
}

#[test]
fn invalid_reserves_fail_closed_in_the_independent_planner() {
    for margin in [f64::NAN, f64::INFINITY, -0.1, 0., 0.09, 1.01] {
        let path = planner(Some(margin)).plan(ego(0.), &road(4.5, 0.), &[]);
        assert_eq!(path.mode, DrivingMode::Emergency);
        assert!(path.points.is_empty());
    }
}
