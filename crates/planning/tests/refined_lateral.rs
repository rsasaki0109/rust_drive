use rustdriving_core::{DrivingMode, EgoState, Planner, Pose, Prediction, Route, Vec2};
use rustdriving_planning::LatticePlanner;

fn object(x: f64, y: f64, radius: f64) -> Prediction {
    Prediction {
        id: (y.abs() * 10.) as u64,
        positions: vec![Vec2::new(x, y); 41],
        radius,
        additional_radius_m: vec![],
        dt: 0.2,
    }
}
fn road(width: f64) -> Route {
    Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], width).unwrap()
}
fn driver(refined: bool) -> LatticePlanner {
    let mut planner = LatticePlanner::default();
    planner.cruise_speed = 4.;
    planner.lateral_candidate_margin_m = Some(0.35);
    planner.refine_lateral_candidates = refined;
    planner
}
fn ego() -> EgoState {
    EgoState {
        pose: Pose::default(),
        speed: 4.,
    }
}
fn confirmed(
    planner: &mut LatticePlanner,
    ego: EgoState,
    route: &Route,
    objects: &[Prediction],
) -> rustdriving_core::Trajectory {
    for stamp in [0., 0.1] {
        planner.observe_refinement_scene(Some(stamp));
        planner.plan(ego, route, objects);
    }
    planner.observe_refinement_scene(Some(0.2));
    planner.plan(ego, route, objects)
}

// Independent exact static-disc/polyline distance, without the planner sweep.
fn clearance(path: &rustdriving_core::Trajectory, object: &Prediction, radius: f64) -> f64 {
    path.points
        .windows(2)
        .map(|p| {
            let d = p[1].position.minus(p[0].position);
            let q = object.positions[0].minus(p[0].position);
            let norm = d.x * d.x + d.y * d.y;
            let f = if norm == 0. {
                0.
            } else {
                ((q.x * d.x + q.y * d.y) / norm).clamp(0., 1.)
            };
            object.positions[0].distance(p[0].position.plus(d.scaled(f))) - radius - object.radius
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn intermediate_path_resolves_the_three_target_clearance_failure_without_losing_the_corridor() {
    let objects = [
        object(30., 0., 0.5),
        object(30., 7.1, 1.5),
        object(30., -7.1, 1.5),
    ];
    let previous = driver(false).plan(ego(), &road(5.5), &objects);
    assert_eq!(previous.lateral_target, 3.5);
    assert!(
        objects
            .iter()
            .map(|o| clearance(&previous, o, 1.25))
            .fold(f64::INFINITY, f64::min)
            < 1.
    );
    let current = confirmed(&mut driver(true), ego(), &road(5.5), &objects);
    assert_eq!(current.lateral_target, 3.0625);
    assert_eq!(current.mode, DrivingMode::Avoid);
    assert!(current.points.last().unwrap().position.x >= 39.9);
    assert!(objects.iter().all(|o| clearance(&current, o, 1.25) >= 1.));
    assert!(
        current
            .points
            .iter()
            .all(|p| p.position.y.abs() + 1.25 <= 5.5)
    );
}

#[test]
fn a_measured_left_conflict_uses_the_same_checked_right_intermediate_path() {
    let objects = [
        object(30., 0., 0.5),
        object(30., 7.1, 1.5),
        object(30., -7.1, 1.5),
        object(15., 3., 1.),
    ];
    let path = confirmed(&mut driver(true), ego(), &road(5.5), &objects);
    assert_eq!(path.lateral_target, -3.0625);
    assert!(objects.iter().all(|o| clearance(&path, o, 1.25) >= 1.));
}

#[test]
fn refinement_cannot_invent_room_or_bypass_missing_width_calibration() {
    let mut missing = LatticePlanner::default();
    missing.refine_lateral_candidates = true;
    assert_eq!(
        missing.plan(ego(), &road(5.5), &[]).mode,
        DrivingMode::Emergency
    );
    for width in [2., 3., 4.5, 5.5] {
        let path = driver(true).plan(ego(), &road(width), &[object(25., 0., 3.)]);
        assert!(matches!(
            path.mode,
            DrivingMode::Yield | DrivingMode::Emergency
        ));
        assert!(
            path.points
                .iter()
                .all(|p| p.position.y.abs() + 1.25 <= width)
        );
    }
}

#[test]
fn an_open_route_does_not_create_an_unnecessary_lateral_maneuver() {
    for refined in [false, true] {
        let path = driver(refined).plan(ego(), &road(5.5), &[]);
        assert_eq!(path.lateral_target, 0.);
        assert_eq!(path.mode, DrivingMode::Cruise);
    }
}

#[test]
fn a_later_complete_refinement_can_replace_a_low_reserve_side_despite_switching_cost() {
    let mut planner = driver(true);
    let mut objects = [
        object(30., 0., 0.85),
        object(30., 7.1, 1.5),
        object(30., -7.1, 1.5),
    ];
    assert_eq!(
        planner.plan(ego(), &road(5.5), &objects).lateral_target,
        3.5
    );
    objects[0].radius = 0.5;
    let measured = EgoState {
        pose: Pose {
            position: Vec2::new(5., 0.4),
            yaw: 0.,
        },
        speed: 4.,
    };
    let next = confirmed(&mut planner, measured, &road(5.5), &objects);
    assert_eq!(next.lateral_target, 3.0625);
    assert!(objects.iter().all(|o| clearance(&next, o, 1.25) >= 1.));
}

#[test]
fn cached_scene_calls_and_interrupted_support_cannot_confirm_an_escape() {
    let objects = [
        object(30., 0., 0.5),
        object(30., 7.1, 1.5),
        object(30., -7.1, 1.5),
    ];
    let mut planner = driver(true);
    for _ in 0..20 {
        planner.observe_refinement_scene(Some(0.));
        assert_eq!(
            planner.plan(ego(), &road(5.5), &objects).lateral_target,
            3.5
        );
    }
    planner.observe_refinement_scene(Some(0.1));
    planner.plan(ego(), &road(5.5), &objects);
    planner.observe_refinement_scene(None);
    planner.observe_refinement_scene(Some(0.2));
    assert_eq!(
        planner.plan(ego(), &road(5.5), &objects).lateral_target,
        3.5
    );
    planner.observe_refinement_scene(Some(0.3));
    planner.plan(ego(), &road(5.5), &objects);
    planner.observe_refinement_scene(Some(0.4));
    assert_eq!(
        planner.plan(ego(), &road(5.5), &objects).lateral_target,
        3.0625
    );
    planner.reset_route_state();
    assert_eq!(
        planner.plan(ego(), &road(5.5), &objects).lateral_target,
        3.5
    );
}
