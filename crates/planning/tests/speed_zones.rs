use rustdriving_core::{DrivingMode, EgoState, Planner, Prediction, Route, RouteSpeedZone, Vec2};
use rustdriving_planning::LatticePlanner;

fn planner(zones: Vec<RouteSpeedZone>) -> LatticePlanner {
    let mut planner = LatticePlanner::default();
    planner.cruise_speed = 6.;
    planner.speed_zones = zones;
    planner
}
fn route() -> Route {
    Route::new(vec![Vec2::default(), Vec2::new(120., 0.)], 2.1).unwrap()
}
fn zone(start_s_m: f64, end_s_m: f64, speed_m_s: f64) -> RouteSpeedZone {
    RouteSpeedZone {
        start_s_m,
        end_s_m,
        speed_m_s,
    }
}

#[test]
fn anticipation_and_release_use_local_caps_instead_of_a_mission_wide_minimum() {
    let path = planner(vec![zone(20., 30., 2.)]).plan(EgoState::default(), &route(), &[]);
    assert_ne!(path.mode, DrivingMode::Emergency);
    assert!(
        path.points
            .iter()
            .any(|p| p.position.x < 10. && p.speed > 5.)
    );
    assert!(
        path.points
            .iter()
            .filter(|p| (18.75..=31.25).contains(&p.position.x))
            .all(|p| p.speed <= 2. + 1e-10)
    );
    assert!(
        path.points
            .iter()
            .any(|p| p.position.x > 37. && p.speed > 4.)
    );
}

#[test]
fn a_subsample_zone_caps_the_entire_crossing_interval() {
    let path = planner(vec![zone(25.11, 25.12, 2.)]).plan(EgoState::default(), &route(), &[]);
    assert_ne!(path.mode, DrivingMode::Emergency);
    for pair in path.points.windows(2) {
        let dx = pair[1].position.x - pair[0].position.x;
        if dx <= 0. {
            continue;
        }
        for i in 0..=100 {
            let fraction = i as f64 / 100.;
            let x = pair[0].position.x + fraction * dx;
            if (25.11 - 1.25..=25.12 + 1.25).contains(&x) {
                let speed_squared =
                    (1. - fraction) * pair[0].speed.powi(2) + fraction * pair[1].speed.powi(2);
                assert!(speed_squared <= 4. + 1e-9);
            }
        }
    }
}

#[test]
fn overlap_and_obstacle_retiming_keep_the_lowest_active_cap() {
    let object = Prediction {
        id: 4,
        positions: vec![Vec2::new(36., 0.); 41],
        radius: 0.7,
        dt: 0.2,
        additional_radius_m: vec![],
    };
    let path = planner(vec![zone(20., 30., 3.), zone(22., 24., 1.5)]).plan(
        EgoState::default(),
        &route(),
        &[object],
    );
    assert_eq!(path.mode, DrivingMode::Yield);
    assert_eq!(path.points.last().unwrap().speed, 0.);
    assert!(path.points.iter().any(|p| p.position.x >= 24.));
    assert!(
        path.points
            .iter()
            .filter(|p| (20.75..=25.25).contains(&p.position.x))
            .all(|p| p.speed <= 1.5 + 1e-10)
    );
    assert!(
        (path.points.last().unwrap().time - path.points[path.points.len() - 2].time - 8.).abs()
            < 1e-10
    );
}

#[test]
fn an_already_active_limit_recovers_measured_overspeed_with_bounded_braking() {
    let ego = EgoState {
        speed: 8.,
        pose: rustdriving_core::Pose {
            position: Vec2::new(25., 0.),
            yaw: 0.,
        },
    };
    let path = planner(vec![zone(20., 80., 2.)]).plan(ego, &route(), &[]);
    assert_ne!(path.mode, DrivingMode::Emergency);
    assert_eq!(path.points[0].speed, 8.);
    let a =
        (path.points[1].speed - path.points[0].speed) / (path.points[1].time - path.points[0].time);
    assert!((a + 2.5).abs() < 1e-9);
    assert!(
        path.points
            .iter()
            .filter(|p| p.position.x >= 38.)
            .all(|p| p.speed <= 2. + 1e-9)
    );
}

#[test]
fn unreachable_future_caps_and_malformed_zones_are_not_silently_ignored() {
    let path = planner(vec![zone(5., 10., 2.)]).plan(
        EgoState {
            speed: 8.,
            ..EgoState::default()
        },
        &route(),
        &[],
    );
    assert_eq!(path.mode, DrivingMode::Emergency);
    assert!(path.points.is_empty());
    for invalid in [
        zone(-1., 10., 2.),
        zone(10., 5., 2.),
        zone(5., 10., 0.),
        zone(5., 10., f64::NAN),
    ] {
        let path = planner(vec![invalid]).plan(EgoState::default(), &route(), &[]);
        assert_eq!(path.mode, DrivingMode::Emergency);
    }
}

#[test]
fn surveyed_arc_intervals_are_independent_of_world_axis() {
    let mut a = planner(vec![zone(20., 30., 2.)]);
    let mut b = planner(vec![zone(20., 30., 2.)]);
    let east = a.plan(EgoState::default(), &route(), &[]);
    let north_route = Route::new(vec![Vec2::default(), Vec2::new(0., 120.)], 2.1).unwrap();
    let north = b.plan(
        EgoState {
            pose: rustdriving_core::Pose {
                position: Vec2::default(),
                yaw: std::f64::consts::FRAC_PI_2,
            },
            speed: 0.,
        },
        &north_route,
        &[],
    );
    assert_eq!(east.points.len(), north.points.len());
    for (x, y) in east.points.iter().zip(&north.points) {
        assert!((x.speed - y.speed).abs() < 1e-9);
        assert!((x.time - y.time).abs() < 1e-9);
    }
}
