use super::*;
use crate::{
    RoadNetwork,
    lanes::{Lane, LaneMapSpec, LaneNeighbor},
};

fn spec(side: LaneSide, angle: f64) -> LaneMapSpec {
    let sign = if side == LaneSide::Left { 1.0 } else { -1.0 };
    let point = |x: f64, y: f64| {
        Vec2::new(
            100.0 + x * angle.cos() - y * angle.sin(),
            -50.0 + x * angle.sin() + y * angle.cos(),
        )
    };
    let mut source = Lane {
        id: "source".into(),
        centerline: vec![point(0.0, 0.0), point(80.0, 0.0)],
        width_m: 3.0,
        speed_limit_m_s: 4.0,
        successors: vec![],
        left: None,
        right: None,
    };
    let mut target = Lane {
        id: "target".into(),
        centerline: vec![
            point(0.0, sign * 3.5),
            point(25.0, sign * 3.5),
            point(80.0, sign * 3.5),
        ],
        width_m: 4.0,
        speed_limit_m_s: 3.0,
        successors: vec!["exit".into()],
        left: None,
        right: None,
    };
    let exit = Lane {
        id: "exit".into(),
        centerline: vec![point(80.0, sign * 3.5), point(120.0, sign * 3.5)],
        width_m: 3.2,
        speed_limit_m_s: 2.0,
        successors: vec![],
        left: None,
        right: None,
    };
    let neighbor = |id: &str| {
        Some(LaneNeighbor {
            lane_id: id.into(),
            change_allowed: true,
        })
    };
    match side {
        LaneSide::Left => {
            source.left = neighbor("target");
            target.right = neighbor("source");
        }
        LaneSide::Right => {
            source.right = neighbor("target");
            target.left = neighbor("source");
        }
    }
    LaneMapSpec {
        schema_version: 1,
        lanes: vec![source, target, exit],
    }
}

fn request(side: LaneSide) -> LaneChangeRequest {
    LaneChangeRequest {
        side,
        start_m: 10.0,
        length_m: 40.0,
    }
}

// Independent de Casteljau evaluation: quintic Bezier controls [0,0,0,1,1,1].
fn bezier(controls: &[f64], u: f64) -> f64 {
    let mut values = controls.to_vec();
    for n in (1..values.len()).rev() {
        for i in 0..n {
            values[i] = (1.0 - u) * values[i] + u * values[i + 1];
        }
    }
    values[0]
}

#[test]
fn continuous_curve_matches_independent_bezier_and_fits_unequal_lane_bands_in_both_directions() {
    for angle in [0.0, 0.37, std::f64::consts::FRAC_PI_2, std::f64::consts::PI] {
        for side in [LaneSide::Left, LaneSide::Right] {
            let network = LaneNetwork::new(spec(side, angle)).unwrap();
            let change = network
                .lane_change_route(
                    "source",
                    "exit",
                    &[],
                    request(side),
                    VehicleConfig::default(),
                )
                .unwrap();
            let sign = if side == LaneSide::Left { 1.0 } else { -1.0 };
            let local = |p: Vec2| {
                (
                    (p.x - 100.0) * angle.cos() + (p.y + 50.0) * angle.sin(),
                    -(p.x - 100.0) * angle.sin() + (p.y + 50.0) * angle.cos(),
                )
            };
            let plan = &change.plan;
            assert_eq!(plan.lane_ids, ["source", "target", "exit"]);
            assert_eq!(plan.speed_limit_m_s, 2.0);
            assert_eq!(plan.route.half_width, 1.5);
            assert_eq!(change.maneuver.corridor_tracking_margin_m, 0.25);
            for &point in &plan.route.points {
                let (x, y) = local(point);
                let expected = sign
                    * 3.5
                    * bezier(
                        &[0., 0., 0., 1., 1., 1.],
                        ((x - 10.0) / 40.0).clamp(0.0, 1.0),
                    );
                assert!(
                    (y - expected).abs() < 1e-10,
                    "angle {angle}, side {side:?}, {x},{y}"
                );
                let y = sign * y;
                assert!(y - 1.25 >= -1.5 - 1e-10 && y + 1.25 <= 5.5 + 1e-10);
            }
            let mut dense_length = 80.0;
            let mut previous = (10.0, 0.0);
            for i in 1..=20_000 {
                let u = i as f64 / 20_000.0;
                let current = (10.0 + 40.0 * u, 3.5 * bezier(&[0., 0., 0., 1., 1., 1.], u));
                dense_length += (current.0 - previous.0).hypot(current.1 - previous.1);
                previous = current;
                let dy = 3.5 * bezier(&[0., 0., 5., 0., 0.], u);
                let ddy = 3.5 * bezier(&[0., 20., -20., 0.], u);
                let curvature = (40.0 * ddy).abs() / (1600.0 + dy * dy).powf(1.5);
                assert!(curvature <= change.maneuver.curvature_bound_m_inv + 1e-12);
            }
            assert!((plan.distance_m - dense_length).abs() < 1e-4);
            assert!(plan.distance_m > 120.0 && plan.distance_m < 121.0);
            let resolved = RoadNetwork::new(plan.network.clone())
                .unwrap()
                .shortest_route(&plan.start_node, &plan.goal_node, &[])
                .unwrap();
            assert_eq!(resolved.route.points, plan.route.points);
            assert_eq!(resolved.route.half_width, plan.route.half_width);
        }
    }
}

#[test]
fn map_permission_closures_and_reachable_target_successors_are_required() {
    let network = LaneNetwork::new(spec(LaneSide::Left, 0.0)).unwrap();
    for (start, goal, closed, side) in [
        ("source", "exit", vec![], LaneSide::Right),
        ("source", "exit", vec!["source".into()], LaneSide::Left),
        ("source", "exit", vec!["target".into()], LaneSide::Left),
        ("source", "exit", vec!["exit".into()], LaneSide::Left),
        ("source", "exit", vec!["unknown".into()], LaneSide::Left),
        (
            "source",
            "exit",
            vec!["exit".into(), "exit".into()],
            LaneSide::Left,
        ),
        ("unknown", "exit", vec![], LaneSide::Left),
        ("source", "unknown", vec![], LaneSide::Left),
        ("source", "source", vec![], LaneSide::Left),
    ] {
        assert!(
            network
                .lane_change_route(
                    start,
                    goal,
                    &closed,
                    request(side),
                    VehicleConfig::default()
                )
                .is_err()
        );
    }
    let mut forbidden = spec(LaneSide::Left, 0.0);
    forbidden.lanes[0].left.as_mut().unwrap().change_allowed = false;
    let network = LaneNetwork::new(forbidden).unwrap();
    assert!(
        network
            .lane_change_route(
                "source",
                "target",
                &[],
                request(LaneSide::Left),
                VehicleConfig::default()
            )
            .is_err()
    );
    // The reciprocal map permission remains independent and usable.
    assert!(
        network
            .lane_change_route(
                "target",
                "source",
                &[],
                request(LaneSide::Right),
                VehicleConfig::default()
            )
            .is_ok()
    );
}

#[test]
fn invalid_distance_calibration_curvature_and_envelope_fail_before_a_plan_is_returned() {
    let network = LaneNetwork::new(spec(LaneSide::Left, 0.0)).unwrap();
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
        let mut r = request(LaneSide::Left);
        r.start_m = bad;
        assert!(
            network
                .lane_change_route("source", "target", &[], r, VehicleConfig::default())
                .is_err()
        );
        r = request(LaneSide::Left);
        r.length_m = bad;
        assert!(
            network
                .lane_change_route("source", "target", &[], r, VehicleConfig::default())
                .is_err()
        );
    }
    for (start, length) in [(10., 0.), (10., 2.), (70., 20.), (f64::MAX, f64::MAX)] {
        let r = LaneChangeRequest {
            side: LaneSide::Left,
            start_m: start,
            length_m: length,
        };
        assert!(
            network
                .lane_change_route("source", "target", &[], r, VehicleConfig::default())
                .is_err()
        );
    }
    for vehicle in [
        VehicleConfig {
            wheelbase: 0.0,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            wheelbase: f64::NAN,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            radius: 0.0,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            radius: f64::INFINITY,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            radius: 1.46,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            max_steer: 0.0,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            max_steer: f64::NAN,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            max_steer: std::f64::consts::FRAC_PI_2,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            max_steer: 0.001,
            ..VehicleConfig::default()
        },
    ] {
        assert!(
            network
                .lane_change_route("source", "target", &[], request(LaneSide::Left), vehicle)
                .is_err()
        );
    }
}

#[test]
fn complete_section_and_zero_start_keep_exact_endpoints_and_a_target_goal() {
    let network = LaneNetwork::new(spec(LaneSide::Left, 0.0)).unwrap();
    let r = LaneChangeRequest {
        side: LaneSide::Left,
        start_m: 0.0,
        length_m: 80.0,
    };
    let change = network
        .lane_change_route("source", "target", &[], r, VehicleConfig::default())
        .unwrap();
    assert_eq!(
        change.plan.route.points.first(),
        network.lane("source").unwrap().centerline.first()
    );
    assert_eq!(
        change.plan.route.points.last(),
        network.lane("target").unwrap().centerline.last()
    );
    assert_eq!(change.plan.lane_ids, ["source", "target"]);
    assert_eq!(change.plan.route.points.len(), 321);
    assert!(
        change
            .plan
            .route
            .points
            .windows(2)
            .all(|p| p[0].distance(p[1]) > 0.0)
    );
}

#[test]
fn sampling_budget_is_bounded_before_allocation_and_tolerated_neighbor_drift_is_included() {
    let mut long = spec(LaneSide::Left, 0.0);
    long.lanes.pop();
    long.lanes[1].successors.clear();
    for lane in &mut long.lanes {
        for p in &mut lane.centerline {
            p.x = 100.0 + (p.x - 100.0) * 100.0;
        }
    }
    let network = LaneNetwork::new(long).unwrap();
    let r = LaneChangeRequest {
        side: LaneSide::Left,
        start_m: 0.0,
        length_m: 1024.25,
    };
    assert!(
        network
            .lane_change_route("source", "target", &[], r, VehicleConfig::default())
            .unwrap_err()
            .contains("budget")
    );
    let mut drift = spec(LaneSide::Left, 0.0);
    drift.lanes.pop();
    drift.lanes[1].successors.clear();
    drift.lanes[1].centerline = vec![
        Vec2::new(100.0 + 1e-7, -46.5),
        Vec2::new(180.0 + 2e-7, -46.5 + 1e-7),
    ];
    let network = LaneNetwork::new(drift).unwrap();
    let change = network
        .lane_change_route(
            "source",
            "target",
            &[],
            request(LaneSide::Left),
            VehicleConfig::default(),
        )
        .unwrap();
    assert!(change.maneuver.curvature_bound_m_inv > 3.5 * 10.0 / 3.0_f64.sqrt() / 1600.0);
    assert_eq!(
        change.plan.route.points.last(),
        network.lane("target").unwrap().centerline.last()
    );
}
