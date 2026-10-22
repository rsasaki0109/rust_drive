use super::*;

fn request() -> LaneReturnRequest {
    LaneReturnRequest {
        start: Pose {
            position: Vec2::new(28.0, 0.2),
            yaw: 0.08,
        },
        curvature_m_inv: 0.015,
        lane_start: Vec2::default(),
        lane_end: Vec2::new(80.0, 0.0),
        half_width_m: 1.75,
        distance_m: 30.0,
    }
}

#[test]
fn measured_start_and_straight_end_match_independent_quintic_boundary_solution() {
    for angle in [0.0, 0.37, std::f64::consts::FRAC_PI_2, std::f64::consts::PI] {
        for sign in [-1.0, 1.0] {
            let rotate = |p: Vec2| {
                Vec2::new(
                    100.0 + p.x * angle.cos() - p.y * angle.sin(),
                    -50.0 + p.x * angle.sin() + p.y * angle.cos(),
                )
            };
            let mut r = request();
            r.start.position.y *= sign;
            r.start.yaw = angle + r.start.yaw * sign;
            r.curvature_m_inv *= sign;
            r.start.position = rotate(r.start.position);
            r.lane_start = rotate(r.lane_start);
            r.lane_end = rotate(r.lane_end);
            let plan = plan(r, VehicleConfig::default()).unwrap();
            let y0 = sign * 0.2;
            let slope = (sign * 0.08_f64).tan();
            let a1 = slope * 30.0;
            let a2 = 0.5 * r.curvature_m_inv * (1.0 + slope * slope).powf(1.5) * 900.0;
            // Solve y,y',y'' at u=0 and u=1 directly in the power basis.
            let a3 = -10.0 * y0 - 6.0 * a1 - 3.0 * a2;
            let a4 = 15.0 * y0 + 8.0 * a1 + 3.0 * a2;
            let a5 = -6.0 * y0 - 3.0 * a1 - a2;
            for (i, p) in plan.route.points.iter().enumerate() {
                let u = i as f64 / (plan.route.points.len() - 1) as f64;
                let y = y0 + a1 * u + a2 * u * u + a3 * u.powi(3) + a4 * u.powi(4) + a5 * u.powi(5);
                assert!(p.distance(rotate(Vec2::new(28.0 + 30.0 * u, y))) < 1e-10);
            }
            for i in 0..=10_000 {
                let u = i as f64 / 10_000.0;
                let dy = (a1
                    + 2.0 * a2 * u
                    + 3.0 * a3 * u * u
                    + 4.0 * a4 * u.powi(3)
                    + 5.0 * a5 * u.powi(4))
                    / 30.0;
                let ddy =
                    (2.0 * a2 + 6.0 * a3 * u + 12.0 * a4 * u * u + 20.0 * a5 * u.powi(3)) / 900.0;
                assert!(
                    ddy.abs() / (1.0 + dy * dy).powf(1.5) <= plan.curvature_bound_m_inv + 1e-12
                );
            }
            assert_eq!(plan.route.points[0], r.start.position);
            assert!(
                plan.route
                    .points
                    .last()
                    .unwrap()
                    .distance(rotate(Vec2::new(58.0, 0.0)))
                    < 1e-10
            );
            let b = plan.lateral_controls_m;
            assert!((5.0 * (b[1] - b[0]) / 30.0 - slope).abs() < 1e-12);
            assert!(
                (20.0 * (b[2] - 2.0 * b[1] + b[0]) / 900.0 / (1.0 + slope * slope).powf(1.5)
                    - r.curvature_m_inv)
                    .abs()
                    < 1e-12
            );
            assert_eq!(&b[3..], &[0.0, 0.0, 0.0]);
            assert_eq!(plan.route.points.len(), 121);
        }
    }
}

#[test]
fn nonfinite_late_backward_unbounded_and_unsteerable_returns_are_rejected() {
    for case in 0..14 {
        let mut r = request();
        match case {
            0 => r.start.yaw = f64::NAN,
            1 => r.start.position.x = f64::INFINITY,
            2 => r.curvature_m_inv = f64::NAN,
            3 => r.distance_m = f64::MAX,
            4 => r.distance_m = 0.0,
            5 => r.half_width_m = f64::NAN,
            6 => r.half_width_m = 1.0,
            7 => r.lane_end = r.lane_start,
            8 => r.lane_end.x = 1e7,
            9 => r.start.position.y = 0.5,
            10 => r.start.position.x = 60.0,
            11 => r.start.yaw = std::f64::consts::PI,
            12 => r.curvature_m_inv = 0.4,
            _ => r.lane_start.y = f64::NEG_INFINITY,
        }
        assert!(plan(r, VehicleConfig::default()).is_err(), "case {case}");
    }
    for v in [
        VehicleConfig {
            wheelbase: 0.0,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            radius: f64::NAN,
            ..VehicleConfig::default()
        },
        VehicleConfig {
            max_steer: 0.001,
            ..VehicleConfig::default()
        },
    ] {
        assert!(plan(request(), v).is_err());
    }
}
