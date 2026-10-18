use super::*;

fn fixture() -> (LaneResumeRequest, Route) {
    let route = Route::new(
        vec![
            Vec2::default(),
            Vec2::new(25., 0.),
            Vec2::new(55., 3.5),
            Vec2::new(80., 3.5),
            Vec2::new(160., 3.5),
        ],
        1.75,
    )
    .unwrap();
    let request = LaneResumeRequest {
        start: Pose {
            position: Vec2::new(42., 0.04),
            yaw: 0.01,
        },
        source_start: Vec2::default(),
        source_end: Vec2::new(80., 0.),
        source_width_m: 3.5,
        target_start: Vec2::new(0., 3.5),
        target_end: Vec2::new(80., 3.5),
        target_width_m: 3.5,
        original_exit_s_m: route.project(Vec2::new(55., 3.5)).0,
        distance_m: 30.,
    };
    (request, route)
}

#[test]
fn stopped_connector_matches_independent_boundary_polynomial_and_retains_the_original_tail() {
    for angle in [
        0.0_f64,
        0.37,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
    ] {
        for sign in [-1., 1.] {
            let transform = |p: Vec2| {
                Vec2::new(
                    10. + p.x * angle.cos() - sign * p.y * angle.sin(),
                    -20. + p.x * angle.sin() + sign * p.y * angle.cos(),
                )
            };
            let (mut r, original) = fixture();
            let original = Route::new(
                original.points.iter().map(|p| transform(*p)).collect(),
                1.75,
            )
            .unwrap();
            r.original_exit_s_m = original.project(transform(Vec2::new(55., 3.5))).0;
            r.start.position = transform(r.start.position);
            r.start.yaw = angle + 0.01 * sign;
            r.source_start = transform(r.source_start);
            r.source_end = transform(r.source_end);
            r.target_start = transform(r.target_start);
            r.target_end = transform(r.target_end);
            let p = plan(r, &original, VehicleConfig::default()).unwrap();
            let y0 = 0.04 - 3.5;
            let a1 = 0.01_f64.tan() * 30.;
            let (a3, a4, a5) = (-10. * y0 - 6. * a1, 15. * y0 + 8. * a1, -6. * y0 - 3. * a1);
            for i in 0..=120 {
                let u = i as f64 / 120.;
                let y = 3.5 + y0 + a1 * u + a3 * u.powi(3) + a4 * u.powi(4) + a5 * u.powi(5);
                assert!(p.route.points[i].distance(transform(Vec2::new(42. + 30. * u, y))) < 1e-10);
            }
            for i in 0..=10_000 {
                let u = i as f64 / 10_000.;
                let dy = (a1 + 3. * a3 * u * u + 4. * a4 * u.powi(3) + 5. * a5 * u.powi(4)) / 30.;
                let ddy = (6. * a3 * u + 12. * a4 * u * u + 20. * a5 * u.powi(3)) / 900.;
                assert!(ddy.abs() / (1. + dy * dy).powf(1.5) <= p.curvature_bound_m_inv + 1e-12);
            }
            assert_eq!(&p.route.points[121..], &original.points[3..]);
            assert_eq!(p.route.points[0], r.start.position);
            assert!((p.join_s_m - r.original_exit_s_m - 17.).abs() < 1e-10);
            assert!(p.connector_length_m > 30.);
        }
    }
}

#[test]
fn resume_rejects_wrong_source_short_room_and_a_forged_or_missing_target_tail() {
    for case in 0..14 {
        let (mut r, mut original) = fixture();
        match case {
            0 => r.start.position.y = 0.6,
            1 => r.start.position.x = 65.,
            2 => r.source_end.y = 1.,
            3 => r.source_width_m = 4.,
            4 => r.distance_m = f64::NAN,
            5 => r.start.yaw = f64::INFINITY,
            6 => r.original_exit_s_m = f64::NAN,
            7 => r.original_exit_s_m = original.length(),
            8 => original.lengths[2] += 1.,
            9 => {
                original = Route::new(
                    vec![
                        Vec2::default(),
                        Vec2::new(25., 0.),
                        Vec2::new(55., 3.5),
                        Vec2::new(80., 4.),
                        Vec2::new(160., 3.5),
                    ],
                    1.75,
                )
                .unwrap()
            }
            10 => r.target_start.x = f64::NAN,
            11 => r.target_width_m = 0.,
            12 => r.start.position.x = 0.,
            _ => r.source_start.x = 1e7,
        }
        assert!(
            plan(r, &original, VehicleConfig::default()).is_err(),
            "case {case}"
        );
    }
}
