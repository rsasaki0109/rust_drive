use rustdriving_core::{Pose, Prediction, Vec2};
use rustdriving_prediction::cover_pose_uncertainty;

fn forecast() -> Prediction {
    Prediction {
        id: 9,
        positions: vec![Vec2::new(45., 0.), Vec2::new(-20., 30.)],
        radius: 1.,
        dt: 0.2,
        additional_radius_m: vec![0.2, 0.3],
    }
}
fn unchanged(a: &Prediction, b: &Prediction) {
    assert_eq!(a.positions, b.positions);
    assert_eq!(a.radius, b.radius);
    assert_eq!(a.additional_radius_m, b.additional_radius_m);
}
fn covariance(l: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| l[i][k] * l[j][k]).sum()))
}

#[test]
fn translation_uses_the_major_axis_and_preserves_nominal_geometry() {
    let mut p = forecast();
    let before = p.clone();
    cover_pose_uncertainty(
        &mut p,
        Pose::default(),
        [[0.04, 0., 0.], [0., 0.09, 0.], [0., 0., 0.]],
        3.,
    )
    .unwrap();
    assert_eq!(p.positions, before.positions);
    assert_eq!(p.radius, before.radius);
    for (a, b) in p.additional_radius_m.iter().zip(before.additional_radius_m) {
        assert!((a - b - 0.9).abs() < 1e-12);
    }
    let before = p.clone();
    cover_pose_uncertainty(&mut p, Pose::default(), [[0.; 3]; 3], 3.).unwrap();
    unchanged(&p, &before);
}

#[test]
fn heading_has_a_range_lever_arm_and_an_exact_rotation_remainder() {
    let mut p = forecast();
    cover_pose_uncertainty(
        &mut p,
        Pose::default(),
        [[0., 0., 0.], [0., 0., 0.], [0., 0., 0.0001]],
        3.,
    )
    .unwrap();
    assert!((p.additional_radius_m[0] - 0.2 - 1.35 - 0.02025).abs() < 1e-12);
    let radius = 20_f64.hypot(30.);
    assert!((p.additional_radius_m[1] - 0.3 - radius * (0.03 + 0.00045)).abs() < 1e-12);
}

#[test]
fn joint_translation_heading_correlation_is_retained() {
    let mut p = forecast();
    p.positions[0] = Vec2::new(0., 50.);
    cover_pose_uncertainty(
        &mut p,
        Pose::default(),
        [[1., 0., 0.02], [0., 0., 0.], [0.02, 0., 0.0004]],
        3.,
    )
    .unwrap();
    // At this offset the correlated linear displacement cancels exactly.
    assert!((p.additional_radius_m[0] - 0.2 - 0.09).abs() < 1e-12);
}

#[test]
fn exact_joint_pose_perturbations_fit_at_knots_and_between_them() {
    let l = [[0.15, 0., 0.], [0.04, 0.11, 0.], [0.002, -0.003, 0.006]];
    let mut p = forecast();
    p.additional_radius_m = vec![0.; 2];
    cover_pose_uncertainty(&mut p, Pose::default(), covariance(l), 3.).unwrap();
    for step in 0..11 {
        let f = step as f64 / 10.;
        let q = p.positions[0].scaled(1. - f).plus(p.positions[1].scaled(f));
        let bound = (1. - f) * p.additional_radius_m[0] + f * p.additional_radius_m[1];
        for i in 0..73 {
            for j in 0..37 {
                let theta = std::f64::consts::TAU * i as f64 / 72.;
                let phi = std::f64::consts::PI * j as f64 / 36.;
                let unit = [phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos()];
                let delta: [f64; 3] =
                    std::array::from_fn(|r| 3. * (0..3).map(|c| l[r][c] * unit[c]).sum::<f64>());
                let actual = q.rotated(delta[2]).plus(Vec2::new(delta[0], delta[1]));
                assert!(q.distance(actual) <= bound + 1e-12);
            }
        }
    }
}

#[test]
fn changing_world_origin_and_orientation_preserves_the_margin() {
    let matrix = covariance([[0.15, 0., 0.], [0.04, 0.11, 0.], [0.002, -0.003, 0.006]]);
    let angle = 1.2_f64;
    let t = [
        [angle.cos(), -angle.sin(), 0.],
        [angle.sin(), angle.cos(), 0.],
        [0., 0., 1.],
    ];
    let rotated = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            (0..3)
                .flat_map(|a| (0..3).map(move |b| t[i][a] * matrix[a][b] * t[j][b]))
                .sum()
        })
    });
    let mut a = forecast();
    let mut b = a.clone();
    let offset = Vec2::new(100., -20.);
    b.positions = b
        .positions
        .iter()
        .map(|q| q.rotated(angle).plus(offset))
        .collect();
    cover_pose_uncertainty(&mut a, Pose::default(), matrix, 3.).unwrap();
    cover_pose_uncertainty(
        &mut b,
        Pose {
            position: offset,
            yaw: angle,
        },
        rotated,
        3.,
    )
    .unwrap();
    for (x, y) in a.additional_radius_m.iter().zip(b.additional_radius_m) {
        assert!((x - y).abs() < 1e-11);
    }
}

#[test]
fn malformed_covariance_and_multiplier_leave_the_prediction_unchanged() {
    for p in [
        [[1., 0., 0.], [0., -1., 0.], [0., 0., 1.]],
        [[1., 2., 0.], [2., 1., 0.], [0., 0., 1.]],
        [[1., 0.9, 0.9], [0.9, 1., -0.9], [0.9, -0.9, 1.]],
        [[1., 0.2, 0.], [0.3, 1., 0.], [0., 0., 1.]],
        [[0., 0., 0.1], [0., 1., 0.], [0.1, 0., 1.]],
        [[f64::NAN, 0., 0.], [0., 1., 0.], [0., 0., 1.]],
    ] {
        let mut actual = forecast();
        let original = actual.clone();
        assert!(cover_pose_uncertainty(&mut actual, Pose::default(), p, 3.).is_err());
        unchanged(&actual, &original);
    }
    for sigma in [0., 0.9, 5.1, f64::INFINITY, f64::NAN] {
        let mut actual = forecast();
        let original = actual.clone();
        assert!(cover_pose_uncertainty(&mut actual, Pose::default(), [[0.; 3]; 3], sigma).is_err());
        unchanged(&actual, &original);
    }
}

#[test]
fn malformed_nominal_occupancy_and_overflow_cannot_partially_expand_a_forecast() {
    let mut p = forecast();
    p.additional_radius_m.pop();
    let original = p.clone();
    assert!(cover_pose_uncertainty(&mut p, Pose::default(), [[0.; 3]; 3], 3.).is_err());
    unchanged(&p, &original);
    let mut p = forecast();
    p.positions[1] = Vec2::new(f64::MAX, f64::MAX);
    let original = p.clone();
    assert!(
        cover_pose_uncertainty(
            &mut p,
            Pose::default(),
            [[0., 0., 0.], [0., 0., 0.], [0., 0., 1.]],
            3.
        )
        .is_err()
    );
    unchanged(&p, &original);
}
