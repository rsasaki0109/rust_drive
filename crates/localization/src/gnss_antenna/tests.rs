use super::*;

fn fix(stamp: f64, position: Vec2) -> Gnss {
    Gnss {
        stamp,
        position,
        variance: 0.09,
    }
}

#[test]
fn antenna_linearization_has_the_closed_form_position_heading_posterior() {
    let mut e = Ekf::with_pose_covariance(
        Pose::default(),
        [[0.25, 0., 0.], [0., 0.36, 0.], [0., 0., 0.04]],
    )
    .unwrap()
    .with_gnss_antenna_offset(Vec2::new(1., 0.))
    .unwrap();
    assert_eq!(
        e.correct(fix(0., Vec2::new(1.1, 0.2))),
        GnssDecision::Accepted
    );
    assert!((e.state().pose.position.x - 0.25 * 0.1 / 0.34).abs() < 1e-14);
    assert!((e.state().pose.position.y - 0.36 * 0.2 / 0.49).abs() < 1e-14);
    assert!((e.state().pose.yaw - 0.04 * 0.2 / 0.49).abs() < 1e-14);
    let p = e.pose_covariance();
    assert!((p[0][0] - 0.25 * 0.09 / 0.34).abs() < 1e-14);
    assert!((p[1][1] - (0.36 - 0.36 * 0.36 / 0.49)).abs() < 1e-14);
    assert!((p[1][2] + 0.36 * 0.04 / 0.49).abs() < 1e-14);
    assert!((p[2][2] - (0.04 - 0.04 * 0.04 / 0.49)).abs() < 1e-14);
    assert!(
        (e.diagnostics().last_nis.unwrap() - (0.1 * 0.1 / 0.34 + 0.2 * 0.2 / 0.49)).abs() < 1e-14
    );
}

#[test]
fn yaw_uncertainty_belongs_in_the_antenna_innovation_gate() {
    let mut e = Ekf::with_pose_covariance(
        Pose::default(),
        [[0.001, 0., 0.], [0., 0.001, 0.], [0., 0., 0.1]],
    )
    .unwrap()
    .with_gnss_antenna_offset(Vec2::new(2., 0.))
    .unwrap();
    // Without the yaw column this one-meter transverse innovation is rejected.
    let mut sample = fix(0., Vec2::new(2., 1.));
    sample.variance = 0.001;
    assert_eq!(e.correct(sample), GnssDecision::Accepted);
    assert!((e.diagnostics().last_nis.unwrap() - 1. / 0.402).abs() < 1e-12);
    assert!(e.state().pose.yaw > 0.4);
    let prior = e.clone();
    assert_eq!(e.correct(sample), GnssDecision::IgnoredTimestamp);
    assert_eq!(e.pose_covariance(), prior.pose_covariance());
    assert_eq!(e.diagnostics().accepted_fixes, 1);
    assert_eq!(
        e.correct(fix(1., Vec2::new(50., 50.))),
        GnssDecision::RejectedInnovation
    );
    assert_eq!(e.pose_covariance(), prior.pose_covariance());
    assert_eq!(e.state().pose.position, prior.state().pose.position);
    assert_eq!(e.last_gnss, 0.);
}

#[test]
fn body_mount_and_full_posterior_rotate_with_the_world_across_yaw_wrap() {
    let angle = 0.7_f64;
    let rotation = [
        [angle.cos(), -angle.sin(), 0.],
        [angle.sin(), angle.cos(), 0.],
        [0., 0., 1.],
    ];
    let covariance = [[0.25, 0.04, -0.01], [0.04, 0.36, 0.02], [-0.01, 0.02, 0.04]];
    let mut transformed: [[f64; 3]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            (0..3)
                .flat_map(|a| {
                    (0..3).map(move |b| rotation[i][a] * covariance[a][b] * rotation[j][b])
                })
                .sum()
        })
    });
    for i in 0..3 {
        for j in 0..i {
            let symmetric = transformed[i][j] / 2. + transformed[j][i] / 2.;
            transformed[i][j] = symmetric;
            transformed[j][i] = symmetric;
        }
    }
    let pose = Pose {
        position: Vec2::new(3., -2.),
        yaw: 3.13,
    };
    let mount = Vec2::new(-1.2, 0.35);
    let translation = Vec2::new(8., 4.);
    let z = pose.to_world(mount).plus(Vec2::new(0.05, -0.08));
    let mut a = Ekf::with_pose_covariance(pose, covariance)
        .unwrap()
        .with_gnss_antenna_offset(mount)
        .unwrap();
    let mut b = Ekf::with_pose_covariance(
        Pose {
            position: pose.position.rotated(angle).plus(translation),
            yaw: wrap_angle(pose.yaw + angle),
        },
        transformed,
    )
    .unwrap()
    .with_gnss_antenna_offset(mount)
    .unwrap();
    assert_eq!(a.correct(fix(0., z)), GnssDecision::Accepted);
    assert_eq!(
        b.correct(fix(0., z.rotated(angle).plus(translation))),
        GnssDecision::Accepted
    );
    assert!(
        a.state()
            .pose
            .position
            .rotated(angle)
            .plus(translation)
            .distance(b.state().pose.position)
            < 1e-12
    );
    assert!(wrap_angle(a.state().pose.yaw + angle - b.state().pose.yaw).abs() < 1e-12);
    for i in 0..3 {
        for j in 0..3 {
            let pa = a.pose_covariance();
            let expected: f64 = (0..3)
                .flat_map(|k| (0..3).map(move |l| rotation[i][k] * pa[k][l] * rotation[j][l]))
                .sum();
            assert!((expected - b.pose_covariance()[i][j]).abs() < 1e-12);
        }
    }
}

#[test]
fn explicit_zero_offset_preserves_both_legacy_correction_paths_bit_for_bit() {
    for shape in [None, Some([[1., 0.9], [0.9, 1.]])] {
        let mut a = Ekf::with_calibration(Pose::default(), None, shape).unwrap();
        let mut b = a.clone().with_gnss_antenna_offset(Vec2::default()).unwrap();
        for i in 0..30 {
            let stamp = f64::from(i) * 0.2;
            let motion = Odometry {
                stamp,
                speed: 2.,
                yaw_rate: 0.15,
            };
            a.predict_chassis(motion, 0.2, 0.7);
            b.predict_chassis(motion, 0.2, 0.7);
            let z = a.state().pose.position.plus(Vec2::new(0.02, -0.03));
            assert_eq!(a.correct(fix(stamp, z)), b.correct(fix(stamp, z)));
            assert_eq!(a.pose_covariance(), b.pose_covariance());
            assert_eq!(a.state().pose.position, b.state().pose.position);
            assert_eq!(a.state().pose.yaw.to_bits(), b.state().pose.yaw.to_bits());
            assert_eq!(a.diagnostics().last_nis, b.diagnostics().last_nis);
        }
    }
}

#[test]
fn invalid_mounts_and_numeric_failure_cannot_mutate_a_filter() {
    for offset in [
        Vec2::new(f64::NAN, 0.),
        Vec2::new(0., f64::INFINITY),
        Vec2::new(10., 0.01),
    ] {
        assert!(
            Ekf::new(Pose::default())
                .with_gnss_antenna_offset(offset)
                .is_err()
        );
    }
    let mut e = Ekf::with_calibration(Pose::default(), None, Some([[1.9, 0.], [0., 0.1]]))
        .unwrap()
        .with_gnss_antenna_offset(Vec2::new(1., 0.3))
        .unwrap();
    let before = e.clone();
    let mut bad = fix(0., Vec2::new(1., 0.3));
    bad.variance = f64::MAX;
    assert_eq!(e.correct(bad), GnssDecision::Invalid);
    assert_eq!(e.pose_covariance(), before.pose_covariance());
    assert_eq!(e.state().pose.position, before.state().pose.position);
    assert_eq!(e.diagnostics().last_observed_stamp, None);
}
