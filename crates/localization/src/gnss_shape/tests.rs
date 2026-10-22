use super::*;

fn prior() -> [[f64; 3]; 3] {
    [[0.4, 0.1, 0.02], [0.1, 0.3, -0.01], [0.02, -0.01, 0.04]]
}
fn assert_filter_close(a: &Ekf, b: &Ekf) {
    assert!(a.state().pose.position.distance(b.state().pose.position) < 1e-12);
    assert!(wrap_angle(a.state().pose.yaw - b.state().pose.yaw).abs() < 1e-12);
    for (x, y) in a
        .pose_covariance()
        .iter()
        .flatten()
        .zip(b.pose_covariance().iter().flatten())
    {
        assert!((x - y).abs() < 1e-12);
    }
}

#[test]
fn correlated_noise_matches_joint_conditional_gaussian_and_nis() {
    let p = prior();
    let mut f =
        Ekf::with_calibration(Pose::default(), Some(p), Some([[1.4, 0.5], [0.5, 0.6]])).unwrap();
    let residual = [0.1, -0.2];
    // Independent joint Gaussian conditioning P - P H' (H P H' + R)^-1 H P.
    let (a, b, c): (f64, f64, f64) = (0.68, 0.2, 0.42);
    let determinant = a * c - b * b;
    let inverse = [
        [c / determinant, -b / determinant],
        [-b / determinant, a / determinant],
    ];
    let gain: [[f64; 2]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..2).map(|k| p[i][k] * inverse[k][j]).sum())
    });
    assert_eq!(
        f.correct(Gnss {
            stamp: 0.,
            position: Vec2::new(residual[0], residual[1]),
            variance: 0.2
        }),
        GnssDecision::Accepted
    );
    let actual = [
        f.state().pose.position.x,
        f.state().pose.position.y,
        f.state().pose.yaw,
    ];
    for i in 0..3 {
        assert!((actual[i] - (0..2).map(|k| gain[i][k] * residual[k]).sum::<f64>()).abs() < 1e-14);
        for j in 0..3 {
            let expected = p[i][j] - (0..2).map(|k| gain[i][k] * p[k][j]).sum::<f64>();
            assert!((f.pose_covariance()[i][j] - expected).abs() < 1e-14);
        }
    }
    let nis: f64 = (0..2)
        .flat_map(|i| (0..2).map(move |j| residual[i] * inverse[i][j] * residual[j]))
        .sum();
    assert!((f.diagnostics().last_nis.unwrap() - nis).abs() < 1e-14);
    assert_eq!(f.diagnostics().accepted_fixes, 1);
}

#[test]
fn directional_gate_rejects_narrow_axis_error_accepted_by_isotropic_noise() {
    let p = [[0.01, 0., 0.], [0., 0.01, 0.], [0., 0., 0.01]];
    for (position, expected, nis) in [
        (Vec2::new(1., 0.), GnssDecision::Accepted, 1. / 0.048),
        (
            Vec2::new(0., 1.),
            GnssDecision::RejectedInnovation,
            1. / 0.012,
        ),
    ] {
        let mut f =
            Ekf::with_calibration(Pose::default(), Some(p), Some([[1.9, 0.], [0., 0.1]])).unwrap();
        let fix = Gnss {
            stamp: 0.,
            position,
            variance: 0.02,
        };
        assert_eq!(f.correct(fix), expected);
        assert!((f.diagnostics().last_nis.unwrap() - nis).abs() < 1e-11);
        let mut isotropic = Ekf::with_pose_covariance(Pose::default(), p).unwrap();
        assert_eq!(isotropic.correct(fix), GnssDecision::Accepted);
        let before = f.clone();
        assert_eq!(f.correct(fix), GnssDecision::IgnoredTimestamp);
        assert_filter_close(&f, &before);
        if expected == GnssDecision::RejectedInnovation {
            assert_eq!(f.last_gnss, f64::NEG_INFINITY);
            assert_eq!(f.diagnostics().rejected_fixes, 1);
        }
    }
}

#[test]
fn malformed_calibration_and_nonfinite_computed_noise_are_not_substituted() {
    for m in [
        [[1., 0.2], [0.3, 1.]],
        [[1., 1.], [1., 1.]],
        [[0., 0.], [0., 2.]],
        [[-1., 0.], [0., 3.]],
        [[2., 0.], [0., 2.]],
        [[f64::NAN, 0.], [0., 1.]],
        [[1., f64::INFINITY], [f64::INFINITY, 1.]],
    ] {
        assert!(Ekf::with_calibration(Pose::default(), None, Some(m)).is_err());
    }
    for (m, variance) in [
        ([[1.9, 0.], [0., 0.1]], f64::MAX),
        ([[1e-300, 0.], [0., 2.]], 1e-300),
    ] {
        let mut f = Ekf::with_calibration(Pose::default(), None, Some(m)).unwrap();
        let before = f.clone();
        assert_eq!(
            f.correct(Gnss {
                stamp: 0.,
                position: Vec2::new(0.1, 0.1),
                variance
            }),
            GnssDecision::Invalid
        );
        assert_filter_close(&f, &before);
        assert_eq!(f.diagnostics().last_observed_stamp, None);
        assert_eq!(f.diagnostics().accepted_fixes, 0);
    }
}

#[test]
fn delayed_gnss_and_maps_preserve_shape_in_both_arrival_orders() {
    for offset in [0., 1.2] {
        let original =
            Ekf::with_calibration(Pose::default(), Some(prior()), Some([[1., 0.9], [0.9, 1.]]))
                .unwrap();
        let motion = Odometry {
            stamp: 0.1,
            speed: 1.5,
            yaw_rate: 0.2,
        };
        let pose = Pose {
            position: Vec2::new(0.07, 0.002),
            yaw: 0.01,
        };
        let noise = [[0.04, 0., 0.], [0., 0.04, 0.], [0., 0., 0.003]];
        let fix = Gnss {
            stamp: 0.075,
            position: Vec2::new(0.12, 0.01),
            variance: 0.02,
        };
        let mut expected = original.clone();
        expected.predict_chassis(motion, 0.05, offset);
        assert!(expected.correct_map_pose(pose, noise));
        expected.predict_chassis(motion, 0.025, offset);
        assert!(expected.update(fix));
        expected.predict_chassis(motion, 0.025, offset);
        for gnss_first in [false, true] {
            let mut actual = original.clone();
            let mut history = latency::GnssHistory::with_delayed_maps();
            history.reset(0.);
            history
                .predict(&mut actual, motion, 0., 0.1, offset)
                .unwrap();
            if gnss_first {
                assert_eq!(
                    history.correct(&mut actual, fix, 0.1).unwrap(),
                    GnssDecision::Accepted
                );
            }
            assert!(
                history
                    .correct_map_pose_at(&mut actual, pose, noise, 0.05, 0.1)
                    .unwrap()
            );
            if !gnss_first {
                assert_eq!(
                    history.correct(&mut actual, fix, 0.1).unwrap(),
                    GnssDecision::Accepted
                );
            }
            assert_filter_close(&actual, &expected);
            assert_eq!(actual.diagnostics().accepted_fixes, 1);
            assert_eq!(actual.diagnostics().last_accepted_stamp, Some(0.075));
        }
    }
}

#[test]
fn rotating_enu_transports_the_measurement_shape_and_posterior() {
    let angle: f64 = 0.73;
    let (s, c) = angle.sin_cos();
    let t = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]];
    let shape = [[1., 0.9], [0.9, 1.]];
    let mut rotated = [[0.; 2]; 2];
    for i in 0..2 {
        for j in 0..=i {
            let v: f64 = (0..2)
                .flat_map(|k| (0..2).map(move |l| t[i][k] * shape[k][l] * t[j][l]))
                .sum();
            rotated[i][j] = v;
            rotated[j][i] = v;
        }
    }
    for offset in [0., 1.2] {
        let mut a = Ekf::with_calibration(Pose::default(), None, Some(shape)).unwrap();
        let mut b = Ekf::with_calibration(
            Pose {
                yaw: angle,
                ..Pose::default()
            },
            None,
            Some(rotated),
        )
        .unwrap();
        for i in 0..30 {
            let stamp = i as f64 * 0.05;
            let motion = Odometry {
                stamp,
                speed: 2.,
                yaw_rate: 0.1,
            };
            a.predict_chassis(motion, 0.05, offset);
            b.predict_chassis(motion, 0.05, offset);
            let position = Vec2::new(2. * stamp + 0.1, 0.1 * stamp + 0.02);
            assert_eq!(
                a.correct(Gnss {
                    stamp,
                    position,
                    variance: 0.02
                }),
                b.correct(Gnss {
                    stamp,
                    position: position.rotated(angle),
                    variance: 0.02
                })
            );
            assert!(
                a.state()
                    .pose
                    .position
                    .rotated(angle)
                    .distance(b.state().pose.position)
                    < 1e-12
            );
            assert!(wrap_angle(b.state().pose.yaw - a.state().pose.yaw - angle).abs() < 1e-12);
            let p = a.pose_covariance();
            for row in 0..3 {
                for col in 0..3 {
                    let expected: f64 = (0..3)
                        .flat_map(|k| (0..3).map(move |l| t[row][k] * p[k][l] * t[col][l]))
                        .sum();
                    assert!((expected - b.pose_covariance()[row][col]).abs() < 1e-12);
                }
            }
        }
    }
}
