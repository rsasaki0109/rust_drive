use super::*;

fn prior() -> [[f64; 3]; 3] {
    [
        [0.25, 0.05, 0.005],
        [0.05, 0.17, -0.005],
        [0.005, -0.005, 0.000725],
    ]
}

#[test]
fn correlated_prior_matches_closed_form_joint_gaussian_conditioning() {
    let p = prior();
    let mut f = Ekf::with_pose_covariance(Pose::default(), p).unwrap();
    assert_eq!(f.pose_covariance(), p);
    let r = 0.02;
    let residual = [0.12, -0.07];
    let a = p[0][0] + r;
    let b = p[0][1];
    let c = p[1][1] + r;
    let determinant = a * c - b * b;
    let inverse = [
        [c / determinant, -b / determinant],
        [-b / determinant, a / determinant],
    ];
    let gain: [[f64; 2]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..2).map(|k| p[i][k] * inverse[k][j]).sum())
    });
    assert!(f.update(Gnss {
        stamp: 0.,
        position: Vec2::new(residual[0], residual[1]),
        variance: r
    }));
    let actual = [
        f.state().pose.position.x,
        f.state().pose.position.y,
        f.state().pose.yaw,
    ];
    for i in 0..3 {
        assert!((actual[i] - (0..2).map(|k| gain[i][k] * residual[k]).sum::<f64>()).abs() < 1e-14);
        for j in 0..3 {
            let conditional = p[i][j] - (0..2).map(|k| gain[i][k] * p[k][j]).sum::<f64>();
            assert!((f.pose_covariance()[i][j] - conditional).abs() < 1e-14);
        }
    }
    assert_eq!(f.diagnostics().accepted_fixes, 1);
    assert_eq!(f.last_gnss, 0.);
}

#[test]
fn invalid_covariances_and_poses_are_rejected_without_fallback() {
    for (i, j) in [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)] {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut p = prior();
            p[i][j] = value;
            p[j][i] = value;
            assert!(Ekf::with_pose_covariance(Pose::default(), p).is_err());
        }
    }
    for i in 0..3 {
        let mut p = prior();
        p[i][i] = 0.;
        assert!(Ekf::with_pose_covariance(Pose::default(), p).is_err());
        p[i][i] = -1.;
        assert!(Ekf::with_pose_covariance(Pose::default(), p).is_err());
    }
    let mut p = prior();
    p[1][0] += 0.001;
    assert!(Ekf::with_pose_covariance(Pose::default(), p).is_err());
    p = prior();
    p[0][1] = 1.;
    p[1][0] = 1.;
    assert!(Ekf::with_pose_covariance(Pose::default(), p).is_err());
    assert!(
        Ekf::with_pose_covariance(
            Pose {
                yaw: f64::NAN,
                ..Pose::default()
            },
            prior()
        )
        .is_err()
    );
    assert!(
        Ekf::with_pose_covariance(
            Pose {
                position: Vec2::new(f64::INFINITY, 0.),
                ..Pose::default()
            },
            prior()
        )
        .is_err()
    );
}

#[test]
fn overflowing_position_trace_and_unbounded_circular_heading_are_rejected() {
    let mut p = prior();
    p[2][2] = std::f64::consts::PI.powi(2) + 1.;
    assert!(Ekf::with_pose_covariance(Pose::default(), p).is_err());
    assert!(
        Ekf::with_pose_covariance(
            Pose::default(),
            [[1e308, 0., 0.], [0., 1e308, 0.], [0., 0., 0.03]]
        )
        .is_err()
    );
}

fn rotate(p: [[f64; 3]; 3], angle: f64) -> [[f64; 3]; 3] {
    let t = [
        [angle.cos(), -angle.sin(), 0.],
        [angle.sin(), angle.cos(), 0.],
        [0., 0., 1.],
    ];
    let mut result = [[0.; 3]; 3];
    for i in 0..3 {
        for j in 0..=i {
            let value = (0..3)
                .flat_map(|k| (0..3).map(move |l| t[i][k] * p[k][l] * t[j][l]))
                .sum();
            result[i][j] = value;
            result[j][i] = value;
        }
    }
    result
}
#[test]
fn rotating_enu_transports_position_heading_cross_covariance() {
    let angle = 1.1;
    let mut a = Ekf::with_pose_covariance(Pose::default(), prior()).unwrap();
    let mut b = Ekf::with_pose_covariance(
        Pose {
            yaw: angle,
            ..Pose::default()
        },
        rotate(prior(), angle),
    )
    .unwrap();
    for i in 0..30 {
        let t = i as f64 * 0.05;
        let motion = Odometry {
            stamp: t,
            speed: 2.,
            yaw_rate: 0.1,
        };
        a.predict_chassis(motion, 0.05, 1.2);
        b.predict_chassis(motion, 0.05, 1.2);
        let position = Vec2::new(2. * t + 0.1, 0.1 * t + 0.02);
        assert_eq!(
            a.correct(Gnss {
                stamp: t,
                position,
                variance: 0.02
            }),
            b.correct(Gnss {
                stamp: t,
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
        let expected = rotate(a.pose_covariance(), angle);
        for (row, actual) in expected.iter().zip(b.pose_covariance()) {
            for (x, y) in row.iter().zip(actual) {
                assert!((x - y).abs() < 1e-12);
            }
        }
    }
}

#[test]
fn delayed_fractional_fix_replays_the_entire_supplied_prior() {
    for offset in [0., 1.2] {
        let pose = Pose {
            position: Vec2::new(0.3, -0.2),
            yaw: 0.07,
        };
        let mut delayed = Ekf::with_pose_covariance(pose, prior()).unwrap();
        let mut direct = delayed.clone();
        let mut h = latency::GnssHistory::default();
        h.reset(0.);
        let motion = Odometry {
            stamp: 0.1,
            speed: 1.5,
            yaw_rate: 0.2,
        };
        h.predict(&mut delayed, motion, 0., 0.1, offset).unwrap();
        let fix = Gnss {
            stamp: 0.075,
            position: Vec2::new(0.4, -0.18),
            variance: 0.02,
        };
        direct.predict_chassis(motion, 0.075, offset);
        assert!(direct.update(fix));
        direct.predict_chassis(motion, 0.025, offset);
        assert_eq!(
            h.correct(&mut delayed, fix, 0.1).unwrap(),
            GnssDecision::Accepted
        );
        assert!(
            delayed
                .state()
                .pose
                .position
                .distance(direct.state().pose.position)
                < 1e-14
        );
        for (a, b) in delayed
            .pose_covariance()
            .iter()
            .zip(direct.pose_covariance())
        {
            for (x, y) in a.iter().zip(b) {
                assert!((x - y).abs() < 1e-14);
            }
        }
    }
}
