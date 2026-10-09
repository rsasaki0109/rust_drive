//! Three-state EKF: wheel speed / gyro prediction and scalar GNSS updates.
pub mod registration;
pub mod registration3d;
use rustdrive_core::{
    EgoState, Gnss, GnssDecision, LocalizationDiagnostics, Odometry, Pose, Vec2, wrap_angle,
};
/// Conservative observation floors for an externally registered fixed-map pose.
pub const MAP_POSITION_VARIANCE_FLOOR_M2: f64 = 0.01;
pub const MAP_YAW_VARIANCE_FLOOR_RAD2: f64 = 1e-4;
pub struct Ekf {
    state: EgoState,
    covariance: [[f64; 3]; 3],
    pub last_gnss: f64,
    diagnostics: LocalizationDiagnostics,
}
impl Ekf {
    /// Initial heading comes from the configured spawn calibration, not live simulator truth.
    pub fn new(pose: Pose) -> Self {
        Self {
            state: EgoState { pose, speed: 0.0 },
            covariance: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.03]],
            last_gnss: f64::NEG_INFINITY,
            diagnostics: LocalizationDiagnostics::default(),
        }
    }
    pub fn predict(&mut self, odom: Odometry, dt: f64) {
        if !dt.is_finite() || dt <= 0.0 || !odom.speed.is_finite() || !odom.yaw_rate.is_finite() {
            return;
        }
        let yaw = self.state.pose.yaw;
        let v = odom.speed.max(0.0);
        self.state.pose.position = self
            .state
            .pose
            .position
            .plus(Vec2::new(yaw.cos(), yaw.sin()).scaled(v * dt));
        self.state.pose.yaw = wrap_angle(yaw + odom.yaw_rate * dt);
        self.state.speed = v;
        let f = [
            [1.0, 0.0, -v * dt * yaw.sin()],
            [0.0, 1.0, v * dt * yaw.cos()],
            [0.0, 0.0, 1.0],
        ];
        let mut next = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    for l in 0..3 {
                        next[i][j] += f[i][k] * self.covariance[k][l] * f[j][l];
                    }
                }
            }
        }
        for (i, row) in next.iter_mut().enumerate() {
            row[i] += [0.015, 0.015, 0.0005][i] * dt;
        }
        self.covariance = next;
    }
    /// Low-speed no-slip chassis prediction. Wheel speed is body longitudinal
    /// speed; lateral chassis speed is rear offset times yaw rate. Pose yaw
    /// remains body heading; state speed is course speed. No tire-slip estimate.
    /// Zero offset preserves the original predictor exactly.
    pub fn predict_chassis(&mut self, odom: Odometry, dt: f64, rear_axle_offset_m: f64) {
        if !rear_axle_offset_m.is_finite() || rear_axle_offset_m < 0.0 {
            return;
        }
        if rear_axle_offset_m == 0.0 {
            self.predict(odom, dt);
            return;
        }
        if !dt.is_finite() || dt <= 0.0 || !odom.speed.is_finite() || !odom.yaw_rate.is_finite() {
            return;
        }
        let yaw = self.state.pose.yaw;
        let v = odom.speed.max(0.0);
        let lateral = rear_axle_offset_m * odom.yaw_rate;
        let midpoint_yaw = yaw + odom.yaw_rate * dt / 2.0;
        let displacement = Vec2::new(
            v * midpoint_yaw.cos() - lateral * midpoint_yaw.sin(),
            v * midpoint_yaw.sin() + lateral * midpoint_yaw.cos(),
        )
        .scaled(dt);
        let position = self.state.pose.position.plus(displacement);
        let next_yaw = wrap_angle(yaw + odom.yaw_rate * dt);
        let speed = v.hypot(lateral);
        if !position.finite() || !next_yaw.is_finite() || !speed.is_finite() {
            return;
        }
        let f = [
            [1.0, 0.0, -displacement.y],
            [0.0, 1.0, displacement.x],
            [0.0, 0.0, 1.0],
        ];
        let mut next = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    for l in 0..3 {
                        next[i][j] += f[i][k] * self.covariance[k][l] * f[j][l];
                    }
                }
            }
        }
        for (i, row) in next.iter_mut().enumerate() {
            row[i] += [0.015, 0.015, 0.0005][i] * dt;
        }
        if next.iter().flatten().any(|value| !value.is_finite()) {
            return;
        }
        self.state = EgoState {
            pose: Pose {
                position,
                yaw: next_yaw,
            },
            speed,
        };
        self.covariance = next;
    }
    /// Joint fixed-map x/y/body-yaw correction with conservative observation
    /// floors, a wrapped-yaw innovation gate and Joseph covariance update.
    /// Map observations never refresh GNSS timestamps or GNSS diagnostics.
    pub fn correct_map_pose(&mut self, pose: Pose, covariance: [[f64; 3]; 3]) -> bool {
        if !pose.position.finite()
            || !pose.yaw.is_finite()
            || covariance.iter().flatten().any(|value| !value.is_finite())
        {
            return false;
        }
        let mut noise = covariance;
        for i in 0..3 {
            for j in 0..3 {
                let tolerance = 1e-10 * covariance[i][j].abs().max(covariance[j][i].abs()).max(1.0);
                if (covariance[i][j] - covariance[j][i]).abs() > tolerance {
                    return false;
                }
                noise[i][j] = covariance[i][j] / 2.0 + covariance[j][i] / 2.0;
            }
        }
        if cholesky3(noise).is_none() {
            return false;
        }
        noise[0][0] = noise[0][0].max(MAP_POSITION_VARIANCE_FLOOR_M2);
        noise[1][1] = noise[1][1].max(MAP_POSITION_VARIANCE_FLOOR_M2);
        noise[2][2] = noise[2][2].max(MAP_YAW_VARIANCE_FLOOR_RAD2);
        let innovation = [
            pose.position.x - self.state.pose.position.x,
            pose.position.y - self.state.pose.position.y,
            wrap_angle(pose.yaw - self.state.pose.yaw),
        ];
        if innovation.iter().any(|value| !value.is_finite()) {
            return false;
        }
        let mut sum = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                sum[i][j] = self.covariance[i][j] + noise[i][j];
            }
        }
        let Some(lower) = cholesky3(sum) else {
            return false;
        };
        let mut whitened = [0.0; 3];
        for i in 0..3 {
            let residual = innovation[i] - (0..i).map(|j| lower[i][j] * whitened[j]).sum::<f64>();
            whitened[i] = residual / lower[i][i];
        }
        let nis = whitened.iter().map(|value| value * value).sum::<f64>();
        if !nis.is_finite() || nis > 36.0 {
            return false;
        }
        let gain = self.covariance.map(|row| solve_cholesky3(lower, row));
        if gain.iter().flatten().any(|value| !value.is_finite()) {
            return false;
        }
        let correction: [f64; 3] =
            std::array::from_fn(|i| (0..3).map(|j| gain[i][j] * innovation[j]).sum());
        let next_pose = Pose {
            position: self
                .state
                .pose
                .position
                .plus(Vec2::new(correction[0], correction[1])),
            yaw: wrap_angle(self.state.pose.yaw + correction[2]),
        };
        let a: [[f64; 3]; 3] =
            std::array::from_fn(|i| std::array::from_fn(|j| f64::from(i == j) - gain[i][j]));
        let mut next = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for (k, noise_row) in noise.iter().enumerate() {
                    for (l, noise_value) in noise_row.iter().enumerate() {
                        next[i][j] += a[i][k] * self.covariance[k][l] * a[j][l]
                            + gain[i][k] * noise_value * gain[j][l];
                    }
                }
            }
        }
        // Remove roundoff asymmetry before validating and committing the update.
        for i in 0..3 {
            for j in 0..i {
                let symmetric = next[i][j] / 2.0 + next[j][i] / 2.0;
                next[i][j] = symmetric;
                next[j][i] = symmetric;
            }
        }
        if !next_pose.position.finite() || !next_pose.yaw.is_finite() || cholesky3(next).is_none() {
            return false;
        }
        self.state.pose = next_pose;
        self.covariance = next;
        true
    }
    /// Accept only strictly new fixes within the joint 2D innovation gate.
    pub fn update(&mut self, fix: Gnss) -> bool {
        self.correct(fix) == GnssDecision::Accepted
    }
    pub fn correct(&mut self, fix: Gnss) -> GnssDecision {
        if !fix.position.finite()
            || !fix.variance.is_finite()
            || fix.variance <= 0.0
            || !fix.stamp.is_finite()
            || fix.stamp < 0.0
        {
            return GnssDecision::Invalid;
        }
        if self
            .diagnostics
            .last_observed_stamp
            .is_some_and(|stamp| fix.stamp <= stamp)
        {
            return GnssDecision::IgnoredTimestamp;
        }
        let residual = fix.position.minus(self.state.pose.position);
        // LDL^T whitening of S = H P H^T + R, including x/y correlation.
        // Avoid an explicit determinant/inverse, which can overflow for large R.
        let xx = self.covariance[0][0] + fix.variance;
        let xy = 0.5 * (self.covariance[0][1] + self.covariance[1][0]);
        let yy = self.covariance[1][1] + fix.variance;
        let ratio = xy / xx;
        let conditional = yy - xy * ratio;
        if !xx.is_finite() || xx <= 0.0 || !conditional.is_finite() || conditional <= 0.0 {
            return GnssDecision::Invalid;
        }
        let nis = (residual.x / xx.sqrt()).powi(2)
            + ((residual.y - ratio * residual.x) / conditional.sqrt()).powi(2);
        self.diagnostics.last_observed_stamp = Some(fix.stamp);
        self.diagnostics.last_nis = nis.is_finite().then_some(nis);
        if !nis.is_finite() || nis > 36.0 {
            self.diagnostics.last_decision = Some(GnssDecision::RejectedInnovation);
            self.diagnostics.rejected_fixes = self.diagnostics.rejected_fixes.saturating_add(1);
            return GnssDecision::RejectedInnovation;
        }
        // Independent x/y measurement noise permits sequential scalar correction.
        // Joseph form preserves covariance symmetry/positive semidefiniteness.
        for axis in 0..2 {
            let measurement = if axis == 0 {
                fix.position.x
            } else {
                fix.position.y
            };
            let estimate = if axis == 0 {
                self.state.pose.position.x
            } else {
                self.state.pose.position.y
            };
            let mut k = [0.0; 3];
            let denom = self.covariance[axis][axis] + fix.variance;
            for (i, value) in k.iter_mut().enumerate() {
                *value = self.covariance[i][axis] / denom;
            }
            let innovation = measurement - estimate;
            self.state.pose.position.x += k[0] * innovation;
            self.state.pose.position.y += k[1] * innovation;
            self.state.pose.yaw = wrap_angle(self.state.pose.yaw + k[2] * innovation);
            let old = self.covariance;
            let mut a = [[0.0; 3]; 3];
            for (i, row) in a.iter_mut().enumerate() {
                for (j, value) in row.iter_mut().enumerate() {
                    *value = f64::from(i == j) - if j == axis { k[i] } else { 0.0 };
                }
            }
            let mut next = [[0.0; 3]; 3];
            for (i, row) in next.iter_mut().enumerate() {
                for (j, value) in row.iter_mut().enumerate() {
                    *value = (k[i] * fix.variance) * k[j];
                    for (m, old_row) in old.iter().enumerate() {
                        for (n, old_value) in old_row.iter().enumerate() {
                            *value += a[i][m] * old_value * a[j][n];
                        }
                    }
                }
            }
            self.covariance = next;
        }
        self.last_gnss = fix.stamp;
        self.diagnostics.last_accepted_stamp = Some(fix.stamp);
        self.diagnostics.last_decision = Some(GnssDecision::Accepted);
        self.diagnostics.accepted_fixes = self.diagnostics.accepted_fixes.saturating_add(1);
        GnssDecision::Accepted
    }
    pub fn diagnostics(&self) -> LocalizationDiagnostics {
        self.diagnostics
    }
    pub fn state(&self) -> EgoState {
        self.state
    }
    pub fn position_variance(&self) -> f64 {
        self.covariance[0][0] + self.covariance[1][1]
    }
}
fn cholesky3(matrix: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    if matrix.iter().flatten().any(|value| !value.is_finite()) {
        return None;
    }
    let mut lower = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..=i {
            let residual = matrix[i][j] - (0..j).map(|k| lower[i][k] * lower[j][k]).sum::<f64>();
            if i == j {
                if residual <= 0.0 || !residual.is_finite() {
                    return None;
                }
                lower[i][j] = residual.sqrt();
            } else {
                lower[i][j] = residual / lower[j][j];
            }
        }
    }
    lower
        .iter()
        .flatten()
        .all(|value| value.is_finite())
        .then_some(lower)
}
fn solve_cholesky3(lower: [[f64; 3]; 3], rhs: [f64; 3]) -> [f64; 3] {
    let mut forward = [0.0; 3];
    for i in 0..3 {
        forward[i] = (rhs[i] - (0..i).map(|j| lower[i][j] * forward[j]).sum::<f64>()) / lower[i][i];
    }
    let mut result = [0.0; 3];
    for i in (0..3).rev() {
        result[i] =
            (forward[i] - ((i + 1)..3).map(|j| lower[j][i] * result[j]).sum::<f64>()) / lower[i][i];
    }
    result
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn map_pose_diagonal_joint_correction_matches_analytic_posterior() {
        let mut e = Ekf::new(Pose::default());
        let noise = [[0.04, 0.0, 0.0], [0.0, 0.09, 0.0], [0.0, 0.0, 0.002]];
        assert!(e.correct_map_pose(
            Pose {
                position: Vec2::new(0.2, -0.1),
                yaw: 0.1
            },
            noise
        ));
        let prior = [1.0, 1.0, 0.03];
        let residual = [0.2, -0.1, 0.1];
        let actual = [
            e.state().pose.position.x,
            e.state().pose.position.y,
            e.state().pose.yaw,
        ];
        for i in 0..3 {
            let gain = prior[i] / (prior[i] + noise[i][i]);
            assert!((actual[i] - gain * residual[i]).abs() < 1e-12);
            assert!(
                (e.covariance[i][i] - prior[i] * noise[i][i] / (prior[i] + noise[i][i])).abs()
                    < 1e-12
            );
        }
    }
    #[test]
    fn map_pose_wraps_yaw_and_preserves_gnss_freshness_and_diagnostics() {
        let mut e = Ekf::new(Pose {
            position: Vec2::default(),
            yaw: 3.13,
        });
        assert!(e.update(Gnss {
            stamp: 1.0,
            position: Vec2::default(),
            variance: 0.02
        }));
        let before = e.diagnostics();
        assert!(e.correct_map_pose(
            Pose {
                position: Vec2::new(0.02, -0.01),
                yaw: -3.13
            },
            [[0.02, 0.0, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0, 0.002]]
        ));
        assert!(wrap_angle(e.state().pose.yaw - 3.13).abs() < 0.03);
        assert_eq!(e.last_gnss, 1.0);
        let after = e.diagnostics();
        assert_eq!(before.last_observed_stamp, after.last_observed_stamp);
        assert_eq!(before.last_accepted_stamp, after.last_accepted_stamp);
        assert_eq!(before.last_decision, after.last_decision);
        assert_eq!(before.last_nis, after.last_nis);
        assert_eq!(before.accepted_fixes, after.accepted_fixes);
        assert_eq!(before.rejected_fixes, after.rejected_fixes);
    }
    #[test]
    fn map_pose_covariance_floors_prevent_false_precision() {
        let mut e = Ekf::new(Pose::default());
        assert!(e.correct_map_pose(
            Pose::default(),
            [[1e-12, 0.0, 0.0], [0.0, 1e-12, 0.0], [0.0, 0.0, 1e-12]]
        ));
        assert!(
            (e.covariance[0][0]
                - MAP_POSITION_VARIANCE_FLOOR_M2 / (1.0 + MAP_POSITION_VARIANCE_FLOOR_M2))
                .abs()
                < 1e-12
        );
        assert!(
            (e.covariance[2][2]
                - 0.03 * MAP_YAW_VARIANCE_FLOOR_RAD2 / (0.03 + MAP_YAW_VARIANCE_FLOOR_RAD2))
                .abs()
                < 1e-12
        );
    }
    #[test]
    fn invalid_or_gated_joint_map_fixes_do_not_mutate_estimate() {
        let mut e = Ekf::new(Pose::default());
        let before = e.covariance;
        for noise in [
            [[0.02, 0.03, 0.0], [0.03, 0.02, 0.0], [0.0, 0.0, 0.001]],
            [[0.02, 0.001, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0, 0.001]],
            [[f64::NAN, 0.0, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0, 0.001]],
            [[0.0; 3]; 3],
        ] {
            assert!(!e.correct_map_pose(Pose::default(), noise));
        }
        assert!(!e.correct_map_pose(
            Pose {
                position: Vec2::new(100.0, 0.0),
                yaw: 0.0
            },
            [[0.02, 0.0, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0, 0.001]]
        ));
        assert_eq!(e.covariance, before);
        assert_eq!(e.state().pose.position, Vec2::default());
        assert_eq!(e.state().pose.yaw, 0.0);
        assert_eq!(e.last_gnss, f64::NEG_INFINITY);
    }
    #[test]
    fn correlated_map_joseph_updates_stay_positive_and_symmetric() {
        let mut e = Ekf::new(Pose::default());
        let noise = [
            [0.05, 0.01, 0.002],
            [0.01, 0.06, -0.001],
            [0.002, -0.001, 0.002],
        ];
        for i in 0..100 {
            e.predict(
                Odometry {
                    stamp: i as f64 * 0.05,
                    speed: 2.0,
                    yaw_rate: 0.1,
                },
                0.05,
            );
            assert!(e.correct_map_pose(e.state().pose, noise));
            assert!(cholesky3(e.covariance).is_some());
            for a in 0..3 {
                for b in 0..3 {
                    assert!((e.covariance[a][b] - e.covariance[b][a]).abs() < 1e-12);
                }
            }
        }
        assert_eq!(e.last_gnss, f64::NEG_INFINITY);
        assert_eq!(e.diagnostics().accepted_fixes, 0);
    }
    #[test]
    fn chassis_turns_match_analytic_displacement_speed_and_covariance() {
        for direction in [-1.0_f64, 1.0] {
            let mut e = Ekf::new(Pose::default());
            let dt = 0.05;
            e.predict_chassis(
                Odometry {
                    stamp: 0.0,
                    speed: 4.0,
                    yaw_rate: direction * 0.2,
                },
                dt,
                1.3,
            );
            let midpoint = direction * 0.005;
            let lateral = direction * 0.26;
            let dx = (4.0 * midpoint.cos() - lateral * midpoint.sin()) * dt;
            let dy = (4.0 * midpoint.sin() + lateral * midpoint.cos()) * dt;
            assert!((e.state().pose.position.x - dx).abs() < 1e-12);
            assert!((e.state().pose.position.y - dy).abs() < 1e-12);
            assert!((e.state().pose.yaw - direction * 0.01).abs() < 1e-12);
            assert!((e.state().speed - 4.0_f64.hypot(0.26)).abs() < 1e-12);
            let expected = [
                [
                    1.0 + 0.03 * dy * dy + 0.015 * dt,
                    -0.03 * dx * dy,
                    -0.03 * dy,
                ],
                [
                    -0.03 * dx * dy,
                    1.0 + 0.03 * dx * dx + 0.015 * dt,
                    0.03 * dx,
                ],
                [-0.03 * dy, 0.03 * dx, 0.03 + 0.0005 * dt],
            ];
            for (row, expected_row) in e.covariance.iter().zip(expected) {
                for (value, expected_value) in row.iter().zip(expected_row) {
                    assert!((value - expected_value).abs() < 1e-12);
                }
            }
        }
    }
    #[test]
    fn zero_chassis_offset_preserves_existing_predictor_and_corrector_exactly() {
        let initial = Pose {
            position: Vec2::new(1.0, 2.0),
            yaw: 0.4,
        };
        let mut baseline = Ekf::new(initial);
        let mut chassis = Ekf::new(initial);
        for i in 0..100 {
            let odometry = Odometry {
                stamp: i as f64 * 0.05,
                speed: 3.0,
                yaw_rate: (i as f64 * 0.1).sin() * 0.2,
            };
            baseline.predict(odometry, 0.05);
            chassis.predict_chassis(odometry, 0.05, 0.0);
            let fix = Gnss {
                stamp: odometry.stamp,
                position: baseline.state().pose.position.plus(Vec2::new(0.01, -0.01)),
                variance: 0.02,
            };
            assert_eq!(baseline.correct(fix), chassis.correct(fix));
            assert_eq!(
                baseline.state().pose.position,
                chassis.state().pose.position
            );
            assert_eq!(
                baseline.state().pose.yaw.to_bits(),
                chassis.state().pose.yaw.to_bits()
            );
            assert_eq!(
                baseline.state().speed.to_bits(),
                chassis.state().speed.to_bits()
            );
            assert_eq!(baseline.covariance, chassis.covariance);
            assert_eq!(baseline.last_gnss, chassis.last_gnss);
        }
    }
    #[test]
    fn invalid_chassis_offset_and_numeric_overflow_cannot_mutate_estimate() {
        let mut e = Ekf::new(Pose::default());
        let before = e.covariance;
        for rear in [-1.0, f64::NAN, f64::INFINITY] {
            e.predict_chassis(
                Odometry {
                    stamp: 0.0,
                    speed: 4.0,
                    yaw_rate: 0.2,
                },
                0.05,
                rear,
            );
        }
        e.predict_chassis(
            Odometry {
                stamp: 0.0,
                speed: f64::MAX,
                yaw_rate: f64::MAX,
            },
            10.0,
            1.3,
        );
        assert_eq!(e.state().pose.position, Vec2::default());
        assert_eq!(e.state().pose.yaw, 0.0);
        assert_eq!(e.state().speed, 0.0);
        assert_eq!(e.covariance, before);
        assert_eq!(e.last_gnss, f64::NEG_INFINITY);
    }
    #[test]
    fn chassis_covariance_stays_symmetric_and_positive_under_repeated_corrections() {
        let mut e = Ekf::new(Pose::default());
        for i in 0..200 {
            e.predict_chassis(
                Odometry {
                    stamp: i as f64 * 0.05,
                    speed: 3.0,
                    yaw_rate: 0.4 * (i as f64 * 0.05).sin(),
                },
                0.05,
                1.3,
            );
            assert!(e.update(Gnss {
                stamp: i as f64 * 0.05,
                position: e.state().pose.position,
                variance: 0.02
            }));
            for a in 0..3 {
                assert!(e.covariance[a][a] > 0.0);
                for b in 0..3 {
                    assert!((e.covariance[a][b] - e.covariance[b][a]).abs() < 1e-9);
                }
            }
        }
    }
    #[test]
    fn correlated_innovation_is_rejected_without_mutation() {
        let mut e = Ekf::new(Pose::default());
        // Positive-definite covariance: this direction has little uncertainty.
        e.covariance = [[1.0, 0.99, 0.0], [0.99, 1.0, 0.0], [0.0, 0.0, 0.03]];
        let before = e.covariance;
        assert!(!e.update(Gnss {
            stamp: 1.0,
            position: Vec2::new(1.0, -1.0),
            variance: 0.02
        }));
        assert_eq!(e.state().pose.position, Vec2::default());
        assert_eq!(e.covariance, before);
        assert_eq!(e.last_gnss, f64::NEG_INFINITY);
    }
    #[test]
    fn correlated_consistent_innovation_is_accepted() {
        let mut e = Ekf::new(Pose::default());
        e.covariance = [[1.0, 0.99, 0.0], [0.99, 1.0, 0.0], [0.0, 0.0, 0.03]];
        assert!(e.update(Gnss {
            stamp: 1.0,
            position: Vec2::new(1.0, 1.0),
            variance: 0.02
        }));
        assert!((e.diagnostics().last_nis.unwrap() - 2.0 / 2.01).abs() < 1e-12);
    }
    #[test]
    fn sequential_joseph_correction_matches_an_independent_joint_solution() {
        let mut e = Ekf::new(Pose::default());
        let prior = [[1.0, 0.5, 0.2], [0.5, 2.0, 0.1], [0.2, 0.1, 0.3]];
        e.covariance = prior;
        let r = 0.2;
        let det = (prior[0][0] + r) * (prior[1][1] + r) - prior[0][1].powi(2);
        let inverse = [
            [(prior[1][1] + r) / det, -prior[0][1] / det],
            [-prior[0][1] / det, (prior[0][0] + r) / det],
        ];
        let gain: [[f64; 2]; 3] = std::array::from_fn(|i| {
            std::array::from_fn(|j| prior[i][0] * inverse[0][j] + prior[i][1] * inverse[1][j])
        });
        assert!(e.update(Gnss {
            stamp: 0.0,
            position: Vec2::new(0.2, -0.1),
            variance: r
        }));
        let state = [
            e.state().pose.position.x,
            e.state().pose.position.y,
            e.state().pose.yaw,
        ];
        for i in 0..3 {
            assert!((state[i] - (gain[i][0] * 0.2 - gain[i][1] * 0.1)).abs() < 1e-12);
            for j in 0..3 {
                let expected = prior[i][j] - gain[i][0] * prior[0][j] - gain[i][1] * prior[1][j];
                assert!((e.covariance[i][j] - expected).abs() < 1e-12);
            }
        }
    }
    #[test]
    fn rejected_fixes_do_not_refresh_acceptance_or_allow_timestamp_rollback() {
        let mut e = Ekf::new(Pose::default());
        let fix = Gnss {
            stamp: 0.0,
            position: Vec2::default(),
            variance: 0.02,
        };
        assert!(e.update(fix));
        assert_eq!(
            e.correct(Gnss {
                stamp: 1.0,
                position: Vec2::new(30.0, -25.0),
                ..fix
            }),
            GnssDecision::RejectedInnovation
        );
        assert_eq!(
            e.correct(Gnss { stamp: 0.5, ..fix }),
            GnssDecision::IgnoredTimestamp
        );
        assert_eq!(
            e.correct(Gnss { stamp: 1.0, ..fix }),
            GnssDecision::IgnoredTimestamp
        );
        assert_eq!(e.last_gnss, 0.0);
        assert_eq!(e.diagnostics().last_observed_stamp, Some(1.0));
        assert_eq!(e.diagnostics().rejected_fixes, 1);
        assert!(e.update(Gnss { stamp: 1.2, ..fix }));
        assert_eq!(e.diagnostics().last_accepted_stamp, Some(1.2));
        assert_eq!(e.diagnostics().accepted_fixes, 2);
    }
    #[test]
    fn extreme_numeric_fixes_are_finite_or_rejected_without_state_mutation() {
        let mut e = Ekf::new(Pose::default());
        assert_eq!(
            e.correct(Gnss {
                stamp: 0.0,
                position: Vec2::new(f64::MAX, f64::MAX),
                variance: 0.02
            }),
            GnssDecision::RejectedInnovation
        );
        assert_eq!(e.state().pose.position, Vec2::default());
        assert!(e.diagnostics().last_nis.is_none());
        assert!(e.update(Gnss {
            stamp: 0.2,
            position: Vec2::new(1e150, -1e150),
            variance: 1e308
        }));
        assert!(e.state().pose.position.finite());
        assert!(e.position_variance().is_finite());
        assert_eq!(
            e.correct(Gnss {
                stamp: 0.4,
                position: Vec2::default(),
                variance: f64::NAN
            }),
            GnssDecision::Invalid
        );
        assert_eq!(e.diagnostics().last_observed_stamp, Some(0.2));
    }
    #[test]
    fn corrects_drift_and_rejects_outlier() {
        let mut e = Ekf::new(Pose::default());
        for i in 0..100 {
            e.predict(
                Odometry {
                    stamp: i as f64 * 0.1,
                    speed: 1.1,
                    yaw_rate: 0.0,
                },
                0.1,
            );
            assert!(e.update(Gnss {
                stamp: i as f64 * 0.1,
                position: Vec2::new((i + 1) as f64 * 0.1, 0.0),
                variance: 0.04
            }));
        }
        assert!((e.state().pose.position.x - 10.0).abs() < 0.2);
        assert!(!e.update(Gnss {
            stamp: 11.0,
            position: Vec2::new(1000.0, 0.0),
            variance: 0.04
        }));
    }
    #[test]
    fn covariance_stays_symmetric_positive() {
        let mut e = Ekf::new(Pose::default());
        for i in 0..200 {
            e.predict(
                Odometry {
                    stamp: i as f64,
                    speed: 4.0,
                    yaw_rate: 0.1,
                },
                0.05,
            );
            let p = e.state().pose.position;
            assert!(e.update(Gnss {
                stamp: i as f64,
                position: p,
                variance: 0.1
            }));
            for a in 0..3 {
                assert!(e.covariance[a][a] > 0.0);
                for b in 0..3 {
                    assert!((e.covariance[a][b] - e.covariance[b][a]).abs() < 1e-9);
                }
            }
        }
    }
}
