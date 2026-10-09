//! Three-state EKF: wheel speed / gyro prediction and scalar GNSS updates.
use rustdrive_core::{
    EgoState, Gnss, GnssDecision, LocalizationDiagnostics, Odometry, Pose, Vec2, wrap_angle,
};
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
#[cfg(test)]
mod tests {
    use super::*;
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
