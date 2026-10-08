//! Three-state EKF: wheel speed / gyro prediction and scalar GNSS updates.
use rustdrive_core::{EgoState, Gnss, Odometry, Pose, Vec2, wrap_angle};
pub struct Ekf {
    state: EgoState,
    covariance: [[f64; 3]; 3],
    pub last_gnss: f64,
}
impl Ekf {
    /// Initial heading comes from the configured spawn calibration, not live simulator truth.
    pub fn new(pose: Pose) -> Self {
        Self {
            state: EgoState { pose, speed: 0.0 },
            covariance: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.03]],
            last_gnss: f64::NEG_INFINITY,
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
    /// Reject non-finite readings, stale measurements, and >6-sigma innovations.
    pub fn update(&mut self, fix: Gnss) -> bool {
        if !fix.position.finite()
            || !fix.variance.is_finite()
            || fix.variance <= 0.0
            || !fix.stamp.is_finite()
            || fix.stamp < self.last_gnss
        {
            return false;
        }
        let residual = fix.position.minus(self.state.pose.position);
        let nis = residual.x.powi(2) / (self.covariance[0][0] + fix.variance)
            + residual.y.powi(2) / (self.covariance[1][1] + fix.variance);
        if nis > 36.0 {
            return false;
        }
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
            for (i, row) in self.covariance.iter_mut().enumerate() {
                for (j, value) in row.iter_mut().enumerate() {
                    *value = old[i][j] - k[i] * old[axis][j];
                }
            }
        }
        self.last_gnss = fix.stamp;
        true
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
