//! Optional correlated horizontal GNSS calibration; legacy scalar math is separate.
use super::*;

/// Trace two keeps `Gnss::variance` the mean horizontal-axis variance (m²).
/// Shape entries are dimensionless in world ENU, not a body-frame covariance.
pub fn validate_gnss_covariance_shape(shape: [[f64; 2]; 2]) -> Result<(), String> {
    let conditional = shape[1][1] - shape[0][1] * (shape[0][1] / shape[0][0]);
    if shape.iter().flatten().any(|v| !v.is_finite())
        || shape[0][1] != shape[1][0]
        || (shape[0][0] + shape[1][1] - 2.).abs() > 1e-12
        || shape[0][0] <= 0.
        || !conditional.is_finite()
        || conditional <= 0.
    {
        return Err(
            "GNSS covariance shape must be finite, symmetric, positive definite and have trace two"
                .into(),
        );
    }
    Ok(())
}

impl Ekf {
    pub(super) fn correct_shaped(&mut self, fix: Gnss, shape: [[f64; 2]; 2]) -> GnssDecision {
        let noise = shape.map(|row| row.map(|v| v * fix.variance));
        let ratio = noise[0][1] / noise[0][0];
        let conditional = noise[1][1] - noise[0][1] * ratio;
        if noise.iter().flatten().any(|v| !v.is_finite())
            || !ratio.is_finite()
            || noise[0][0] <= 0.
            || !conditional.is_finite()
            || conditional <= 0.
        {
            return GnssDecision::Invalid;
        }
        let residual = fix.position.minus(self.state.pose.position);
        let xx = self.covariance[0][0] + noise[0][0];
        let xy = self.covariance[0][1] / 2. + self.covariance[1][0] / 2. + noise[0][1];
        let yy = self.covariance[1][1] + noise[1][1];
        let innovation_ratio = xy / xx;
        let innovation_conditional = yy - xy * innovation_ratio;
        if !xx.is_finite()
            || xx <= 0.
            || !innovation_conditional.is_finite()
            || innovation_conditional <= 0.
        {
            return GnssDecision::Invalid;
        }
        let nis = (residual.x / xx.sqrt()).powi(2)
            + ((residual.y - innovation_ratio * residual.x) / innovation_conditional.sqrt())
                .powi(2);
        let mut candidate = self.clone();
        candidate.diagnostics.last_observed_stamp = Some(fix.stamp);
        candidate.diagnostics.last_nis = nis.is_finite().then_some(nis);
        if !nis.is_finite() || nis > 36. {
            candidate.diagnostics.last_decision = Some(GnssDecision::RejectedInnovation);
            candidate.diagnostics.rejected_fixes =
                candidate.diagnostics.rejected_fixes.saturating_add(1);
            *self = candidate;
            return GnssDecision::RejectedInnovation;
        }
        // LDL whitening of R gives independent scalar measurements x and
        // y - (Rxy/Rxx)*x. Keep the original joint NIS gate and Joseph updates.
        if !candidate.correct_scalar(fix.position, [1., 0., 0.], noise[0][0])
            || !candidate.correct_scalar(fix.position, [-ratio, 1., 0.], conditional)
        {
            return GnssDecision::Invalid;
        }
        candidate.last_gnss = fix.stamp;
        candidate.diagnostics.last_accepted_stamp = Some(fix.stamp);
        candidate.diagnostics.last_decision = Some(GnssDecision::Accepted);
        candidate.diagnostics.accepted_fixes =
            candidate.diagnostics.accepted_fixes.saturating_add(1);
        *self = candidate;
        GnssDecision::Accepted
    }

    fn correct_scalar(&mut self, measurement: Vec2, h: [f64; 3], variance: f64) -> bool {
        let projected: [f64; 3] =
            std::array::from_fn(|i| (0..3).map(|j| self.covariance[i][j] * h[j]).sum());
        let denominator = variance + (0..3).map(|i| h[i] * projected[i]).sum::<f64>();
        if !denominator.is_finite() || denominator <= 0. {
            return false;
        }
        let gain = projected.map(|v| v / denominator);
        let residual = measurement.minus(self.state.pose.position);
        let innovation = h[0] * residual.x + h[1] * residual.y;
        let pose = Pose {
            position: self
                .state
                .pose
                .position
                .plus(Vec2::new(gain[0] * innovation, gain[1] * innovation)),
            yaw: wrap_angle(self.state.pose.yaw + gain[2] * innovation),
        };
        let a: [[f64; 3]; 3] =
            std::array::from_fn(|i| std::array::from_fn(|j| f64::from(i == j) - gain[i] * h[j]));
        let mut next = [[0.; 3]; 3];
        for (i, row) in next.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = (gain[i] * variance) * gain[j];
                for (m, old) in self.covariance.iter().enumerate() {
                    for (n, p) in old.iter().enumerate() {
                        *value += a[i][m] * p * a[j][n];
                    }
                }
            }
        }
        for i in 0..3 {
            for j in 0..i {
                let v = next[i][j] / 2. + next[j][i] / 2.;
                next[i][j] = v;
                next[j][i] = v;
            }
        }
        if !pose.position.finite() || !pose.yaw.is_finite() || cholesky3(next).is_none() {
            return false;
        }
        self.state.pose = pose;
        self.covariance = next;
        true
    }
}

#[cfg(test)]
mod tests;
