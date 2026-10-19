//! Explicit antenna displacement from the estimated body reference point.
use super::*;

/// Body forward/left meters, bounded to a ten-meter planar mounting radius.
pub fn validate_gnss_antenna_offset(offset: Vec2) -> Result<(), String> {
    if !offset.finite() || offset.x.hypot(offset.y) > 10. {
        return Err(
            "GNSS antenna offset must be finite body forward/left meters within 10 m".into(),
        );
    }
    Ok(())
}

impl Ekf {
    /// Calibrate GNSS antenna position relative to this filter's pose origin.
    /// This is supplied mounting geometry, never inferred from observations.
    pub fn with_gnss_antenna_offset(mut self, offset: Vec2) -> Result<Self, String> {
        validate_gnss_antenna_offset(offset)?;
        self.gnss_antenna_offset_m = Some(offset);
        Ok(self)
    }

    pub(super) fn correct_antenna(&mut self, fix: Gnss, offset: Vec2) -> GnssDecision {
        let arm = offset.rotated(self.state.pose.yaw);
        let residual = fix.position.minus(self.state.pose.position.plus(arm));
        // h(p) = position + R(yaw)*offset; linearize both rows at the same prior.
        let h = [[1., 0., -arm.y], [0., 1., arm.x]];
        let shape = self.gnss_covariance_shape.unwrap_or([[1., 0.], [0., 1.]]);
        let noise = shape.map(|row| row.map(|v| v * fix.variance));
        let ratio = noise[0][1] / noise[0][0];
        let conditional = noise[1][1] - noise[0][1] * ratio;
        if noise.iter().flatten().any(|v| !v.is_finite())
            || !ratio.is_finite()
            || !conditional.is_finite()
            || noise[0][0] <= 0.
            || conditional <= 0.
        {
            return GnssDecision::Invalid;
        }
        let projected: [[f64; 3]; 2] = std::array::from_fn(|axis| {
            std::array::from_fn(|i| (0..3).map(|j| self.covariance[i][j] * h[axis][j]).sum())
        });
        let s: [[f64; 2]; 2] = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                noise[i][j] + (0..3).map(|k| h[i][k] * projected[j][k]).sum::<f64>()
            })
        });
        let sr = (s[0][1] / 2. + s[1][0] / 2.) / s[0][0];
        let sc = s[1][1] - (s[0][1] / 2. + s[1][0] / 2.) * sr;
        if !s[0][0].is_finite() || s[0][0] <= 0. || !sc.is_finite() || sc <= 0. {
            return GnssDecision::Invalid;
        }
        let nis = (residual.x / s[0][0].sqrt()).powi(2)
            + ((residual.y - sr * residual.x) / sc.sqrt()).powi(2);
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
        // Whiten R and apply sequential scalar Joseph updates to the frozen
        // linearization. Accumulate an unwrapped tangent correction, so the
        // second row does not relinearize or see an artificial 2*pi jump.
        let rows = [h[0], std::array::from_fn(|i| h[1][i] - ratio * h[0][i])];
        let innovations = [residual.x, residual.y - ratio * residual.x];
        let mut delta = [0.; 3];
        for (axis, variance) in [noise[0][0], conditional].into_iter().enumerate() {
            let row = rows[axis];
            let ph: [f64; 3] =
                std::array::from_fn(|i| (0..3).map(|j| candidate.covariance[i][j] * row[j]).sum());
            let denominator = variance + (0..3).map(|i| row[i] * ph[i]).sum::<f64>();
            if !denominator.is_finite() || denominator <= 0. {
                return GnssDecision::Invalid;
            }
            let gain = ph.map(|p| p / denominator);
            let innovation = innovations[axis] - (0..3).map(|i| row[i] * delta[i]).sum::<f64>();
            for i in 0..3 {
                delta[i] += gain[i] * innovation;
            }
            let a: [[f64; 3]; 3] = std::array::from_fn(|i| {
                std::array::from_fn(|j| f64::from(i == j) - gain[i] * row[j])
            });
            let mut next = [[0.; 3]; 3];
            for (i, values) in next.iter_mut().enumerate() {
                for (j, value) in values.iter_mut().enumerate() {
                    *value = (gain[i] * variance) * gain[j];
                    for (m, old) in candidate.covariance.iter().enumerate() {
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
            if cholesky3(next).is_none() {
                return GnssDecision::Invalid;
            }
            candidate.covariance = next;
        }
        candidate.state.pose = Pose {
            position: self.state.pose.position.plus(Vec2::new(delta[0], delta[1])),
            yaw: wrap_angle(self.state.pose.yaw + delta[2]),
        };
        if !candidate.state.pose.position.finite() || !candidate.state.pose.yaw.is_finite() {
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
}

#[cfg(test)]
mod tests;
