use crate::{Ekf, cholesky3};
use rustdriving_core::{AbsoluteHeading, HeadingDecision, HeadingDiagnostics, wrap_angle};

impl Ekf {
    /// Synchronous correction at the filter's current pose. History wrappers
    /// must retain this event before permitting any acquisition-time rewind.
    pub fn correct_heading(&mut self, sample: AbsoluteHeading) -> HeadingDecision {
        if !sample.valid() {
            return HeadingDecision::Invalid;
        }
        if self
            .heading_diagnostics
            .last_observed_stamp
            .is_some_and(|t| sample.stamp <= t)
        {
            return HeadingDecision::IgnoredTimestamp;
        }
        let innovation = wrap_angle(sample.yaw_rad - self.state.pose.yaw);
        let variance = self.covariance[2][2] + sample.variance_rad2;
        if !innovation.is_finite() || !variance.is_finite() || variance <= 0. {
            return HeadingDecision::Invalid;
        }
        let nis = (innovation / variance.sqrt()).powi(2);
        if !nis.is_finite() || nis > 9. {
            self.heading_diagnostics.last_observed_stamp = Some(sample.stamp);
            self.heading_diagnostics.last_nis = nis.is_finite().then_some(nis);
            self.heading_diagnostics.last_decision = Some(HeadingDecision::RejectedInnovation);
            self.heading_diagnostics.rejected_samples =
                self.heading_diagnostics.rejected_samples.saturating_add(1);
            return HeadingDecision::RejectedInnovation;
        }
        let gain: [f64; 3] = std::array::from_fn(|i| self.covariance[i][2] / variance);
        let position = self.state.pose.position.plus(rustdriving_core::Vec2::new(
            gain[0] * innovation,
            gain[1] * innovation,
        ));
        let yaw = wrap_angle(self.state.pose.yaw + gain[2] * innovation);
        let a: [[f64; 3]; 3] = std::array::from_fn(|i| {
            std::array::from_fn(|j| f64::from(i == j) - if j == 2 { gain[i] } else { 0. })
        });
        let mut next = [[0.; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                next[i][j] = gain[i] * sample.variance_rad2 * gain[j];
                for (k, row) in self.covariance.iter().enumerate() {
                    for (l, value) in row.iter().enumerate() {
                        next[i][j] += a[i][k] * value * a[j][l];
                    }
                }
            }
        }
        for i in 0..3 {
            for j in 0..i {
                let symmetric = next[i][j] / 2. + next[j][i] / 2.;
                next[i][j] = symmetric;
                next[j][i] = symmetric;
            }
        }
        if !position.finite() || !yaw.is_finite() || cholesky3(next).is_none() {
            return HeadingDecision::Invalid;
        }
        self.state.pose.position = position;
        self.state.pose.yaw = yaw;
        self.covariance = next;
        self.heading_diagnostics.last_observed_stamp = Some(sample.stamp);
        self.heading_diagnostics.last_accepted_stamp = Some(sample.stamp);
        self.heading_diagnostics.last_decision = Some(HeadingDecision::Accepted);
        self.heading_diagnostics.last_nis = Some(nis);
        self.heading_diagnostics.accepted_samples =
            self.heading_diagnostics.accepted_samples.saturating_add(1);
        HeadingDecision::Accepted
    }

    pub fn heading_diagnostics(&self) -> HeadingDiagnostics {
        self.heading_diagnostics
    }
}
