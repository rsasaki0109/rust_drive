use crate::{HealthIssue, SensorFrame};
use rustdriving_core::HeadingDecision;
use rustdriving_localization::{Ekf, latency::GnssHistory};

pub const MAX_ABSOLUTE_HEADING_AGE_S: f64 = 0.35;

#[derive(Default)]
pub(super) struct HeadingMonitor {
    fault: Option<(f64, HealthIssue)>,
}
impl HeadingMonitor {
    pub(super) fn observe(
        &mut self,
        ekf: &mut Ekf,
        history: Option<&mut GnssHistory>,
        input: &SensorFrame,
        enabled: bool,
        motion_covered: bool,
        health: &mut Vec<HealthIssue>,
    ) {
        if !enabled {
            if input.absolute_heading.is_some() {
                health.push(HealthIssue::InvalidAbsoluteHeading);
            }
            return;
        }
        if let Some(sample) = input.absolute_heading.filter(|_| motion_covered) {
            // Check malformed data even on duplicate/old deliveries. Valid old
            // samples are ignored and cannot refresh accepted age or clear a fault.
            if !sample.valid() {
                self.fault = Some((input.time, HealthIssue::InvalidAbsoluteHeading));
            } else if ekf
                .heading_diagnostics()
                .last_observed_stamp
                .is_none_or(|t| sample.stamp > t)
            {
                if (input.time - sample.stamp).abs() > 1e-9 {
                    self.fault = Some((input.time, HealthIssue::InvalidAbsoluteHeading));
                } else {
                    let decision = if let Some(history) = history {
                        history
                            .correct_heading(ekf, sample, input.time)
                            .unwrap_or(HeadingDecision::Invalid)
                    } else {
                        ekf.correct_heading(sample)
                    };
                    match decision {
                        HeadingDecision::Accepted => {
                            if self.fault.as_ref().is_none_or(|(t, _)| sample.stamp > *t) {
                                self.fault = None;
                            }
                        }
                        HeadingDecision::RejectedInnovation => {
                            self.fault = Some((input.time, HealthIssue::HeadingInnovationHold));
                        }
                        HeadingDecision::Invalid => {
                            self.fault = Some((input.time, HealthIssue::InvalidAbsoluteHeading));
                        }
                        HeadingDecision::IgnoredTimestamp => {}
                    }
                }
            }
        }
        if let Some((_, issue)) = &self.fault {
            health.push(issue.clone());
        }
        if ekf
            .heading_diagnostics()
            .last_accepted_stamp
            .is_none_or(|t| input.time - t > MAX_ABSOLUTE_HEADING_AGE_S + 1e-9)
        {
            health.push(HealthIssue::StaleAbsoluteHeading);
        }
    }
}
