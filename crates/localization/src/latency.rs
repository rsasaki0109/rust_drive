//! Bounded acquisition-time GNSS correction with measured-motion replay.
use crate::Ekf;
use rustdriving_core::{Gnss, GnssDecision, Odometry};
use std::collections::VecDeque;

pub const MAX_GNSS_DELAY_S: f64 = 0.5;
pub const MAX_PREDICTIONS: usize = 64;
const EPS: f64 = 1e-9;

#[derive(Clone)]
struct Prediction {
    start: f64,
    end: f64,
    before: Ekf,
    odometry: Odometry,
    rear_axle_offset_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HistoryError {
    InvalidClockOrMotion,
    Unavailable,
}

/// No measurements or calibration enter from evaluation truth. Only this
/// wrapper may correct the supplied EKF while its history is active; external
/// pose corrections require `reset` and a new covered acquisition interval.
#[derive(Default)]
pub struct GnssHistory {
    predictions: VecDeque<Prediction>,
    current_time: Option<f64>,
}

fn advance(filter: &mut Ekf, odometry: Odometry, dt: f64, offset: f64) {
    filter.predict_chassis(odometry, dt, offset);
}

impl GnssHistory {
    pub fn reset(&mut self, now: f64) {
        self.predictions.clear();
        self.current_time = now.is_finite().then_some(now);
    }

    pub fn len(&self) -> usize {
        self.predictions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.predictions.is_empty()
    }

    /// Record exactly the prediction applied by the ordinary filter. A missing
    /// motion interval starts a new history epoch, never an invented bridge.
    pub fn predict(
        &mut self,
        filter: &mut Ekf,
        odometry: Odometry,
        start: f64,
        end: f64,
        rear_axle_offset_m: f64,
    ) -> Result<(), HistoryError> {
        if !start.is_finite()
            || !end.is_finite()
            || start < 0.
            || end <= start
            || end - start > 0.25 + EPS
            || !odometry.speed.is_finite()
            || !odometry.yaw_rate.is_finite()
            || !rear_axle_offset_m.is_finite()
            || rear_axle_offset_m < 0.
            || self.current_time.is_some_and(|now| start < now - EPS)
        {
            return Err(HistoryError::InvalidClockOrMotion);
        }
        if self
            .current_time
            .is_none_or(|now| (now - start).abs() > EPS)
        {
            self.reset(start);
        }
        let before = filter.clone();
        advance(filter, odometry, end - start, rear_axle_offset_m);
        if self.predictions.len() == MAX_PREDICTIONS {
            self.predictions.pop_front();
        }
        self.predictions.push_back(Prediction {
            start,
            end,
            before,
            odometry,
            rear_axle_offset_m,
        });
        self.current_time = Some(end);
        while self.predictions.len() > MAX_PREDICTIONS
            || self
                .predictions
                .front()
                .is_some_and(|p| p.end < end - MAX_GNSS_DELAY_S - EPS)
        {
            self.predictions.pop_front();
        }
        Ok(())
    }

    /// Gate the innovation at acquisition time, then replay the same bounded
    /// odometry intervals. Diagnostic counts and accepted freshness remain on
    /// original acquisition clocks; neither is refreshed by internal replay.
    pub fn correct(
        &mut self,
        filter: &mut Ekf,
        fix: Gnss,
        now: f64,
    ) -> Result<GnssDecision, HistoryError> {
        if !now.is_finite()
            || self
                .current_time
                .is_none_or(|time| (time - now).abs() > EPS)
            || !fix.stamp.is_finite()
            || fix.stamp < 0.
            || fix.stamp > now + EPS
        {
            return Err(HistoryError::InvalidClockOrMotion);
        }
        if !fix.position.finite() || !fix.variance.is_finite() || fix.variance <= 0. {
            return Ok(GnssDecision::Invalid);
        }
        if filter
            .diagnostics
            .last_observed_stamp
            .is_some_and(|stamp| fix.stamp <= stamp)
        {
            return Ok(GnssDecision::IgnoredTimestamp);
        }
        if now - fix.stamp <= EPS {
            let decision = filter.correct(fix);
            self.predictions.clear();
            return Ok(decision);
        }
        if now - fix.stamp > MAX_GNSS_DELAY_S + EPS {
            return Err(HistoryError::Unavailable);
        }
        let Some(index) = self
            .predictions
            .iter()
            .position(|p| p.start <= fix.stamp + EPS && fix.stamp <= p.end + EPS)
        else {
            return Err(HistoryError::Unavailable);
        };
        let step = &self.predictions[index];
        let mut candidate = step.before.clone();
        candidate.diagnostics = filter.diagnostics;
        candidate.last_gnss = filter.last_gnss;
        let prefix = fix.stamp - step.start;
        if prefix > EPS {
            advance(
                &mut candidate,
                step.odometry,
                prefix,
                step.rear_axle_offset_m,
            );
        }
        let decision = candidate.correct(fix);
        if decision != GnssDecision::Accepted {
            // Rejection updates evidence only, never the live pose/covariance.
            filter.diagnostics = candidate.diagnostics;
            return Ok(decision);
        }
        let mut replayed = VecDeque::new();
        for (i, original) in self.predictions.iter().enumerate().skip(index) {
            let start = if i == index {
                fix.stamp
            } else {
                original.start
            };
            if original.end - start <= EPS {
                continue;
            }
            let before = candidate.clone();
            advance(
                &mut candidate,
                original.odometry,
                original.end - start,
                original.rear_axle_offset_m,
            );
            replayed.push_back(Prediction {
                start,
                before,
                ..original.clone()
            });
        }
        *filter = candidate;
        self.predictions = replayed;
        Ok(decision)
    }
}

#[cfg(test)]
mod tests;
