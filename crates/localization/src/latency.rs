//! Bounded acquisition-time GNSS correction with measured-motion replay.
use crate::Ekf;
use rustdriving_core::{AbsoluteHeading, Gnss, GnssDecision, Odometry, Pose};
use std::collections::VecDeque;

pub const MAX_GNSS_DELAY_S: f64 = 0.5;
pub const MAX_PREDICTIONS: usize = 64;
pub const MAX_MAP_CORRECTIONS: usize = 64;
pub const MAX_GNSS_CORRECTIONS: usize = 64;
pub const MAX_HEADING_CORRECTIONS: usize = 64;
pub const MAX_MAP_DELAY_S: f64 = 0.2;
pub const MAX_HEADING_DELAY_S: f64 = 0.2;
const EPS: f64 = 1e-9;
mod asynchronous;

#[derive(Clone)]
struct Prediction {
    start: f64,
    end: f64,
    before: Ekf,
    odometry: Odometry,
    rear_axle_offset_m: f64,
}

#[derive(Clone)]
struct MapCorrection {
    stamp: f64,
    before: Ekf,
    pose: Pose,
    covariance: [[f64; 3]; 3],
}

#[derive(Clone)]
struct GnssCorrection {
    before: Ekf,
    fix: Gnss,
}

#[derive(Clone)]
struct HeadingCorrection {
    before: Ekf,
    sample: AbsoluteHeading,
}

#[derive(Clone)]
enum Event {
    Prediction(Prediction),
    Map(MapCorrection),
    Gnss(GnssCorrection),
    Heading(HeadingCorrection),
}
impl Event {
    fn priority(&self) -> u8 {
        match self {
            Self::Prediction(_) => 0,
            Self::Gnss(_) => 1,
            Self::Heading(_) => 2,
            Self::Map(_) => 3,
        }
    }

    fn start(&self) -> f64 {
        match self {
            Self::Prediction(p) => p.start,
            Self::Map(m) => m.stamp,
            Self::Gnss(g) => g.fix.stamp,
            Self::Heading(h) => h.sample.stamp,
        }
    }
    fn end(&self) -> f64 {
        match self {
            Self::Prediction(p) => p.end,
            Self::Map(m) => m.stamp,
            Self::Gnss(g) => g.fix.stamp,
            Self::Heading(h) => h.sample.stamp,
        }
    }
    fn before(&self) -> &Ekf {
        match self {
            Self::Prediction(p) => &p.before,
            Self::Map(m) => &m.before,
            Self::Gnss(g) => &g.before,
            Self::Heading(h) => &h.before,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HistoryError {
    InvalidClockOrMotion,
    Unavailable,
    /// The delayed fix would invalidate a previously accepted map correction.
    MapCorrectionConflict,
    /// A new earlier map observation would invalidate an accepted GNSS fix.
    GnssCorrectionConflict,
    /// Rewinding would invalidate a previously accepted heading observation.
    HeadingCorrectionConflict,
}

/// No measurements or calibration enter from evaluation truth. Only this
/// wrapper may correct the supplied EKF while its history is active. Synchronous
/// map measurements must enter through `correct_map_pose`, and synchronous
/// heading through `correct_heading` or `correct_heading_at` with retained corrections enabled. Other
/// external corrections require `reset` and a new covered acquisition interval.
#[derive(Clone, Default)]
pub struct GnssHistory {
    events: VecDeque<Event>,
    // A discarded correction may already be folded into the first snapshot.
    // Earlier equal-time priorities can no longer be reconstructed from it.
    coverage_floor: Option<(f64, u8)>,
    current_time: Option<f64>,
    replayed_map_corrections: u64,
    replayed_gnss_corrections: u64,
    replayed_heading_corrections: u64,
    retain_gnss: bool,
}

fn advance(filter: &mut Ekf, odometry: Odometry, dt: f64, offset: f64) {
    filter.predict_chassis(odometry, dt, offset);
}

impl GnssHistory {
    /// Retain accepted GNSS events too, permitting later map observations to
    /// arrive before or after them. Default history keeps its original behavior.
    pub fn with_delayed_maps() -> Self {
        Self::with_retained_corrections()
    }

    /// Retain admitted GNSS, heading and map events in acquisition order.
    /// Synchronous heading can then survive an earlier GNSS/map insertion.
    pub fn with_retained_corrections() -> Self {
        Self {
            retain_gnss: true,
            ..Self::default()
        }
    }

    pub fn reset(&mut self, now: f64) {
        self.events.clear();
        self.coverage_floor = None;
        self.current_time = (now.is_finite() && now >= 0.).then_some(now);
    }

    pub fn len(&self) -> usize {
        self.events
            .iter()
            .filter(|e| matches!(e, Event::Prediction(_)))
            .count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn map_corrections(&self) -> usize {
        self.events
            .iter()
            .filter(|e| matches!(e, Event::Map(_)))
            .count()
    }

    pub fn gnss_corrections(&self) -> usize {
        self.events
            .iter()
            .filter(|e| matches!(e, Event::Gnss(_)))
            .count()
    }

    pub fn replayed_gnss_corrections(&self) -> u64 {
        self.replayed_gnss_corrections
    }

    pub fn heading_corrections(&self) -> usize {
        self.events
            .iter()
            .filter(|e| matches!(e, Event::Heading(_)))
            .count()
    }

    pub fn replayed_heading_corrections(&self) -> u64 {
        self.replayed_heading_corrections
    }

    pub fn replayed_map_corrections(&self) -> u64 {
        self.replayed_map_corrections
    }

    fn prune(&mut self, now: f64) {
        while self.len() > MAX_PREDICTIONS
            || self.map_corrections() > MAX_MAP_CORRECTIONS
            || self.gnss_corrections() > MAX_GNSS_CORRECTIONS
            || self.heading_corrections() > MAX_HEADING_CORRECTIONS
            || self
                .events
                .front()
                .is_some_and(|e| e.end() < now - MAX_GNSS_DELAY_S - EPS)
        {
            if let Some(event) = self.events.pop_front() {
                self.coverage_floor = Some((event.end(), event.priority()));
            }
        }
    }

    /// Admit a new synchronous map measurement once, with the ordinary EKF
    /// gate. Rejected matches are never revived by later GNSS delivery.
    pub fn correct_map_pose(
        &mut self,
        filter: &mut Ekf,
        pose: Pose,
        covariance: [[f64; 3]; 3],
        now: f64,
    ) -> Result<bool, HistoryError> {
        if self.retain_gnss {
            return self.correct_map_pose_at(filter, pose, covariance, now, now);
        }
        if !now.is_finite()
            || self
                .current_time
                .is_none_or(|time| (time - now).abs() > EPS)
            || self
                .events
                .iter()
                .any(|e| matches!(e, Event::Map(m) if m.stamp >= now - EPS))
        {
            return Err(HistoryError::InvalidClockOrMotion);
        }
        let before = filter.clone();
        if !filter.correct_map_pose(pose, covariance) {
            return Ok(false);
        }
        self.events.push_back(Event::Map(MapCorrection {
            stamp: now,
            before,
            pose,
            covariance,
        }));
        self.prune(now);
        Ok(true)
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
        self.events.push_back(Event::Prediction(Prediction {
            start,
            end,
            before,
            odometry,
            rear_axle_offset_m,
        }));
        self.current_time = Some(end);
        self.prune(end);
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
        if self.retain_gnss {
            return self.correct_retained_gnss(filter, fix, now);
        }
        let map_at_now = self
            .events
            .back()
            .is_some_and(|e| matches!(e, Event::Map(m) if (m.stamp - now).abs() <= EPS));
        if now - fix.stamp <= EPS && !map_at_now {
            let decision = filter.correct(fix);
            self.events.clear();
            return Ok(decision);
        }
        if now - fix.stamp > MAX_GNSS_DELAY_S + EPS {
            return Err(HistoryError::Unavailable);
        }
        let Some(index) = self
            .events
            .iter()
            .position(|e| e.start() <= fix.stamp + EPS && fix.stamp <= e.end() + EPS)
        else {
            return Err(HistoryError::Unavailable);
        };
        let step = &self.events[index];
        let mut candidate = step.before().clone();
        candidate.diagnostics = filter.diagnostics;
        candidate.last_gnss = filter.last_gnss;
        let prefix = fix.stamp - step.start();
        if let Event::Prediction(step) = step
            && prefix > EPS
        {
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
        let mut map_count = 0_u64;
        for (i, original) in self.events.iter().enumerate().skip(index) {
            match original {
                Event::Prediction(original) => {
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
                    replayed.push_back(Event::Prediction(Prediction {
                        start,
                        before,
                        ..original.clone()
                    }));
                }
                Event::Map(original) => {
                    let before = candidate.clone();
                    if !candidate.correct_map_pose(original.pose, original.covariance) {
                        // Atomic refusal: keep the live filter, history and all
                        // observation ownership unchanged, including freshness.
                        return Err(HistoryError::MapCorrectionConflict);
                    }
                    replayed.push_back(Event::Map(MapCorrection {
                        before,
                        ..original.clone()
                    }));
                    map_count += 1;
                }
                Event::Gnss(_) | Event::Heading(_) => {
                    return Err(HistoryError::InvalidClockOrMotion);
                }
            }
        }
        *filter = candidate;
        self.events = replayed;
        self.replayed_map_corrections = self.replayed_map_corrections.saturating_add(map_count);
        Ok(decision)
    }
}

#[cfg(test)]
mod tests;
