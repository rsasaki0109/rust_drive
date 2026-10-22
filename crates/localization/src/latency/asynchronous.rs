//! Optional chronological replay of accepted GNSS, heading and map measurements.
use super::*;
use rustdriving_core::HeadingDecision;

impl Event {
    fn set_before(&mut self, filter: &Ekf) {
        match self {
            Self::Prediction(p) => p.before = filter.clone(),
            Self::Gnss(g) => g.before = filter.clone(),
            Self::Heading(h) => h.before = filter.clone(),
            Self::Map(m) => m.before = filter.clone(),
        }
    }

    fn apply(&self, filter: &mut Ekf) -> Result<(), HistoryError> {
        match self {
            Self::Prediction(p) => {
                advance(filter, p.odometry, p.end - p.start, p.rear_axle_offset_m);
            }
            Self::Gnss(g) => {
                // Replay gates a measurement again without treating it as a
                // newly delivered observation. Caller restores owned evidence.
                filter.diagnostics = Default::default();
                if filter.correct(g.fix) != GnssDecision::Accepted {
                    return Err(HistoryError::GnssCorrectionConflict);
                }
            }
            Self::Map(m) => {
                if !filter.correct_map_pose(m.pose, m.covariance) {
                    return Err(HistoryError::MapCorrectionConflict);
                }
            }
            Self::Heading(h) => {
                filter.heading_diagnostics = Default::default();
                if filter.correct_heading(h.sample) != HeadingDecision::Accepted {
                    return Err(HistoryError::HeadingCorrectionConflict);
                }
            }
        }
        Ok(())
    }
}

impl GnssHistory {
    fn check_coverage(&self, stamp: f64, now: f64) -> Result<(), HistoryError> {
        if !now.is_finite()
            || now < 0.
            || !stamp.is_finite()
            || stamp < 0.
            || stamp > now + EPS
            || self
                .current_time
                .is_none_or(|time| (time - now).abs() > EPS)
        {
            return Err(HistoryError::InvalidClockOrMotion);
        }
        if now - stamp > MAX_GNSS_DELAY_S + EPS
            || self
                .events
                .front()
                .map_or(now - stamp > EPS, |e| stamp < e.start() - EPS)
        {
            return Err(HistoryError::Unavailable);
        }
        Ok(())
    }

    /// Reconstruct the prior for an observation. GNSS, heading, then map at equal
    /// acquisition time; none sees corrections acquired in its future.
    fn prior_at(
        &self,
        filter: &Ekf,
        stamp: f64,
        priority: u8,
        now: f64,
    ) -> Result<Ekf, HistoryError> {
        self.check_coverage(stamp, now)?;
        if self
            .coverage_floor
            .is_some_and(|(time, discarded_priority)| {
                (stamp - time).abs() <= EPS && priority <= discarded_priority
            })
        {
            return Err(HistoryError::Unavailable);
        }
        let mut prior = self
            .events
            .front()
            .map_or_else(|| filter.clone(), |e| e.before().clone());
        for event in &self.events {
            if let Event::Prediction(p) = event {
                if p.start >= stamp - EPS {
                    break;
                }
                let end = p.end.min(stamp);
                advance(&mut prior, p.odometry, end - p.start, p.rear_axle_offset_m);
                if p.end > stamp + EPS {
                    break;
                }
            } else if event.end() < stamp - EPS
                || ((event.end() - stamp).abs() <= EPS && event.priority() < priority)
            {
                event.apply(&mut prior)?;
            } else {
                break;
            }
        }
        Ok(prior)
    }

    /// Estimated acquisition pose for map registration, after equal-time GNSS
    /// and before the new map measurement. No correction is applied by querying.
    pub fn map_prior_pose(&self, filter: &Ekf, stamp: f64, now: f64) -> Result<Pose, HistoryError> {
        if !self.retain_gnss || now - stamp > MAX_MAP_DELAY_S + EPS {
            return Err(HistoryError::Unavailable);
        }
        Ok(self.prior_at(filter, stamp, 3, now)?.state.pose)
    }

    /// Insert one admitted measurement and rebuild the bounded interval. All
    /// old accepted measurements must still pass their ordinary gates. Changes
    /// remain local until every replay succeeds, including later GNSS fixes.
    fn insert(
        &mut self,
        filter: &mut Ekf,
        measurement: Event,
        owned: &Ekf,
        now: f64,
    ) -> Result<(), HistoryError> {
        let stamp = measurement.end();
        let priority = measurement.priority();
        self.check_coverage(stamp, now)?;
        let mut candidate = self
            .events
            .front()
            .map_or_else(|| filter.clone(), |e| e.before().clone());
        let mut replayed = VecDeque::new();
        for original in &self.events {
            if let Event::Prediction(p) = original
                && p.start < stamp - EPS
                && stamp < p.end - EPS
            {
                replayed.push_back(Event::Prediction(Prediction {
                    end: stamp,
                    ..p.clone()
                }));
                replayed.push_back(Event::Prediction(Prediction {
                    start: stamp,
                    ..p.clone()
                }));
            } else {
                replayed.push_back(original.clone());
            }
        }
        replayed.push_back(measurement);
        replayed.make_contiguous().sort_by(|a, b| {
            a.end()
                .total_cmp(&b.end())
                .then(a.priority().cmp(&b.priority()))
        });
        let mut maps = 0_u64;
        let mut fixes = 0_u64;
        let mut headings = 0_u64;
        for event in &mut replayed {
            event.set_before(&candidate);
            event.apply(&mut candidate)?;
            if event.end() > stamp + EPS
                || ((event.end() - stamp).abs() <= EPS && event.priority() > priority)
            {
                match event {
                    Event::Map(_) => maps += 1,
                    Event::Gnss(_) => fixes += 1,
                    Event::Heading(_) => headings += 1,
                    Event::Prediction(_) => {}
                }
            }
        }
        // Counts, freshness and last NIS describe original delivery processing,
        // not another observation manufactured by a rewind.
        candidate.diagnostics = owned.diagnostics;
        candidate.last_gnss = owned.last_gnss;
        candidate.heading_diagnostics = owned.heading_diagnostics;
        *filter = candidate;
        self.events = replayed;
        self.replayed_map_corrections = self.replayed_map_corrections.saturating_add(maps);
        self.replayed_gnss_corrections = self.replayed_gnss_corrections.saturating_add(fixes);
        self.replayed_heading_corrections =
            self.replayed_heading_corrections.saturating_add(headings);
        self.prune(now);
        Ok(())
    }

    pub(super) fn correct_retained_gnss(
        &mut self,
        filter: &mut Ekf,
        fix: Gnss,
        now: f64,
    ) -> Result<GnssDecision, HistoryError> {
        let mut prior = self.prior_at(filter, fix.stamp, 1, now)?;
        prior.diagnostics = filter.diagnostics;
        prior.last_gnss = filter.last_gnss;
        prior.heading_diagnostics = filter.heading_diagnostics;
        let decision = prior.correct(fix);
        if decision != GnssDecision::Accepted {
            filter.diagnostics = prior.diagnostics;
            return Ok(decision);
        }
        self.insert(
            filter,
            Event::Gnss(GnssCorrection {
                before: prior.clone(),
                fix,
            }),
            &prior,
            now,
        )?;
        Ok(decision)
    }

    /// Admit only a synchronous heading. Replays retain the original delivery
    /// diagnostics and may neither revive rejected samples nor renew freshness.
    pub fn correct_heading(
        &mut self,
        filter: &mut Ekf,
        sample: AbsoluteHeading,
        now: f64,
    ) -> Result<HeadingDecision, HistoryError> {
        self.correct_heading_with_delay(filter, sample, now, 0.)
    }

    /// Fuse an acquired heading up to 0.2 s old at its original acquisition
    /// pose, then replay admitted GNSS, heading and map events transactionally.
    /// Delivery never changes the acquisition-owned freshness or sample counts.
    pub fn correct_heading_at(
        &mut self,
        filter: &mut Ekf,
        sample: AbsoluteHeading,
        now: f64,
    ) -> Result<HeadingDecision, HistoryError> {
        self.correct_heading_with_delay(filter, sample, now, MAX_HEADING_DELAY_S)
    }

    fn correct_heading_with_delay(
        &mut self,
        filter: &mut Ekf,
        sample: AbsoluteHeading,
        now: f64,
        maximum_delay: f64,
    ) -> Result<HeadingDecision, HistoryError> {
        if !self.retain_gnss {
            return Err(HistoryError::Unavailable);
        }
        if !sample.valid() {
            return Ok(HeadingDecision::Invalid);
        }
        self.check_coverage(now, now)?;
        if filter
            .heading_diagnostics
            .last_observed_stamp
            .is_some_and(|stamp| sample.stamp <= stamp)
        {
            return Ok(HeadingDecision::IgnoredTimestamp);
        }
        if sample.stamp > now + EPS || (maximum_delay == 0. && now - sample.stamp > EPS) {
            return Err(HistoryError::InvalidClockOrMotion);
        }
        if now - sample.stamp > maximum_delay + EPS {
            return Err(HistoryError::Unavailable);
        }
        let mut prior = self.prior_at(filter, sample.stamp, 2, now)?;
        prior.heading_diagnostics = filter.heading_diagnostics;
        let decision = prior.correct_heading(sample);
        if decision != HeadingDecision::Accepted {
            filter.heading_diagnostics = prior.heading_diagnostics;
            return Ok(decision);
        }
        let mut owned = filter.clone();
        owned.heading_diagnostics = prior.heading_diagnostics;
        self.insert(
            filter,
            Event::Heading(HeadingCorrection {
                before: prior,
                sample,
            }),
            &owned,
            now,
        )?;
        Ok(decision)
    }

    /// Admit a delayed map pose only with explicit retained-GNSS history. Map
    /// acquisition clocks must advance, and the existing 0.2 s freshness limit
    /// also bounds supported delay. Any conflict leaves live state unchanged.
    pub fn correct_map_pose_at(
        &mut self,
        filter: &mut Ekf,
        pose: Pose,
        covariance: [[f64; 3]; 3],
        stamp: f64,
        now: f64,
    ) -> Result<bool, HistoryError> {
        if !self.retain_gnss || now - stamp > MAX_MAP_DELAY_S + EPS {
            return Err(HistoryError::Unavailable);
        }
        if self
            .events
            .iter()
            .any(|e| matches!(e, Event::Map(m) if m.stamp >= stamp - EPS))
        {
            return Err(HistoryError::InvalidClockOrMotion);
        }
        let mut prior = self.prior_at(filter, stamp, 3, now)?;
        if !prior.correct_map_pose(pose, covariance) {
            return Ok(false);
        }
        let owned = filter.clone();
        self.insert(
            filter,
            Event::Map(MapCorrection {
                stamp,
                before: prior,
                pose,
                covariance,
            }),
            &owned,
            now,
        )?;
        Ok(true)
    }
}

#[cfg(test)]
mod tests;
