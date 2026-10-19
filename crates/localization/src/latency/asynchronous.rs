//! Optional chronological replay of accepted GNSS and map measurements.
use super::*;

impl Event {
    fn priority(&self) -> u8 {
        match self {
            Self::Prediction(_) => 0,
            Self::Gnss(_) => 1,
            Self::Map(_) => 2,
        }
    }

    fn set_before(&mut self, filter: &Ekf) {
        match self {
            Self::Prediction(p) => p.before = filter.clone(),
            Self::Gnss(g) => g.before = filter.clone(),
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

    /// Reconstruct the prior for an observation. GNSS precedes map at equal
    /// acquisition time; neither sees corrections acquired in its future.
    fn prior_at(
        &self,
        filter: &Ekf,
        stamp: f64,
        priority: u8,
        now: f64,
    ) -> Result<Ekf, HistoryError> {
        self.check_coverage(stamp, now)?;
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
        Ok(self.prior_at(filter, stamp, 2, now)?.state.pose)
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
        for event in &mut replayed {
            event.set_before(&candidate);
            event.apply(&mut candidate)?;
            if event.end() > stamp + EPS
                || ((event.end() - stamp).abs() <= EPS && event.priority() > priority)
            {
                match event {
                    Event::Map(_) => maps += 1,
                    Event::Gnss(_) => fixes += 1,
                    Event::Prediction(_) => {}
                }
            }
        }
        // Counts, freshness and last NIS describe original delivery processing,
        // not another observation manufactured by a rewind.
        candidate.diagnostics = owned.diagnostics;
        candidate.last_gnss = owned.last_gnss;
        *filter = candidate;
        self.events = replayed;
        self.replayed_map_corrections = self.replayed_map_corrections.saturating_add(maps);
        self.replayed_gnss_corrections = self.replayed_gnss_corrections.saturating_add(fixes);
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
        let mut prior = self.prior_at(filter, stamp, 2, now)?;
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
