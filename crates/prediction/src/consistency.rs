//! Detect a forecast failing to enclose a later acquired proxy disc.
//! This observes model consistency; it does not calibrate physical uncertainty.
use rustdriving_core::{
    ClearSweepEvidence, ClearSweepRecoveryDiagnostics, ClearSweepRelease,
    ForecastConsistencyDiagnostics, ForecastConsistencyFault as Fault, ForecastConsistencyMiss,
    ForecastConsistencyState as State, Prediction, Track, TrackMeasurement,
};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

const MAX_IDENTITIES: usize = 128;
const MAX_REFERENCES: usize = 16;
const MAX_KNOTS: usize = 64;

struct Reference {
    origin: f64,
    acquisition: f64,
    prediction: Prediction,
}

/// The caller validates full acquired rays and synchronous stopped measured
/// motion before supplying clear-sweep evidence. Other faults remain upstream.
pub struct ForecastConsistencyAdmission {
    pub upstream_healthy: bool,
    pub clear_sweep: Option<ClearSweepEvidence>,
}

#[derive(Default)]
pub struct ForecastConsistencyGuard {
    references: BTreeMap<u64, VecDeque<Reference>>,
    held: BTreeSet<u64>,
    previous_time: Option<f64>,
    last_recovery_acquisition: Option<f64>,
    reset_required: bool,
    diagnostic: ForecastConsistencyDiagnostics,
}

impl ForecastConsistencyGuard {
    pub fn enable_clear_sweep_recovery(&mut self) {
        self.diagnostic
            .clear_sweep
            .get_or_insert_with(ClearSweepRecoveryDiagnostics::default);
    }
    pub fn diagnostics(&self) -> &ForecastConsistencyDiagnostics {
        &self.diagnostic
    }

    pub fn blocking(&self) -> bool {
        self.diagnostic.fault.is_some()
    }

    fn reset_confirmation(&mut self) {
        self.diagnostic.recovery_samples = 0;
        self.diagnostic.recovery_first_acquisition_s = None;
        self.last_recovery_acquisition = None;
        self.reset_clear_confirmation();
    }

    fn reset_clear_confirmation(&mut self) {
        if let Some(clear) = &mut self.diagnostic.clear_sweep {
            clear.confirmation_samples = 0;
            clear.first_acquisition_s = None;
            clear.last_acquisition_s = None;
        }
    }

    fn release(&mut self) {
        self.diagnostic.fault = None;
        self.diagnostic.fault_control_time_s = None;
        self.diagnostic.recovery_after_control_s = None;
        self.held.clear();
        self.reset_confirmation();
    }

    fn fault(&mut self, now: f64, cause: Fault, reset_required: bool) {
        self.diagnostic.fault = Some(cause);
        self.diagnostic.fault_control_time_s = now.is_finite().then_some(now);
        self.diagnostic.recovery_after_control_s = now.is_finite().then_some(now);
        self.reset_required |= reset_required;
        self.reset_confirmation();
    }

    fn refresh(&mut self) {
        self.diagnostic.state = if self.reset_required {
            State::ResetRequired
        } else if !self.blocking() {
            State::Observing
        } else if self.diagnostic.recovery_samples > 0
            || self
                .diagnostic
                .clear_sweep
                .as_ref()
                .is_some_and(|c| c.confirmation_samples > 0)
        {
            State::Confirming
        } else {
            State::Holding
        };
        self.diagnostic.held_identities = self.held.iter().copied().collect();
        self.diagnostic.retained_references = self.references.values().map(VecDeque::len).sum();
    }

    /// An uncovered motion epoch cannot share identities or reference geometry
    /// with a later one. Construction of a new guard accompanies pose reset.
    pub fn invalidate_motion_epoch(&mut self, now: f64) {
        self.references.clear();
        self.fault(now, Fault::MotionDiscontinuity, true);
        self.refresh();
    }

    /// `scan` is the last accepted acquisition stamp. Measurements and forecasts
    /// must share continuous tracking coordinates. A cached stamp cannot vote.
    /// `healthy` reports upstream sensor/localization health, excluding this
    /// guard's own hold. Independent navigation/rule faults keep their own brake
    /// obligations after measured recovery.
    pub fn observe(
        &mut self,
        now: f64,
        scan: Option<f64>,
        measurements: &[TrackMeasurement],
        tracks: &[Track],
        predictions: &[Prediction],
        healthy: bool,
    ) {
        self.observe_with_clear_sweep(
            now,
            scan,
            measurements,
            tracks,
            predictions,
            ForecastConsistencyAdmission {
                upstream_healthy: healthy,
                clear_sweep: None,
            },
        );
    }

    pub fn observe_with_clear_sweep(
        &mut self,
        now: f64,
        scan: Option<f64>,
        measurements: &[TrackMeasurement],
        tracks: &[Track],
        predictions: &[Prediction],
        admission: ForecastConsistencyAdmission,
    ) {
        let healthy = admission.upstream_healthy;
        if !now.is_finite() || now < 0. || self.previous_time.is_some_and(|last| now < last) {
            self.fault(self.previous_time.unwrap_or(0.), Fault::InvalidInput, true);
            self.refresh();
            return;
        }
        self.previous_time = Some(now);
        if self.reset_required {
            self.refresh();
            return;
        }
        if tracks.len() > MAX_IDENTITIES
            || measurements.len() > MAX_IDENTITIES
            || predictions.len() > MAX_IDENTITIES
        {
            self.fault(now, Fault::Capacity, true);
            self.refresh();
            return;
        }
        let Some(stamp) = scan else {
            if !healthy && self.blocking() {
                self.diagnostic.recovery_after_control_s = Some(now);
                self.reset_confirmation();
            }
            self.refresh();
            return;
        };
        if !stamp.is_finite() || stamp < 0. || stamp > now {
            self.fault(now, Fault::InvalidInput, false);
            self.refresh();
            return;
        }
        if !healthy {
            if self.blocking() {
                self.diagnostic.recovery_after_control_s = Some(now);
                self.reset_confirmation();
            }
            if self
                .diagnostic
                .last_scan_acquisition_s
                .is_none_or(|old| stamp > old)
            {
                self.diagnostic.last_scan_acquisition_s = Some(stamp);
            }
            self.refresh();
            return;
        }
        if self
            .diagnostic
            .last_scan_acquisition_s
            .is_some_and(|old| stamp <= old)
        {
            self.refresh();
            return;
        }
        let active: BTreeMap<_, _> = tracks.iter().map(|t| (t.id, t)).collect();
        let forecasts: BTreeMap<_, _> = predictions.iter().map(|p| (p.id, p)).collect();
        let acquired: BTreeSet<_> = measurements.iter().map(|m| m.id).collect();
        let invalid = active.len() != tracks.len()
            || forecasts.len() != predictions.len()
            || acquired.len() != measurements.len()
            || active.keys().ne(forecasts.keys())
            || tracks.iter().any(|t| {
                !t.position.finite()
                    || !t.velocity.finite()
                    || !t.radius.is_finite()
                    || t.radius < 0.
                    || !t.last_seen.is_finite()
                    || t.last_seen < 0.
                    || t.last_seen > stamp
            })
            || predictions.iter().any(|p| {
                p.positions.is_empty()
                    || p.positions.len() > MAX_KNOTS
                    || !p.dt.is_finite()
                    || p.dt <= 0.
                    || !p.valid_radii()
                    || !(p.dt * (p.positions.len() - 1) as f64).is_finite()
                    || p.positions.iter().any(|position| !position.finite())
            })
            || measurements.iter().any(|m| {
                m.stamp != stamp
                    || !m.center.finite()
                    || !m.radius.is_finite()
                    || m.radius < 0.
                    || active.get(&m.id).is_none_or(|t| t.last_seen != stamp)
            });
        self.diagnostic.last_scan_acquisition_s = Some(stamp);
        if invalid {
            self.fault(now, Fault::InvalidInput, false);
            self.refresh();
            return;
        }
        let clear_evidence = admission
            .clear_sweep
            .filter(|_| self.diagnostic.clear_sweep.is_some());
        if let Some(proof) = &clear_evidence {
            if proof.acquisition_s != stamp
                || stamp != now
                || !measurements.is_empty()
                || !(64..=4096).contains(&proof.ray_count)
                || ![
                    proof.checked_center_radius_m,
                    proof.minimum_resolved_disc_radius_m,
                    proof.maximum_angular_gap_rad,
                    proof.maximum_transverse_gap_m,
                ]
                .iter()
                .all(|v| v.is_finite())
                || proof.checked_center_radius_m <= 0.
                || proof.minimum_resolved_disc_radius_m <= 0.
                || proof.maximum_angular_gap_rad <= 0.
                || proof.maximum_transverse_gap_m < 0.
                || proof.maximum_transverse_gap_m >= proof.minimum_resolved_disc_radius_m
            {
                self.fault(now, Fault::InvalidInput, false);
                self.refresh();
                return;
            }
            self.diagnostic.clear_sweep.as_mut().unwrap().last_evidence = Some(proof.clone());
        }
        let mut checked = BTreeSet::new();
        let mut misses = Vec::new();
        for measured in measurements {
            let reference = self.references.get(&measured.id).and_then(|history| {
                history.iter().rev().find(|r| {
                    r.acquisition < stamp
                        && r.origin <= stamp
                        && stamp - r.origin
                            <= r.prediction.dt * (r.prediction.positions.len() - 1) as f64
                })
            });
            let Some(reference) = reference else {
                continue;
            };
            let elapsed = stamp - reference.origin;
            let step = elapsed / reference.prediction.dt;
            let index = (step.floor() as usize).min(reference.prediction.positions.len() - 1);
            let next = (index + 1).min(reference.prediction.positions.len() - 1);
            let a = reference.prediction.positions[index];
            let center = a.plus(
                reference.prediction.positions[next]
                    .minus(a)
                    .scaled((step - index as f64).clamp(0., 1.)),
            );
            let radius = reference.prediction.radius_at(elapsed);
            let shortfall = center.distance(measured.center) + measured.radius - radius;
            if !shortfall.is_finite() {
                self.fault(now, Fault::InvalidInput, false);
                self.refresh();
                return;
            }
            checked.insert(measured.id);
            if shortfall > 1e-8 {
                misses.push(ForecastConsistencyMiss {
                    id: measured.id,
                    reference_origin_s: reference.origin,
                    reference_acquisition_s: reference.acquisition,
                    measurement_acquisition_s: stamp,
                    predicted_center: center,
                    predicted_radius_m: radius,
                    measured_center: measured.center,
                    measured_radius_m: measured.radius,
                    shortfall_m: shortfall,
                });
            }
        }
        self.diagnostic.evaluated_acquisition_s = Some(stamp);
        self.diagnostic.evaluated_at_control_s = Some(now);
        self.diagnostic.checked_measurements = checked.len();
        self.diagnostic.unmonitored_measurements = measurements.len() - checked.len();
        self.diagnostic.missed_measurements = misses.len();
        self.diagnostic.complete_scene_check = active.keys().all(|id| checked.contains(id))
            && self.held.iter().all(|id| checked.contains(id));
        if !misses.is_empty() {
            self.held.extend(misses.iter().map(|m| m.id));
            self.fault(now, Fault::MeasuredMiss, false);
            if self.held.len() > MAX_IDENTITIES {
                self.held = self.held.iter().take(MAX_IDENTITIES).copied().collect();
                self.fault(now, Fault::Capacity, true);
            }
        } else if self.blocking() {
            if clear_evidence.is_some()
                && self.diagnostic.fault == Some(Fault::MeasuredMiss)
                && !self.held.is_empty()
                && self
                    .diagnostic
                    .recovery_after_control_s
                    .is_some_and(|after| stamp > after)
            {
                self.diagnostic.recovery_samples = 0;
                self.diagnostic.recovery_first_acquisition_s = None;
                self.last_recovery_acquisition = None;
                if self
                    .diagnostic
                    .clear_sweep
                    .as_ref()
                    .unwrap()
                    .last_acquisition_s
                    .is_none_or(|last| stamp - last > 0.35 + 1e-9)
                {
                    self.reset_clear_confirmation();
                    self.diagnostic
                        .clear_sweep
                        .as_mut()
                        .unwrap()
                        .first_acquisition_s = Some(stamp);
                }
                let clear = self.diagnostic.clear_sweep.as_mut().unwrap();
                clear.confirmation_samples = clear.confirmation_samples.saturating_add(1).min(3);
                clear.last_acquisition_s = Some(stamp);
                if clear.confirmation_samples >= 3
                    && stamp - clear.first_acquisition_s.unwrap() >= 0.2 - 1e-9
                {
                    clear.release_count = clear.release_count.saturating_add(1);
                    clear.last_release = Some(ClearSweepRelease {
                        identities: self.held.iter().copied().collect(),
                        control_time_s: now,
                        first_acquisition_s: clear.first_acquisition_s.unwrap(),
                        last_acquisition_s: stamp,
                        acquisition_count: clear.confirmation_samples,
                    });
                    self.release();
                }
            } else if self.diagnostic.complete_scene_check
                && self
                    .diagnostic
                    .recovery_after_control_s
                    .is_some_and(|after| stamp > after)
            {
                self.reset_clear_confirmation();
                if self
                    .last_recovery_acquisition
                    .is_none_or(|last| stamp - last > 0.35 + 1e-9)
                {
                    self.reset_confirmation();
                    self.diagnostic.recovery_first_acquisition_s = Some(stamp);
                }
                self.diagnostic.recovery_samples =
                    self.diagnostic.recovery_samples.saturating_add(1).min(3);
                self.last_recovery_acquisition = Some(stamp);
                if self.diagnostic.recovery_samples >= 3
                    && self
                        .diagnostic
                        .recovery_first_acquisition_s
                        .is_some_and(|first| stamp - first >= 0.2 - 1e-9)
                {
                    self.release();
                }
            } else {
                self.reset_confirmation();
            }
        }
        self.diagnostic.misses = misses;
        self.references.retain(|id, _| active.contains_key(id));
        for measured in measurements {
            let history = self.references.entry(measured.id).or_default();
            if history.len() == MAX_REFERENCES {
                history.pop_front();
            }
            history.push_back(Reference {
                origin: now,
                acquisition: stamp,
                prediction: (*forecasts[&measured.id]).clone(),
            });
        }
        self.refresh();
    }
}
