//! Explicit caller-clock freshness supervision. No wall clock, threads or drive permission.
use crate::{Camera, Pose, Tracker, Update};
use serde::Serialize;

const MAX_AGE_S: f64 = 0.2;

pub enum Observation<'a> {
    Idle,
    Failed,
    Depth { timestamp: f64, depth_m: &'a [f64] },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TemporalStatus {
    Waiting,
    Fresh,
    Hold,
    Lost,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Reason {
    AcquisitionFailed,
    InvalidAcquisitionTime,
    DuplicateOrOlderAcquisition,
    StaleAcquisition,
    BeforeFaultEpoch,
    EstimatorRejected,
    AcceptedPoseExpired,
}

#[derive(Debug, Serialize)]
pub struct SupervisedUpdate {
    pub now: f64,
    pub status: TemporalStatus,
    pub reason: Option<Reason>,
    pub last_accepted_timestamp: Option<f64>,
    pub accepted_age_s: Option<f64>,
    pub last_observed_acquisition: Option<f64>,
    pub fault_epoch: Option<f64>,
    /// Cached optical pose only while temporally fresh; not an accuracy guarantee.
    pub first_from_last_accepted: Option<Pose>,
    /// Actual estimator work only. Idle/failure/metadata rejection never fabricates depth.
    pub measurement: Option<Update>,
}

pub struct Supervisor {
    tracker: Tracker,
    last_now: Option<f64>,
    last_observed: Option<f64>,
    last_accepted: Option<f64>,
    last_pose: Option<Pose>,
    status: TemporalStatus,
    reason: Option<Reason>,
    fault_epoch: Option<f64>,
}

impl Supervisor {
    pub fn new(camera: Camera) -> Result<Self, String> {
        Ok(Self {
            tracker: Tracker::new(camera)?,
            last_now: None,
            last_observed: None,
            last_accepted: None,
            last_pose: None,
            status: TemporalStatus::Waiting,
            reason: None,
            fault_epoch: None,
        })
    }

    /// `now` and acquisition timestamps must share one monotonic epoch.
    /// Invalid receipt clocks return Err before any state mutation; callers must hold.
    /// Loss is terminal for this instance. There is no implicit map/origin reset.
    pub fn tick(
        &mut self,
        now: f64,
        observation: Observation<'_>,
    ) -> Result<SupervisedUpdate, String> {
        if !now.is_finite()
            || !(0.0..=1e12).contains(&now)
            || self.last_now.is_some_and(|previous| now <= previous)
        {
            return Err(
                "invalid, duplicate or regressing supervisor clock; caller must hold".into(),
            );
        }
        self.last_now = Some(now);
        if self
            .last_accepted
            .is_some_and(|stamp| now - stamp > MAX_AGE_S)
        {
            self.status = TemporalStatus::Lost;
            self.reason = Some(Reason::AcceptedPoseExpired);
        }
        let mut measurement = None;
        if self.status != TemporalStatus::Lost {
            match observation {
                Observation::Idle => {}
                Observation::Failed => self.hold(now, Reason::AcquisitionFailed),
                Observation::Depth { timestamp, depth_m } => {
                    if !timestamp.is_finite() || timestamp < 0.0 || timestamp > now {
                        self.hold(now, Reason::InvalidAcquisitionTime);
                    } else if self.last_observed.is_some_and(|stamp| timestamp <= stamp) {
                        self.hold(now, Reason::DuplicateOrOlderAcquisition);
                    } else {
                        self.last_observed = Some(timestamp);
                        if now - timestamp > MAX_AGE_S {
                            self.hold(now, Reason::StaleAcquisition);
                        } else if self.fault_epoch.is_some_and(|epoch| timestamp <= epoch) {
                            // Quarantined backlog does not move the fault epoch forward.
                            self.reason = Some(Reason::BeforeFaultEpoch);
                        } else {
                            let update = self.tracker.step(timestamp, depth_m);
                            if update.accepted {
                                self.last_accepted = Some(timestamp);
                                self.last_pose = update.first_from_current;
                                self.status = TemporalStatus::Fresh;
                                self.reason = None;
                                self.fault_epoch = None;
                            } else if update.lost {
                                self.status = TemporalStatus::Lost;
                                self.reason = Some(Reason::AcceptedPoseExpired);
                            } else {
                                self.hold(now, Reason::EstimatorRejected);
                            }
                            measurement = Some(update);
                        }
                    }
                }
            }
        }
        Ok(SupervisedUpdate {
            now,
            status: self.status,
            reason: self.reason,
            last_accepted_timestamp: self.last_accepted,
            accepted_age_s: self.last_accepted.map(|stamp| now - stamp),
            last_observed_acquisition: self.last_observed,
            fault_epoch: self.fault_epoch,
            first_from_last_accepted: if self.status == TemporalStatus::Fresh {
                self.last_pose
            } else {
                None
            },
            measurement,
        })
    }

    fn hold(&mut self, now: f64, reason: Reason) {
        self.status = TemporalStatus::Hold;
        self.reason = Some(reason);
        self.fault_epoch = Some(now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn camera() -> Camera {
        Camera {
            width: 128,
            height: 96,
            fx: 110.,
            fy: 110.,
            cx: 63.5,
            cy: 47.5,
        }
    }
    fn depth() -> Vec<f64> {
        (0..128 * 96)
            .map(|i| {
                1.5 + 0.12 * ((i % 128) as f64 / 9.).sin() + 0.08 * ((i / 128) as f64 / 11.).cos()
            })
            .collect()
    }
    fn frame(timestamp: f64, depth_m: &[f64]) -> Observation<'_> {
        Observation::Depth { timestamp, depth_m }
    }

    #[test]
    fn silence_expires_without_fabricated_acquisitions_and_never_revives() {
        let mut s = Supervisor::new(camera()).unwrap();
        let d = depth();
        assert_eq!(
            s.tick(0., frame(0., &d)).unwrap().status,
            TemporalStatus::Fresh
        );
        let boundary = s.tick(0.2, Observation::Idle).unwrap();
        assert_eq!(boundary.status, TemporalStatus::Fresh);
        assert!(boundary.measurement.is_none());
        let lost = s.tick(0.201, Observation::Idle).unwrap();
        assert_eq!(lost.status, TemporalStatus::Lost);
        assert!(lost.first_from_last_accepted.is_none());
        assert_eq!(s.tracker.last_observed, Some(0.));
        let later = s.tick(0.21, frame(0.21, &d)).unwrap();
        assert_eq!(later.status, TemporalStatus::Lost);
        assert!(later.measurement.is_none());
        assert_eq!(s.tracker.last_accepted, Some(0.));
    }

    #[test]
    fn fault_epoch_quarantines_backlog_then_fresh_fit_recovers_same_origin() {
        let mut s = Supervisor::new(camera()).unwrap();
        let d = depth();
        s.tick(1., frame(1., &d)).unwrap();
        assert_eq!(
            s.tick(1.03, Observation::Failed).unwrap().status,
            TemporalStatus::Hold
        );
        let old = s.tick(1.04, frame(1.02, &d)).unwrap();
        assert_eq!(old.reason, Some(Reason::BeforeFaultEpoch));
        assert_eq!(old.fault_epoch, Some(1.03));
        assert!(old.measurement.is_none() && old.first_from_last_accepted.is_none());
        assert_eq!(s.tracker.last_observed, Some(1.));
        let fresh = s.tick(1.06, frame(1.05, &d)).unwrap();
        assert_eq!(fresh.status, TemporalStatus::Fresh);
        assert_eq!(fresh.last_accepted_timestamp, Some(1.05));
        assert!(fresh.fault_epoch.is_none());
        assert!(
            fresh
                .first_from_last_accepted
                .unwrap()
                .translation
                .iter()
                .all(|v| v.abs() < 1e-12)
        );
    }

    #[test]
    fn malformed_measurement_cannot_corrupt_the_next_fit_or_refresh_age() {
        let d = depth();
        let mut s = Supervisor::new(camera()).unwrap();
        let mut clean = Tracker::new(camera()).unwrap();
        s.tick(1., frame(1., &d)).unwrap();
        clean.step(1., &d);
        let bad = s.tick(1.02, frame(1.019, &[])).unwrap();
        assert_eq!(bad.status, TemporalStatus::Hold);
        assert_eq!(bad.last_accepted_timestamp, Some(1.));
        let recovered = s.tick(1.04, frame(1.039, &d)).unwrap();
        assert_eq!(recovered.status, TemporalStatus::Fresh);
        let expected = clean.step(1.039, &d);
        assert_eq!(
            serde_json::to_value(recovered.measurement.unwrap()).unwrap(),
            serde_json::to_value(expected).unwrap()
        );
    }

    #[test]
    fn invalid_receipt_clocks_leave_all_state_unchanged() {
        let mut s = Supervisor::new(camera()).unwrap();
        let d = depth();
        s.tick(1., frame(1., &d)).unwrap();
        for now in [f64::NAN, f64::INFINITY, -1., 0.9, 1., 1e13] {
            assert!(s.tick(now, Observation::Failed).is_err());
            assert_eq!(s.last_now, Some(1.));
            assert_eq!(s.status, TemporalStatus::Fresh);
            assert!(s.fault_epoch.is_none());
            assert_eq!(s.tracker.last_accepted, Some(1.));
        }
        assert_eq!(
            s.tick(1.01, Observation::Idle).unwrap().status,
            TemporalStatus::Fresh
        );
    }

    #[test]
    fn future_invalid_duplicate_and_stale_acquisitions_do_not_fit_or_refresh() {
        let d = depth();
        for stamp in [f64::NAN, f64::INFINITY, -1., 2., 1.] {
            let mut s = Supervisor::new(camera()).unwrap();
            s.tick(1., frame(1., &d)).unwrap();
            let bad = s.tick(1.01, frame(stamp, &d)).unwrap();
            assert_eq!(bad.status, TemporalStatus::Hold);
            assert!(bad.measurement.is_none());
            assert_eq!(s.tracker.last_observed, Some(1.));
        }
        let mut s = Supervisor::new(camera()).unwrap();
        let stale = s.tick(1., frame(0., &d)).unwrap();
        assert_eq!(stale.reason, Some(Reason::StaleAcquisition));
        assert!(s.tracker.model.is_none());
    }

    #[test]
    fn repeated_faults_and_rejected_depth_cannot_postpone_silence_loss() {
        let d = depth();
        let mut s = Supervisor::new(camera()).unwrap();
        s.tick(1., frame(1., &d)).unwrap();
        for now in [1.05, 1.1, 1.15] {
            assert_eq!(
                s.tick(now, Observation::Failed).unwrap().status,
                TemporalStatus::Hold
            );
        }
        let expired = s.tick(1.201, frame(1.2, &d)).unwrap();
        assert_eq!(expired.status, TemporalStatus::Lost);
        assert!(expired.measurement.is_none());
        assert_eq!(expired.last_accepted_timestamp, Some(1.));
    }
}
