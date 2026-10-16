//! Two-clock result admission. Availability here is not a vehicle-control permit.
use rustdriving_image_tracking::{Acquisition, Config, Detection, Frame, Phase, Snapshot, Tracker};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Policy {
    pub max_age_s: f64,
    pub recovery_samples: u32,
}
impl Default for Policy {
    fn default() -> Self {
        Self {
            max_age_s: 0.2,
            recovery_samples: 2,
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Input {
    Sample {
        acquired_s: f64,
        image_sha256: String,
        detections: Vec<Detection>,
    },
    Tick,
    Failed,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Event {
    pub now_s: f64,
    pub input: Input,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldReason {
    Initializing,
    AcquisitionFailed,
    DeadlineExceeded,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Decision {
    Accepted,
    Polled,
    AcquisitionFailed,
    InvalidAcquisitionTime,
    FutureSample,
    BeforeFault,
    OutOfOrder,
    StaleSample,
    InvalidPayload,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Trigger {
    Sample,
    AcquisitionFailure,
    DeadlineExpiry,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct BackendUpdate {
    pub trigger: Trigger,
    pub frame: Frame,
    pub snapshot: Snapshot,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ProjectedTrack {
    pub track_id: u64,
    pub class_index: usize,
    pub bbox_at_now: [f64; 4],
    pub velocity_px_s: [f64; 2],
    pub observed_at_s: f64,
    pub observation_age_s: f64,
    pub last_observed_confidence: f64,
    pub extrapolated: bool,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Output {
    pub now_s: f64,
    pub decision: Decision,
    pub ready: bool,
    pub hold_reason: Option<HoldReason>,
    pub fault_epoch_s: Option<f64>,
    pub recovery_count: u32,
    pub last_accepted_acquired_s: Option<f64>,
    pub result_age_s: Option<f64>,
    pub backend_updates: Vec<BackendUpdate>,
    /// Only confirmed, age-bounded hypotheses are exposed, and only when ready.
    pub tracks: Vec<ProjectedTrack>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Guard {
    tracker: Tracker,
    tracker_config: Config,
    policy: Policy,
    last_now: Option<f64>,
    last_acquired: Option<f64>,
    armed: bool,
    fault_epoch: Option<f64>,
    hold_reason: Option<HoldReason>,
    recovery_count: u32,
    last_snapshot: Option<Snapshot>,
}
fn clock(value: f64) -> bool {
    value.is_finite() && (0.0..=1e12).contains(&value)
}

impl Guard {
    pub fn new(width: u32, height: u32, config: Config, policy: Policy) -> Result<Self, String> {
        if !policy.max_age_s.is_finite()
            || !(0.001..=2.).contains(&policy.max_age_s)
            || !(1..=10).contains(&policy.recovery_samples)
        {
            return Err("invalid bounded freshness policy".into());
        }
        Ok(Self {
            tracker: Tracker::new(width, height, config.clone())?,
            tracker_config: config,
            policy,
            last_now: None,
            last_acquired: None,
            armed: false,
            fault_epoch: None,
            hold_reason: Some(HoldReason::Initializing),
            recovery_count: 0,
            last_snapshot: None,
        })
    }

    /// Invalid delivery clocks are fatal/atomic. A rejected sample advances
    /// delivery time and can trigger expiry, but never refreshes acquisition.
    pub fn step(&mut self, event: &Event) -> Result<Output, String> {
        if !clock(event.now_s)
            || self
                .last_now
                .is_some_and(|last| event.now_s - last < 0.000001)
        {
            return Err(
                "delivery clock must be finite, in [0,1e12], and advance at least one microsecond"
                    .into(),
            );
        }
        let mut next = self.clone();
        let result = next.advance(event)?;
        *self = next;
        Ok(result)
    }

    fn update(
        &mut self,
        frame: Frame,
        trigger: Trigger,
        updates: &mut Vec<BackendUpdate>,
    ) -> Result<(), String> {
        let snapshot = self.tracker.step(&frame)?;
        self.last_snapshot = Some(snapshot.clone());
        updates.push(BackendUpdate {
            trigger,
            frame,
            snapshot,
        });
        Ok(())
    }
    fn fault(
        &mut self,
        at: f64,
        reason: HoldReason,
        updates: &mut Vec<BackendUpdate>,
    ) -> Result<(), String> {
        self.update(
            Frame {
                timestamp_s: at,
                acquisition: Acquisition::Failed,
            },
            if reason == HoldReason::DeadlineExceeded {
                Trigger::DeadlineExpiry
            } else {
                Trigger::AcquisitionFailure
            },
            updates,
        )?;
        self.armed = false;
        self.fault_epoch = Some(at);
        self.hold_reason = Some(reason);
        self.recovery_count = 0;
        self.last_snapshot = None;
        Ok(())
    }
    fn reject(&mut self, decision: Decision) -> Decision {
        if self.hold_reason.is_some() {
            self.recovery_count = 0;
        }
        decision
    }

    fn advance(&mut self, event: &Event) -> Result<Output, String> {
        self.last_now = Some(event.now_s);
        let mut updates = Vec::new();
        let decision = if event.input == Input::Failed {
            self.fault(event.now_s, HoldReason::AcquisitionFailed, &mut updates)?;
            Decision::AcquisitionFailed
        } else {
            if self.armed {
                let deadline =
                    self.last_acquired.ok_or("missing armed acquisition")? + self.policy.max_age_s;
                if event.now_s > deadline {
                    // Latch the actual deadline once, not every later poll time.
                    self.fault(deadline, HoldReason::DeadlineExceeded, &mut updates)?;
                }
            }
            match &event.input {
                Input::Tick => Decision::Polled,
                Input::Sample {
                    acquired_s,
                    image_sha256,
                    detections,
                } => {
                    if !clock(*acquired_s) {
                        self.reject(Decision::InvalidAcquisitionTime)
                    } else if *acquired_s > event.now_s {
                        self.reject(Decision::FutureSample)
                    } else if self
                        .fault_epoch
                        .is_some_and(|barrier| *acquired_s <= barrier)
                    {
                        self.reject(Decision::BeforeFault)
                    } else if self
                        .last_acquired
                        .is_some_and(|last| *acquired_s - last < 0.000001)
                    {
                        self.reject(Decision::OutOfOrder)
                    } else if event.now_s > *acquired_s + self.policy.max_age_s {
                        self.reject(Decision::StaleSample)
                    } else {
                        let frame = Frame {
                            timestamp_s: *acquired_s,
                            acquisition: Acquisition::Detected {
                                image_sha256: image_sha256.clone(),
                                detections: detections.clone(),
                            },
                        };
                        // The original tracker's complete payload/hash checks are
                        // atomic; bad deliveries cannot mutate its IDs/history.
                        match self.tracker.step(&frame) {
                            Err(_) => self.reject(Decision::InvalidPayload),
                            Ok(snapshot) => {
                                self.last_snapshot = Some(snapshot.clone());
                                updates.push(BackendUpdate {
                                    trigger: Trigger::Sample,
                                    frame,
                                    snapshot,
                                });
                                self.last_acquired = Some(*acquired_s);
                                self.armed = true;
                                self.recovery_count =
                                    (self.recovery_count + 1).min(self.policy.recovery_samples);
                                if self.recovery_count >= self.policy.recovery_samples {
                                    self.hold_reason = None;
                                }
                                Decision::Accepted
                            }
                        }
                    }
                }
                Input::Failed => unreachable!(),
            }
        };
        let ready = self.hold_reason.is_none();
        let mut tracks = Vec::new();
        if ready {
            let age_limit = self.policy.max_age_s.min(self.tracker_config.coast_seconds);
            if let Some(snapshot) = &self.last_snapshot {
                for track in &snapshot.tracks {
                    if track.phase == Phase::Tentative
                        || event.now_s > track.last_observed_timestamp_s + age_limit
                    {
                        continue;
                    }
                    let age = event.now_s - track.last_observed_timestamp_s;
                    tracks.push(ProjectedTrack {
                        track_id: track.track_id,
                        class_index: track.class_index,
                        bbox_at_now: std::array::from_fn(|axis| {
                            track.last_observed_bbox[axis] + track.velocity_px_s[axis % 2] * age
                        }),
                        velocity_px_s: track.velocity_px_s,
                        observed_at_s: track.last_observed_timestamp_s,
                        observation_age_s: age,
                        last_observed_confidence: track.last_observed_confidence,
                        extrapolated: age > 0.,
                    });
                }
            }
        }
        Ok(Output {
            now_s: event.now_s,
            decision,
            ready,
            hold_reason: self.hold_reason,
            fault_epoch_s: self.fault_epoch,
            recovery_count: self.recovery_count,
            last_accepted_acquired_s: self.last_acquired,
            result_age_s: self.last_acquired.map(|time| event.now_s - time),
            backend_updates: updates,
            tracks,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample(now: f64, acquired: f64, identity: u64, left: f64) -> Event {
        Event {
            now_s: now,
            input: Input::Sample {
                acquired_s: acquired,
                image_sha256: format!("{identity:064x}"),
                detections: vec![Detection {
                    class_index: 2,
                    confidence: 0.8,
                    bbox: [left, 10., left + 20., 30.],
                }],
            },
        }
    }
    fn new() -> Guard {
        Guard::new(100, 100, Config::default(), Policy::default()).unwrap()
    }
    fn warmed() -> Guard {
        let mut guard = new();
        assert!(!guard.step(&sample(0., 0., 1, 10.)).unwrap().ready);
        assert!(guard.step(&sample(0.1, 0.1, 2, 11.)).unwrap().ready);
        guard
    }
    #[test]
    fn polling_projects_without_renewing_observations_or_acquisition_clock() {
        let mut guard = warmed();
        let poll = guard
            .step(&Event {
                now_s: 0.15,
                input: Input::Tick,
            })
            .unwrap();
        assert!(poll.ready && poll.backend_updates.is_empty());
        assert_eq!(poll.last_accepted_acquired_s, Some(0.1));
        assert_eq!(poll.tracks[0].observed_at_s, 0.1);
        assert!((poll.tracks[0].bbox_at_now[0] - 11.5).abs() < 1e-10);
        assert_eq!(poll.tracks[0].last_observed_confidence, 0.8);
        let delayed = guard.step(&sample(0.18, 0.14, 3, 11.4)).unwrap();
        assert_eq!(delayed.decision, Decision::Accepted);
        assert_eq!(delayed.backend_updates[0].frame.timestamp_s, 0.14);
        assert_eq!(delayed.tracks[0].track_id, 1);
    }
    #[test]
    fn expiry_is_latched_and_only_post_fault_frames_can_recover() {
        let mut guard = warmed();
        let expired = guard
            .step(&Event {
                now_s: 0.31,
                input: Input::Tick,
            })
            .unwrap();
        let barrier = 0.1 + 0.2;
        assert_eq!(expired.fault_epoch_s, Some(barrier));
        assert_eq!(expired.hold_reason, Some(HoldReason::DeadlineExceeded));
        assert_eq!(expired.backend_updates[0].snapshot.retired[0].track_id, 1);
        let idle = guard
            .step(&Event {
                now_s: 0.6,
                input: Input::Tick,
            })
            .unwrap();
        assert_eq!(idle.fault_epoch_s, Some(barrier));
        assert!(idle.backend_updates.is_empty() && idle.tracks.is_empty());
        assert_eq!(
            guard.step(&sample(0.61, 0.2, 3, 12.)).unwrap().decision,
            Decision::BeforeFault
        );
        let one = guard.step(&sample(0.62, 0.62, 4, 12.)).unwrap();
        assert!(!one.ready && one.tracks.is_empty());
        let two = guard.step(&sample(0.7, 0.7, 5, 13.)).unwrap();
        assert!(two.ready);
        assert_eq!(two.tracks[0].track_id, 2);
    }
    #[test]
    fn fresh_arrival_after_a_gap_records_expiry_before_new_acquisition() {
        let mut guard = warmed();
        let one = guard.step(&sample(0.41, 0.4, 3, 12.)).unwrap();
        assert_eq!(one.backend_updates.len(), 2);
        assert_eq!(one.backend_updates[0].trigger, Trigger::DeadlineExpiry);
        assert_eq!(one.backend_updates[1].trigger, Trigger::Sample);
        assert!(!one.ready);
        assert!(guard.step(&sample(0.51, 0.5, 4, 13.)).unwrap().ready);
    }
    #[test]
    fn explicit_failure_clears_ids_and_queued_frames_cannot_clear_the_hold() {
        let mut guard = warmed();
        let fault = guard
            .step(&Event {
                now_s: 0.15,
                input: Input::Failed,
            })
            .unwrap();
        assert!(fault.tracks.is_empty() && !fault.ready);
        assert_eq!(fault.fault_epoch_s, Some(0.15));
        assert_eq!(
            guard.step(&sample(0.16, 0.14, 3, 12.)).unwrap().decision,
            Decision::BeforeFault
        );
        assert!(!guard.step(&sample(0.17, 0.17, 4, 12.)).unwrap().ready);
        let recovered = guard.step(&sample(0.18, 0.18, 5, 12.)).unwrap();
        assert!(recovered.ready && recovered.tracks[0].track_id > 1);
    }
    #[test]
    fn rejected_payload_does_not_refresh_time_and_breaks_recovery_streak() {
        let mut guard = new();
        guard.step(&sample(0., 0., 1, 10.)).unwrap();
        let repeated = guard.step(&sample(0.05, 0.05, 1, 11.)).unwrap();
        assert_eq!(repeated.decision, Decision::InvalidPayload);
        assert_eq!(repeated.recovery_count, 0);
        assert_eq!(repeated.last_accepted_acquired_s, Some(0.));
        assert!(!guard.step(&sample(0.1, 0.1, 2, 11.)).unwrap().ready);
        assert!(guard.step(&sample(0.15, 0.15, 3, 12.)).unwrap().ready);
        let future = guard.step(&sample(0.2, 0.3, 4, 13.)).unwrap();
        assert_eq!(future.decision, Decision::FutureSample);
        assert_eq!(future.last_accepted_acquired_s, Some(0.15));
    }
    #[test]
    fn delivery_clock_errors_are_atomic() {
        let mut guard = warmed();
        let before = guard.clone();
        for time in [0.1, 0.0, f64::NAN, f64::INFINITY, -1., 1e13] {
            assert!(
                guard
                    .step(&Event {
                        now_s: time,
                        input: Input::Failed
                    })
                    .is_err()
            );
            assert_eq!(guard, before);
        }
    }
    #[test]
    fn inclusive_deadline_is_consistent_for_relative_and_unix_clocks() {
        for epoch in [0., 1_800_000_000.] {
            let mut guard = new();
            guard.step(&sample(epoch, epoch, 1, 10.)).unwrap();
            guard
                .step(&sample(epoch + 0.125, epoch + 0.125, 2, 11.))
                .unwrap();
            let deadline = epoch + 0.125 + 0.2;
            assert!(
                guard
                    .step(&Event {
                        now_s: deadline,
                        input: Input::Tick
                    })
                    .unwrap()
                    .ready
            );
            assert!(
                !guard
                    .step(&Event {
                        now_s: deadline + 0.00001,
                        input: Input::Tick
                    })
                    .unwrap()
                    .ready
            );
        }
    }
    #[test]
    fn healthy_empty_images_can_recover_without_inventing_objects() {
        let mut guard = new();
        for i in 0..2 {
            let event = Event {
                now_s: i as f64 * 0.1,
                input: Input::Sample {
                    acquired_s: i as f64 * 0.1,
                    image_sha256: format!("{i:064x}"),
                    detections: vec![],
                },
            };
            let out = guard.step(&event).unwrap();
            assert_eq!(out.ready, i == 1);
            assert!(out.tracks.is_empty());
        }
    }
    #[test]
    fn each_track_ages_even_when_new_images_keep_camera_ready() {
        let mut guard = new();
        guard.step(&sample(0., 0., 1, 10.)).unwrap();
        guard.step(&sample(0.05, 0.05, 2, 11.)).unwrap();
        guard
            .step(&Event {
                now_s: 0.1,
                input: Input::Sample {
                    acquired_s: 0.1,
                    image_sha256: format!("{:064x}", 3),
                    detections: vec![],
                },
            })
            .unwrap();
        let out = guard
            .step(&Event {
                now_s: 0.26,
                input: Input::Tick,
            })
            .unwrap();
        assert!(out.ready && out.tracks.is_empty());
    }
    #[test]
    fn invalid_policy_is_rejected() {
        for policy in [
            Policy {
                max_age_s: f64::NAN,
                ..Policy::default()
            },
            Policy {
                max_age_s: 0.,
                ..Policy::default()
            },
            Policy {
                recovery_samples: 0,
                ..Policy::default()
            },
        ] {
            assert!(Guard::new(100, 100, Config::default(), policy).is_err());
        }
    }
}
