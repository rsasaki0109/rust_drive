//! Image-space tracking. Positions are pixels, velocities are pixels/second.
//! A track ID is a correspondence hypothesis, not a physical-object guarantee.
mod assignment;

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

pub const MAX_DETECTIONS: usize = 300;
pub const MAX_TRACKS: usize = 300;
pub const IMAGE_HISTORY: usize = 256;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Detection {
    pub class_index: usize,
    pub confidence: f64,
    /// Original image pixels: [left, top, right, bottom].
    pub bbox: [f64; 4],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub min_iou: f64,
    pub coast_seconds: f64,
    pub confirmation_hits: u32,
    pub velocity_gain: f64,
    pub max_tracks: usize,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            min_iou: 0.3,
            coast_seconds: 0.5,
            confirmation_hits: 2,
            velocity_gain: 0.5,
            max_tracks: MAX_TRACKS,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case", deny_unknown_fields)]
pub enum Acquisition {
    Detected {
        image_sha256: String,
        detections: Vec<Detection>,
    },
    Missing,
    Failed,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Frame {
    pub timestamp_s: f64,
    pub acquisition: Acquisition,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    Tentative,
    Confirmed,
    Coasting,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Estimate {
    pub track_id: u64,
    pub class_index: usize,
    pub phase: Phase,
    /// Predicted boxes can extend outside the image; they are never observations.
    pub bbox: [f64; 4],
    pub velocity_px_s: [f64; 2],
    pub last_observed_bbox: [f64; 4],
    pub last_observed_timestamp_s: f64,
    pub last_observed_confidence: f64,
    pub observation_count: u32,
    pub detection_index: Option<usize>,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Health {
    Observed,
    Missing,
    AcquisitionFailed,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RetirementReason {
    Expired,
    TentativeMiss,
    AcquisitionFailed,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Retirement {
    pub track_id: u64,
    pub reason: RetirementReason,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Snapshot {
    pub sequence_number: u64,
    pub timestamp_s: f64,
    pub health: Health,
    pub source_image_sha256: Option<String>,
    pub tracks: Vec<Estimate>,
    pub retired: Vec<Retirement>,
    pub untracked_detection_indices: Vec<usize>,
    pub assignment_work: usize,
}

#[derive(Clone, Debug, PartialEq)]
struct Track {
    id: u64,
    observed: Detection,
    last_seen: f64,
    velocity: [f64; 2],
    hits: u32,
    confirmed: bool,
}
impl Track {
    fn predicted(&self, timestamp: f64) -> [f64; 4] {
        let age = timestamp - self.last_seen;
        let [l, t, r, b] = self.observed.bbox;
        let [vx, vy] = self.velocity;
        [l + vx * age, t + vy * age, r + vx * age, b + vy * age]
    }
    fn estimate(&self, timestamp: f64, index: Option<usize>) -> Estimate {
        Estimate {
            track_id: self.id,
            class_index: self.observed.class_index,
            phase: if !self.confirmed {
                Phase::Tentative
            } else if index.is_some() {
                Phase::Confirmed
            } else {
                Phase::Coasting
            },
            bbox: self.predicted(timestamp),
            velocity_px_s: self.velocity,
            last_observed_bbox: self.observed.bbox,
            last_observed_timestamp_s: self.last_seen,
            last_observed_confidence: self.observed.confidence,
            observation_count: self.hits,
            detection_index: index,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Tracker {
    width: u32,
    height: u32,
    config: Config,
    last_timestamp: Option<f64>,
    sequence: u64,
    next_id: u64,
    tracks: Vec<Track>,
    recent_images: VecDeque<String>,
}

pub fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|v| v.is_ascii_digit() || (b'a'..=b'f').contains(&v))
}

impl Tracker {
    pub fn new(width: u32, height: u32, config: Config) -> Result<Self, String> {
        if width == 0
            || height == 0
            || width > 16384
            || height > 16384
            || u64::from(width) * u64::from(height) > 16_000_000
        {
            return Err("invalid fixed image dimensions".into());
        }
        if !config.min_iou.is_finite()
            || !(0.01..=1.).contains(&config.min_iou)
            || !config.coast_seconds.is_finite()
            || !(0.001..=2.).contains(&config.coast_seconds)
            || !(1..=10).contains(&config.confirmation_hits)
            || !config.velocity_gain.is_finite()
            || !(0.0..=1.).contains(&config.velocity_gain)
            || !(1..=MAX_TRACKS).contains(&config.max_tracks)
        {
            return Err("invalid bounded tracker configuration".into());
        }
        Ok(Self {
            width,
            height,
            config,
            last_timestamp: None,
            sequence: 0,
            next_id: 1,
            tracks: vec![],
            recent_images: VecDeque::new(),
        })
    }

    /// Errors leave all state unchanged. Callers must not reuse a prior snapshot
    /// as a current observation when this method returns Err.
    pub fn step(&mut self, frame: &Frame) -> Result<Snapshot, String> {
        if !frame.timestamp_s.is_finite()
            || frame.timestamp_s < 0.
            || self
                .last_timestamp
                .is_some_and(|last| frame.timestamp_s - last < 0.000001)
        {
            return Err("clock must advance by at least one microsecond".into());
        }
        if let Acquisition::Detected {
            image_sha256,
            detections,
        } = &frame.acquisition
        {
            if !valid_sha256(image_sha256) || self.recent_images.contains(image_sha256) {
                return Err("invalid or reused image identity in recent 256 acquisitions".into());
            }
            if detections.len() > MAX_DETECTIONS {
                return Err("detection count budget exceeded".into());
            }
            for d in detections {
                let [l, t, r, b] = d.bbox;
                if d.class_index >= 80
                    || !d.confidence.is_finite()
                    || !(0.0..=1.).contains(&d.confidence)
                    || d.bbox.iter().any(|n| !n.is_finite())
                    || l < 0.
                    || t < 0.
                    || r <= l
                    || b <= t
                    || r > f64::from(self.width)
                    || b > f64::from(self.height)
                {
                    return Err("invalid detection class, confidence or image bounds".into());
                }
            }
        }
        let mut next = self.clone();
        let result = next.advance(frame)?;
        *self = next;
        Ok(result)
    }

    fn advance(&mut self, frame: &Frame) -> Result<Snapshot, String> {
        self.sequence = self.sequence.checked_add(1).ok_or("sequence exhausted")?;
        self.last_timestamp = Some(frame.timestamp_s);
        let mut retired = vec![];
        if frame.acquisition == Acquisition::Failed {
            retired.extend(self.tracks.drain(..).map(|t| Retirement {
                track_id: t.id,
                reason: RetirementReason::AcquisitionFailed,
            }));
            return Ok(Snapshot {
                sequence_number: self.sequence,
                timestamp_s: frame.timestamp_s,
                health: Health::AcquisitionFailed,
                source_image_sha256: None,
                tracks: vec![],
                retired,
                untracked_detection_indices: vec![],
                assignment_work: 0,
            });
        }
        let (detections, image_sha) = match &frame.acquisition {
            Acquisition::Detected {
                image_sha256,
                detections,
            } => (detections.as_slice(), Some(image_sha256.clone())),
            _ => (&[][..], None),
        };
        self.tracks.retain(|t| {
            let keep = frame.timestamp_s - t.last_seen <= self.config.coast_seconds;
            if !keep {
                retired.push(Retirement {
                    track_id: t.id,
                    reason: RetirementReason::Expired,
                });
            }
            keep
        });
        let boxes: Vec<_> = self
            .tracks
            .iter()
            .map(|t| (t.observed.class_index, t.predicted(frame.timestamp_s)))
            .collect();
        let (matches, assignment_work) =
            assignment::associate(&boxes, detections, self.config.min_iou)?;
        let mut used = vec![false; detections.len()];
        let mut kept = vec![];
        let mut estimates = vec![];
        for (mut track, matched) in self.tracks.drain(..).zip(matches) {
            if let Some(index) = matched {
                used[index] = true;
                let d = &detections[index];
                let dt = frame.timestamp_s - track.last_seen;
                let gain = if track.hits == 1 {
                    1.
                } else {
                    self.config.velocity_gain
                };
                for axis in 0..2 {
                    let old = (track.observed.bbox[axis] + track.observed.bbox[axis + 2]) * 0.5;
                    let new = (d.bbox[axis] + d.bbox[axis + 2]) * 0.5;
                    track.velocity[axis] =
                        (1. - gain) * track.velocity[axis] + gain * (new - old) / dt;
                }
                track.observed = d.clone();
                track.last_seen = frame.timestamp_s;
                track.hits = track.hits.saturating_add(1);
                track.confirmed |= track.hits >= self.config.confirmation_hits;
            } else if !track.confirmed {
                retired.push(Retirement {
                    track_id: track.id,
                    reason: RetirementReason::TentativeMiss,
                });
                continue;
            }
            estimates.push(track.estimate(frame.timestamp_s, matched));
            kept.push(track);
        }
        let mut new: Vec<_> = (0..detections.len()).filter(|&i| !used[i]).collect();
        new.sort_by(|&a, &b| {
            detections[b]
                .confidence
                .total_cmp(&detections[a].confidence)
                .then(a.cmp(&b))
        });
        let mut untracked_detection_indices = vec![];
        for index in new {
            if kept.len() == self.config.max_tracks {
                untracked_detection_indices.push(index);
                continue;
            }
            let track = Track {
                id: self.next_id,
                observed: detections[index].clone(),
                last_seen: frame.timestamp_s,
                velocity: [0.; 2],
                hits: 1,
                confirmed: self.config.confirmation_hits == 1,
            };
            self.next_id = self.next_id.checked_add(1).ok_or("track ID exhausted")?;
            estimates.push(track.estimate(frame.timestamp_s, Some(index)));
            kept.push(track);
        }
        self.tracks = kept;
        if let Some(sha) = &image_sha {
            self.recent_images.push_back(sha.clone());
            if self.recent_images.len() > IMAGE_HISTORY {
                self.recent_images.pop_front();
            }
        }
        retired.sort_by_key(|r| r.track_id);
        untracked_detection_indices.sort_unstable();
        Ok(Snapshot {
            sequence_number: self.sequence,
            timestamp_s: frame.timestamp_s,
            health: if image_sha.is_some() {
                Health::Observed
            } else {
                Health::Missing
            },
            source_image_sha256: image_sha,
            tracks: estimates,
            retired,
            untracked_detection_indices,
            assignment_work,
        })
    }
}
