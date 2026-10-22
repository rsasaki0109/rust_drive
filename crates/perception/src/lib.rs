//! Range-adaptive clustering and globally associated alpha-beta tracking.
mod association;
mod clustering;
pub mod ground3d;
pub mod image_features;
mod initialization;
pub mod objects3d;
pub mod planar_sweep;
mod recent_motion;
pub mod terrain_adaptive;
pub use ground3d::{GroundClassification, TerrainConfig, TerrainDiagnostics, classify_ground};
pub use objects3d::{ObjectAabb, ObjectClassification, ObjectClusterConfig, cluster_objects};
pub use terrain_adaptive::{
    AdaptiveGroundClassification, AdaptiveTerrainConfig, AdaptiveTerrainDiagnostics,
    classify_ground_adaptive,
};

use rustdriving_core::{Detection, LidarScan, Perception, Pose, Track, TrackMeasurement, Vec2};
use std::collections::BTreeMap;
#[derive(Default)]
pub struct LidarClusters;
impl Perception for LidarClusters {
    fn detect(&mut self, scan: &LidarScan, pose: Pose) -> Vec<Detection> {
        detect_clusters(scan, pose, false)
    }
}

impl LidarClusters {
    /// Admit bounded circular hypotheses from three/four visible surface returns.
    /// Geometric support is a heuristic, not calibrated physical-shape confidence.
    pub fn detect_with_sparse_circles(&mut self, scan: &LidarScan, pose: Pose) -> Vec<Detection> {
        detect_clusters(scan, pose, true)
    }
}

fn detect_clusters(scan: &LidarScan, pose: Pose, sparse_circles: bool) -> Vec<Detection> {
    let points: Vec<_> = scan
        .points
        .iter()
        .copied()
        .filter(|p| p.finite() && p.x.hypot(p.y) > 0.3)
        .collect();
    let mut result = Vec::new();
    for queue in clustering::Components::new(&points) {
        if queue.len() < 3 {
            continue;
        }
        let mut center = Vec2::default();
        for &j in &queue {
            center = center.plus(points[j]);
        }
        center = center.scaled(1.0 / queue.len() as f64);
        // Visible surfaces underestimate centers: conservative enclosing proxy, not a shape classifier.
        let extent = queue
            .iter()
            .map(|&j| center.distance(points[j]))
            .fold(0.0, f64::max);
        let cluster: Vec<_> = queue.iter().map(|&j| points[j]).collect();
        let fitted = fit_circle(&cluster).or_else(|| {
            sparse_circles
                .then(|| fit_sparse_circle(&cluster))
                .flatten()
        });
        let (center, radius) = fitted.unwrap_or((center, (extent + 0.65).clamp(0.85, 3.0)));
        result.push(Detection {
            center: pose.to_world(center),
            radius,
        });
    }
    result
}
/// Algebraic least-squares circle fit, with bounds and residual checks.
fn fit_circle(points: &[Vec2]) -> Option<(Vec2, f64)> {
    if points.len() < 5 {
        return None;
    }
    fit_circle_parameters(points)
}

/// Sparse arcs require both a baseline and curvature above the degeneracy floor.
/// All points must lie on the near visible branch of the fitted circle.
fn fit_sparse_circle(points: &[Vec2]) -> Option<(Vec2, f64)> {
    if !(3..=4).contains(&points.len()) {
        return None;
    }
    let (mut baseline, mut twice_area) = (0_f64, 0_f64);
    for (i, a) in points.iter().enumerate() {
        for (j, b) in points.iter().enumerate().skip(i + 1) {
            baseline = baseline.max(a.distance(*b));
            for c in points.iter().skip(j + 1) {
                let u = b.minus(*a);
                let v = c.minus(*a);
                twice_area = twice_area.max((u.x * v.y - u.y * v.x).abs());
            }
        }
    }
    if baseline < 0.5 || twice_area / baseline < 0.05 {
        return None;
    }
    let (center, radius) = fit_circle_parameters(points)?;
    if points.iter().any(|p| {
        let normal = p.minus(center);
        p.x * normal.x + p.y * normal.y > 0.12 * p.x.hypot(p.y)
    }) {
        return None;
    }
    Some((center, radius))
}

fn fit_circle_parameters(points: &[Vec2]) -> Option<(Vec2, f64)> {
    let origin = points
        .iter()
        .fold(Vec2::default(), |sum, p| sum.plus(*p))
        .scaled(1.0 / points.len() as f64);
    let mut augmented = [[0.0; 4]; 3];
    for point in points {
        let q = point.minus(origin);
        let a = [2.0 * q.x, 2.0 * q.y, 1.0];
        let b = q.x * q.x + q.y * q.y;
        for i in 0..3 {
            for j in 0..3 {
                augmented[i][j] += a[i] * a[j];
            }
            augmented[i][3] += a[i] * b;
        }
    }
    for col in 0..3 {
        let pivot = (col..3)
            .max_by(|&a, &b| augmented[a][col].abs().total_cmp(&augmented[b][col].abs()))?;
        if augmented[pivot][col].abs() < 1e-8 {
            return None;
        }
        augmented.swap(col, pivot);
        let divisor = augmented[col][col];
        for j in col..4 {
            augmented[col][j] /= divisor;
        }
        for i in 0..3 {
            if i == col {
                continue;
            }
            let factor = augmented[i][col];
            for j in col..4 {
                augmented[i][j] -= factor * augmented[col][j];
            }
        }
    }
    let local = Vec2::new(augmented[0][3], augmented[1][3]);
    let radius = (augmented[2][3] + local.x * local.x + local.y * local.y).sqrt();
    let center = origin.plus(local);
    if !radius.is_finite()
        || !(0.4..=2.5).contains(&radius)
        || points
            .iter()
            .any(|p| (p.distance(center) - radius).abs() > 0.12)
    {
        return None;
    }
    Some((center, radius + 0.12))
}
#[derive(Clone, Default)]
pub struct Tracker {
    tracks: Vec<Track>,
    next_id: u64,
    accepted_scan_stamp: Option<f64>,
    confirm_initialization: bool,
    fit_recent_motion: bool,
    enclose_measurements: bool,
    pending: BTreeMap<u64, initialization::Initialization>,
    recent: BTreeMap<u64, recent_motion::RecentMotion>,
    measurements: Vec<TrackMeasurement>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackerUpdateError {
    InvalidTimestamp,
    InvalidDetection { index: usize },
    IdentifierExhausted,
    AssociationLimit,
}

impl std::fmt::Display for TrackerUpdateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidTimestamp => write!(
                f,
                "tracking timestamp must be finite, nonnegative and newer than the last accepted scan"
            ),
            Self::InvalidDetection { index } => write!(
                f,
                "tracking detection {index} requires a finite center and finite nonnegative radius"
            ),
            Self::IdentifierExhausted => write!(f, "tracking identifier space exhausted"),
            Self::AssociationLimit => {
                write!(f, "tracking association work or component limit exceeded")
            }
        }
    }
}
impl std::error::Error for TrackerUpdateError {}

impl Tracker {
    /// Refit acquired moving-object position/velocity in a bounded stable-shape window.
    /// Unsupported, static and unconfirmed slow motion retain alpha-beta updates.
    pub fn with_recent_motion_fit() -> Self {
        Self {
            fit_recent_motion: true,
            ..Self::default()
        }
    }

    /// Opt-in cold-track velocity confirmation from stable acquired detections.
    pub fn with_confirmed_initialization() -> Self {
        Self {
            confirm_initialization: true,
            ..Self::default()
        }
    }

    /// Preserve the entire newest measured detection disc at the filtered center.
    /// This is measured geometry, without a future-motion uncertainty claim.
    pub fn with_measurement_enclosure(mut self) -> Self {
        self.enclose_measurements = true;
        self
    }

    /// Raw detections from the last successful scan, in the continuous frame.
    /// A successful empty scan clears these; rejected updates preserve them.
    pub fn measurements(&self) -> &[TrackMeasurement] {
        &self.measurements
    }

    /// Updates from a successful scan. Rejected inputs return the unchanged
    /// tracks; use `try_update` to distinguish rejection from a successful scan.
    pub fn update(&mut self, detections: &[Detection], stamp: f64) -> Vec<Track> {
        self.try_update(detections, stamp)
            .unwrap_or_else(|_| self.tracks.clone())
    }

    /// Accepts finite, nonnegative, strictly increasing acquisition timestamps
    /// and finite detections with nonnegative radii. Any error leaves tracks,
    /// identity allocation and the accepted scan time unchanged. A successful
    /// empty scan advances acceptance time without renewing track observations.
    pub fn try_update(
        &mut self,
        detections: &[Detection],
        stamp: f64,
    ) -> Result<Vec<Track>, TrackerUpdateError> {
        if !stamp.is_finite()
            || stamp < 0.
            || self.accepted_scan_stamp.is_some_and(|last| stamp <= last)
        {
            return Err(TrackerUpdateError::InvalidTimestamp);
        }
        if let Some(index) = detections
            .iter()
            .position(|d| !d.center.finite() || !d.radius.is_finite() || d.radius < 0.)
        {
            return Err(TrackerUpdateError::InvalidDetection { index });
        }
        let mut next = self.clone();
        next.accepted_scan_stamp = Some(stamp);
        next.apply(detections, stamp)?;
        let result = next.tracks.clone();
        *self = next;
        Ok(result)
    }

    fn apply(&mut self, detections: &[Detection], stamp: f64) -> Result<(), TrackerUpdateError> {
        self.tracks.retain(|t| stamp - t.last_seen < 0.6);
        self.pending
            .retain(|id, _| self.tracks.iter().any(|t| t.id == *id));
        self.recent
            .retain(|id, _| self.tracks.iter().any(|t| t.id == *id));
        self.measurements.clear();
        let matches = association::associate(&self.tracks, detections, stamp)?;
        for (d, matched) in detections.iter().zip(matches) {
            let id;
            if let Some(i) = matched {
                let track = &mut self.tracks[i];
                id = track.id;
                // The residual spans this track's missed observations, even
                // when other objects were detected in intervening scans.
                let elapsed = stamp - track.last_seen;
                let predicted = track.position.plus(track.velocity.scaled(elapsed));
                let residual = d.center.minus(predicted);
                track.position = predicted.plus(residual.scaled(0.75));
                if let Some(pending) = self.pending.get_mut(&id) {
                    if let Some((position, velocity)) = pending.observe(d, stamp) {
                        track.position = position;
                        track.velocity = velocity;
                        self.pending.remove(&id);
                    }
                } else {
                    track.velocity = track
                        .velocity
                        .plus(residual.scaled(0.12 / elapsed.clamp(0.01, 1.0)));
                }
                if let Some(recent) = self.recent.get_mut(&id)
                    && let Some((position, velocity)) = recent.observe(d, stamp)
                {
                    track.position = position;
                    track.velocity = velocity;
                }
                let speed = track.velocity.x.hypot(track.velocity.y);
                if speed > 15.0 {
                    track.velocity = track.velocity.scaled(15.0 / speed);
                }
                track.radius = 0.8 * track.radius + 0.2 * d.radius;
                if self.enclose_measurements {
                    track.radius = track
                        .radius
                        .max(track.position.distance(d.center) + d.radius);
                }
                track.last_seen = stamp;
            } else {
                self.next_id = self
                    .next_id
                    .checked_add(1)
                    .ok_or(TrackerUpdateError::IdentifierExhausted)?;
                id = self.next_id;
                if self.confirm_initialization {
                    self.pending
                        .insert(id, initialization::Initialization::new(d, stamp));
                }
                if self.fit_recent_motion {
                    self.recent
                        .insert(id, recent_motion::RecentMotion::new(d, stamp));
                }
                self.tracks.push(Track {
                    id: self.next_id,
                    position: d.center,
                    velocity: Vec2::default(),
                    radius: d.radius,
                    last_seen: stamp,
                });
            }
            if self.confirm_initialization || self.enclose_measurements || self.fit_recent_motion {
                self.measurements.push(TrackMeasurement {
                    id,
                    stamp,
                    center: d.center,
                    radius: d.radius,
                });
            }
        }
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exhausted_ids_reject_without_partial_track_or_clock_updates() {
        let mut t = Tracker::default();
        let detection = |x| Detection {
            center: Vec2::new(x, 0.),
            radius: 1.,
        };
        let before = t.try_update(&[detection(0.)], 0.).unwrap();
        t.next_id = u64::MAX - 1;
        assert_eq!(
            t.try_update(&[detection(0.2), detection(20.), detection(40.)], 0.1)
                .unwrap_err(),
            TrackerUpdateError::IdentifierExhausted
        );
        assert_eq!(t.next_id, u64::MAX - 1);
        assert_eq!(t.accepted_scan_stamp, Some(0.));
        assert_eq!(t.tracks.len(), 1);
        assert_eq!(t.tracks[0].position, before[0].position);
        assert_eq!(t.tracks[0].velocity, before[0].velocity);
        assert_eq!(t.tracks[0].last_seen, before[0].last_seen);
        let repaired = t
            .try_update(&[detection(0.2), detection(20.)], 0.1)
            .unwrap();
        assert_eq!(repaired[1].id, u64::MAX);
        assert!(
            t.try_update(&[detection(0.3), detection(20.1)], 0.2)
                .is_ok()
        );
    }

    #[test]
    fn clusters_two_objects_without_labels() {
        let scan = LidarScan {
            sweep: None,
            stamp: 0.0,
            points: vec![
                Vec2::new(5.0, 0.0),
                Vec2::new(5.0, 0.1),
                Vec2::new(5.1, 0.0),
                Vec2::new(10.0, 3.0),
                Vec2::new(10.1, 3.0),
                Vec2::new(10.0, 3.1),
            ],
        };
        assert_eq!(LidarClusters.detect(&scan, Pose::default()).len(), 2);
    }
    #[test]
    fn tracks_velocity_and_expires() {
        let mut t = Tracker::default();
        for i in 0..50 {
            t.update(
                &[Detection {
                    center: Vec2::new(i as f64 * 0.2, 0.0),
                    radius: 1.0,
                }],
                i as f64 * 0.1,
            );
        }
        assert!((t.tracks[0].velocity.x - 2.0).abs() < 0.1);
        assert_eq!(t.tracks[0].id, 1);
        assert!(t.update(&[], 6.0).is_empty());
    }
    #[test]
    fn circle_fit_recovers_hidden_center_from_visible_arc() {
        let points: Vec<_> = (0..30)
            .map(|i| {
                let a = 2.0 + 2.0 * i as f64 / 29.0;
                Vec2::new(10.0 + a.cos(), 2.0 + a.sin())
            })
            .collect();
        let (center, radius) = fit_circle(&points).unwrap();
        assert!(center.distance(Vec2::new(10.0, 2.0)) < 1e-6);
        assert!((radius - 1.12).abs() < 1e-6);
    }
    #[test]
    fn nan_points_are_ignored() {
        assert!(
            LidarClusters
                .detect(
                    &LidarScan {
                        sweep: None,
                        stamp: 0.0,
                        points: vec![Vec2::new(f64::NAN, 0.0)]
                    },
                    Pose::default()
                )
                .is_empty()
        );
    }
}
