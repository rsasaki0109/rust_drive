//! Range-adaptive Euclidean clustering and nearest-neighbor alpha-beta tracking.
pub mod ground3d;
pub mod image_features;
pub mod objects3d;
pub mod terrain_adaptive;
pub use ground3d::{GroundClassification, TerrainConfig, TerrainDiagnostics, classify_ground};
pub use objects3d::{ObjectAabb, ObjectClassification, ObjectClusterConfig, cluster_objects};
pub use terrain_adaptive::{
    AdaptiveGroundClassification, AdaptiveTerrainConfig, AdaptiveTerrainDiagnostics,
    classify_ground_adaptive,
};

use rustdriving_core::{Detection, LidarScan, Perception, Pose, Track, Vec2};
#[derive(Default)]
pub struct LidarClusters;
impl Perception for LidarClusters {
    fn detect(&mut self, scan: &LidarScan, pose: Pose) -> Vec<Detection> {
        let points: Vec<_> = scan
            .points
            .iter()
            .copied()
            .filter(|p| p.finite() && p.x.hypot(p.y) > 0.3)
            .collect();
        let mut visited = vec![false; points.len()];
        let mut result = Vec::new();
        for i in 0..points.len() {
            if visited[i] {
                continue;
            }
            visited[i] = true;
            let mut queue = vec![i];
            let mut cursor = 0;
            while cursor < queue.len() {
                let p = points[queue[cursor]];
                cursor += 1;
                let tolerance = (0.28 + p.x.hypot(p.y) * 0.015).min(0.85);
                for j in 0..points.len() {
                    if !visited[j] && p.distance(points[j]) < tolerance {
                        visited[j] = true;
                        queue.push(j);
                    }
                }
            }
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
            let fitted = fit_circle(&queue.iter().map(|&j| points[j]).collect::<Vec<_>>());
            let (center, radius) = fitted.unwrap_or((center, (extent + 0.65).clamp(0.85, 3.0)));
            result.push(Detection {
                center: pose.to_world(center),
                radius,
            });
        }
        result
    }
}
/// Algebraic least-squares circle fit, with bounds and residual checks.
fn fit_circle(points: &[Vec2]) -> Option<(Vec2, f64)> {
    if points.len() < 5 {
        return None;
    }
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
#[derive(Default)]
pub struct Tracker {
    tracks: Vec<Track>,
    next_id: u64,
    last_stamp: Option<f64>,
}
impl Tracker {
    pub fn update(&mut self, detections: &[Detection], stamp: f64) -> Vec<Track> {
        let dt = self
            .last_stamp
            .map_or(0.1, |s| (stamp - s).clamp(0.01, 1.0));
        self.last_stamp = Some(stamp);
        self.tracks.retain(|t| stamp - t.last_seen < 0.6);
        let mut matched = vec![false; self.tracks.len()];
        for d in detections {
            let best = self
                .tracks
                .iter()
                .enumerate()
                .filter(|(i, _)| !matched[*i])
                .map(|(i, t)| {
                    (
                        i,
                        t.position
                            .plus(t.velocity.scaled(stamp - t.last_seen))
                            .distance(d.center),
                    )
                })
                .filter(|(_, dist)| *dist < 3.0)
                .min_by(|a, b| a.1.total_cmp(&b.1));
            if let Some((i, _)) = best {
                let track = &mut self.tracks[i];
                let predicted = track
                    .position
                    .plus(track.velocity.scaled(stamp - track.last_seen));
                let residual = d.center.minus(predicted);
                track.position = predicted.plus(residual.scaled(0.75));
                track.velocity = track.velocity.plus(residual.scaled(0.12 / dt));
                let speed = track.velocity.x.hypot(track.velocity.y);
                if speed > 15.0 {
                    track.velocity = track.velocity.scaled(15.0 / speed);
                }
                track.radius = 0.8 * track.radius + 0.2 * d.radius;
                track.last_seen = stamp;
                matched[i] = true;
            } else {
                self.next_id += 1;
                self.tracks.push(Track {
                    id: self.next_id,
                    position: d.center,
                    velocity: Vec2::default(),
                    radius: d.radius,
                    last_seen: stamp,
                });
                matched.push(true);
            }
        }
        self.tracks.clone()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn clusters_two_objects_without_labels() {
        let scan = LidarScan {
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
                        stamp: 0.0,
                        points: vec![Vec2::new(f64::NAN, 0.0)]
                    },
                    Pose::default()
                )
                .is_empty()
        );
    }
}
