//! Local log-odds occupancy map, independent of the supplied navigation route.
use rustdriving_core::{LidarScan, Pose, Vec2};
pub struct OccupancyGrid {
    pub origin: Vec2,
    pub resolution: f64,
    pub width: usize,
    pub height: usize,
    log_odds: Vec<f32>,
}
impl OccupancyGrid {
    pub fn new(origin: Vec2, width: usize, height: usize, resolution: f64) -> Self {
        assert!(
            origin.finite()
                && width > 0
                && height > 0
                && resolution.is_finite()
                && resolution > 0.0
        );
        Self {
            origin,
            resolution,
            width,
            height,
            log_odds: vec![0.0; width * height],
        }
    }
    fn index(&self, p: Vec2) -> Option<usize> {
        let x = ((p.x - self.origin.x) / self.resolution).floor();
        let y = ((p.y - self.origin.y) / self.resolution).floor();
        if x >= 0.0 && y >= 0.0 && x < (self.width as f64) && y < (self.height as f64) {
            Some(y as usize * self.width + x as usize)
        } else {
            None
        }
    }
    pub fn update(&mut self, scan: &LidarScan, pose: Pose) {
        for &point in &scan.points {
            if !point.finite() {
                continue;
            }
            let end = pose.to_world(point);
            let distance = pose.position.distance(end);
            let steps = (distance / self.resolution).ceil() as usize;
            for i in 0..steps {
                let p = pose.position.plus(
                    end.minus(pose.position)
                        .scaled(i as f64 / steps.max(1) as f64),
                );
                if let Some(j) = self.index(p) {
                    self.log_odds[j] = (self.log_odds[j] - 0.15).max(-4.0);
                }
            }
            if let Some(j) = self.index(end) {
                self.log_odds[j] = (self.log_odds[j] + 0.85).min(4.0);
            }
        }
    }
    pub fn probability(&self, p: Vec2) -> Option<f64> {
        self.index(p)
            .map(|i| 1.0 / (1.0 + (-self.log_odds[i] as f64).exp()))
    }
    pub fn occupied_cells(&self) -> Vec<Vec2> {
        self.log_odds
            .iter()
            .enumerate()
            .filter(|(_, l)| **l > 1.0)
            .map(|(i, _)| {
                Vec2::new(
                    self.origin.x
                        + (i % self.width) as f64 * self.resolution
                        + self.resolution / 2.0,
                    self.origin.y
                        + (i / self.width) as f64 * self.resolution
                        + self.resolution / 2.0,
                )
            })
            .collect()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn marks_hit_and_clears_ray() {
        let mut g = OccupancyGrid::new(Vec2::new(-1.0, -1.0), 20, 10, 0.5);
        let scan = LidarScan {
            stamp: 0.0,
            points: vec![Vec2::new(5.0, 0.0)],
        };
        for _ in 0..4 {
            g.update(&scan, Pose::default());
        }
        assert!(g.probability(Vec2::new(5.0, 0.0)).unwrap() > 0.9);
        assert!(g.probability(Vec2::new(2.0, 0.0)).unwrap() < 0.5);
        assert!(g.probability(Vec2::new(100.0, 0.0)).is_none());
    }
}
