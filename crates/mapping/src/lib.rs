//! Local log-odds occupancy map, independent of the supplied navigation route.
mod ray;
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
    /// Each return clears each crossed cell once, excluding its hit cell.
    /// Cells are half-open; a corner touch alone supplies no free evidence.
    /// Empty scans do not clear space. Invalid poses/returns are ignored.
    pub fn update(&mut self, scan: &LidarScan, pose: Pose) {
        if !pose.position.finite() || !pose.yaw.is_finite() {
            return;
        }
        let start = Vec2::new(
            (pose.position.x - self.origin.x) / self.resolution,
            (pose.position.y - self.origin.y) / self.resolution,
        );
        if !start.finite() {
            return;
        }
        for &point in &scan.points {
            if !point.finite() {
                continue;
            }
            let end = pose.to_world(point);
            let normalized_end = Vec2::new(
                (end.x - self.origin.x) / self.resolution,
                (end.y - self.origin.y) / self.resolution,
            );
            if !normalized_end.finite() {
                continue;
            }
            let hit = self.index(end);
            if let Some(cells) = ray::Cells::new(start, normalized_end, self.width, self.height) {
                for (x, y) in cells {
                    let j = y * self.width + x;
                    if Some(j) != hit {
                        self.log_odds[j] = (self.log_odds[j] - 0.15).max(-4.0);
                    }
                }
            }
            if let Some(j) = hit {
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
mod tests;
