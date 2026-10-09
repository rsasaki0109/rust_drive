//! Bounded, calibrated height selection before the ordinary planar algorithms.
use rustdrive_core::{LidarScan, MultiHeightLidarScan};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

const HEIGHT_TOLERANCE_M: f64 = 1e-9;
const VOXEL_M: f64 = 0.05;

/// Expected synchronized planes and the vehicle's calibrated vertical collision
/// envelope. This does not establish coverage between the measured planes.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MultiHeightLidarConfig {
    pub heights_m: Vec<f64>,
    pub collision_bottom_m: f64,
    pub collision_top_m: f64,
}
impl MultiHeightLidarConfig {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if !(2..=16).contains(&self.heights_m.len())
            || !self.collision_bottom_m.is_finite()
            || !self.collision_top_m.is_finite()
            || !(-5.0..=10.0).contains(&self.collision_bottom_m)
            || !(-5.0..=10.0).contains(&self.collision_top_m)
            || self.collision_top_m - self.collision_bottom_m < 0.1
            || self
                .heights_m
                .iter()
                .any(|h| !h.is_finite() || !(-5.0..=10.0).contains(h))
            || self
                .heights_m
                .iter()
                .enumerate()
                .any(|(i, h)| self.heights_m[..i].iter().any(|old| (old - h).abs() < 0.01))
            || !self.heights_m.iter().any(|h| self.relevant(*h))
        {
            return Err("invalid multi-height LiDAR calibration".into());
        }
        Ok(())
    }
    fn relevant(&self, height: f64) -> bool {
        height >= self.collision_bottom_m && height <= self.collision_top_m
    }
    pub(crate) fn fuse(&self, scan: &MultiHeightLidarScan) -> Result<LidarScan, ()> {
        if scan.planes.len() != self.heights_m.len() {
            return Err(());
        }
        let mut matched = BTreeSet::new();
        let mut planes = Vec::with_capacity(scan.planes.len());
        let mut count = 0usize;
        for plane in &scan.planes {
            let Some(index) = self.heights_m.iter().position(|h| {
                plane.height_m.is_finite() && (plane.height_m - h).abs() <= HEIGHT_TOLERANCE_M
            }) else {
                return Err(());
            };
            count = count.checked_add(plane.points.len()).ok_or(())?;
            if !matched.insert(index)
                || count > 20_000
                || plane
                    .points
                    .iter()
                    .any(|p| !p.finite() || p.x.hypot(p.y) > 200.0)
            {
                return Err(());
            }
            planes.push((index, plane));
        }
        planes.sort_by(|(a, _), (b, _)| self.heights_m[*a].total_cmp(&self.heights_m[*b]));
        let mut cells = BTreeSet::new();
        let mut points = vec![];
        for (index, plane) in planes {
            if !self.relevant(self.heights_m[index]) {
                continue;
            }
            for point in &plane.points {
                let cell = (
                    (point.x / VOXEL_M).floor() as i64,
                    (point.y / VOXEL_M).floor() as i64,
                );
                if cells.insert(cell) {
                    points.push(*point);
                }
            }
        }
        Ok(LidarScan {
            stamp: scan.stamp,
            points,
        })
    }
}
