//! Validate genuine tilted XYZ returns before bounded planar projection.
use rustdrive_core::{Lidar3dScan, LidarScan, Vec2};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::f64::consts::{PI, TAU};

const DIRECTION_TOLERANCE: f64 = 1e-4;
const VOXEL_M: f64 = 0.05;

/// Uniform elevation rings and a full azimuth sweep from -PI to PI inclusive.
/// Ordinals are column * elevation_rings + ring. Beam direction is body
/// (cos(e)*cos(a), -cos(e)*sin(a), sin(e)); points add mount height to Z.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lidar3dConfig {
    pub azimuth_columns: usize,
    pub elevation_rings: usize,
    pub min_elevation_rad: f64,
    pub max_elevation_rad: f64,
    pub mount_height_m: f64,
    pub min_range_m: f64,
    pub max_range_m: f64,
    pub collision_bottom_m: f64,
    pub collision_top_m: f64,
}
impl Lidar3dConfig {
    pub(crate) fn validate(&self) -> Result<(), String> {
        let ray_count = self.azimuth_columns.checked_mul(self.elevation_rings);
        if !(2..=2048).contains(&self.azimuth_columns)
            || !(2..=64).contains(&self.elevation_rings)
            || ray_count.is_none_or(|n| n > 20_000)
            || ![
                self.min_elevation_rad,
                self.max_elevation_rad,
                self.mount_height_m,
                self.min_range_m,
                self.max_range_m,
                self.collision_bottom_m,
                self.collision_top_m,
            ]
            .iter()
            .all(|v| v.is_finite())
            || !(-PI / 3.0..=PI / 3.0).contains(&self.min_elevation_rad)
            || !(-PI / 3.0..=PI / 3.0).contains(&self.max_elevation_rad)
            || self.max_elevation_rad - self.min_elevation_rad < 1e-4
            || !(-5.0..=10.0).contains(&self.mount_height_m)
            || !(0.1..=10.0).contains(&self.min_range_m)
            || !(0.2..=200.0).contains(&self.max_range_m)
            || self.max_range_m - self.min_range_m < 0.1
            || !(-5.0..=10.0).contains(&self.collision_bottom_m)
            || !(-5.0..=10.0).contains(&self.collision_top_m)
            || self.collision_top_m - self.collision_bottom_m < 0.1
        {
            return Err("invalid 3D LiDAR calibration".into());
        }
        Ok(())
    }
    pub(crate) fn project(&self, scan: &Lidar3dScan) -> Result<LidarScan, ()> {
        let ray_count = self
            .azimuth_columns
            .checked_mul(self.elevation_rings)
            .ok_or(())?;
        if scan.returns.len() > ray_count || scan.returns.len() > 20_000 {
            return Err(());
        }
        let mut indices = BTreeSet::new();
        for measured in &scan.returns {
            let p = measured.point;
            if measured.ray_index >= ray_count || !indices.insert(measured.ray_index) || !p.finite()
            {
                return Err(());
            }
            let up = p.z - self.mount_height_m;
            let range = p.x.hypot(p.y).hypot(up);
            if !range.is_finite()
                || range < self.min_range_m - 1e-7
                || range > self.max_range_m + 1e-7
            {
                return Err(());
            }
            let column = measured.ray_index / self.elevation_rings;
            let ring = measured.ray_index % self.elevation_rings;
            let azimuth = -PI + TAU * column as f64 / (self.azimuth_columns - 1) as f64;
            let elevation = self.min_elevation_rad
                + (self.max_elevation_rad - self.min_elevation_rad) * ring as f64
                    / (self.elevation_rings - 1) as f64;
            let expected = [
                elevation.cos() * azimuth.cos(),
                -elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ];
            if [p.x / range, p.y / range, up / range]
                .iter()
                .zip(expected)
                .any(|(actual, expected)| (actual - expected).abs() > DIRECTION_TOLERANCE)
            {
                return Err(());
            }
        }
        let mut returns: Vec<_> = scan.returns.iter().collect();
        returns.sort_by_key(|r| r.ray_index);
        let mut cells = BTreeSet::new();
        let mut points = vec![];
        for measured in returns {
            let p = measured.point;
            if p.z < self.collision_bottom_m || p.z > self.collision_top_m {
                continue;
            }
            let cell = (
                (p.x / VOXEL_M).floor() as i64,
                (p.y / VOXEL_M).floor() as i64,
            );
            if cells.insert(cell) {
                points.push(Vec2::new(p.x, p.y));
            }
        }
        Ok(LidarScan {
            stamp: scan.stamp,
            points,
        })
    }
}
