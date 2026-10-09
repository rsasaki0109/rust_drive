//! Deterministic measured ground fitting, with bounded priors and local support.
use rustdrive_core::{Lidar3dScan, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};

const SECTORS: usize = 8;
const ANCHORS: usize = 16;
const SUPPORT_BAND_M: f64 = 5.0;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundConfig {
    pub reference_height_m: f64,
    pub max_slope: f64,
    pub max_height_offset_m: f64,
    pub residual_threshold_m: f64,
    pub fit_radius_m: f64,
    pub min_inliers: usize,
    pub min_sector_inliers: usize,
    pub min_cell_inliers: usize,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundPlane {
    /// Road-datum Z = a * body X + b * body Y + c.
    pub a: f64,
    pub b: f64,
    pub c: f64,
}
impl GroundPlane {
    fn residual(self, p: Vec3) -> f64 {
        p.z - (self.a * p.x + self.b * p.y + self.c)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundDiagnostics {
    pub stamp: f64,
    pub plane: Option<GroundPlane>,
    pub candidate_points: usize,
    pub inliers: usize,
    pub sector_inliers: Vec<usize>,
    pub removed_points: usize,
    pub preserved_points: usize,
    pub supported_cells: usize,
    pub max_inlier_residual_m: f64,
    pub rms_residual_m: f64,
    /// Geometric fit confidence only; acquisition freshness is checked separately.
    pub confidence: bool,
}
fn sector(point: Vec3, count: usize) -> usize {
    (((point.y.atan2(point.x) + PI) / TAU * count as f64).floor() as usize).min(count - 1)
}
fn cell(point: Vec3) -> (usize, usize) {
    (
        sector(point, SECTORS),
        (point.x.hypot(point.y) / SUPPORT_BAND_M).floor() as usize,
    )
}
impl GroundConfig {
    pub(crate) fn validate(&self, mount_height: f64) -> Result<(), String> {
        if ![
            self.reference_height_m,
            self.max_slope,
            self.max_height_offset_m,
            self.residual_threshold_m,
            self.fit_radius_m,
        ]
        .iter()
        .all(|v| v.is_finite())
            || !(-1.0..=1.0).contains(&self.reference_height_m)
            || mount_height - self.reference_height_m < 0.1
            || !(0.001..=0.1).contains(&self.max_slope)
            || !(0.001..=0.04).contains(&self.max_height_offset_m)
            || !(0.002..=0.03).contains(&self.residual_threshold_m)
            || !(3.0..=12.0).contains(&self.fit_radius_m)
            || !(32..=10_000).contains(&self.min_inliers)
            || !(3..=self.min_inliers / SECTORS).contains(&self.min_sector_inliers)
            || !(3..=256).contains(&self.min_cell_inliers)
        {
            return Err("invalid measured-ground calibration".into());
        }
        Ok(())
    }
    fn plausible(&self, plane: GroundPlane) -> bool {
        plane.a.is_finite()
            && plane.b.is_finite()
            && plane.c.is_finite()
            && plane.a.hypot(plane.b) <= self.max_slope
            && (plane.c - self.reference_height_m).abs() <= self.max_height_offset_m
    }
    /// Returns a raw-return removal mask only after a supported measured fit.
    pub(crate) fn separate(
        &self,
        scan: &Lidar3dScan,
    ) -> Result<(Vec<bool>, GroundDiagnostics), Box<GroundDiagnostics>> {
        let mut diagnostics = GroundDiagnostics {
            stamp: scan.stamp,
            plane: None,
            candidate_points: 0,
            inliers: 0,
            sector_inliers: vec![0; SECTORS],
            removed_points: 0,
            preserved_points: scan.returns.len(),
            supported_cells: 0,
            max_inlier_residual_m: 0.0,
            rms_residual_m: 0.0,
            confidence: false,
        };
        let mut candidates: Vec<_> = scan
            .returns
            .iter()
            .filter(|r| {
                let radius = r.point.x.hypot(r.point.y);
                radius >= 0.5
                    && radius <= self.fit_radius_m
                    && (r.point.z - self.reference_height_m).abs()
                        <= self.max_height_offset_m
                            + self.max_slope * radius
                            + self.residual_threshold_m
            })
            .collect();
        candidates.sort_by_key(|r| r.ray_index);
        diagnostics.candidate_points = candidates.len();
        if candidates.len() < self.min_inliers {
            return Err(Box::new(diagnostics));
        }
        let mut anchors: [Option<Vec3>; ANCHORS] = [None; ANCHORS];
        for measured in &candidates {
            let p = measured.point;
            let anchor = &mut anchors[sector(p, ANCHORS)];
            if anchor.is_none_or(|old| p.x.hypot(p.y) < old.x.hypot(old.y)) {
                *anchor = Some(p);
            }
        }
        let mut best: Option<(GroundPlane, usize, f64)> = None;
        // At most 48 spatially spread hypotheses, independent of point count.
        for first in 0..ANCHORS {
            for offset in [4, 5, 6] {
                let Some(a) = anchors[first] else {
                    continue;
                };
                let Some(b) = anchors[(first + offset) % ANCHORS] else {
                    continue;
                };
                let Some(c) = anchors[(first + 2 * offset) % ANCHORS] else {
                    continue;
                };
                let Some(plane) = three_points(a, b, c) else {
                    continue;
                };
                if !self.plausible(plane) {
                    continue;
                }
                let (count, squared) = candidates.iter().fold((0, 0.0), |(count, sum), r| {
                    let residual = plane.residual(r.point);
                    if residual.abs() <= self.residual_threshold_m {
                        (count + 1, sum + residual * residual)
                    } else {
                        (count, sum)
                    }
                });
                if best.is_none_or(|(_, old_count, old_squared)| {
                    count > old_count || (count == old_count && squared < old_squared)
                }) {
                    best = Some((plane, count, squared));
                }
            }
        }
        let Some((mut plane, _, _)) = best else {
            return Err(Box::new(diagnostics));
        };
        let mut mask: Vec<_> = candidates
            .iter()
            .map(|r| plane.residual(r.point).abs() <= self.residual_threshold_m)
            .collect();
        let mut stable = false;
        // Threshold-adjacent points can enter or leave after refinement. Publish
        // a plane only when it fits the very same inliers its diagnostics report.
        // Eight refinements bound work; oscillating or unconverged fits abstain.
        for _ in 0..8 {
            let inliers: Vec<_> = candidates
                .iter()
                .zip(&mask)
                .filter_map(|(r, included)| included.then_some(r.point))
                .collect();
            let Some(refined) = least_squares(&inliers) else {
                return Err(Box::new(diagnostics));
            };
            if !self.plausible(refined) {
                return Err(Box::new(diagnostics));
            }
            plane = refined;
            let next: Vec<_> = candidates
                .iter()
                .map(|r| plane.residual(r.point).abs() <= self.residual_threshold_m)
                .collect();
            if next == mask {
                stable = true;
                break;
            }
            mask = next;
        }
        if !stable {
            return Err(Box::new(diagnostics));
        }
        diagnostics.plane = Some(plane);
        let mut squared = 0.0;
        for measured in &candidates {
            let residual = plane.residual(measured.point).abs();
            if residual <= self.residual_threshold_m {
                diagnostics.inliers += 1;
                diagnostics.sector_inliers[sector(measured.point, SECTORS)] += 1;
                diagnostics.max_inlier_residual_m = diagnostics.max_inlier_residual_m.max(residual);
                squared += residual * residual;
            }
        }
        if diagnostics.inliers > 0 {
            diagnostics.rms_residual_m = (squared / diagnostics.inliers as f64).sqrt();
        }
        if diagnostics.inliers < self.min_inliers
            || diagnostics
                .sector_inliers
                .iter()
                .any(|n| *n < self.min_sector_inliers)
        {
            return Err(Box::new(diagnostics));
        }
        let mut support = BTreeMap::new();
        for measured in &scan.returns {
            if plane.residual(measured.point).abs() <= self.residual_threshold_m {
                *support.entry(cell(measured.point)).or_insert(0usize) += 1;
            }
        }
        diagnostics.supported_cells = support
            .values()
            .filter(|n| **n >= self.min_cell_inliers)
            .count();
        let removed: Vec<_> = scan
            .returns
            .iter()
            .map(|r| {
                plane.residual(r.point).abs() <= self.residual_threshold_m
                    && support
                        .get(&cell(r.point))
                        .is_some_and(|n| *n >= self.min_cell_inliers)
            })
            .collect();
        diagnostics.removed_points = removed.iter().filter(|removed| **removed).count();
        diagnostics.preserved_points -= diagnostics.removed_points;
        diagnostics.confidence = true;
        Ok((removed, diagnostics))
    }
}
fn three_points(a: Vec3, b: Vec3, c: Vec3) -> Option<GroundPlane> {
    let (bx, by, bz) = (b.x - a.x, b.y - a.y, b.z - a.z);
    let (cx, cy, cz) = (c.x - a.x, c.y - a.y, c.z - a.z);
    let determinant = bx * cy - by * cx;
    if determinant.abs() < 0.1 {
        return None;
    }
    let slope_x = (bz * cy - by * cz) / determinant;
    let slope_y = (bx * cz - bz * cx) / determinant;
    Some(GroundPlane {
        a: slope_x,
        b: slope_y,
        c: a.z - slope_x * a.x - slope_y * a.y,
    })
}
fn least_squares(points: &[Vec3]) -> Option<GroundPlane> {
    if points.len() < 3 {
        return None;
    }
    let mut matrix = [[0.0; 4]; 3];
    for p in points {
        let row = [p.x, p.y, 1.0];
        for i in 0..3 {
            for j in 0..3 {
                matrix[i][j] += row[i] * row[j];
            }
            matrix[i][3] += row[i] * p.z;
        }
    }
    for column in 0..3 {
        let pivot = (column..3).max_by(|a, b| {
            matrix[*a][column]
                .abs()
                .total_cmp(&matrix[*b][column].abs())
        })?;
        if matrix[pivot][column].abs() < 1e-9 {
            return None;
        }
        matrix.swap(column, pivot);
        let divisor = matrix[column][column];
        for value in &mut matrix[column][column..] {
            *value /= divisor;
        }
        let pivot_row = matrix[column];
        for (index, row) in matrix.iter_mut().enumerate() {
            if index != column {
                let factor = row[column];
                for j in column..4 {
                    row[j] -= factor * pivot_row[j];
                }
            }
        }
    }
    Some(GroundPlane {
        a: matrix[0][3],
        b: matrix[1][3],
        c: matrix[2][3],
    })
}
