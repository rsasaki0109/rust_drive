//! Bounded ray/point ownership and conditional full-azimuth resolution checks.
use rustdriving_core::{
    ClearSweepEvidence, ClearSweepRecoveryConfig, LidarScan, PlanarLidarCalibration,
    PlanarRayOutcome, Vec2,
};
use std::f64::consts::TAU;

pub fn validate_calibration(c: &PlanarLidarCalibration) -> Result<f64, &'static str> {
    let step = c.angular_step_rad.abs();
    let span = step * c.ray_count.saturating_sub(1) as f64;
    if !(64..=4096).contains(&c.ray_count)
        || ![
            c.first_angle_rad,
            step,
            c.min_range_m,
            c.max_range_m,
            c.range_error_m,
        ]
        .iter()
        .all(|v| v.is_finite())
        || c.first_angle_rad.abs() > TAU
        || !(1e-5..=0.1).contains(&step)
        || span < TAU - step - 1e-9
        || span > TAU + 1e-9
        || !(0.0..=1.0).contains(&c.min_range_m)
        || !(1.0..=200.0).contains(&c.max_range_m)
        || c.max_range_m <= c.min_range_m
        || !(0.0..=0.1).contains(&c.range_error_m)
    {
        return Err("invalid full-azimuth planar calibration");
    }
    Ok(step.max((TAU - span).max(0.)))
}

pub fn validate_recovery(c: &ClearSweepRecoveryConfig) -> Result<(), &'static str> {
    let gap = validate_calibration(&c.lidar)?;
    if !c.checked_center_radius_m.is_finite()
        || !c.minimum_resolved_disc_radius_m.is_finite()
        || !(1.0..=100.0).contains(&c.checked_center_radius_m)
        || !(0.2..=3.0).contains(&c.minimum_resolved_disc_radius_m)
        || c.checked_center_radius_m + c.minimum_resolved_disc_radius_m + c.lidar.range_error_m
            > c.lidar.max_range_m
        || c.lidar.min_range_m >= c.minimum_resolved_disc_radius_m
        || c.checked_center_radius_m * (gap / 2.).sin() >= c.minimum_resolved_disc_radius_m
    {
        return Err("unresolved planar empty-sweep region");
    }
    Ok(())
}

/// Validate the complete acquisition, including one-to-one measured-point
/// ownership. Unavailable rays cannot turn into negative sensing evidence.
pub fn validate_sweep(scan: &LidarScan) -> Result<bool, &'static str> {
    let sweep = scan.sweep.as_ref().ok_or("planar sweep evidence missing")?;
    let c = sweep.calibration;
    validate_calibration(&c)?;
    if sweep.rays.len() != c.ray_count
        || scan.points.len() > c.ray_count
        || !scan.stamp.is_finite()
        || scan.stamp < 0.
    {
        return Err("incomplete planar sweep");
    }
    let mut seen = vec![false; scan.points.len()];
    for (index, ray) in sweep.rays.iter().enumerate() {
        match *ray {
            PlanarRayOutcome::Unavailable => return Err("unavailable planar ray"),
            PlanarRayOutcome::Clear => {}
            PlanarRayOutcome::Return(point) => {
                let p = *scan
                    .points
                    .get(point)
                    .ok_or("invalid ray return ownership")?;
                let angle = c.first_angle_rad + c.angular_step_rad * index as f64;
                let direction = Vec2::new(angle.cos(), angle.sin());
                let along = p.x * direction.x + p.y * direction.y;
                let across = (p.x * direction.y - p.y * direction.x).abs();
                if seen[point]
                    || !p.finite()
                    || along <= 0.
                    || along < c.min_range_m - c.range_error_m
                    || along > c.max_range_m + c.range_error_m
                    || across > 1e-5 * (1. + along)
                {
                    return Err("measured point differs from commanded ray");
                }
                seen[point] = true;
            }
        }
    }
    if seen.iter().any(|v| !v) {
        return Err("unowned measured point");
    }
    Ok(scan.points.is_empty())
}

pub fn certify_empty_sweep(
    scan: &LidarScan,
    c: &ClearSweepRecoveryConfig,
) -> Result<Option<ClearSweepEvidence>, &'static str> {
    validate_recovery(c)?;
    if scan.sweep.as_ref().is_none_or(|s| s.calibration != c.lidar) {
        return Err("planar sweep differs from declared calibration");
    }
    if !validate_sweep(scan)? {
        return Ok(None);
    }
    let gap = validate_calibration(&c.lidar)?;
    Ok(Some(ClearSweepEvidence {
        acquisition_s: scan.stamp,
        checked_center_radius_m: c.checked_center_radius_m,
        minimum_resolved_disc_radius_m: c.minimum_resolved_disc_radius_m,
        maximum_angular_gap_rad: gap,
        maximum_transverse_gap_m: c.checked_center_radius_m * (gap / 2.).sin(),
        ray_count: c.lidar.ray_count,
    }))
}
