//! One forward connector from a stopped source pose to the original target tail.
use crate::lane_return::{LaneReturnRequest, plan_with_start_offset};
use rustdriving_core::{Pose, Route, Vec2, VehicleConfig};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneResumeRequest {
    /// Estimated stationary pose; initial curvature is zero in this adapter.
    pub start: Pose,
    pub source_start: Vec2,
    pub source_end: Vec2,
    pub source_width_m: f64,
    pub target_start: Vec2,
    pub target_end: Vec2,
    pub target_width_m: f64,
    pub original_exit_s_m: f64,
    pub distance_m: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct LaneResumePlan {
    pub route: Route,
    pub curvature_bound_m_inv: f64,
    pub lateral_controls_m: [f64; 6],
    pub connector_length_m: f64,
    pub join_s_m: f64,
}

pub fn plan(
    r: LaneResumeRequest,
    original: &Route,
    vehicle: VehicleConfig,
) -> Result<LaneResumePlan, String> {
    if original.points.len() > 5000
        || original
            .points
            .iter()
            .any(|p| !p.finite() || p.x.abs() > 1e6 || p.y.abs() > 1e6)
        || !r.source_start.finite()
        || !r.source_end.finite()
        || !r.target_start.finite()
        || !r.target_end.finite()
        || !r.original_exit_s_m.is_finite()
        || r.original_exit_s_m < 0.0
        || !r.source_width_m.is_finite()
        || !(2.6..=10.0).contains(&r.source_width_m)
        || !r.target_width_m.is_finite()
        || !(2.6..=10.0).contains(&r.target_width_m)
    {
        return Err("invalid resume lanes or original route".into());
    }
    let canonical = Route::new(original.points.clone(), original.half_width)?;
    if canonical.lengths != original.lengths || canonical.length() > 1500.0 {
        return Err("invalid resume route arc metadata".into());
    }
    let vector = r.target_end.minus(r.target_start);
    let length = vector.x.hypot(vector.y);
    if !(20.0..=1000.0).contains(&length)
        || r.source_end.minus(r.source_start).distance(vector) > 1e-6
        || r.source_start.x.abs() > 1e6
        || r.source_start.y.abs() > 1e6
        || r.source_end.x.abs() > 1e6
        || r.source_end.y.abs() > 1e6
    {
        return Err("resume requires parallel coextensive straight lanes".into());
    }
    let axis = vector.scaled(1.0 / length);
    let coords = |p: Vec2| {
        let q = p.minus(r.target_start);
        (q.x * axis.x + q.y * axis.y, axis.x * q.y - axis.y * q.x)
    };
    let (source_x, source_y) = coords(r.source_start);
    let (_, start_y) = coords(r.start.position);
    let (exit_x, exit_y) = coords(original.sample(r.original_exit_s_m, 0.0).0);
    if source_x.abs() > 1e-6
        || (source_y.abs() - (r.source_width_m + r.target_width_m) / 2.0).abs() > 1e-6
        || (start_y - source_y).abs() + vehicle.radius + 0.05 > r.source_width_m / 2.0
        || original.half_width > r.source_width_m.min(r.target_width_m) / 2.0
        || r.original_exit_s_m >= original.length()
        || exit_y.abs() > 1e-6
        || !(0.0..length).contains(&exit_x)
    {
        return Err("resume requires source containment and the original target tail".into());
    }
    let curve = plan_with_start_offset(
        LaneReturnRequest {
            start: r.start,
            curvature_m_inv: 0.0,
            lane_start: r.target_start,
            lane_end: r.target_end,
            half_width_m: original.half_width,
            distance_m: r.distance_m,
        },
        vehicle,
        source_y.abs() + r.source_width_m / 2.0,
    )?;
    let lower = (-r.target_width_m / 2.0).min(source_y - r.source_width_m / 2.0);
    let upper = (r.target_width_m / 2.0).max(source_y + r.source_width_m / 2.0);
    if curve
        .lateral_controls_m
        .iter()
        .any(|y| y - vehicle.radius - 0.05 < lower || y + vehicle.radius + 0.05 > upper)
    {
        return Err("resume curve leaves the physical lane envelope".into());
    }
    let join = *curve.route.points.last().unwrap();
    let join_x = coords(join).0;
    let join_s = r.original_exit_s_m + join_x - exit_x;
    let target_end_s = r.original_exit_s_m + length - exit_x;
    if join_x < exit_x
        || original.sample(join_s, 0.0).0.distance(join) > 1e-6
        || original.sample(target_end_s, 0.0).0.distance(r.target_end) > 1e-6
        || target_end_s > original.length()
    {
        return Err(
            "resume join must be ahead of the original change on its target section".into(),
        );
    }
    let mut previous_x = exit_x;
    for (&p, &s) in original
        .points
        .iter()
        .zip(&original.lengths)
        .filter(|(_, s)| **s > r.original_exit_s_m && **s <= target_end_s + 1e-6)
    {
        let (x, y) = coords(p);
        if y.abs() > 1e-6 || x < previous_x || (s - r.original_exit_s_m - x + exit_x).abs() > 1e-6 {
            return Err("resume requires an unchanged straight target section".into());
        }
        previous_x = x;
    }
    let connector_length = curve.route.length();
    let mut points = curve.route.points;
    points.extend(
        original
            .points
            .iter()
            .zip(&original.lengths)
            .filter(|(_, s)| **s > join_s + 1e-6)
            .map(|(p, _)| *p),
    );
    Ok(LaneResumePlan {
        route: Route::new(points, original.half_width)?,
        curvature_bound_m_inv: curve.curvature_bound_m_inv,
        lateral_controls_m: curve.lateral_controls_m,
        connector_length_m: connector_length,
        join_s_m: join_s,
    })
}

#[cfg(test)]
mod tests;
