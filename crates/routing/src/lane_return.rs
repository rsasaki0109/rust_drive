//! A bounded forward return to a straight lane, starting at measured pose/curvature.
use rustdriving_core::{Pose, Route, Vec2, VehicleConfig, wrap_angle};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneReturnRequest {
    pub start: Pose,
    pub curvature_m_inv: f64,
    pub lane_start: Vec2,
    pub lane_end: Vec2,
    pub half_width_m: f64,
    pub distance_m: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct LaneReturnPlan {
    pub route: Route,
    pub curvature_bound_m_inv: f64,
    pub lateral_controls_m: [f64; 6],
}

pub fn plan(request: LaneReturnRequest, vehicle: VehicleConfig) -> Result<LaneReturnPlan, String> {
    let r = request;
    if !r.start.position.finite()
        || !r.start.yaw.is_finite()
        || !r.curvature_m_inv.is_finite()
        || !r.lane_start.finite()
        || !r.lane_end.finite()
        || [
            r.start.position.x,
            r.start.position.y,
            r.lane_start.x,
            r.lane_start.y,
            r.lane_end.x,
            r.lane_end.y,
        ]
        .iter()
        .any(|v| v.abs() > 1e6)
        || !r.distance_m.is_finite()
        || !(10.0..=60.0).contains(&r.distance_m)
        || !r.half_width_m.is_finite()
        || !(0.5..=6.0).contains(&r.half_width_m)
        || !vehicle.radius.is_finite()
        || vehicle.radius <= 0.0
        || r.half_width_m < vehicle.radius + 0.05
        || !vehicle.wheelbase.is_finite()
        || vehicle.wheelbase <= 0.0
        || !vehicle.max_steer.is_finite()
        || vehicle.max_steer <= 0.0
        || vehicle.max_steer >= std::f64::consts::FRAC_PI_2
    {
        return Err("invalid lane-return request or vehicle calibration".into());
    }
    let vector = r.lane_end.minus(r.lane_start);
    let length = vector.x.hypot(vector.y);
    if !(20.0..=1000.0).contains(&length) {
        return Err("return lane must be 20..1000 m long".into());
    }
    let axis = vector.scaled(1.0 / length);
    let relative = r.start.position.minus(r.lane_start);
    let x = relative.x * axis.x + relative.y * axis.y;
    let y = axis.x * relative.y - axis.y * relative.x;
    let heading = wrap_angle(r.start.yaw - axis.y.atan2(axis.x));
    if x < 0.0
        || x + r.distance_m + vehicle.radius + 1.0 > length
        || y.abs() > r.half_width_m - vehicle.radius - 0.05
        || heading.abs() > 0.35
    {
        return Err("return requires an early aligned pose and forward stopping room".into());
    }
    let slope = heading.tan();
    let second = r.curvature_m_inv * (1.0 + slope * slope).powf(1.5);
    // Quintic Bezier boundary derivatives: position, heading and curvature match
    // the measured start; all lateral derivatives vanish on the final centerline.
    let controls = [
        y,
        y + slope * r.distance_m / 5.0,
        y + 2.0 * slope * r.distance_m / 5.0 + second * r.distance_m.powi(2) / 20.0,
        0.0,
        0.0,
        0.0,
    ];
    let bound = controls
        .windows(3)
        .map(|p| (20.0 * (p[2] - 2.0 * p[1] + p[0]) / r.distance_m.powi(2)).abs())
        .fold(0.0_f64, f64::max);
    let limit = vehicle.max_steer.tan() / vehicle.wheelbase / 1.005;
    if controls.iter().any(|v| !v.is_finite())
        || !bound.is_finite()
        || !limit.is_finite()
        || limit <= 0.0
        || bound > limit
    {
        return Err("lane return exceeds the calibrated continuous curvature bound".into());
    }
    let normal = Vec2::new(-axis.y, axis.x);
    let count = (r.distance_m / 0.25).ceil() as usize;
    let mut points = Vec::with_capacity(count + 1);
    for i in 0..=count {
        let u = i as f64 / count as f64;
        let mut values = controls;
        for n in (1..6).rev() {
            for j in 0..n {
                values[j] = (1.0 - u) * values[j] + u * values[j + 1];
            }
        }
        points.push(
            r.lane_start
                .plus(axis.scaled(x + r.distance_m * u))
                .plus(normal.scaled(values[0])),
        );
    }
    points[0] = r.start.position;
    Ok(LaneReturnPlan {
        route: Route::new(points, r.half_width_m)?,
        curvature_bound_m_inv: bound,
        lateral_controls_m: controls,
    })
}

#[cfg(test)]
mod tests;
