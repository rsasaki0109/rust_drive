//! Conditional world-position occupancy from a current ENU pose ellipsoid.
use rustdriving_core::{Pose, Prediction};

/// Add pose uncertainty after transforming nominal forecasts into world ENU.
///
/// Covariance units are m², m·rad and rad² for (x, y, yaw). For a point at
/// offset q from the current ego, J = [[1, 0, -q.y], [0, 1, q.x]]. The linear
/// ellipsoid fits in sigma * sqrt(lambda_max(J P Jᵀ)). Exact rotation adds at
/// most |q| * sigma² * P[yaw,yaw] / 2 by the Taylor remainder bound.
/// This is conditional on the supplied joint pose ellipsoid, not a calibrated
/// probability, object-shape bound or future ego/process covariance estimate.
/// Errors leave the prediction unchanged; nominal centers/radius never change.
pub fn cover_pose_uncertainty(
    prediction: &mut Prediction,
    ego: Pose,
    covariance: [[f64; 3]; 3],
    sigma: f64,
) -> Result<(), String> {
    if !sigma.is_finite() || !(1.0..=5.0).contains(&sigma) {
        return Err("pose forecast sigma must be finite and between 1 and 5".into());
    }
    if !ego.position.finite() || !ego.yaw.is_finite() {
        return Err("pose forecast requires a finite ENU reference pose".into());
    }
    let p = symmetric_psd(covariance)?;
    if prediction.positions.is_empty() {
        return Ok(());
    }
    if !prediction.valid_radii()
        || !prediction.dt.is_finite()
        || prediction.dt <= 0.
        || prediction.positions.iter().any(|q| !q.finite())
    {
        return Err("pose forecast requires finite valid nominal occupancy".into());
    }
    let mut extra = Vec::with_capacity(prediction.positions.len());
    for (i, position) in prediction.positions.iter().enumerate() {
        let q = position.minus(ego.position);
        let a = p[0][0] - 2. * q.y * p[0][2] + q.y * q.y * p[2][2];
        let d = p[1][1] + 2. * q.x * p[1][2] + q.x * q.x * p[2][2];
        let b = p[0][1] + q.x * p[0][2] - q.y * p[1][2] - q.x * q.y * p[2][2];
        let eigenvalue = 0.5 * (a + d + (a - d).hypot(2. * b));
        let padding =
            sigma * eigenvalue.max(0.).sqrt() + 0.5 * q.x.hypot(q.y) * sigma * sigma * p[2][2];
        let radius = prediction.additional_radius_m.get(i).copied().unwrap_or(0.) + padding;
        if !a.is_finite()
            || !b.is_finite()
            || !d.is_finite()
            || !eigenvalue.is_finite()
            || !padding.is_finite()
            || !radius.is_finite()
            || !(prediction.radius + radius).is_finite()
        {
            return Err("pose forecast uncertainty overflow".into());
        }
        extra.push(radius);
    }
    prediction.additional_radius_m = extra;
    Ok(())
}

fn symmetric_psd(mut p: [[f64; 3]; 3]) -> Result<[[f64; 3]; 3], String> {
    let invalid =
        || "pose forecast requires finite symmetric positive semidefinite covariance".to_owned();
    if p.iter().flatten().any(|x| !x.is_finite()) || (0..3).any(|i| p[i][i] < 0.) {
        return Err(invalid());
    }
    let scales = [p[0][0].sqrt(), p[1][1].sqrt(), p[2][2].sqrt()];
    let mut correlation = [[0.; 3]; 3];
    for i in 0..3 {
        correlation[i][i] = if scales[i] > 0. { 1. } else { 0. };
        for j in 0..i {
            if scales[i] == 0. || scales[j] == 0. {
                if p[i][j] != 0. || p[j][i] != 0. {
                    return Err(invalid());
                }
                continue;
            }
            let lower = p[i][j] / scales[i] / scales[j];
            let upper = p[j][i] / scales[i] / scales[j];
            if !lower.is_finite() || !upper.is_finite() || (lower - upper).abs() > 1e-10 {
                return Err(invalid());
            }
            let value = 0.5 * lower + 0.5 * upper;
            if value.abs() > 1. {
                return Err(invalid());
            }
            correlation[i][j] = value;
            correlation[j][i] = value;
            p[i][j] = 0.5 * p[i][j] + 0.5 * p[j][i];
            p[j][i] = p[i][j];
        }
    }
    let c = correlation;
    let determinant = c[0][0] * (c[1][1] * c[2][2] - c[1][2] * c[1][2])
        - c[0][1] * (c[0][1] * c[2][2] - c[0][2] * c[1][2])
        + c[0][2] * (c[0][1] * c[1][2] - c[0][2] * c[1][1]);
    if determinant < 0. {
        return Err(invalid());
    }
    Ok(p)
}
