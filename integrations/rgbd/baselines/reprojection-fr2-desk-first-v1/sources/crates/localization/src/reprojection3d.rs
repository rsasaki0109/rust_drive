//! Bounded RGB-D pixel reprojection refinement on fixed measured landmarks.
//!
//! Points belong to the previous optical camera frame (z forward); pixels belong
//! to the current image. The supplied and returned pose is previous_from_current.
//! Optimize its inverse with left SE(3) increments. No landmarks are dropped,
//! no reference truth is consumed, and no calibrated covariance is claimed.
use crate::registration3d::{Pose3, Quaternion};
use rustdriving_core::Vec3;

const HARD_OBSERVATIONS: usize = 256;
const HARD_ITERATIONS: usize = 8;
const HARD_LINE_SEARCH: usize = 8;
const HARD_CHECKS: usize = HARD_OBSERVATIONS * (1 + HARD_ITERATIONS * (1 + HARD_LINE_SEARCH));

#[derive(Clone, Copy, Debug)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct ReprojectionObservation3d {
    pub previous: Vec3,
    pub current_pixel: [f64; 2],
}
#[derive(Clone, Debug)]
pub struct ReprojectionConfig3d {
    pub max_observations: usize,
    pub max_iterations: usize,
    pub max_line_search_steps: usize,
    /// Individual projection checks; excludes bounded 6x6 matrix algebra.
    pub max_point_checks: usize,
    pub min_observations: usize,
    pub huber_delta_px: f64,
    pub min_depth_m: f64,
    pub max_depth_m: f64,
    /// Condition number of the diagonally scaled weighted normal matrix.
    pub max_condition_number: f64,
    pub max_translation_m: f64,
    pub max_rotation_rad: f64,
    pub translation_tolerance_m: f64,
    pub rotation_tolerance_rad: f64,
}
impl Default for ReprojectionConfig3d {
    fn default() -> Self {
        Self {
            max_observations: 256,
            max_iterations: 8,
            max_line_search_steps: 8,
            max_point_checks: HARD_CHECKS,
            min_observations: 12,
            huber_delta_px: 3.0,
            min_depth_m: 0.1,
            max_depth_m: 10.0,
            max_condition_number: 1e8,
            max_translation_m: 0.5,
            max_rotation_rad: 0.35,
            translation_tolerance_m: 1e-6,
            rotation_tolerance_rad: 1e-6,
        }
    }
}
impl ReprojectionConfig3d {
    pub fn validate(&self) -> Result<(), String> {
        let pos = |x: f64, cap: f64| x.is_finite() && x > 0.0 && x <= cap;
        if self.max_observations > HARD_OBSERVATIONS
            || self.min_observations < 12
            || self.min_observations > self.max_observations
            || !(1..=HARD_ITERATIONS).contains(&self.max_iterations)
            || !(1..=HARD_LINE_SEARCH).contains(&self.max_line_search_steps)
            || !(1..=HARD_CHECKS).contains(&self.max_point_checks)
            || !pos(self.huber_delta_px, 1000.0)
            || !pos(self.min_depth_m, 10.0)
            || !pos(self.max_depth_m, 1000.0)
            || self.max_depth_m <= self.min_depth_m
            || !self.max_condition_number.is_finite()
            || !(1.0..=1e8).contains(&self.max_condition_number)
            || !pos(self.max_translation_m, 0.5)
            || !pos(self.max_rotation_rad, 0.35)
            || !pos(self.translation_tolerance_m, 1e-3)
            || !pos(self.rotation_tolerance_rad, 1e-3)
        {
            return Err("invalid bounded reprojection configuration".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct ReprojectionIteration3d {
    /// Accepted pose after this step, in the optimized inverse convention.
    pub current_from_previous: Pose3,
    pub huber_cost_before: f64,
    pub huber_cost_after: f64,
    pub scale: f64,
    /// Unscaled left increment: translation metres then rotation vector radians.
    pub increment: [f64; 6],
    pub normal_condition_number: f64,
    pub line_search_trials: usize,
}
#[derive(Clone, Debug)]
pub struct ReprojectionResult3d {
    pub pose: Pose3,
    /// RMS of the two-dimensional residual norm (not per coordinate).
    pub initial_rms_px: f64,
    pub final_rms_px: f64,
    pub initial_huber_cost: f64,
    pub final_huber_cost: f64,
    pub iterations: usize,
    pub accepted_steps: usize,
    pub point_checks: usize,
    pub valid_support: usize,
    /// False at the bounded iteration cap: no stationarity guarantee is claimed.
    pub converged: bool,
    pub max_normal_condition_number: f64,
    pub trace: Vec<ReprojectionIteration3d>,
}

pub fn refine_reprojection(
    observations: &[ReprojectionObservation3d],
    camera: &CameraIntrinsics,
    initial_previous_from_current: Pose3,
    cfg: &ReprojectionConfig3d,
) -> Result<ReprojectionResult3d, String> {
    cfg.validate()?;
    if !camera.fx.is_finite()
        || !camera.fy.is_finite()
        || !(1.0..=1e5).contains(&camera.fx)
        || !(1.0..=1e5).contains(&camera.fy)
        || !bounded(camera.cx)
        || !bounded(camera.cy)
    {
        return Err("invalid reprojection camera intrinsics".into());
    }
    if observations.len() < cfg.min_observations || observations.len() > cfg.max_observations {
        return Err("insufficient or excessive reprojection observations".into());
    }
    for o in observations {
        if !o.previous.finite()
            || norm(o.previous) > 1e6
            || !(cfg.min_depth_m..=cfg.max_depth_m).contains(&o.previous.z)
            || !o.current_pixel.iter().all(|v| bounded(*v))
        {
            return Err("invalid reprojection observation".into());
        }
    }
    validate_pose(initial_previous_from_current, cfg)?;
    let mut pose = Pose3 {
        rotation: initial_previous_from_current.rotation.normalized()?,
        ..initial_previous_from_current
    }
    .inverse();
    let mut checks = 0;
    let initial = evaluate(observations, camera, pose, cfg, &mut checks, false)?;
    let mut score = initial;
    let mut trace = Vec::with_capacity(cfg.max_iterations);
    let mut iterations = 0;
    let mut converged = false;
    let mut max_condition: f64 = 0.0;
    for _ in 0..cfg.max_iterations {
        iterations += 1;
        let linear = evaluate(observations, camera, pose, cfg, &mut checks, true)?;
        let (increment, condition) = solve_normal(linear.h, linear.g, cfg.max_condition_number)?;
        max_condition = max_condition.max(condition);
        if increment_norm(&increment, 0) <= cfg.translation_tolerance_m
            && increment_norm(&increment, 3) <= cfg.rotation_tolerance_rad
        {
            converged = true;
            break;
        }
        let mut accepted = None;
        for trial in 0..cfg.max_line_search_steps {
            let alpha = 2.0_f64.powi(-(trial as i32));
            let candidate = left_increment(pose, increment, alpha)?;
            if validate_pose(candidate.inverse(), cfg).is_err() {
                continue;
            }
            match evaluate(observations, camera, candidate, cfg, &mut checks, false) {
                Ok(next) if next.cost < score.cost => {
                    accepted = Some((candidate, next, alpha, trial + 1));
                    break;
                }
                Err(e) if e == "reprojection point-check budget exhausted" => return Err(e),
                _ => {}
            }
        }
        let Some((candidate, next, alpha, trials)) = accepted else {
            return Err("reprojection line search failed".into());
        };
        trace.push(ReprojectionIteration3d {
            current_from_previous: candidate,
            huber_cost_before: score.cost,
            huber_cost_after: next.cost,
            scale: alpha,
            increment,
            normal_condition_number: condition,
            line_search_trials: trials,
        });
        pose = candidate;
        score = next;
    }
    let output = pose.inverse();
    validate_pose(output, cfg)?;
    Ok(ReprojectionResult3d {
        pose: output,
        initial_rms_px: initial.rms,
        final_rms_px: score.rms,
        initial_huber_cost: initial.cost,
        final_huber_cost: score.cost,
        iterations,
        accepted_steps: trace.len(),
        point_checks: checks,
        valid_support: observations.len(),
        converged,
        max_normal_condition_number: max_condition,
        trace,
    })
}
fn bounded(x: f64) -> bool {
    x.is_finite() && x.abs() <= 1e6
}
fn norm(p: Vec3) -> f64 {
    p.x.hypot(p.y).hypot(p.z)
}
fn increment_norm(d: &[f64; 6], start: usize) -> f64 {
    d[start].hypot(d[start + 1]).hypot(d[start + 2])
}
fn validate_pose(p: Pose3, cfg: &ReprojectionConfig3d) -> Result<(), String> {
    let q = p.rotation;
    let qn = q.w.hypot(q.x).hypot(q.y).hypot(q.z);
    if !p.translation.finite()
        || !qn.is_finite()
        || (qn - 1.0).abs() > 1e-10
        || norm(p.translation) > cfg.max_translation_m
        || q.angular_distance(Quaternion::identity()) > cfg.max_rotation_rad
    {
        return Err("invalid or excessive reprojection pose".into());
    }
    Ok(())
}
fn left_increment(p: Pose3, d: [f64; 6], alpha: f64) -> Result<Pose3, String> {
    let axis = Vec3::new(d[3] * alpha, d[4] * alpha, d[5] * alpha);
    let angle = norm(axis);
    let rotation = if angle < 1e-12 {
        Quaternion::identity()
    } else {
        Quaternion::from_axis_angle(axis, angle)?
    };
    let mut next = Pose3 {
        translation: Vec3::new(d[0] * alpha, d[1] * alpha, d[2] * alpha),
        rotation,
    }
    .compose(p);
    next.rotation = next.rotation.normalized()?;
    Ok(next)
}
#[derive(Clone, Copy)]
struct Evaluation {
    rms: f64,
    cost: f64,
    h: [[f64; 6]; 6],
    g: [f64; 6],
}
fn evaluate(
    observations: &[ReprojectionObservation3d],
    camera: &CameraIntrinsics,
    pose: Pose3,
    cfg: &ReprojectionConfig3d,
    checks: &mut usize,
    linearize: bool,
) -> Result<Evaluation, String> {
    let mut out = Evaluation {
        rms: 0.0,
        cost: 0.0,
        h: [[0.0; 6]; 6],
        g: [0.0; 6],
    };
    let mut squared = 0.0;
    for observation in observations {
        if *checks == cfg.max_point_checks {
            return Err("reprojection point-check budget exhausted".into());
        }
        *checks += 1;
        let p = pose.transform(observation.previous);
        if !p.finite() || !(cfg.min_depth_m..=cfg.max_depth_m).contains(&p.z) {
            return Err("invalid projected reprojection depth".into());
        }
        let r = [
            camera.fx * p.x / p.z + camera.cx - observation.current_pixel[0],
            camera.fy * p.y / p.z + camera.cy - observation.current_pixel[1],
        ];
        let length = r[0].hypot(r[1]);
        if !length.is_finite() {
            return Err("nonfinite reprojection residual".into());
        }
        squared += length * length;
        let weight = if length <= cfg.huber_delta_px {
            out.cost += 0.5 * length * length;
            1.0
        } else {
            out.cost += cfg.huber_delta_px * (length - 0.5 * cfg.huber_delta_px);
            cfg.huber_delta_px / length
        };
        if linearize {
            let j = jacobian(p, camera);
            for i in 0..6 {
                out.g[i] += weight * (j[0][i] * r[0] + j[1][i] * r[1]);
                for k in 0..6 {
                    out.h[i][k] += weight * (j[0][i] * j[0][k] + j[1][i] * j[1][k]);
                }
            }
        }
    }
    out.rms = (squared / observations.len() as f64).sqrt();
    if !out.rms.is_finite() || !out.cost.is_finite() {
        return Err("nonfinite reprojection objective".into());
    }
    Ok(out)
}
fn jacobian(p: Vec3, c: &CameraIntrinsics) -> [[f64; 6]; 2] {
    let x = p.x / p.z;
    let y = p.y / p.z;
    [
        [
            c.fx / p.z,
            0.0,
            -c.fx * x / p.z,
            -c.fx * x * y,
            c.fx * (1.0 + x * x),
            -c.fx * y,
        ],
        [
            0.0,
            c.fy / p.z,
            -c.fy * y / p.z,
            -c.fy * (1.0 + y * y),
            c.fy * x * y,
            c.fy * x,
        ],
    ]
}
fn solve_normal(
    h: [[f64; 6]; 6],
    g: [f64; 6],
    max_condition: f64,
) -> Result<([f64; 6], f64), String> {
    let mut scale = [0.0; 6];
    let mut a = [[0.0; 6]; 6];
    for i in 0..6 {
        if !h[i][i].is_finite() || h[i][i] <= 0.0 || !g[i].is_finite() {
            return Err("ill-conditioned reprojection normal matrix".into());
        }
        scale[i] = h[i][i].sqrt();
    }
    for i in 0..6 {
        for j in 0..6 {
            a[i][j] = h[i][j] / (scale[i] * scale[j]);
            if !a[i][j].is_finite() {
                return Err("nonfinite reprojection normal matrix".into());
            }
        }
    }
    let eigen = eigenvalues(a);
    let smallest = eigen.iter().copied().fold(f64::INFINITY, f64::min);
    let largest = eigen.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let condition = largest / smallest;
    if smallest <= 0.0 || !condition.is_finite() || condition > max_condition {
        return Err("ill-conditioned reprojection normal matrix".into());
    }
    let mut l = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..=i {
            let mut value = a[i][j];
            for k in 0..j {
                value -= l[i][k] * l[j][k];
            }
            if i == j {
                if value <= 0.0 || !value.is_finite() {
                    return Err("singular reprojection normal solve".into());
                }
                l[i][j] = value.sqrt();
            } else {
                l[i][j] = value / l[j][j];
            }
        }
    }
    let mut y = [0.0; 6];
    for i in 0..6 {
        let mut value = -g[i] / scale[i];
        for (j, previous) in y.iter().enumerate().take(i) {
            value -= l[i][j] * previous;
        }
        y[i] = value / l[i][i];
    }
    let mut d = [0.0; 6];
    for i in (0..6).rev() {
        let mut value = y[i];
        for j in i + 1..6 {
            value -= l[j][i] * d[j];
        }
        d[i] = value / l[i][i];
    }
    for i in 0..6 {
        d[i] /= scale[i];
        if !d[i].is_finite() {
            return Err("nonfinite reprojection increment".into());
        }
    }
    Ok((d, condition))
}
/// Maximum-pivot symmetric Jacobi, bounded to 256 rotations on a 6x6 matrix.
fn eigenvalues(mut a: [[f64; 6]; 6]) -> [f64; 6] {
    for _ in 0..256 {
        let (mut p, mut q) = (0, 1);
        let mut maximum = a[p][q].abs();
        for (i, row) in a.iter().enumerate() {
            for (j, value) in row.iter().enumerate().skip(i + 1) {
                if value.abs() > maximum {
                    maximum = value.abs();
                    p = i;
                    q = j;
                }
            }
        }
        if maximum <= 1e-14 {
            break;
        }
        let angle = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
        let (s, c) = angle.sin_cos();
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];
        for k in 0..6 {
            if k != p && k != q {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[p][k] = a[k][p];
                a[k][q] = s * akp + c * akq;
                a[q][k] = a[k][q];
            }
        }
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
    }
    std::array::from_fn(|i| a[i][i])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::visual_odometry3d::{
        Correspondence3d, VisualOdometry3dConfig, register_correspondences,
    };
    fn camera() -> CameraIntrinsics {
        CameraIntrinsics {
            fx: 517.3,
            fy: 516.5,
            cx: 318.6,
            cy: 255.3,
        }
    }
    // Independent Rodrigues matrix projection, without the implementation's
    // quaternion transform, Jacobian, or residual helper.
    fn rotate(p: Vec3, omega: [f64; 3]) -> Vec3 {
        let angle = omega.iter().map(|x| x * x).sum::<f64>().sqrt();
        if angle == 0.0 {
            return p;
        }
        let a = omega.map(|x| x / angle);
        let v = [p.x, p.y, p.z];
        let dot: f64 = (0..3).map(|i| a[i] * v[i]).sum();
        let cross = [
            a[1] * v[2] - a[2] * v[1],
            a[2] * v[0] - a[0] * v[2],
            a[0] * v[1] - a[1] * v[0],
        ];
        let out: [f64; 3] = std::array::from_fn(|i| {
            v[i] * angle.cos() + cross[i] * angle.sin() + a[i] * dot * (1.0 - angle.cos())
        });
        Vec3::new(out[0], out[1], out[2])
    }
    fn observations(omega: [f64; 3], t: [f64; 3]) -> Vec<ReprojectionObservation3d> {
        let c = camera();
        (0..96)
            .map(|i| {
                let previous = Vec3::new(
                    (i % 8) as f64 * 0.14 - 0.49,
                    (i / 8 % 6) as f64 * 0.12 - 0.30,
                    1.8 + (i / 48) as f64 * 0.65 + (i % 5) as f64 * 0.07,
                );
                let r = rotate(previous, omega);
                let p = Vec3::new(r.x + t[0], r.y + t[1], r.z + t[2]);
                ReprojectionObservation3d {
                    previous,
                    current_pixel: [c.fx * p.x / p.z + c.cx, c.fy * p.y / p.z + c.cy],
                }
            })
            .collect()
    }
    fn truth(omega: [f64; 3], t: [f64; 3]) -> Pose3 {
        let angle = omega.iter().map(|x| x * x).sum::<f64>().sqrt();
        Pose3 {
            translation: Vec3::new(t[0], t[1], t[2]),
            rotation: if angle == 0.0 {
                Quaternion::identity()
            } else {
                Quaternion::from_axis_angle(Vec3::new(omega[0], omega[1], omega[2]), angle).unwrap()
            },
        }
        .inverse()
    }
    fn translation_error(a: Pose3, b: Pose3) -> f64 {
        norm(Vec3::new(
            a.translation.x - b.translation.x,
            a.translation.y - b.translation.y,
            a.translation.z - b.translation.z,
        ))
    }
    #[test]
    fn recovers_three_axis_motion_and_monotonic_bounded_trace() {
        let omega = [0.045, -0.035, 0.028];
        let t = [0.075, -0.032, 0.024];
        let obs = observations(omega, t);
        let result = refine_reprojection(
            &obs,
            &camera(),
            Pose3::identity(),
            &ReprojectionConfig3d::default(),
        )
        .unwrap();
        assert!(translation_error(result.pose, truth(omega, t)) < 1e-7);
        assert!(
            result
                .pose
                .rotation
                .angular_distance(truth(omega, t).rotation)
                < 1e-7
        );
        assert!(result.final_rms_px < 1e-6);
        assert!(result.converged);
        assert!(result.final_huber_cost < result.initial_huber_cost);
        assert_eq!(result.valid_support, obs.len());
        assert!(result.iterations <= 8 && result.point_checks <= 18688);
        assert_eq!(result.accepted_steps, result.trace.len());
        for step in &result.trace {
            assert!(step.huber_cost_after < step.huber_cost_before);
            assert!(step.normal_condition_number <= 1e8);
        }
        let optimum = refine_reprojection(
            &obs,
            &camera(),
            truth(omega, t),
            &ReprojectionConfig3d::default(),
        )
        .unwrap();
        assert!(optimum.converged);
        assert_eq!(optimum.accepted_steps, 0);
        assert_eq!(optimum.iterations, 1);
        assert_eq!(optimum.point_checks, 2 * obs.len());
    }
    #[test]
    fn pixel_refinement_removes_measured_current_depth_bias() {
        let omega = [0.018, -0.022, 0.014];
        let t = [0.040, -0.025, 0.018];
        let obs = observations(omega, t);
        let c = camera();
        let correspondences: Vec<_> = obs
            .iter()
            .enumerate()
            .map(|(i, o)| {
                let rotated = rotate(o.previous, omega);
                let z = (rotated.z + t[2]) * (1.004 + 0.002 * (i as f64 * 0.7).sin());
                Correspondence3d {
                    previous: o.previous,
                    current: Vec3::new(
                        (o.current_pixel[0] - c.cx) * z / c.fx,
                        (o.current_pixel[1] - c.cy) * z / c.fy,
                        z,
                    ),
                }
            })
            .collect();
        let initial =
            register_correspondences(&correspondences, &VisualOdometry3dConfig::default())
                .unwrap()
                .pose;
        let known = truth(omega, t);
        assert!(translation_error(initial, known) > 0.004);
        let result =
            refine_reprojection(&obs, &c, initial, &ReprojectionConfig3d::default()).unwrap();
        assert!(translation_error(result.pose, known) < 1e-7);
        assert!(result.final_rms_px < result.initial_rms_px * 0.001);
    }
    #[test]
    fn huber_limits_pixel_outlier_influence_without_dropping_support() {
        let omega = [0.014, -0.012, 0.008];
        let t = [0.025, -0.010, 0.020];
        let mut obs = observations(omega, t);
        for (i, o) in obs.iter_mut().enumerate() {
            if i % 5 == 0 {
                o.current_pixel[0] += if i % 2 == 0 { 60.0 } else { -60.0 };
            }
        }
        let cfg = ReprojectionConfig3d::default();
        let result = refine_reprojection(&obs, &camera(), Pose3::identity(), &cfg).unwrap();
        assert_eq!(result.valid_support, 96);
        assert!(result.final_huber_cost < result.initial_huber_cost);
        assert!(translation_error(result.pose, truth(omega, t)) < 0.012);
        assert!(
            result
                .pose
                .rotation
                .angular_distance(truth(omega, t).rotation)
                < 0.008
        );
        assert!(result.final_rms_px > 20.0); // outliers remain in diagnostics.
    }
    #[test]
    fn analytic_left_jacobian_matches_independent_rodrigues_differences() {
        let p = Vec3::new(0.3, -0.2, 2.1);
        let c = camera();
        let analytic = jacobian(p, &c);
        let epsilon = 1e-6;
        for column in 0..6 {
            let project = |sign: f64| {
                let mut omega = [0.0; 3];
                if column >= 3 {
                    omega[column - 3] = sign * epsilon;
                }
                let mut v = rotate(p, omega);
                if column < 3 {
                    match column {
                        0 => v.x += sign * epsilon,
                        1 => v.y += sign * epsilon,
                        _ => v.z += sign * epsilon,
                    }
                }
                [c.fx * v.x / v.z + c.cx, c.fy * v.y / v.z + c.cy]
            };
            let plus = project(1.0);
            let minus = project(-1.0);
            for row in 0..2 {
                assert!(
                    (analytic[row][column] - (plus[row] - minus[row]) / (2.0 * epsilon)).abs()
                        < 1e-6
                );
            }
        }
    }
    #[test]
    fn degenerate_geometry_rejects_even_when_prior_reprojects_perfectly() {
        let obs = vec![
            ReprojectionObservation3d {
                previous: Vec3::new(0.0, 0.0, 2.0),
                current_pixel: [camera().cx, camera().cy]
            };
            12
        ];
        assert!(
            refine_reprojection(
                &obs,
                &camera(),
                Pose3::identity(),
                &ReprojectionConfig3d::default()
            )
            .unwrap_err()
            .contains("conditioned")
        );
        let obs: Vec<_> = (0..20)
            .map(|i| {
                let p = Vec3::new(i as f64 * 0.02, 0.0, 2.0);
                ReprojectionObservation3d {
                    previous: p,
                    current_pixel: [camera().fx * p.x / p.z + camera().cx, camera().cy],
                }
            })
            .collect();
        assert!(
            refine_reprojection(
                &obs,
                &camera(),
                Pose3::identity(),
                &ReprojectionConfig3d::default()
            )
            .is_err()
        );
    }
    #[test]
    fn known_planar_landmarks_are_observable_but_no_projection_may_be_dropped() {
        let c = camera();
        let omega = [0.025, -0.018, 0.012];
        let t = [0.035, -0.021, 0.028];
        let obs: Vec<_> = (0..48)
            .map(|i| {
                let previous = Vec3::new(
                    (i % 8) as f64 * 0.14 - 0.49,
                    (i / 8) as f64 * 0.12 - 0.30,
                    2.0,
                );
                let r = rotate(previous, omega);
                let p = Vec3::new(r.x + t[0], r.y + t[1], r.z + t[2]);
                ReprojectionObservation3d {
                    previous,
                    current_pixel: [c.fx * p.x / p.z + c.cx, c.fy * p.y / p.z + c.cy],
                }
            })
            .collect();
        let cfg = ReprojectionConfig3d::default();
        let result = refine_reprojection(&obs, &c, Pose3::identity(), &cfg).unwrap();
        assert!(translation_error(result.pose, truth(omega, t)) < 1e-7);
        assert!(result.final_rms_px < 1e-6);
        let mut invalid = obs;
        invalid[47].previous.z = cfg.min_depth_m;
        let prior = Pose3 {
            translation: Vec3::new(0.0, 0.0, 0.02),
            ..Pose3::identity()
        };
        // The inverse prior puts this one point behind the permitted near plane.
        // The other 47 points cannot silently constitute a reduced support set.
        assert!(
            refine_reprojection(&invalid, &c, prior, &cfg)
                .unwrap_err()
                .contains("projected reprojection depth")
        );
    }

    #[test]
    fn invalid_inputs_and_hard_resources_reject_transactionally() {
        let obs = observations([0.0; 3], [0.0; 3]);
        let cfg = ReprojectionConfig3d::default();
        let mut c = camera();
        c.fx = 0.0;
        assert!(refine_reprojection(&obs, &c, Pose3::identity(), &cfg).is_err());
        c = camera();
        c.cy = f64::NAN;
        assert!(refine_reprojection(&obs, &c, Pose3::identity(), &cfg).is_err());
        let mut p = Pose3::identity();
        p.rotation.w = 1.01;
        assert!(refine_reprojection(&obs, &camera(), p, &cfg).is_err());
        p = Pose3::identity();
        p.translation.x = 0.51;
        assert!(refine_reprojection(&obs, &camera(), p, &cfg).is_err());
        let mut bad = obs.clone();
        bad[3].current_pixel[1] = f64::NAN;
        assert!(refine_reprojection(&bad, &camera(), Pose3::identity(), &cfg).is_err());
        bad = obs.clone();
        bad[3].previous.z = 0.09;
        assert!(refine_reprojection(&bad, &camera(), Pose3::identity(), &cfg).is_err());
        assert!(refine_reprojection(&obs[..11], &camera(), Pose3::identity(), &cfg).is_err());
        bad = vec![obs[0]; 257];
        assert!(refine_reprojection(&bad, &camera(), Pose3::identity(), &cfg).is_err());
        let cfg = ReprojectionConfig3d {
            max_point_checks: obs.len(),
            ..cfg
        };
        assert!(
            refine_reprojection(&obs, &camera(), Pose3::identity(), &cfg)
                .unwrap_err()
                .contains("budget")
        );
        let cfg = ReprojectionConfig3d {
            max_iterations: 9,
            ..ReprojectionConfig3d::default()
        };
        assert!(cfg.validate().is_err());
        let cfg = ReprojectionConfig3d {
            max_condition_number: f64::NAN,
            ..ReprojectionConfig3d::default()
        };
        assert!(cfg.validate().is_err());
    }
    #[test]
    fn line_search_failure_has_no_success_fallback() {
        let obs = observations([0.0; 3], [0.8, 0.0, 0.0]);
        let cfg = ReprojectionConfig3d {
            max_line_search_steps: 1,
            ..ReprojectionConfig3d::default()
        };
        assert!(
            refine_reprojection(&obs, &camera(), Pose3::identity(), &cfg)
                .unwrap_err()
                .contains("line search failed")
        );
    }
    #[test]
    fn iteration_cap_is_reported_without_convergence_claim() {
        let obs = observations([0.07, -0.05, 0.04], [0.09, -0.05, 0.04]);
        let cfg = ReprojectionConfig3d {
            max_iterations: 1,
            ..ReprojectionConfig3d::default()
        };
        let result = refine_reprojection(&obs, &camera(), Pose3::identity(), &cfg).unwrap();
        assert!(!result.converged);
        assert_eq!(result.iterations, 1);
        assert_eq!(result.accepted_steps, 1);
        assert!(result.final_huber_cost < result.initial_huber_cost);
    }
}
