//! A relative engineering weighting hypothesis, not a measured noise model.

/// For z=fb/disparity, constant disparity variance gives axial variance
/// proportional to z^4. Keep both measured endpoint depths fixed during each
/// normal solve. Fusion/history uncertainty is not modeled by this coefficient.
pub(crate) fn relative(source_z: f64, target_z: f64) -> f64 {
    let source_squared = source_z * source_z;
    let target_squared = target_z * target_z;
    1. / ((source_squared * source_squared + target_squared * target_squared) * 0.5).max(1.)
}

/// The existing accumulator applies a 10 mm Huber multiplier. Supply the
/// coefficient that replaces it with Huber(r*sqrt(relative)) consistently.
pub(crate) fn robust_coefficient(source_z: f64, target_z: f64, residual: f64) -> f64 {
    let relative = relative(source_z, target_z);
    let absolute = residual.abs().max(1e-15);
    let original_huber = (0.01 / absolute).min(1.);
    let standardized_huber = (0.01 / (absolute * relative.sqrt())).min(1.);
    relative * standardized_huber / original_huber
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Camera, Correspondence, Equation, Pose, Residual, add_plane, dot, norm, pyramid, sub,
    };

    #[test]
    fn fourth_power_variance_has_metre_floor_and_symmetric_bounded_endpoints() {
        for (z, expected) in [
            (0.3, 1.),
            (1., 1.),
            (2., 1. / 16.),
            (4., 1. / 256.),
            (5., 1. / 625.),
        ] {
            assert_eq!(relative(z, z), expected);
        }
        assert_eq!(relative(1., 3.), 1. / 41.);
        for a in [0.3, 0.8, 1., 2., 3.5, 5.] {
            for b in [0.3, 0.8, 1., 2., 3.5, 5.] {
                assert_eq!(relative(a, b), relative(b, a));
                assert!(((1. / 625.)..=1.).contains(&relative(a, b)));
            }
        }
    }

    #[test]
    fn all_six_weighted_huber_derivatives_hold_measured_depths_fixed() {
        let pose = Pose::identity().increment([0.02, -0.01, 0.03, 0.01, -0.02, 0.015]);
        let source = [0.2, -0.1, 2.4];
        let target = [0.18, -0.08, 2.5];
        let n = [0.2, -0.3, 1.].map(|v| v / norm([0.2, -0.3, 1.]));
        let weight = relative(source[2], target[2]);
        let q = pose.transform(source);
        let r = dot(n, sub(q, target));
        let mut e = Equation {
            count: 0,
            rms_m: 0.,
            h: [[0.; 6]; 6],
            g: [0.; 6],
            direction_counts: None,
        };
        add_plane(&mut e, n, q, r, weight);
        let cost = |p: Pose| {
            let a = dot(n, sub(p.transform(source), target)).abs();
            weight
                * if a <= 0.01 {
                    0.5 * a * a
                } else {
                    0.01 * (a - 0.005)
                }
        };
        for axis in 0..6 {
            let mut d = [0.; 6];
            d[axis] = 1e-6;
            let plus = cost(pose.increment(d));
            d[axis] = -1e-6;
            let minus = cost(pose.increment(d));
            assert!(((plus - minus) / 2e-6 - e.g[axis]).abs() < 1e-10);
        }
    }

    #[test]
    fn relative_weight_does_not_reduce_raw_rms_or_invent_support() {
        let c = Camera {
            width: 32,
            height: 32,
            fx: 32.,
            fy: 32.,
            cx: 15.5,
            cy: 15.5,
        };
        let levels = pyramid(c, &vec![4.; 32 * 32]);
        let p = Pose::identity().increment([0., 0., 0.005, 0., 0., 0.]);
        let ordinary = crate::equation(
            &levels[2],
            &levels[2],
            p,
            1,
            Correspondence::Forward,
            Residual::TargetPlane,
        );
        for mode in [Residual::DepthVariancePlane, Residual::DepthWhitenedPlane] {
            let weighted =
                crate::equation(&levels[2], &levels[2], p, 1, Correspondence::Forward, mode);
            assert_eq!(weighted.count, ordinary.count);
            assert!(weighted.count >= 200);
            assert_eq!(weighted.rms_m, ordinary.rms_m);
            assert_eq!(weighted.h, ordinary.h.map(|row| row.map(|v| v / 256.)));
            assert_eq!(weighted.g, ordinary.g.map(|v| v / 256.));
        }
    }

    #[test]
    fn standardized_huber_has_correct_six_axis_gradient_and_bounded_cofactor() {
        for z in [0.3, 1., 1.5, 2., 3., 5.] {
            let w = relative(z, z);
            for r in [0., 0.005, 0.02, 0.04, 0.08] {
                let c = robust_coefficient(z, z, r);
                assert!(c >= w * (1. - 1e-14) && c <= w.sqrt() * (1. + 1e-14));
            }
        }
        let p = Pose::identity().increment([0.02, -0.01, 0.03, 0.01, -0.02, 0.015]);
        let source = [0.2, -0.1, 2.4];
        let target = [0.18, -0.08, 2.5];
        let n = [0.2, -0.3, 1.].map(|v| v / norm([0.2, -0.3, 1.]));
        let q = p.transform(source);
        let r = dot(n, sub(q, target));
        let c = robust_coefficient(source[2], target[2], r);
        let mut e = Equation {
            count: 0,
            rms_m: 0.,
            h: [[0.; 6]; 6],
            g: [0.; 6],
            direction_counts: None,
        };
        add_plane(&mut e, n, q, r, c);
        let cost = |p: Pose| {
            let a = (dot(n, sub(p.transform(source), target))
                * relative(source[2], target[2]).sqrt())
            .abs();
            if a <= 0.01 {
                0.5 * a * a
            } else {
                0.01 * (a - 0.005)
            }
        };
        for axis in 0..6 {
            let mut d = [0.; 6];
            d[axis] = 1e-6;
            let plus = cost(p.increment(d));
            d[axis] = -1e-6;
            let minus = cost(p.increment(d));
            assert!(((plus - minus) / 2e-6 - e.g[axis]).abs() < 1e-10);
        }
    }
}
