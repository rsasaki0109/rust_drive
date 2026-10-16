//! One signed residual for two measured planes, in the target optical frame.
use crate::{Equation, Level, Pose, V, add, add_residual, cross, dot, mul, norm, sub};

/// `source_normal` already includes the fitted rotation. Under a left rigid
/// increment it rotates with `query`; the target point/normal remain fixed.
/// The factor 1/2 keeps the ordinary plane scale when both normals agree.
pub(crate) fn midpoint(query: V, target: V, source_normal: V, target_normal: V) -> (f64, [f64; 6]) {
    let n = mul(add(target_normal, source_normal), 0.5);
    let rotation = mul(
        add(cross(query, target_normal), cross(target, source_normal)),
        0.5,
    );
    (
        dot(n, sub(query, target)),
        [n[0], n[1], n[2], rotation[0], rotation[1], rotation[2]],
    )
}

/// Independently search model-to-current correspondences. Accumulate their
/// derivatives directly in the target frame, without an adjoint transformation
/// of a normal equation solved in the opposite frame.
pub(crate) fn reverse(
    e: &mut Equation,
    target: &Level,
    source: &Level,
    pose: Pose,
    scale: usize,
) -> f64 {
    let inverse = pose.inverse();
    let stride = if scale == 1 { 2 } else { 1 };
    let mut squared = 0.;
    for y in (0..target.camera.height).step_by(stride) {
        for x in (0..target.camera.width).step_by(stride) {
            let i = y * target.camera.width + x;
            let nt = target.normals[i];
            if dot(nt, nt) <= 0.25 {
                continue;
            }
            let back = inverse.transform(target.vertices[i]);
            let Some(j) = source.camera.project(back, true) else {
                continue;
            };
            let ns = source.normals[j];
            if dot(ns, ns) <= 0.25
                || norm(sub(back, source.vertices[j])) >= if scale == 4 { 0.12 } else { 0.08 }
                || dot(inverse.rotate(nt), ns) <= 0.8
            {
                continue;
            }
            let (r, jacobian) = midpoint(
                pose.transform(source.vertices[j]),
                target.vertices[i],
                pose.rotate(ns),
                nt,
            );
            add_residual(e, jacobian, r, 1.);
            squared += r * r;
            e.count += 1;
        }
    }
    squared
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Pose, norm};

    fn unit(v: V) -> V {
        mul(v, 1. / norm(v))
    }

    #[test]
    fn one_supported_direction_cannot_substitute_for_the_other() {
        let mut e = Equation {
            count: 900,
            rms_m: 0.,
            h: [[0.; 6]; 6],
            g: [0.; 6],
            direction_counts: Some([199, 701]),
        };
        assert!(!e.supported());
        e.direction_counts = Some([700, 200]);
        assert!(e.supported());
        e.direction_counts = Some([900, 0]);
        assert!(!e.supported());
        e.direction_counts = Some([700, 201]);
        assert!(!e.supported());
    }

    #[test]
    fn all_six_derivatives_include_the_rotating_normal() {
        let pose = Pose::identity().increment([0.02, -0.01, 0.03, 0.05, -0.03, 0.02]);
        let source = [0.4, -0.3, 2.1];
        let target = [0.35, -0.22, 2.14];
        let ns = unit([0.2, -0.3, 1.]);
        let nt = unit([-0.1, 0.2, 1.]);
        let (_, j) = midpoint(pose.transform(source), target, pose.rotate(ns), nt);
        for axis in 0..6 {
            let mut delta = [0.; 6];
            delta[axis] = 1e-6;
            let plus = pose.increment(delta);
            delta[axis] = -1e-6;
            let minus = pose.increment(delta);
            let positive = midpoint(plus.transform(source), target, plus.rotate(ns), nt).0;
            let negative = midpoint(minus.transform(source), target, minus.rotate(ns), nt).0;
            assert!(((positive - negative) / 2e-6 - j[axis]).abs() < 5e-10);
        }
    }

    #[test]
    fn reversing_a_fixed_correspondence_negates_the_residual() {
        let pose = Pose::identity().increment([0.1, -0.03, 0.02, 0.03, 0.02, -0.04]);
        let source = [0.4, -0.3, 2.1];
        let target = [-0.1, 0.2, 2.2];
        let ns = unit([0.1, -0.2, 1.]);
        let nt = unit([-0.15, 0.1, 1.]);
        let forward = midpoint(pose.transform(source), target, pose.rotate(ns), nt).0;
        let inverse = pose.inverse();
        let reverse = midpoint(inverse.transform(target), source, inverse.rotate(nt), ns).0;
        assert!((forward + reverse).abs() < 1e-14);
    }

    #[test]
    fn removes_difference_penalty_and_retains_equal_normal_plane_scale() {
        let query = [0.4, -0.2, 2.];
        let target = [0.3, -0.1, 2.01];
        let nt = unit([-0.2, 0.1, 1.]);
        let ns = unit([0.15, -0.1, 1.]);
        let rt = dot(nt, sub(query, target));
        let rs = dot(ns, sub(query, target));
        let (r, _) = midpoint(query, target, ns, nt);
        assert!((0.5 * (rt * rt + rs * rs) - r * r - 0.25 * (rt - rs).powi(2)).abs() < 1e-15);
        let (same, j) = midpoint(query, target, nt, nt);
        assert_eq!(same, rt);
        for axis in 0..3 {
            assert_eq!(j[axis], nt[axis]);
            // Equal normals have the ordinary derivative only when the
            // point difference is normal: a rotating measured normal still
            // matters for an arbitrary off-surface fixed correspondence.
        }
        let on_plane = sub(target, mul(nt, 0.01));
        let (_, j) = midpoint(on_plane, target, nt, nt);
        let ordinary = cross(on_plane, nt);
        for axis in 0..3 {
            assert!((j[3 + axis] - ordinary[axis]).abs() < 1e-15);
        }
    }
}
