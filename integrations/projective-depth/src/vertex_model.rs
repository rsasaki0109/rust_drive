//! Bounded projective XYZ model. Cells are visibility indices, not camera rays.
use super::{
    Camera, FusionStats, Level, MAX_Z, MIN_Z, Pose, RasterStats, V, add, cross, mul, norm, sub,
};

pub(super) fn pyramid(camera: Camera, mut points: Vec<V>) -> Vec<Level> {
    let mut levels = Vec::with_capacity(3);
    for scale in [1, 2, 4] {
        let c = camera.scaled(scale);
        let mut normals = vec![[0.; 3]; points.len()];
        for y in 1..c.height - 1 {
            for x in 1..c.width - 1 {
                let i = y * c.width + x;
                let z = points[i][2];
                if z <= MIN_Z || z >= MAX_Z {
                    continue;
                }
                if [i + 1, i - 1, i + c.width, i - c.width]
                    .iter()
                    .any(|&j| points[j][2] <= MIN_Z || (points[j][2] - z).abs() >= 0.05 + 0.01 * z)
                {
                    continue;
                }
                let n = cross(
                    sub(points[i + 1], points[i - 1]),
                    sub(points[i + c.width], points[i - c.width]),
                );
                let length = norm(n);
                if length > 1e-10 {
                    normals[i] = mul(n, 1. / length);
                }
            }
        }
        let next = if scale == 4 {
            Vec::new()
        } else {
            (0..c.width * c.height / 4)
                .map(|i| {
                    let k = i / (c.width / 2) * 2 * c.width + i % (c.width / 2) * 2;
                    let block = [
                        points[k],
                        points[k + 1],
                        points[k + c.width],
                        points[k + c.width + 1],
                    ];
                    let lo = block.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
                    let hi = block.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
                    if lo < MIN_Z || hi > MAX_Z || hi - lo > 0.05 + 0.01 * lo {
                        [0.; 3]
                    } else {
                        mul(add(add(add(block[0], block[1]), block[2]), block[3]), 0.25)
                    }
                })
                .collect()
        };
        levels.push(Level {
            camera: c,
            vertices: points,
            normals,
        });
        points = next;
    }
    levels.reverse();
    levels
}

pub(super) fn raster(
    camera: Camera,
    points: &[V],
    weights: &[u8],
    pose: Pose,
) -> (Vec<V>, Vec<u8>, RasterStats) {
    let mut output = vec![[0.; 3]; points.len()];
    let mut support = vec![0; points.len()];
    let mut stats = RasterStats::default();
    for (i, &point) in points.iter().enumerate() {
        if point[2] <= MIN_Z || point[2] >= MAX_Z {
            continue;
        }
        stats.source_valid += 1;
        let p = pose.transform(point);
        if p[2] >= MAX_Z {
            continue;
        }
        let Some(j) = camera.project(p, false) else {
            continue;
        };
        stats.projected += 1;
        if output[j][2] == 0. || p[2] < output[j][2] {
            if output[j][2] == 0. {
                stats.selected += 1;
            }
            output[j] = p;
            support[j] = weights[i];
        }
    }
    (output, support, stats)
}

pub(super) fn fuse(
    camera: Camera,
    model: &mut [V],
    weights: &mut [u8],
    points: &[V],
    pose: Pose,
) -> FusionStats {
    let (new, present, measurement) = raster(camera, points, &vec![1; points.len()], pose);
    let mut stats = FusionStats {
        measurement,
        ..FusionStats::default()
    };
    for i in 0..model.len() {
        if present[i] == 0 {
            continue;
        }
        let old = model[i][2];
        if weights[i] == 0 {
            model[i] = new[i];
            weights[i] = 1;
            stats.filled += 1;
        } else if (new[i][2] - old).abs() < 0.03 + 0.01 * old {
            let w = f64::from(weights[i].min(16));
            model[i] = mul(add(mul(model[i], w), new[i]), 1. / (w + 1.));
            weights[i] = (weights[i] + 1).min(16);
            stats.averaged += 1;
        } else if new[i][2] < old - 0.03 - 0.01 * old {
            model[i] = new[i];
            weights[i] = 1;
            stats.replaced += 1;
        } else {
            stats.ignored += 1;
        }
    }
    stats
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ModelGeometry, Tracker, TrackerConfig, dot};

    fn camera() -> Camera {
        Camera {
            width: 128,
            height: 96,
            fx: 110.,
            fy: 110.,
            cx: 63.5,
            cy: 47.5,
        }
    }
    fn plane(c: Camera) -> (V, Vec<V>) {
        let n = mul([0.3, -0.2, 1.], 1. / norm([0.3, -0.2, 1.]));
        (
            n,
            (0..c.width * c.height)
                .map(|i| c.point(i, 2. / dot(n, c.point(i, 1.))))
                .collect(),
        )
    }
    #[test]
    fn xyz_pyramid_centroids_and_normals_remain_on_a_tilted_plane() {
        let c = camera();
        let (n, points) = plane(c);
        for level in pyramid(c, points) {
            assert!(level.normals.iter().filter(|v| norm(**v) > 0.5).count() > 500);
            for p in &level.vertices {
                assert!((dot(n, *p) - 2.).abs() < 1e-13);
            }
            for normal in level.normals.iter().filter(|v| norm(**v) > 0.5) {
                assert!(norm(sub(*normal, n)) < 1e-12);
            }
        }
    }
    #[test]
    fn forty_view_warps_preserve_the_plane_instead_of_snapping_xy() {
        let c = camera();
        let (n, mut points) = plane(c);
        let mut depth: Vec<_> = points.iter().map(|p| p[2]).collect();
        let mut weights = vec![1; points.len()];
        let mut old_weights = weights.clone();
        let mut previous = Pose::identity();
        let mut peak_depth_rms = 0_f64;
        for index in (1..=20).chain((0..20).rev()) {
            let i = f64::from(index);
            let view = Pose::identity().increment([
                0.004 * i,
                -0.001 * i,
                0.001 * i,
                0.001 * i,
                -0.0015 * i,
                0.0007 * i,
            ]);
            let transform = view.inverse().compose(previous);
            let (next, w, _) = raster(c, &points, &weights, transform);
            points = next;
            weights = w;
            let (next, w, _) = crate::raster(c, &depth, &old_weights, transform);
            depth = next;
            old_weights = w;
            let normal = view.inverse().rotate(n);
            let offset = 2. - dot(n, view.translation);
            for (p, &w) in points.iter().zip(&weights) {
                if w > 0 {
                    assert!((dot(normal, *p) - offset).abs() < 1e-12);
                }
            }
            let mut squared = 0.;
            let mut count = 0;
            for (j, &w) in old_weights.iter().enumerate() {
                if w > 0 {
                    squared += (dot(normal, c.point(j, depth[j])) - offset).powi(2);
                    count += 1;
                }
            }
            peak_depth_rms = peak_depth_rms.max((squared / f64::from(count)).sqrt());
            previous = view;
        }
        assert!(weights.iter().filter(|&&w| w > 0).count() > 10000);
        assert!(peak_depth_rms > 0.01);
    }
    #[test]
    fn capped_fusion_averages_all_three_coordinates() {
        let c = camera();
        let i = 48 * c.width + 64;
        let mut model = vec![[0.; 3]; c.width * c.height];
        let mut weights = vec![0; model.len()];
        let mut source = model.clone();
        let old = c.point(i, 2.);
        let new = add(c.point(i, 2.01), [0.005, -0.002, 0.]);
        model[i] = old;
        weights[i] = 16;
        source[i] = new;
        let stats = fuse(c, &mut model, &mut weights, &source, Pose::identity());
        assert_eq!(stats.averaged, 1);
        assert_eq!(weights[i], 16);
        assert!(norm(sub(model[i], mul(add(mul(old, 16.), new), 1. / 17.))) < 1e-15);
        assert!(norm(sub(model[i], c.point(i, model[i][2]))) > 1e-4);
    }
    #[test]
    fn vertex_raster_keeps_nearest_depth_first_index_ties_and_exact_xyz() {
        let c = camera();
        let i = 48 * c.width + 64;
        let mut points = vec![[0.; 3]; c.width * c.height];
        let mut weights = vec![0; points.len()];
        let p = add(c.point(i, 2.), [0.005, 0., 0.]);
        points[0] = p;
        points[1] = p;
        weights[0] = 2;
        weights[1] = 7;
        let (output, support, stats) = raster(c, &points, &weights, Pose::identity());
        assert_eq!(output[i], p);
        assert_eq!(support[i], 2);
        assert_eq!(stats.selected, 1);
        points[2] = c.point(i, 1.);
        weights[2] = 4;
        let (output, support, stats) = raster(c, &points, &weights, Pose::identity());
        assert_eq!(output[i], points[2]);
        assert_eq!(support[i], 4);
        assert_eq!(stats.selected, 1);
    }
    #[test]
    fn rejected_vertex_update_preserves_the_model_and_accepted_clock() {
        let c = camera();
        let depth: Vec<_> = (0..c.width * c.height)
            .map(|i| {
                2. + 0.2 * ((i % c.width) as f64 * 0.07).sin() * ((i / c.width) as f64 * 0.09).sin()
            })
            .collect();
        let mut tracker = Tracker::with_config(
            c,
            TrackerConfig {
                geometry: ModelGeometry::Vertices,
                ..TrackerConfig::default()
            },
        )
        .unwrap();
        assert!(tracker.step(1., &depth).accepted);
        let model = tracker.model.as_ref().unwrap().pyramid[2].vertices.clone();
        assert!(!tracker.step(1.03, &vec![0.; depth.len()]).accepted);
        assert_eq!(tracker.model.as_ref().unwrap().pyramid[2].vertices, model);
        assert_eq!(tracker.last_accepted, Some(1.));
        assert!(!tracker.step(1.03, &depth).accepted);
        assert!(tracker.step(1.06, &depth).accepted);
    }
}
