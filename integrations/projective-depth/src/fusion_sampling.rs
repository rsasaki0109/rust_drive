//! Visible-surface averaging for model rasterization, separate from pose fitting.
use super::*;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FusionSampling {
    #[default]
    Nearest,
    /// Locate the nearest surface before averaging compatible projected points.
    /// Multiple rays from one acquisition do not increase its frame support.
    VisibleMean,
}

pub(super) fn raster(
    camera: Camera,
    depth: &[f64],
    weights: &[u8],
    pose: Pose,
) -> (Vec<f64>, Vec<u8>, RasterStats) {
    let (nearest, _, mut stats) = raster_with_tangents(camera, depth, weights, pose, None);
    let mut moment = vec![0.; depth.len()];
    let mut total = vec![0.; depth.len()];
    let mut support = vec![0; depth.len()];
    let mut contributing = 0;
    for (i, &z) in depth.iter().enumerate() {
        if z <= MIN_Z || z >= MAX_Z {
            continue;
        }
        let point = pose.transform(camera.point(i, z));
        if point[2] >= MAX_Z {
            continue;
        }
        let Some(j) = camera.project(point, false) else {
            continue;
        };
        if point[2] - nearest[j] < 0.03 + 0.01 * nearest[j] {
            let w = f64::from(weights[i]);
            moment[j] += w * point[2];
            total[j] += w;
            support[j] = support[j].max(weights[i].min(16));
            contributing += 1;
        }
    }
    for (z, w) in moment.iter_mut().zip(total) {
        if w > 0. {
            *z /= w;
        }
    }
    stats.surface_merged = contributing - stats.selected;
    (moment, support, stats)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera() -> Camera {
        Camera {
            width: 32,
            height: 32,
            fx: 32.,
            fy: 32.,
            cx: 15.5,
            cy: 15.5,
        }
    }

    #[test]
    fn surface_mean_excludes_background_and_preserves_frame_support() {
        let c = camera();
        let mut depth = vec![0.; 1024];
        let mut weights = vec![0; 1024];
        depth[16 * 32 + 16] = 1.;
        depth[16 * 32 + 17] = 1.02;
        depth[17 * 32 + 17] = 2.;
        weights[16 * 32 + 16] = 2;
        weights[16 * 32 + 17] = 6;
        weights[17 * 32 + 17] = 16;
        let mut pose = Pose::identity();
        pose.translation[2] = 1.;
        let (map, support, stats) = raster(c, &depth, &weights, pose);
        let j = 16 * 32 + 16;
        assert!((map[j] - 2.015).abs() < 1e-14);
        assert_eq!(support[j], 6);
        assert_eq!(
            (stats.projected, stats.selected, stats.surface_merged),
            (3, 1, 1)
        );
        let (_, support, _) = raster(c, &depth, &vec![1; 1024], pose);
        assert_eq!(support[j], 1);
    }

    #[test]
    fn surface_band_is_anchored_to_minimum_independent_of_traversal() {
        let c = camera();
        let mut depth = vec![0.; 1024];
        // All three points map to one cell. The middle point must not create
        // a chain that admits the farthest point outside the nearest band.
        for values in [[1., 1.04, 1.08], [1.08, 1.04, 1.]] {
            depth[16 * 32 + 16] = values[0];
            depth[16 * 32 + 17] = values[1];
            depth[17 * 32 + 17] = values[2];
            let mut pose = Pose::identity();
            pose.translation[2] = 1.;
            let (map, _, stats) = raster(c, &depth, &vec![1; 1024], pose);
            assert!((map[16 * 32 + 16] - 2.02).abs() < 1e-14);
            assert_eq!(stats.surface_merged, 1);
        }
    }

    #[test]
    fn original_configuration_remains_default_and_mixed_policies_reject() {
        assert_eq!(
            TrackerConfig::default().fusion_sampling,
            FusionSampling::Nearest
        );
        for option in 0..9 {
            let mut config = TrackerConfig {
                fusion_sampling: FusionSampling::VisibleMean,
                ..TrackerConfig::default()
            };
            match option {
                0 => config.geometry = ModelGeometry::Vertices,
                1 => config.reprojection = Reprojection::TangentPlane,
                2 => config.correspondence = Correspondence::Reciprocal,
                3 => config.residual = Residual::DepthVariancePlane,
                4 => config.normals = NormalEstimation::InverseDepthPlane,
                5 => config.photometric = Photometric::Keyframe,
                6 => config.motion_model = MotionModel::ConstantVelocity,
                7 => config.step_control = StepPolicy::FixedSupport,
                _ => config.timing = Timing::PreviousMotion,
            }
            assert!(Tracker::with_config(camera(), config).is_err());
        }
        assert!(
            Tracker::with_config(
                camera(),
                TrackerConfig {
                    fusion_sampling: FusionSampling::VisibleMean,
                    ..TrackerConfig::default()
                }
            )
            .is_ok()
        );
    }
}
