//! Accepted acquisition evidence for surfaces lying in measured free space.
use super::*;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Visibility {
    #[default]
    Disabled,
    FreeSpaceDecay,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct VisibilityStats {
    pub tested: usize,
    pub consistent: usize,
    pub occluded: usize,
    pub free_space: usize,
    pub removed: usize,
}

pub(super) fn decay(
    camera: Camera,
    model: &mut [f64],
    weights: &mut [u8],
    depth: &[f64],
    normals: &[V],
    current_from_model: Pose,
) -> VisibilityStats {
    let mut stats = VisibilityStats::default();
    for (i, z) in model.iter_mut().enumerate() {
        if *z <= MIN_Z || *z >= MAX_Z || weights[i] == 0 {
            continue;
        }
        let query = current_from_model.transform(camera.point(i, *z));
        if query[2] >= MAX_Z {
            continue;
        }
        let Some(j) = camera.project(query, false) else {
            continue;
        };
        if depth[j] <= MIN_Z || depth[j] >= MAX_Z || dot(normals[j], normals[j]) <= 0.25 {
            continue;
        }
        stats.tested += 1;
        let gap = depth[j] - query[2];
        let band = 0.03 + 0.01 * query[2];
        if gap > band {
            stats.free_space += 1;
            weights[i] -= 1;
            if weights[i] == 0 {
                *z = 0.;
                stats.removed += 1;
            }
        } else if gap < -band {
            stats.occluded += 1;
        } else {
            stats.consistent += 1;
        }
    }
    stats
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
    fn only_supported_free_space_exhausts_evidence_and_clears_depth() {
        let mut model = vec![2.; 1024];
        let mut weights = vec![7; 1024];
        let mut depth = model.clone();
        let mut normals = vec![[0., 0., 1.]; 1024];
        let j = 16 * 32 + 12;
        model[j..j + 4].copy_from_slice(&[1., 3., 1., 1.]);
        weights[j..j + 4].copy_from_slice(&[2, 4, 2, 2]);
        depth[j + 2] = 0.;
        normals[j + 3] = [0.; 3];
        let first = decay(
            camera(),
            &mut model,
            &mut weights,
            &depth,
            &normals,
            Pose::identity(),
        );
        assert_eq!(
            (
                first.tested,
                first.consistent,
                first.occluded,
                first.free_space,
                first.removed
            ),
            (1022, 1020, 1, 1, 0)
        );
        assert_eq!(&weights[j..j + 4], &[1, 4, 2, 2]);
        let second = decay(
            camera(),
            &mut model,
            &mut weights,
            &depth,
            &normals,
            Pose::identity(),
        );
        assert_eq!((second.free_space, second.removed), (1, 1));
        assert_eq!((model[j], weights[j]), (0., 0));
        assert_eq!((model[j + 1], weights[j + 1]), (3., 4));
    }

    #[test]
    fn visibility_projects_model_into_the_actual_current_view() {
        let c = camera();
        let mut model = vec![0.; 1024];
        let mut weights = vec![0; 1024];
        let mut depth = vec![0.; 1024];
        let mut normals = vec![[0.; 3]; 1024];
        let i = 16 * 32 + 16;
        model[i] = 2.;
        weights[i] = 1;
        depth[16 * 32 + 18] = 2.1;
        normals[16 * 32 + 18] = [0., 0., 1.];
        let mut pose = Pose::identity();
        pose.translation[0] = 0.1;
        let wrong = decay(
            c,
            &mut model,
            &mut weights,
            &depth,
            &normals,
            pose.inverse(),
        );
        assert_eq!(wrong.tested, 0);
        let correct = decay(c, &mut model, &mut weights, &depth, &normals, pose);
        assert_eq!(
            (correct.tested, correct.free_space, correct.removed),
            (1, 1, 1)
        );
        assert_eq!((model[i], weights[i]), (0., 0));
    }

    #[test]
    fn visibility_is_opt_in_and_unsupported_combinations_reject() {
        assert_eq!(TrackerConfig::default().visibility, Visibility::Disabled);
        for option in 0..10 {
            let mut config = TrackerConfig {
                visibility: Visibility::FreeSpaceDecay,
                ..TrackerConfig::default()
            };
            match option {
                0 => config.fusion_sampling = FusionSampling::VisibleMean,
                1 => config.geometry = ModelGeometry::Vertices,
                2 => config.reprojection = Reprojection::TangentPlane,
                3 => config.correspondence = Correspondence::Bilinear,
                4 => config.residual = Residual::SymmetricPlanes,
                5 => config.normals = NormalEstimation::InverseDepthPlane,
                6 => config.photometric = Photometric::Keyframe,
                7 => config.motion_model = MotionModel::ConstantVelocity,
                8 => config.step_control = StepPolicy::FixedSupport,
                _ => config.timing = Timing::PreviousMotion,
            }
            assert!(Tracker::with_config(camera(), config).is_err());
        }
        assert!(
            Tracker::with_config(
                camera(),
                TrackerConfig {
                    visibility: Visibility::FreeSpaceDecay,
                    ..TrackerConfig::default()
                }
            )
            .is_ok()
        );
    }
}
