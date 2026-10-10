//! Bounded measured-cloud keyframes in a local optical origin.
//!
//! This estimates local relative poses, without global relocalization, loop
//! closure or an accumulated-pose covariance. A rejected fit supplies no pose.
use crate::registration3d::{Pose3, Registration3dConfig, Registration3dResult, match_scan};
use rustdriving_core::Vec3;

#[derive(Clone, Debug)]
pub struct KeyframeConfig3d {
    pub registration: Registration3dConfig,
    /// Replace the active measured cloud only after an accepted fit at this age.
    pub keyframe_interval_s: f64,
    /// Elapsed time since the last accepted fit, rather than last received scan.
    pub max_unobserved_s: f64,
}
impl Default for KeyframeConfig3d {
    fn default() -> Self {
        Self {
            registration: Registration3dConfig {
                max_correspondence_m: 0.15,
                ..Default::default()
            },
            keyframe_interval_s: 0.10,
            max_unobserved_s: 0.20,
        }
    }
}
impl KeyframeConfig3d {
    pub fn validate(&self) -> Result<(), String> {
        self.registration.validate()?;
        if !self.keyframe_interval_s.is_finite()
            || !self.max_unobserved_s.is_finite()
            || self.keyframe_interval_s <= 0.0
            || self.keyframe_interval_s > self.max_unobserved_s
            || self.max_unobserved_s > 10.0
            || self.registration.max_scan_points > self.registration.max_map_points
        {
            return Err("invalid keyframe timing or point bounds".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct KeyframeRegistration3d {
    pub root_origin_frame_index: usize,
    /// First accepted initialization frame from the current sensor frame.
    pub root_from_scan: Pose3,
    /// First accepted initialization frame from the reference before replacement.
    pub root_from_reference: Pose3,
    pub reference_frame_index: usize,
    pub reference_stamp: f64,
    pub initial_pose: Pose3,
    /// Sensor-to-reference fit; covariance is conditional in that reference.
    pub registration: Registration3dResult,
    pub initialized: bool,
    pub keyframe_replaced: bool,
}

struct Keyframe {
    points: Vec<Vec3>,
    frame_index: usize,
    stamp: f64,
    root_from_reference: Pose3,
    last_relative_pose: Pose3,
}

pub struct KeyframeLocalizer3d {
    config: KeyframeConfig3d,
    keyframe: Option<Keyframe>,
    root_origin_frame_index: Option<usize>,
    last_observed: Option<(usize, f64)>,
    last_accepted_stamp: Option<f64>,
    lost: bool,
}
impl KeyframeLocalizer3d {
    pub fn new(config: KeyframeConfig3d) -> Result<Self, String> {
        config.validate()?;
        Ok(Self {
            config,
            keyframe: None,
            root_origin_frame_index: None,
            last_observed: None,
            last_accepted_stamp: None,
            lost: false,
        })
    }
    /// Explicitly start a new independent coordinate origin. This cannot bridge
    /// a lost trajectory; callers must retain the change of map epoch.
    pub fn reset(&mut self) {
        self.keyframe = None;
        self.root_origin_frame_index = None;
        self.last_observed = None;
        self.last_accepted_stamp = None;
        self.lost = false;
    }
    pub fn is_lost(&self) -> bool {
        self.lost
    }
    pub fn last_accepted_stamp(&self) -> Option<f64> {
        self.last_accepted_stamp
    }
    pub fn keyframe_frame_index(&self) -> Option<usize> {
        self.keyframe.as_ref().map(|k| k.frame_index)
    }
    pub fn observe(
        &mut self,
        frame_index: usize,
        stamp: f64,
        scan: &[Vec3],
    ) -> Result<KeyframeRegistration3d, String> {
        if self.lost {
            return Err("keyframe localization lost; explicit new origin required".into());
        }
        if !stamp.is_finite()
            || !(0.0..=1e12).contains(&stamp)
            || self
                .last_observed
                .is_some_and(|(index, previous)| frame_index <= index || stamp <= previous)
        {
            return Err("invalid or non-increasing keyframe observation".into());
        }
        // A finite chronological acquisition advances the observed clock even
        // when its cloud is malformed. It cannot hide expiry or permit a later
        // retrograde acquisition; only an accepted fit renews pose validity.
        self.last_observed = Some((frame_index, stamp));
        if self
            .last_accepted_stamp
            .is_some_and(|accepted| stamp - accepted > self.config.max_unobserved_s + 1e-9)
        {
            self.lost = true;
            return Err("keyframe accepted-pose age exceeded; localization lost".into());
        }
        if scan.len() < self.config.registration.min_pairs
            || scan.len() > self.config.registration.max_scan_points
            || scan.iter().any(|p| !p.finite())
        {
            if self.keyframe.is_none() {
                self.lost = true;
            }
            return Err("invalid or oversized keyframe cloud".into());
        }
        let Some(reference) = self.keyframe.as_mut() else {
            let registration =
                match match_scan(scan, scan, Pose3::identity(), &self.config.registration) {
                    Ok(fit) => fit,
                    Err(reason) => {
                        self.lost = true;
                        return Err(format!("keyframe initialization rejected: {reason}"));
                    }
                };
            self.keyframe = Some(Keyframe {
                points: scan.to_vec(),
                frame_index,
                stamp,
                root_from_reference: Pose3::identity(),
                last_relative_pose: Pose3::identity(),
            });
            self.root_origin_frame_index = Some(frame_index);
            self.last_accepted_stamp = Some(stamp);
            return Ok(KeyframeRegistration3d {
                root_origin_frame_index: frame_index,
                root_from_scan: Pose3::identity(),
                root_from_reference: Pose3::identity(),
                reference_frame_index: frame_index,
                reference_stamp: stamp,
                initial_pose: Pose3::identity(),
                registration,
                initialized: true,
                keyframe_replaced: false,
            });
        };
        let prior = reference.last_relative_pose;
        let registration = match_scan(scan, &reference.points, prior, &self.config.registration)?;
        let root_from_scan = reference.root_from_reference.compose(registration.pose);
        if !root_from_scan.translation.finite() || root_from_scan.rotation.normalized().is_err() {
            self.lost = true;
            return Err("nonfinite accumulated keyframe pose; localization lost".into());
        }
        let replaced = stamp - reference.stamp + 1e-9 >= self.config.keyframe_interval_s;
        let result = KeyframeRegistration3d {
            root_origin_frame_index: self.root_origin_frame_index.unwrap(),
            root_from_scan,
            root_from_reference: reference.root_from_reference,
            reference_frame_index: reference.frame_index,
            reference_stamp: reference.stamp,
            initial_pose: prior,
            registration,
            initialized: false,
            keyframe_replaced: replaced,
        };
        self.last_accepted_stamp = Some(stamp);
        if replaced {
            *reference = Keyframe {
                points: scan.to_vec(),
                frame_index,
                stamp,
                root_from_reference: root_from_scan,
                last_relative_pose: Pose3::identity(),
            };
        } else {
            reference.last_relative_pose = result.registration.pose;
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registration3d::Quaternion;
    fn geometry() -> Vec<Vec3> {
        (0..180)
            .map(|i| {
                let t = i as f64;
                Vec3::new(
                    (t * 1.73).sin() * 1.3 + (t * 0.21).cos() * 0.2,
                    (t * 0.77).cos() * 0.9,
                    (t * 0.49).sin() * 0.7 + (t * 0.11).cos() * 0.1,
                )
            })
            .collect()
    }
    fn assert_pose(actual: Pose3, expected: Pose3) {
        assert!((actual.translation.x - expected.translation.x).abs() < 1e-6);
        assert!((actual.translation.y - expected.translation.y).abs() < 1e-6);
        assert!((actual.translation.z - expected.translation.z).abs() < 1e-6);
        assert!(actual.rotation.angular_distance(expected.rotation) < 1e-6);
    }
    #[test]
    fn replacing_measured_keyframes_preserves_the_original_coordinate_frame() {
        let world = geometry();
        let mut local = KeyframeLocalizer3d::new(KeyframeConfig3d::default()).unwrap();
        assert!(local.observe(0, 0., &world).unwrap().initialized);
        let a = Pose3 {
            translation: Vec3::new(0.03, -0.01, 0.02),
            rotation: Quaternion::from_axis_angle(Vec3::new(0.3, 0.7, 0.2), 0.02).unwrap(),
        };
        let scan: Vec<_> = world.iter().map(|p| a.inverse().transform(*p)).collect();
        let result = local.observe(1, 0.1, &scan).unwrap();
        assert!(result.keyframe_replaced);
        assert_eq!(result.reference_frame_index, 0);
        assert_pose(result.root_from_scan, a);
        let b = Pose3 {
            translation: Vec3::new(0.05, -0.015, 0.035),
            rotation: Quaternion::from_axis_angle(Vec3::new(0.3, 0.7, 0.2), 0.03).unwrap(),
        };
        let scan: Vec<_> = world.iter().map(|p| b.inverse().transform(*p)).collect();
        let result = local.observe(2, 0.15, &scan).unwrap();
        assert_eq!(result.reference_frame_index, 1);
        assert_eq!(result.root_origin_frame_index, 0);
        assert_pose(result.initial_pose, Pose3::identity());
        assert_pose(result.root_from_scan, b);
        assert_eq!(result.registration.conditioning.ambiguity_probes, 12);
    }
    #[test]
    fn rejected_cloud_keeps_map_and_pose_clock_then_a_measured_fit_can_recover() {
        let cloud = geometry();
        let mut local = KeyframeLocalizer3d::new(KeyframeConfig3d::default()).unwrap();
        local.observe(0, 0., &cloud).unwrap();
        let unrelated: Vec<_> = cloud
            .iter()
            .map(|p| Vec3::new(p.x + 20., p.y, p.z))
            .collect();
        assert!(local.observe(1, 0.05, &unrelated).is_err());
        assert_eq!(local.last_accepted_stamp(), Some(0.));
        assert_eq!(local.keyframe_frame_index(), Some(0));
        let fit = local.observe(2, 0.15, &cloud).unwrap();
        assert_eq!(fit.reference_frame_index, 0);
        assert_pose(fit.root_from_scan, Pose3::identity());
    }
    #[test]
    fn expiry_latches_loss_until_explicit_new_origin() {
        let cloud = geometry();
        let mut local = KeyframeLocalizer3d::new(KeyframeConfig3d::default()).unwrap();
        local.observe(0, 0., &cloud).unwrap();
        assert!(local.observe(1, 0.201, &cloud).is_err());
        assert!(local.is_lost());
        assert_eq!(local.last_accepted_stamp(), Some(0.));
        assert!(local.observe(2, 0.21, &cloud).is_err());
        local.reset();
        let result = local.observe(2, 0.21, &cloud).unwrap();
        assert!(result.initialized);
        assert_eq!(result.root_origin_frame_index, 2);
    }
    #[test]
    fn duplicate_or_invalid_acquisitions_never_refresh_permission() {
        let cloud = geometry();
        let mut local = KeyframeLocalizer3d::new(KeyframeConfig3d::default()).unwrap();
        local.observe(0, 0., &cloud).unwrap();
        assert!(local.observe(0, 0.01, &cloud).is_err());
        assert!(local.observe(1, 0., &cloud).is_err());
        assert!(local.observe(1, f64::NAN, &cloud).is_err());
        let mut bad = cloud.clone();
        bad[0].z = f64::INFINITY;
        assert!(local.observe(1, 0.02, &bad).is_err());
        assert_eq!(local.last_accepted_stamp(), Some(0.));
        assert_eq!(local.keyframe_frame_index(), Some(0));
        assert!(local.observe(1, 0.05, &cloud).is_err());
        assert!(local.observe(2, 0.01, &cloud).is_err());
        assert!(local.observe(2, 0.05, &cloud).is_ok());
    }
    #[test]
    fn malformed_acquisition_cannot_hide_expiry_or_allow_retrograde_recovery() {
        let cloud = geometry();
        let mut local = KeyframeLocalizer3d::new(KeyframeConfig3d::default()).unwrap();
        local.observe(0, 0.0, &cloud).unwrap();
        assert!(local.observe(1, 1.0, &[]).is_err());
        assert!(local.is_lost());
        assert_eq!(local.last_accepted_stamp(), Some(0.0));
        assert!(local.observe(1, 0.1, &cloud).is_err());
        assert!(local.observe(2, 1.01, &cloud).is_err());
        local.reset();
        assert!(local.observe(2, 1.01, &cloud).unwrap().initialized);
    }
    #[test]
    fn degenerate_initial_map_is_rejected_without_a_silent_origin_change() {
        let plane: Vec<_> = (0..10)
            .flat_map(|x| (0..10).map(move |y| Vec3::new(x as f64 * 0.06, y as f64 * 0.06, 2.)))
            .collect();
        let mut local = KeyframeLocalizer3d::new(KeyframeConfig3d::default()).unwrap();
        assert!(local.observe(0, 0., &plane).is_err());
        assert!(local.is_lost());
        assert!(local.observe(1, 0.05, &geometry()).is_err());
        assert_eq!(local.last_accepted_stamp(), None);
    }
}
