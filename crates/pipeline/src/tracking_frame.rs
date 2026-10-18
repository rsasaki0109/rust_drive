//! Continuous wheel/gyro coordinates, independent of absolute pose corrections.
use rustdriving_core::{Pose, Prediction, Track, wrap_angle};

pub(crate) struct TrackingFrame {
    pub pose: Pose,
}
impl TrackingFrame {
    pub fn new(pose: Pose) -> Self {
        Self { pose }
    }

    /// Accumulate only the accepted EKF prediction increment, expressed in the
    /// continuous frame. GNSS/map correction increments never enter this pose.
    /// This also preserves the configured chassis/axle prediction convention.
    pub fn advance(&mut self, before: Pose, predicted: Pose) {
        self.pose.position = self.pose.position.plus(
            predicted
                .position
                .minus(before.position)
                .rotated(self.pose.yaw - before.yaw),
        );
        self.pose.yaw = wrap_angle(self.pose.yaw + wrap_angle(predicted.yaw - before.yaw));
    }

    /// Transform from continuous coordinates to the current localized world.
    pub fn to_world(&self, localized: Pose) -> Pose {
        let yaw = wrap_angle(localized.yaw - self.pose.yaw);
        Pose {
            position: localized.position.minus(self.pose.position.rotated(yaw)),
            yaw,
        }
    }
}

pub(crate) fn pose_to_world(frame: Pose, local: Pose) -> Pose {
    Pose {
        position: frame.to_world(local.position),
        yaw: wrap_angle(frame.yaw + local.yaw),
    }
}

pub(crate) fn track_to_world(frame: Pose, mut track: Track) -> Track {
    track.position = frame.to_world(track.position);
    // A velocity is a vector: absolute localization translation and its time
    // derivative are not object motion. Rotate, without adding either term.
    track.velocity = track.velocity.rotated(frame.yaw);
    track
}

pub(crate) fn prediction_to_world(frame: Pose, mut prediction: Prediction) -> Prediction {
    for position in &mut prediction.positions {
        *position = frame.to_world(*position);
    }
    prediction
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::Vec2;

    #[test]
    fn prediction_increments_preserve_turns_and_chassis_translation_across_corrections() {
        let mut frame = TrackingFrame::new(Pose::default());
        // The localized heading has jumped by 90 degrees. A forward + lateral
        // chassis displacement in that frame must retain both components.
        frame.advance(
            Pose {
                position: Vec2::new(100., -20.),
                yaw: std::f64::consts::FRAC_PI_2,
            },
            Pose {
                position: Vec2::new(99.8, -19.),
                yaw: std::f64::consts::FRAC_PI_2 + 0.1,
            },
        );
        assert!(frame.pose.position.distance(Vec2::new(1., 0.2)) < 1e-12);
        assert!((frame.pose.yaw - 0.1).abs() < 1e-12);
        let localized = Pose {
            position: Vec2::new(123., 45.),
            yaw: -3.1,
        };
        let transformed = pose_to_world(frame.to_world(localized), frame.pose);
        assert!(transformed.position.distance(localized.position) < 1e-12);
        assert!(wrap_angle(transformed.yaw - localized.yaw).abs() < 1e-12);
    }
}
