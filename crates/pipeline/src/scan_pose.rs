//! Bounded acquisition-time transforms from estimator history, never world truth.
use rustdriving_core::{Pose, wrap_angle};
use std::collections::VecDeque;

const HISTORY_SECONDS: f64 = 0.35;
const MAX_SAMPLES: usize = 64;

#[derive(Default)]
pub(crate) struct ScanPoseHistory(VecDeque<(f64, Pose)>);
impl ScanPoseHistory {
    pub fn record(&mut self, time: f64, pose: Pose) {
        self.0.push_back((time, pose));
        // Keep one predecessor for interpolation at the age boundary.
        while self.0.len() > MAX_SAMPLES
            || (self.0.len() > 2 && self.0[1].0 < time - HISTORY_SECONDS)
        {
            self.0.pop_front();
        }
    }
    pub fn at(&self, stamp: f64, now: f64) -> Option<Pose> {
        if !stamp.is_finite() || stamp > now + 1e-9 || now - stamp > HISTORY_SECONDS + 1e-9 {
            return None;
        }
        for &(time, pose) in &self.0 {
            if (time - stamp).abs() <= 1e-9 {
                return Some(pose);
            }
        }
        let mut previous: Option<(f64, Pose)> = None;
        for &(time, pose) in &self.0 {
            if let Some((start, from)) = previous
                && start < stamp
                && stamp < time
                && time - start <= 0.25 + 1e-9
            {
                let fraction = (stamp - start) / (time - start);
                return Some(Pose {
                    position: from
                        .position
                        .plus(pose.position.minus(from.position).scaled(fraction)),
                    yaw: wrap_angle(from.yaw + wrap_angle(pose.yaw - from.yaw) * fraction),
                });
            }
            previous = Some((time, pose));
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::Vec2;
    #[test]
    fn interpolates_translation_and_short_yaw_arc_without_extrapolation() {
        let mut h = ScanPoseHistory::default();
        let a = Pose {
            position: Vec2::new(0.0, 1.0),
            yaw: 3.1,
        };
        let b = Pose {
            position: Vec2::new(2.0, 3.0),
            yaw: -3.1,
        };
        h.record(1.0, a);
        h.record(1.1, b);
        let exact_a = h.at(1.0, 1.1).unwrap();
        let exact_b = h.at(1.1, 1.1).unwrap();
        assert_eq!(exact_a.position, a.position);
        assert_eq!(exact_a.yaw, a.yaw);
        assert_eq!(exact_b.position, b.position);
        assert_eq!(exact_b.yaw, b.yaw);
        let mid = h.at(1.05, 1.1).unwrap();
        assert!(mid.position.distance(Vec2::new(1.0, 2.0)) < 1e-12);
        assert!((mid.yaw.abs() - std::f64::consts::PI).abs() < 1e-12);
        assert!(h.at(0.99, 1.1).is_none());
        assert!(h.at(1.11, 1.1).is_none());
        h.record(1.5, b);
        assert!(h.at(1.3, 1.5).is_none()); // Uncovered clock gap.
    }
    #[test]
    fn rejects_expired_history_and_bounds_memory_even_at_high_frequency() {
        let mut h = ScanPoseHistory::default();
        for i in 0..=1000 {
            h.record(i as f64 * 0.0001, Pose::default());
        }
        assert!(h.0.len() <= MAX_SAMPLES);
        assert!(h.at(0.01, 0.1).is_none());
        assert!(h.at(0.1, 0.1).is_some());
        h.record(0.5, Pose::default());
        assert!(h.at(0.1, 0.5).is_none());
    }
}
