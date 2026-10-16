//! Causal SE(3) initialization from accepted depth-acquisition poses only.
use crate::{Pose, V, add, cross, mul, norm, sub};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MotionModel {
    #[default]
    Stationary,
    ConstantVelocity,
}
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Timing {
    #[default]
    Disabled,
    PreviousMotion,
}
#[derive(Clone, Copy)]
pub(super) struct Interval {
    pub previous: f64,
    pub latest: f64,
    twist: Option<[f64; 6]>,
}
#[derive(Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum WarpStatus {
    Coincident,
    Unavailable,
    Compensated,
    IncrementLimit,
}
#[derive(Debug, Serialize)]
pub struct TimeWarp {
    pub depth_timestamp: f64,
    pub image_timestamp: f64,
    pub previous_timestamp: Option<f64>,
    pub latest_timestamp: Option<f64>,
    pub status: WarpStatus,
    pub image_from_depth: Option<Pose>,
}
pub(super) fn time_warp(depth: f64, image: f64, interval: Option<Interval>) -> TimeWarp {
    let mut out = TimeWarp {
        depth_timestamp: depth,
        image_timestamp: image,
        previous_timestamp: interval.map(|v| v.previous),
        latest_timestamp: interval.map(|v| v.latest),
        status: WarpStatus::Unavailable,
        image_from_depth: None,
    };
    if depth == image {
        out.status = WarpStatus::Coincident;
        out.image_from_depth = Some(Pose::identity());
    } else if let Some(rate) = interval
        && let Some(twist) = rate.twist
    {
        let scaled = twist.map(|v| v * ((depth - image) / (rate.latest - rate.previous)));
        out.status = WarpStatus::IncrementLimit;
        if scaled.iter().all(|v| v.is_finite()) && norm([scaled[3], scaled[4], scaled[5]]) <= 0.15 {
            let pose = exp_body(scaled);
            if pose.finite_rigid() && norm(pose.translation) <= 0.1 {
                out.status = WarpStatus::Compensated;
                out.image_from_depth = Some(pose);
            }
        }
    }
    out
}
#[derive(Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    Bootstrap,
    Predicted,
    IncrementLimit,
    AmbiguousRotation,
}
#[derive(Debug, Serialize)]
pub struct Prediction {
    pub status: Status,
    pub previous_timestamp: Option<f64>,
    pub latest_timestamp: Option<f64>,
    pub history_interval_s: Option<f64>,
    pub horizon_s: Option<f64>,
    pub first_from_prediction: Option<Pose>,
    pub latest_from_prediction: Option<Pose>,
}
#[derive(Default)]
pub(super) struct History {
    previous: Option<(f64, Pose)>,
    latest: Option<(f64, Pose)>,
}
impl History {
    pub(super) fn interval(&self) -> Option<Interval> {
        let (a, first) = self.previous?;
        let (b, last) = self.latest?;
        Some(Interval {
            previous: a,
            latest: b,
            twist: log_body(first.inverse().compose(last)),
        })
    }
    pub(super) fn observe(&mut self, stamp: f64, pose: Pose) {
        debug_assert!(self.latest.is_none_or(|(t, _)| stamp > t));
        self.previous = self.latest;
        self.latest = Some((stamp, pose));
    }
    pub(super) fn predict(&self, stamp: f64) -> Prediction {
        let mut report = Prediction {
            status: Status::Bootstrap,
            previous_timestamp: self.previous.map(|p| p.0),
            latest_timestamp: self.latest.map(|p| p.0),
            history_interval_s: None,
            horizon_s: self.latest.map(|p| stamp - p.0),
            first_from_prediction: None,
            latest_from_prediction: None,
        };
        let (Some((a, first)), Some((b, last))) = (self.previous, self.latest) else {
            return report;
        };
        let dt = b - a;
        report.history_interval_s = Some(dt);
        let Some(twist) = log_body(first.inverse().compose(last)) else {
            report.status = Status::AmbiguousRotation;
            return report;
        };
        let scaled = twist.map(|v| v * ((stamp - b) / dt));
        report.status = Status::IncrementLimit;
        if !scaled.iter().all(|v| v.is_finite()) || norm([scaled[3], scaled[4], scaled[5]]) > 0.15 {
            return report;
        }
        let increment = exp_body(scaled);
        if !increment.finite_rigid() || norm(increment.translation) > 0.1 {
            return report;
        }
        let predicted = last.compose(increment);
        if !predicted.finite_rigid() {
            return report;
        }
        report.status = Status::Predicted;
        report.first_from_prediction = Some(predicted);
        report.latest_from_prediction = Some(increment);
        report
    }
}
fn log_body(p: Pose) -> Option<[f64; 6]> {
    let r = p.rotation;
    let skew = [
        (r[2][1] - r[1][2]) * 0.5,
        (r[0][2] - r[2][0]) * 0.5,
        (r[1][0] - r[0][1]) * 0.5,
    ];
    let sine = norm(skew);
    let cosine = ((r[0][0] + r[1][1] + r[2][2] - 1.) * 0.5).clamp(-1., 1.);
    let angle = sine.atan2(cosine);
    if angle > std::f64::consts::PI - 1e-6 {
        return None;
    }
    let w = if angle < 1e-8 {
        skew
    } else {
        mul(skew, angle / sine)
    };
    let coefficient = if angle < 1e-4 {
        1. / 12. + angle.powi(2) / 720. + angle.powi(4) / 30240.
    } else {
        (1. - 0.5 * angle / (0.5 * angle).tan()) / angle.powi(2)
    };
    let v = add(
        sub(p.translation, mul(cross(w, p.translation), 0.5)),
        mul(cross(w, cross(w, p.translation)), coefficient),
    );
    Some([v[0], v[1], v[2], w[0], w[1], w[2]])
}
fn exp_body(twist: [f64; 6]) -> Pose {
    let v: V = [twist[0], twist[1], twist[2]];
    let w = [twist[3], twist[4], twist[5]];
    let angle = norm(w);
    let (a, b) = if angle < 1e-4 {
        (
            0.5 - angle.powi(2) / 24. + angle.powi(4) / 720.,
            1. / 6. - angle.powi(2) / 120. + angle.powi(4) / 5040.,
        )
    } else {
        (
            (1. - angle.cos()) / angle.powi(2),
            (angle - angle.sin()) / angle.powi(3),
        )
    };
    let translation = add(add(v, mul(cross(w, v), a)), mul(cross(w, cross(w, v)), b));
    Pose {
        translation,
        ..Pose::identity().increment([0., 0., 0., w[0], w[1], w[2]])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn signed_exposure_offsets_use_inverse_body_motion_and_bounded_history() {
        let mut history = History::default();
        assert!(time_warp(1., 1.01, None).image_from_depth.is_none());
        assert!(matches!(
            time_warp(1., 1., None).status,
            WarpStatus::Coincident
        ));
        history.observe(1., Pose::identity());
        history.observe(1.02, Pose::identity().increment([0.02, 0., 0., 0., 0., 0.]));
        for offset in [-0.01, 0.01] {
            let warp = time_warp(1.04, 1.04 + offset, history.interval());
            assert!((warp.image_from_depth.unwrap().translation[0] + offset).abs() < 1e-12);
            assert_eq!(warp.latest_timestamp, Some(1.02));
        }
        assert!(matches!(
            time_warp(1.04, 1.24, history.interval()).status,
            WarpStatus::IncrementLimit
        ));
        let mut ambiguous = History::default();
        ambiguous.observe(1., Pose::identity());
        ambiguous.observe(
            1.02,
            Pose::identity().increment([0., 0., 0., std::f64::consts::PI, 0., 0.]),
        );
        assert!(matches!(
            time_warp(1.04, 1.05, ambiguous.interval()).status,
            WarpStatus::Unavailable
        ));
    }
    #[test]
    fn irregular_prediction_follows_a_helix_in_a_rotated_origin() {
        let origin = Pose::identity().increment([0.2, -0.1, 0.3, 0.1, -0.15, 0.2]);
        let arc = |s: f64| Pose {
            translation: [
                0.4 * (0.1 * s).sin(),
                0.4 * (1. - (0.1 * s).cos()),
                0.006 * s,
            ],
            ..Pose::identity().increment([0., 0., 0., 0., 0., 0.1 * s])
        };
        let mut history = History::default();
        history.observe(1., origin);
        history.observe(1.02, origin.compose(arc(1.)));
        let prediction = history.predict(1.03).first_from_prediction.unwrap();
        let error = origin.compose(arc(1.5)).inverse().compose(prediction);
        assert!(norm(error.translation) < 1e-12);
        for (a, b) in error
            .rotation
            .iter()
            .flatten()
            .zip(Pose::identity().rotation.iter().flatten())
        {
            assert!((a - b).abs() < 1e-12);
        }
    }
    #[test]
    fn bootstrap_bounds_and_ambiguous_rotation_never_invent_prediction() {
        let mut history = History::default();
        assert!(history.predict(1.).first_from_prediction.is_none());
        history.observe(1., Pose::identity());
        assert!(history.predict(1.02).first_from_prediction.is_none());
        history.observe(1.02, Pose::identity().increment([0.04, 0., 0., 0., 0., 0.]));
        assert!(matches!(history.predict(1.03).status, Status::Predicted));
        assert!(matches!(
            history.predict(1.1).status,
            Status::IncrementLimit
        ));
        let mut rotation = History::default();
        rotation.observe(1., Pose::identity());
        rotation.observe(
            1.02,
            Pose::identity().increment([0., 0., 0., std::f64::consts::PI, 0., 0.]),
        );
        assert!(matches!(
            rotation.predict(1.03).status,
            Status::AmbiguousRotation
        ));
    }
}
