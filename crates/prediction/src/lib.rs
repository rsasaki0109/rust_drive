//! Motion forecasts from tracked observations; no simulator behavior or intentions.
use rustdrive_core::{Prediction, Predictor, Track};
use std::collections::{BTreeMap, VecDeque};

/// Uses sustained measured braking for at most one second, then coasts.
/// This is a motion hypothesis, not a guarantee that another driver will brake.
#[derive(Default)]
pub struct ObservedBraking {
    baseline: ConstantVelocity,
    history: BTreeMap<u64, VecDeque<Track>>,
}
impl ObservedBraking {
    /// Produces samples starting at `time`, extrapolated from each observation's
    /// acquisition timestamp. The braking hypothesis also starts at acquisition;
    /// repeated calls do not renew its one-second braking interval.
    pub fn predict(&mut self, tracks: &[Track], time: f64) -> Vec<Prediction> {
        assert!(time.is_finite());
        self.history
            .retain(|id, _| tracks.iter().any(|t| t.id == *id));
        let mut predictions = self.baseline.predict(tracks);
        for (track, prediction) in tracks.iter().zip(&mut predictions) {
            let history = self.history.entry(track.id).or_default();
            let age = time - track.last_seen;
            let speed = track.velocity.x.hypot(track.velocity.y);
            if !track.position.finite()
                || !track.velocity.finite()
                || !speed.is_finite()
                || !track.radius.is_finite()
                || track.radius < 0.0
                || !track.last_seen.is_finite()
                || !age.is_finite()
                || age < 0.0
            {
                history.clear();
                // Empty trajectories retain the invalid track identity and cause
                // downstream planning to brake rather than inventing motion.
                prediction.positions.clear();
                continue;
            }
            for (i, position) in prediction.positions.iter_mut().enumerate() {
                let elapsed = if speed < 0.7 {
                    0.0
                } else {
                    age + i as f64 * prediction.dt
                };
                *position = track.position.plus(track.velocity.scaled(elapsed));
            }
            if prediction
                .positions
                .iter()
                .any(|position| !position.finite())
            {
                history.clear();
                prediction.positions.clear();
                continue;
            }
            if history
                .back()
                .is_some_and(|old| track.last_seen < old.last_seen)
            {
                history.clear();
            }
            if history
                .back()
                .is_none_or(|old| track.last_seen > old.last_seen)
            {
                history.push_back(track.clone());
            }
            while history
                .front()
                .is_some_and(|old| track.last_seen - old.last_seen > 0.8 + 1e-9)
                || history.len() > 32
            {
                history.pop_front();
            }
            if age > 0.15 + 1e-9 {
                continue;
            }
            if speed < 0.7 {
                continue;
            }
            // Withdraw braking on the first fresh non-decreasing speed sample.
            if history.len() < 2
                || speed
                    >= history[history.len() - 2]
                        .velocity
                        .x
                        .hypot(history[history.len() - 2].velocity.y)
            {
                continue;
            }
            let direction = track.velocity.scaled(1.0 / speed);
            let mut previous = track;
            let mut braking = 2.0_f64;
            let mut supported = true;
            for offset in [0.2, 0.4, 0.6] {
                let target = track.last_seen - offset;
                let Some(old) = history.iter().min_by(|a, b| {
                    (a.last_seen - target)
                        .abs()
                        .total_cmp(&(b.last_seen - target).abs())
                }) else {
                    supported = false;
                    break;
                };
                let old_speed = old.velocity.x.hypot(old.velocity.y);
                let elapsed = previous.last_seen - old.last_seen;
                let aligned = (old.velocity.x * direction.x + old.velocity.y * direction.y)
                    / old_speed.max(1e-9);
                let deceleration =
                    (old_speed - previous.velocity.x.hypot(previous.velocity.y)) / elapsed;
                if (old.last_seen - target).abs() > 0.025
                    || elapsed <= 0.0
                    || aligned < 0.98
                    || !deceleration.is_finite()
                    || deceleration < 0.3
                {
                    supported = false;
                    break;
                }
                // Use half the weakest observed braking, capped at 2 m/s².
                braking = braking.min(deceleration * 0.5);
                previous = old;
            }
            if supported {
                for (i, position) in prediction.positions.iter_mut().enumerate() {
                    let t = age + i as f64 * prediction.dt;
                    let braking_time = t.min(1.0).min(speed / braking);
                    let distance = speed * braking_time - 0.5 * braking * braking_time.powi(2)
                        + (speed - braking * braking_time).max(0.0) * (t - braking_time);
                    *position = track.position.plus(direction.scaled(distance));
                }
                if prediction
                    .positions
                    .iter()
                    .any(|position| !position.finite())
                {
                    history.clear();
                    prediction.positions.clear();
                }
            }
        }
        predictions
    }
}
pub struct ConstantVelocity {
    pub horizon: f64,
    pub dt: f64,
}
impl Default for ConstantVelocity {
    fn default() -> Self {
        Self {
            horizon: 8.0,
            dt: 0.2,
        }
    }
}
impl Predictor for ConstantVelocity {
    fn predict(&self, tracks: &[Track]) -> Vec<Prediction> {
        assert!(
            self.dt.is_finite() && self.dt > 0.0 && self.horizon.is_finite() && self.horizon >= 0.0
        );
        tracks
            .iter()
            .map(|track| Prediction {
                id: track.id,
                positions: (0..=(self.horizon / self.dt).ceil() as usize)
                    .map(|i| {
                        track.position.plus(track.velocity.scaled(
                            if track.velocity.x.hypot(track.velocity.y) < 0.7 {
                                0.0
                            } else {
                                i as f64 * self.dt
                            },
                        ))
                    })
                    .collect(),
                radius: track.radius,
                dt: self.dt,
            })
            .collect()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use rustdrive_core::Vec2;
    fn track(time: f64, speed: f64) -> Track {
        Track {
            id: 7,
            position: Vec2::new(10.0, 2.0),
            velocity: Vec2::new(speed, 0.0),
            radius: 1.0,
            last_seen: time,
        }
    }
    fn braking_history(predictor: &mut ObservedBraking) -> Track {
        for i in 0..6 {
            let t = i as f64 * 0.1;
            predictor.predict(&[track(t, 4.0 - 2.0 * t)], t);
        }
        track(0.6, 2.8)
    }
    #[test]
    fn sustained_braking_is_bounded_then_coasts_without_reversing() {
        let mut predictor = ObservedBraking::default();
        let latest = braking_history(&mut predictor);
        let forecast = predictor.predict(&[latest], 0.6);
        // Half of measured 2 m/s² for 1 s, followed by coasting at 1.8 m/s.
        assert!((forecast[0].positions[5].x - 12.3).abs() < 1e-8);
        assert!((forecast[0].positions[40].x - 24.9).abs() < 1e-8);
        assert!(
            forecast[0]
                .positions
                .windows(2)
                .all(|p| p[1].x >= p[0].x && p[1].y == p[0].y)
        );
    }
    #[test]
    fn current_epoch_constant_velocity_propagates_acquisition_age() {
        let mut predictor = ObservedBraking::default();
        let mut acquired = track(2.0, 3.0);
        acquired.position = Vec2::new(1.0, -2.0);
        acquired.velocity = Vec2::new(3.0, -4.0);
        let forecast = predictor.predict(std::slice::from_ref(&acquired), 2.1);
        assert!((forecast[0].positions[0].x - 1.3).abs() < 1e-12);
        assert!((forecast[0].positions[0].y + 2.4).abs() < 1e-12);
        assert!((forecast[0].positions[40].x - 25.3).abs() < 1e-12);
        assert!((forecast[0].positions[40].y + 34.4).abs() < 1e-12);
        // Forecasting does not change the acquisition-origin track or the trait
        // baseline used by callers that have no current-time argument.
        assert_eq!(acquired.last_seen, 2.0);
        assert_eq!(acquired.position, Vec2::new(1.0, -2.0));
        assert_eq!(
            ConstantVelocity::default().predict(&[acquired])[0].positions[0],
            Vec2::new(1.0, -2.0)
        );
    }
    #[test]
    fn delayed_duplicate_braking_does_not_renew_the_acquisition_budget() {
        let mut predictor = ObservedBraking::default();
        let acquired = braking_history(&mut predictor);
        predictor.predict(std::slice::from_ref(&acquired), 0.6);
        for _ in 0..20 {
            let forecast = predictor.predict(std::slice::from_ref(&acquired), 0.7);
            // Acquisition speed 2.8, supported braking 1.0: at age 0.1 the
            // distance is 0.275; at age 1.1 it is 2.3 + 0.1 * 1.8.
            assert!((forecast[0].positions[0].x - 10.275).abs() < 1e-10);
            assert!((forecast[0].positions[5].x - 12.48).abs() < 1e-10);
            assert!((forecast[0].positions[40].x - 25.08).abs() < 1e-10);
            let coast_step = forecast[0].positions[6].x - forecast[0].positions[5].x;
            assert!((coast_step - 0.36).abs() < 1e-10);
        }
        assert_eq!(predictor.history[&7].len(), 7);
        assert_eq!(acquired.last_seen, 0.6);
        assert_eq!(acquired.position, Vec2::new(10.0, 2.0));
    }
    #[test]
    fn stale_braking_withdraws_and_propagates_valid_velocity_from_acquisition() {
        let mut predictor = ObservedBraking::default();
        let acquired = braking_history(&mut predictor);
        let fresh = predictor.predict(std::slice::from_ref(&acquired), 0.75);
        assert!((fresh[0].positions[0].x - 10.40875).abs() < 1e-10);
        let stale = predictor.predict(std::slice::from_ref(&acquired), 0.8);
        assert!((stale[0].positions[0].x - 10.56).abs() < 1e-10);
        assert!((stale[0].positions[40].x - 32.96).abs() < 1e-10);
        assert_eq!(predictor.history[&7].len(), 7);
    }
    #[test]
    fn delayed_low_speed_tracks_keep_the_observed_position() {
        let mut predictor = ObservedBraking::default();
        let acquired = track(0.0, 0.69);
        let forecast = predictor.predict(std::slice::from_ref(&acquired), 0.6);
        assert!(
            forecast[0]
                .positions
                .iter()
                .all(|p| *p == acquired.position)
        );
        let moving = track(0.0, 0.7);
        let forecast = predictor.predict(&[moving], 0.6);
        assert!((forecast[0].positions[0].x - 10.42).abs() < 1e-10);
    }
    #[test]
    fn invalid_or_future_tracks_are_not_fabricated_into_valid_forecasts() {
        let valid = track(0.6, 2.8);
        let mut invalid_tracks = Vec::new();
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut invalid = valid.clone();
            invalid.last_seen = value;
            invalid_tracks.push(invalid);
            let mut invalid = valid.clone();
            invalid.position.x = value;
            invalid_tracks.push(invalid);
            let mut invalid = valid.clone();
            invalid.velocity.y = value;
            invalid_tracks.push(invalid);
            let mut invalid = valid.clone();
            invalid.radius = value;
            invalid_tracks.push(invalid);
        }
        let mut future = valid.clone();
        future.last_seen = 0.61;
        invalid_tracks.push(future);
        let mut negative_radius = valid.clone();
        negative_radius.radius = -1.0;
        invalid_tracks.push(negative_radius);
        let mut overflowing_speed = valid.clone();
        overflowing_speed.velocity = Vec2::new(f64::MAX, f64::MAX);
        invalid_tracks.push(overflowing_speed);
        let mut overflowing_position = valid;
        overflowing_position.position = Vec2::new(f64::MAX, 0.0);
        overflowing_position.velocity = Vec2::new(f64::MAX / 2.0, 0.0);
        invalid_tracks.push(overflowing_position);
        for invalid in invalid_tracks {
            let mut predictor = ObservedBraking::default();
            braking_history(&mut predictor);
            let forecast = predictor.predict(&[invalid], 0.6);
            assert_eq!(forecast[0].id, 7);
            assert!(forecast[0].positions.is_empty());
            assert!(predictor.history[&7].is_empty());
        }
    }
    #[test]
    fn a_short_braking_observation_does_not_change_constant_velocity() {
        let mut predictor = ObservedBraking::default();
        for i in 0..6 {
            let t = i as f64 * 0.1;
            let current = track(t, 4.0 - t);
            let forecast = predictor.predict(std::slice::from_ref(&current), t);
            assert_eq!(
                forecast[0].positions,
                ConstantVelocity::default().predict(&[current])[0].positions
            );
        }
    }
    #[test]
    fn stale_duplicate_and_reacquired_tracks_cannot_supply_braking_evidence() {
        let mut predictor = ObservedBraking::default();
        let latest = braking_history(&mut predictor);
        let mut advanced = latest.clone();
        advanced.position = advanced.position.plus(advanced.velocity.scaled(0.2));
        let baseline = ConstantVelocity::default().predict(&[advanced]);
        for _ in 0..100 {
            predictor.predict(std::slice::from_ref(&latest), 0.6);
        }
        assert_eq!(predictor.history[&7].len(), 7);
        let stale = predictor.predict(std::slice::from_ref(&latest), 0.8);
        assert!(
            stale[0]
                .positions
                .iter()
                .zip(&baseline[0].positions)
                .all(|(a, b)| (a.x - b.x).abs() < 1e-10 && a.y == b.y)
        );
        predictor.predict(&[], 0.9);
        let reacquired = track(1.0, 2.0);
        assert_eq!(
            predictor.predict(std::slice::from_ref(&reacquired), 1.0)[0].positions,
            ConstantVelocity::default().predict(&[reacquired])[0].positions
        );
    }
    #[test]
    fn first_observed_reacceleration_withdraws_braking() {
        let mut predictor = ObservedBraking::default();
        let latest = braking_history(&mut predictor);
        predictor.predict(&[latest], 0.6);
        let accelerating = track(0.7, 2.9);
        assert_eq!(
            predictor.predict(std::slice::from_ref(&accelerating), 0.7)[0].positions,
            ConstantVelocity::default().predict(&[accelerating])[0].positions
        );
    }
    #[test]
    fn direction_change_or_interrupted_sensing_uses_the_baseline() {
        for interrupted in [true, false] {
            let mut predictor = ObservedBraking::default();
            let mut latest = braking_history(&mut predictor);
            if interrupted {
                latest.last_seen = 1.0;
            } else {
                latest.velocity = Vec2::new(0.0, 2.8);
            }
            assert_eq!(
                predictor.predict(std::slice::from_ref(&latest), latest.last_seen)[0].positions,
                ConstantVelocity::default().predict(&[latest])[0].positions
            );
        }
    }
    #[test]
    fn extrapolates_velocity() {
        let tracks = vec![Track {
            id: 7,
            position: Vec2::new(1.0, 2.0),
            velocity: Vec2::new(2.0, -1.0),
            radius: 1.0,
            last_seen: 0.0,
        }];
        let p = ConstantVelocity {
            horizon: 2.0,
            dt: 0.5,
        }
        .predict(&tracks);
        assert_eq!(p[0].positions[4], Vec2::new(5.0, 0.0));
    }
}
