//! A bounded acquisition window for current position/velocity, without future intent.
use crate::initialization::Initialization;
use rustdriving_core::{Detection, Vec2};
use std::collections::VecDeque;

#[derive(Clone)]
pub(crate) struct RecentMotion {
    samples: VecDeque<(Detection, f64)>,
    last_moving_acquisition: Option<f64>,
}

impl RecentMotion {
    pub(crate) fn new(detection: &Detection, stamp: f64) -> Self {
        Self {
            samples: VecDeque::from([(detection.clone(), stamp)]),
            last_moving_acquisition: None,
        }
    }

    pub(crate) fn observe(&mut self, detection: &Detection, stamp: f64) -> Option<(Vec2, Vec2)> {
        if self
            .samples
            .back()
            .is_some_and(|(_, last)| stamp - last > 0.35 + 1e-9)
        {
            self.samples.clear();
            self.last_moving_acquisition = None;
        }
        self.samples.push_back((detection.clone(), stamp));
        // Keep the shortest recent suffix spanning 0.2 s, with at least three
        // actual acquisitions. Cached control ticks never enter this window.
        while self.samples.len() > 3 && stamp - self.samples[1].1 >= 0.2 - 1e-9 {
            self.samples.pop_front();
        }
        while self.samples.len() > 32
            || self
                .samples
                .front()
                .is_some_and(|(_, first)| stamp - first > 0.6 + 1e-9)
        {
            self.samples.pop_front();
        }
        let minimum_radius = self
            .samples
            .iter()
            .map(|(d, _)| d.radius)
            .fold(f64::INFINITY, f64::min);
        let maximum_radius = self
            .samples
            .iter()
            .map(|(d, _)| d.radius)
            .fold(f64::NEG_INFINITY, f64::max);
        if maximum_radius - minimum_radius > 0.12 + 1e-9 {
            return None;
        }
        let (first, first_stamp) = self.samples.front()?;
        let mut fit = Initialization::new(first, *first_stamp);
        let mut result = None;
        for (sample, acquired) in self.samples.iter().skip(1) {
            result = fit.observe(sample, *acquired);
        }
        let (position, velocity) = result?;
        // Use the existing ordinary moving-object cutoff. Do not replace stable
        // static/slow filtering with differentiation of small surface jitter.
        if velocity.x.hypot(velocity.y) >= 0.7 {
            self.last_moving_acquisition = Some(stamp);
        }
        // A recently observed moving object can still be fitted while stopping.
        // Low-speed fits and cached ticks cannot renew this acquisition clock.
        self.last_moving_acquisition
            .is_some_and(|last| stamp - last <= 0.6 + 1e-9)
            .then_some((position, velocity))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oversampling_cannot_accumulate_unbounded_or_falsely_mature_support() {
        let point = Detection {
            center: Vec2::default(),
            radius: 1.,
        };
        let mut recent = RecentMotion::new(&point, 0.);
        for i in 1..=1000 {
            assert!(recent.observe(&point, i as f64 * 0.0001).is_none());
            assert!(recent.samples.len() <= 32);
        }
    }

    #[test]
    fn stopping_fits_expire_without_renewal_by_low_speed_observations() {
        let sample = |x| Detection {
            center: Vec2::new(x, 0.),
            radius: 1.,
        };
        let mut recent = RecentMotion::new(&sample(0.), 0.);
        assert!(recent.observe(&sample(0.4), 0.1).is_none());
        assert!(recent.observe(&sample(0.8), 0.2).is_some());
        assert!(recent.observe(&sample(0.8), 0.3).is_some());
        for i in 4..=9 {
            assert!(
                recent
                    .observe(&sample(0.8), i as f64 * 0.1)
                    .unwrap()
                    .1
                    .x
                    .abs()
                    < 1e-10
            );
        }
        assert!(recent.observe(&sample(0.8), 1.).is_none());
    }
}
