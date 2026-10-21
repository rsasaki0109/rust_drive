//! Bounded cold-track fit, using every admitted detection in a stable shape window.
use rustdriving_core::{Detection, Vec2};

#[derive(Clone)]
pub(crate) struct Initialization {
    first_stamp: f64,
    origin: Vec2,
    count: u64,
    mean_time: f64,
    mean_position: Vec2,
    time_variation: f64,
    position_variation: f64,
    time_position: Vec2,
    minimum_radius: f64,
    maximum_radius: f64,
}

impl Initialization {
    pub(crate) fn new(detection: &Detection, stamp: f64) -> Self {
        Self {
            first_stamp: stamp,
            origin: detection.center,
            count: 1,
            mean_time: 0.,
            mean_position: Vec2::default(),
            time_variation: 0.,
            position_variation: 0.,
            time_position: Vec2::default(),
            minimum_radius: detection.radius,
            maximum_radius: detection.radius,
        }
    }

    /// Returns the fitted current position and velocity only after confirmation.
    /// Radius/fit limits are declared heuristics, not calibrated confidence.
    pub(crate) fn observe(&mut self, detection: &Detection, stamp: f64) -> Option<(Vec2, Vec2)> {
        let elapsed = stamp - self.first_stamp;
        let minimum_radius = self.minimum_radius.min(detection.radius);
        let maximum_radius = self.maximum_radius.max(detection.radius);
        if elapsed > 0.6 + 1e-9 || maximum_radius - minimum_radius > 0.12 + 1e-9 {
            *self = Self::new(detection, stamp);
            return None;
        }
        self.minimum_radius = minimum_radius;
        self.maximum_radius = maximum_radius;
        self.count += 1;
        let position = detection.center.minus(self.origin);
        let delta_time = elapsed - self.mean_time;
        let delta_position = position.minus(self.mean_position);
        self.mean_time += delta_time / self.count as f64;
        self.mean_position = self
            .mean_position
            .plus(delta_position.scaled(1. / self.count as f64));
        self.time_variation += delta_time * (elapsed - self.mean_time);
        let remaining = position.minus(self.mean_position);
        self.position_variation += delta_position.x * remaining.x + delta_position.y * remaining.y;
        self.time_position = self.time_position.plus(remaining.scaled(delta_time));
        if self.count < 3 || elapsed < 0.2 - 1e-9 || self.time_variation <= 1e-12 {
            return None;
        }
        let velocity = self.time_position.scaled(1. / self.time_variation);
        let explained = (self.time_position.x * self.time_position.x
            + self.time_position.y * self.time_position.y)
            / self.time_variation;
        let rms = ((self.position_variation - explained).max(0.) / self.count as f64).sqrt();
        let fitted = self
            .origin
            .plus(self.mean_position)
            .plus(velocity.scaled(elapsed - self.mean_time));
        (velocity.finite()
            && fitted.finite()
            && rms.is_finite()
            && rms <= 0.12
            && velocity.x.hypot(velocity.y) <= 15.)
            .then_some((fitted, velocity))
    }
}
