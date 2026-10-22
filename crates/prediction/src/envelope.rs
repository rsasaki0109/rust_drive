//! Cover the nominal motion and a bounded-error constant-velocity alternative.
use rustdriving_core::{ForecastEnvelopeConfig, Prediction, Track};

pub(super) fn cover(
    bounds: ForecastEnvelopeConfig,
    track: &Track,
    now: f64,
    forecast: &mut Prediction,
) {
    if forecast.positions.is_empty() {
        return;
    }
    let age = now - track.last_seen;
    if age < 0.0 || !age.is_finite() || !forecast.valid_radii() {
        forecast.positions.clear();
        forecast.additional_radius_m.clear();
        return;
    }
    let mut extra = Vec::with_capacity(forecast.positions.len());
    for (i, center) in forecast.positions.iter_mut().enumerate() {
        let elapsed = age + i as f64 * forecast.dt;
        let continuation = track.position.plus(track.velocity.scaled(elapsed));
        let error = bounds.position_error_m
            + bounds.velocity_error_m_s * elapsed
            + 0.5 * bounds.acceleration_error_m_s2 * elapsed.powi(2);
        let old = forecast.additional_radius_m.get(i).copied().unwrap_or(0.0);
        let delta = continuation.minus(*center);
        let distance = delta.x.hypot(delta.y);
        let radius = if old >= distance + error {
            old
        } else if error >= distance + old {
            *center = continuation;
            error
        } else {
            let enclosing = (distance + old + error) * 0.5;
            *center = center.plus(delta.scaled((enclosing - old) / distance));
            enclosing
        };
        if !center.finite() || !radius.is_finite() || !(forecast.radius + radius).is_finite() {
            forecast.positions.clear();
            forecast.additional_radius_m.clear();
            return;
        }
        extra.push(radius);
    }
    forecast.additional_radius_m = extra;
}
