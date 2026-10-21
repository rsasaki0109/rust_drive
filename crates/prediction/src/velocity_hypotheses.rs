//! Preserve recent acquired velocity point hypotheses without moving nominal centers.
use rustdriving_core::{
    ForecastEnvelopeConfig, Prediction, Track, Vec2, VelocityHypothesisDiagnostics,
    VelocityHypothesisTrack,
};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

const WINDOW_S: f64 = 0.8;
const MAXIMUM_SAMPLES: usize = 32;
const MAXIMUM_GAP_S: f64 = 0.35;

#[derive(Clone)]
struct Sample {
    stamp: f64,
    velocity: Vec2,
}

/// A bounded set of measured point alternatives, not calibrated velocity tubes.
#[derive(Clone)]
pub struct RecentVelocityHypotheses {
    position_error_m: f64,
    consumed_scene: Option<f64>,
    last_control: Option<f64>,
    history: BTreeMap<u64, VecDeque<Sample>>,
}

impl RecentVelocityHypotheses {
    pub fn new(bounds: ForecastEnvelopeConfig) -> Result<Self, String> {
        bounds.validate()?;
        Ok(Self {
            position_error_m: bounds.position_error_m,
            consumed_scene: None,
            last_control: None,
            history: BTreeMap::new(),
        })
    }

    pub fn diagnostics(&self) -> VelocityHypothesisDiagnostics {
        VelocityHypothesisDiagnostics {
            window_s: WINDOW_S,
            maximum_samples_per_identity: MAXIMUM_SAMPLES,
            tracks: self
                .history
                .iter()
                .map(|(id, history)| VelocityHypothesisTrack {
                    id: *id,
                    sample_count: history.len(),
                    first_acquisition_s: history.front().map(|s| s.stamp),
                    last_acquisition_s: history.back().map(|s| s.stamp),
                })
                .collect(),
        }
    }

    /// The caller owns velocity support. Only fresh accepted scene acquisitions
    /// for supported identities can enter history. Errors leave both outputs and
    /// history unchanged; cached control ticks consume age without adding samples.
    pub fn apply(
        &mut self,
        tracks: &[Track],
        supported_ids: &[u64],
        scene: Option<f64>,
        now: f64,
        scene_failed: bool,
        forecasts: &mut [Prediction],
    ) -> Result<(), String> {
        let ids: BTreeSet<_> = tracks.iter().map(|t| t.id).collect();
        let supported: BTreeSet<_> = supported_ids.iter().copied().collect();
        if !now.is_finite()
            || now < 0.
            || self.last_control.is_some_and(|old| now < old)
            || tracks.len() > 128
            || tracks.len() != forecasts.len()
            || ids.len() != tracks.len()
            || supported.len() != supported_ids.len()
            || !supported.is_subset(&ids)
        {
            return Err("invalid bounded recent-velocity input".into());
        }
        let mut next = self.clone();
        next.last_control = Some(now);
        next.history.retain(|id, _| ids.contains(id));
        let fresh = scene.is_some_and(|stamp| {
            stamp.is_finite()
                && stamp >= 0.
                && stamp <= now
                && now - stamp <= MAXIMUM_GAP_S + 1e-9
                && next.consumed_scene.is_none_or(|old| stamp > old)
        });
        if scene_failed {
            next.history.clear();
            next.consumed_scene = Some(next.consumed_scene.map_or(now, |t| t.max(now)));
        } else if fresh {
            next.consumed_scene = scene;
        }
        for track in tracks {
            if !track.position.finite()
                || !track.velocity.finite()
                || track.velocity.x.hypot(track.velocity.y) > 15. + 1e-9
                || !track.radius.is_finite()
                || track.radius < 0.
                || !track.last_seen.is_finite()
                || track.last_seen < 0.
                || track.last_seen > now
            {
                return Err("invalid acquired recent velocity".into());
            }
            let history = next.history.entry(track.id).or_default();
            if scene_failed
                || !supported.contains(&track.id)
                || now - track.last_seen > MAXIMUM_GAP_S + 1e-9
            {
                history.clear();
                continue;
            }
            if history
                .back()
                .is_some_and(|s| now - s.stamp > MAXIMUM_GAP_S + 1e-9)
            {
                history.clear();
            }
            if fresh && scene == Some(track.last_seen) {
                history.push_back(Sample {
                    stamp: track.last_seen,
                    velocity: track.velocity,
                });
            }
            while history.len() > MAXIMUM_SAMPLES
                || history
                    .front()
                    .is_some_and(|s| now - s.stamp > WINDOW_S + 1e-9)
            {
                history.pop_front();
            }
        }
        let mut expanded = forecasts.to_vec();
        for (track, forecast) in tracks.iter().zip(&mut expanded) {
            if forecast.id != track.id
                || !forecast.valid_radii()
                || !forecast.dt.is_finite()
                || forecast.dt <= 0.
                || forecast.positions.iter().any(|p| !p.finite())
            {
                return Err("invalid nominal recent-velocity forecast".into());
            }
            let history = &next.history[&track.id];
            if history.is_empty() || forecast.positions.is_empty() {
                continue;
            }
            let mut radii = Vec::with_capacity(forecast.positions.len());
            for (i, center) in forecast.positions.iter().enumerate() {
                let elapsed = now - track.last_seen + i as f64 * forecast.dt;
                if !elapsed.is_finite() {
                    return Err("recent-velocity horizon overflow".into());
                }
                let mut radius = forecast.additional_radius_m.get(i).copied().unwrap_or(0.);
                for sample in history {
                    let point = track.position.plus(sample.velocity.scaled(elapsed));
                    let floor = center.distance(point) + next.position_error_m;
                    if !point.finite() || !floor.is_finite() {
                        return Err("recent-velocity point overflow".into());
                    }
                    radius = radius.max(floor);
                }
                if !radius.is_finite() || !(forecast.radius + radius).is_finite() {
                    return Err("recent-velocity occupancy overflow".into());
                }
                radii.push(radius);
            }
            forecast.additional_radius_m = radii;
        }
        *self = next;
        forecasts.clone_from_slice(&expanded);
        Ok(())
    }
}
