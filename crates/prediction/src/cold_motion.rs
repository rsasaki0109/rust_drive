//! Retain bounded reachable occupancy until measured cold velocity confirmation.
use rustdriving_core::{
    ColdMotionDiagnostics, ColdMotionTrack, ForecastEnvelopeConfig, Prediction, Track,
    TrackMeasurement, Vec2,
};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

#[derive(Clone)]
struct Evidence {
    samples: VecDeque<TrackMeasurement>,
    report: ColdMotionTrack,
}
impl Evidence {
    fn new(id: u64) -> Self {
        Self {
            samples: VecDeque::new(),
            report: ColdMotionTrack {
                id,
                confirmed: false,
                first_acquisition_s: None,
                last_acquisition_s: None,
                acquisition_count: 0,
                last_observed_acquisition_s: None,
                confirmation_acquisition_s: None,
                velocity_residual_m_s: None,
            },
        }
    }
    fn observe(
        &mut self,
        sample: &TrackMeasurement,
        track: &Track,
        error: f64,
        maximum_speed: f64,
    ) -> Result<(), String> {
        if self
            .report
            .last_observed_acquisition_s
            .is_some_and(|t| sample.stamp - t > 0.35 + 1e-9)
        {
            *self = Self::new(track.id);
        }
        self.report.last_observed_acquisition_s = Some(sample.stamp);
        if self.report.confirmed {
            return Ok(());
        }
        self.samples.push_back(sample.clone());
        while self.samples.len() > 3 && sample.stamp - self.samples[1].stamp >= 0.2 - 1e-9 {
            self.samples.pop_front();
        }
        while self.samples.len() > 32
            || self
                .samples
                .front()
                .is_some_and(|m| sample.stamp - m.stamp > 0.6 + 1e-9)
        {
            self.samples.pop_front();
        }
        self.report.first_acquisition_s = self.samples.front().map(|m| m.stamp);
        self.report.last_acquisition_s = self.samples.back().map(|m| m.stamp);
        self.report.acquisition_count = self.samples.len();
        self.report.velocity_residual_m_s = None;
        if let Some(velocity) = fitted(&self.samples) {
            if velocity.x.hypot(velocity.y) > maximum_speed + 1e-9 {
                return Err("measured cold motion exceeds its declared speed bound".into());
            }
            let residual = velocity.distance(track.velocity);
            self.report.velocity_residual_m_s = Some(residual);
            if residual <= error + 1e-9 {
                self.report.confirmed = true;
                self.report.confirmation_acquisition_s = Some(sample.stamp);
                self.samples.clear();
            }
        }
        Ok(())
    }
}

fn fitted(samples: &VecDeque<TrackMeasurement>) -> Option<Vec2> {
    let first = samples.front()?;
    if samples.len() < 3 || samples.back()?.stamp - first.stamp < 0.2 - 1e-9 {
        return None;
    }
    let low = samples
        .iter()
        .map(|m| m.radius)
        .fold(f64::INFINITY, f64::min);
    let high = samples.iter().map(|m| m.radius).fold(0., f64::max);
    if high - low > 0.12 + 1e-9 {
        return None;
    }
    let n = samples.len() as f64;
    let mean_t = samples.iter().map(|m| m.stamp - first.stamp).sum::<f64>() / n;
    let mean = samples
        .iter()
        .fold(Vec2::default(), |sum, m| {
            sum.plus(m.center.minus(first.center))
        })
        .scaled(1. / n);
    let variance = samples
        .iter()
        .map(|m| (m.stamp - first.stamp - mean_t).powi(2))
        .sum::<f64>();
    if variance <= 1e-12 {
        return None;
    }
    let velocity = samples
        .iter()
        .fold(Vec2::default(), |sum, m| {
            sum.plus(
                m.center
                    .minus(first.center)
                    .minus(mean)
                    .scaled(m.stamp - first.stamp - mean_t),
            )
        })
        .scaled(1. / variance);
    let rms = (samples
        .iter()
        .map(|m| {
            let residual = m
                .center
                .minus(first.center)
                .minus(mean)
                .minus(velocity.scaled(m.stamp - first.stamp - mean_t));
            residual.x * residual.x + residual.y * residual.y
        })
        .sum::<f64>()
        / n)
        .sqrt();
    (velocity.finite() && rms.is_finite() && rms <= 0.12).then_some(velocity)
}

#[derive(Clone)]
pub struct ColdMotionEnvelope {
    maximum_speed_m_s: f64,
    bounds: ForecastEnvelopeConfig,
    consumed_scene: Option<f64>,
    evidence: BTreeMap<u64, Evidence>,
}

impl ColdMotionEnvelope {
    pub fn new(maximum_speed_m_s: f64, bounds: ForecastEnvelopeConfig) -> Result<Self, String> {
        bounds.validate()?;
        if !maximum_speed_m_s.is_finite() || !(0.7..=15.).contains(&maximum_speed_m_s) {
            return Err(
                "cold motion requires a finite maximum speed between 0.7 and 15 m/s".into(),
            );
        }
        Ok(Self {
            maximum_speed_m_s,
            bounds,
            consumed_scene: None,
            evidence: BTreeMap::new(),
        })
    }

    pub fn diagnostics(&self) -> ColdMotionDiagnostics {
        ColdMotionDiagnostics {
            maximum_speed_m_s: self.maximum_speed_m_s,
            tracks: self.evidence.values().map(|e| e.report.clone()).collect(),
        }
    }

    /// Uses only accepted continuous-frame tracks and their raw associated discs.
    /// A failed transaction leaves both state and forecasts unchanged.
    pub fn apply(
        &mut self,
        tracks: &[Track],
        measurements: &[TrackMeasurement],
        scene: Option<f64>,
        now: f64,
        scene_failed: bool,
        forecasts: &mut [Prediction],
    ) -> Result<(), String> {
        if !now.is_finite()
            || now < 0.
            || tracks.len() > 128
            || measurements.len() > 128
            || tracks.len() != forecasts.len()
            || tracks.iter().map(|t| t.id).collect::<BTreeSet<_>>().len() != tracks.len()
            || measurements
                .iter()
                .map(|m| m.id)
                .collect::<BTreeSet<_>>()
                .len()
                != measurements.len()
            || measurements
                .iter()
                .any(|m| !tracks.iter().any(|t| t.id == m.id))
        {
            return Err("invalid bounded cold-motion input".into());
        }
        let mut next = self.clone();
        next.evidence
            .retain(|id, _| tracks.iter().any(|t| t.id == *id));
        let fresh = scene.is_some_and(|stamp| {
            stamp.is_finite()
                && stamp >= 0.
                && stamp <= now + 1e-9
                && now - stamp <= 0.35 + 1e-9
                && next.consumed_scene.is_none_or(|old| stamp > old)
        });
        if scene_failed {
            next.evidence.clear();
            next.consumed_scene = Some(next.consumed_scene.map_or(now, |t| t.max(now)));
        } else if fresh {
            next.consumed_scene = scene;
        }
        for track in tracks {
            if !track.position.finite()
                || !track.velocity.finite()
                || !track.radius.is_finite()
                || track.radius < 0.
                || !track.last_seen.is_finite()
                || track.last_seen < 0.
                || track.last_seen > now
            {
                return Err("invalid acquired cold-motion track".into());
            }
            let evidence = next
                .evidence
                .entry(track.id)
                .or_insert_with(|| Evidence::new(track.id));
            if evidence
                .report
                .last_observed_acquisition_s
                .is_some_and(|t| now - t > 0.35 + 1e-9)
            {
                *evidence = Evidence::new(track.id);
            }
            if fresh
                && !scene_failed
                && let Some(sample) = measurements.iter().find(|m| m.id == track.id)
            {
                if Some(sample.stamp) != scene
                    || sample.stamp != track.last_seen
                    || !sample.center.finite()
                    || !sample.radius.is_finite()
                    || sample.radius < 0.
                {
                    return Err("invalid cold-motion acquisition ownership".into());
                }
                evidence.observe(
                    sample,
                    track,
                    next.bounds.velocity_error_m_s,
                    next.maximum_speed_m_s,
                )?;
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
                return Err("invalid nominal cold-motion forecast".into());
            }
            if next.evidence[&track.id].report.confirmed || forecast.positions.is_empty() {
                continue;
            }
            let age = now - track.last_seen;
            let mut radii = Vec::with_capacity(forecast.positions.len());
            for (i, center) in forecast.positions.iter().enumerate() {
                let floor = center.distance(track.position)
                    + next.bounds.position_error_m
                    + next.maximum_speed_m_s * (age + i as f64 * forecast.dt);
                let radius = floor.max(forecast.additional_radius_m.get(i).copied().unwrap_or(0.));
                if !radius.is_finite() || !(forecast.radius + radius).is_finite() {
                    return Err("cold-motion reachable occupancy overflow".into());
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
