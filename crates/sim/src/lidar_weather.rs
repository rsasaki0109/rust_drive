//! Deterministic planar LiDAR weather stressor; not a calibrated atmosphere.
//! Only the returned cloud enters perception. Ray effects and source labels are
//! evaluation metadata. Uncalibrated weather clouds carry no clear-sweep proof.
use crate::{Rng, WorldObject, lidar};
use rustdriving_core::{LidarScan, PlanarRayOutcome, Pose, Vec2};
use rustdriving_perception::planar_sweep::validate_sweep;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WeatherConfig {
    /// Synthetic one-way extinction; surface survival is exp(-2 * alpha * r).
    pub extinction_m_inv: f64,
    /// Effective detectable-particle rate along a ray, not particle density.
    pub backscatter_rate_m_inv: f64,
    /// Additional bounded uniform range perturbation, independent of base noise.
    pub range_jitter_m: f64,
}
impl WeatherConfig {
    pub const CLEAR: Self = Self {
        extinction_m_inv: 0.0,
        backscatter_rate_m_inv: 0.0,
        range_jitter_m: 0.0,
    };
    pub fn validate(self) -> Result<(), String> {
        if !self.extinction_m_inv.is_finite()
            || !(0.0..=0.1).contains(&self.extinction_m_inv)
            || !self.backscatter_rate_m_inv.is_finite()
            || !(0.0..=0.1).contains(&self.backscatter_rate_m_inv)
            || !self.range_jitter_m.is_finite()
            || !(0.0..=1.0).contains(&self.range_jitter_m)
        {
            return Err(
                "weather coefficients must be finite and within declared stress bounds".into(),
            );
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ReturnSource {
    Surface { input: usize },
    Precipitation { beam: usize },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BeamEffect {
    pub input_range_m: Option<f64>,
    pub particle_range_m: Option<f64>,
    pub loss_uniform: f64,
    pub jitter_unit: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WeatherScan {
    pub scan: LidarScan,
    /// Evaluation-only outcomes, never substituted for calibrated sweep evidence.
    pub rays: Vec<PlanarRayOutcome>,
    /// One source per returned point, kept outside the detector input.
    pub sources: Vec<ReturnSource>,
    pub effects: Vec<BeamEffect>,
}
fn uniform(seed: u64, stamp: f64, beam: usize, channel: u64) -> f64 {
    let time = if stamp == 0.0 { 0 } else { stamp.to_bits() };
    let mut z = seed
        ^ time.rotate_left(17)
        ^ (beam as u64).wrapping_mul(0x9e3779b97f4a7c15)
        ^ channel.wrapping_mul(0xd1b54a32d192ed03);
    z = z.wrapping_add(0x9e3779b97f4a7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
    z ^= z >> 31;
    (z >> 11) as f64 / (1_u64 << 53) as f64
}

/// Freeze a complete acquisition from the existing first-return ray/circle
/// sensor. ENU ego/object coordinates are metres; output is body forward/left.
pub fn reference_scan(
    ego: Pose,
    objects: &[WorldObject],
    stamp: f64,
    seed: u64,
) -> Result<LidarScan, String> {
    if !ego.position.finite()
        || !ego.yaw.is_finite()
        || !stamp.is_finite()
        || stamp < 0.0
        || objects.len() > 256
        || objects.iter().any(|o| {
            !o.position.finite() || !o.radius.is_finite() || !(0.01..=10.0).contains(&o.radius)
        })
    {
        return Err("invalid frozen reference acquisition".into());
    }
    let scan = lidar(ego, objects, stamp, &mut Rng::new(seed), true);
    validate_sweep(&scan).map_err(str::to_owned)?;
    Ok(scan)
}

/// Apply common counter-based draws to a complete baseline acquisition.
/// Coefficients and source labels do not enter the driving pipeline.
pub fn apply(input: &LidarScan, config: WeatherConfig, seed: u64) -> Result<WeatherScan, String> {
    config.validate()?;
    validate_sweep(input).map_err(str::to_owned)?;
    let sweep = input
        .sweep
        .as_ref()
        .ok_or("complete baseline sweep required")?;
    let c = sweep.calibration;
    let clear = config == WeatherConfig::CLEAR;
    let mut result = WeatherScan {
        scan: LidarScan {
            stamp: input.stamp,
            points: Vec::new(),
            sweep: None,
        },
        rays: Vec::with_capacity(c.ray_count),
        sources: Vec::new(),
        effects: Vec::with_capacity(c.ray_count),
    };
    for (beam, ray) in sweep.rays.iter().enumerate() {
        let surface = match ray {
            PlanarRayOutcome::Return(index) => Some((*index, input.points[*index])),
            PlanarRayOutcome::Clear => None,
            PlanarRayOutcome::Unavailable => return Err("incomplete baseline acquisition".into()),
        };
        let range = surface.map(|(_, p)| p.x.hypot(p.y));
        let loss = uniform(seed, input.stamp, beam, 0);
        let particle = if config.backscatter_rate_m_inv == 0.0 {
            None
        } else {
            let distance = c.min_range_m
                - (-uniform(seed, input.stamp, beam, 1)).ln_1p() / config.backscatter_rate_m_inv;
            (distance < c.max_range_m).then_some(distance)
        };
        let jitter = uniform(seed, input.stamp, beam, 2) * 2.0 - 1.0;
        result.effects.push(BeamEffect {
            input_range_m: range,
            particle_range_m: particle,
            loss_uniform: loss,
            jitter_unit: jitter,
        });
        let chosen = if let Some(distance) =
            particle.filter(|distance| *distance < range.unwrap_or(c.max_range_m))
        {
            let angle = c.first_angle_rad + c.angular_step_rad * beam as f64;
            Some((
                Vec2::new(angle.cos(), angle.sin()),
                distance,
                ReturnSource::Precipitation { beam },
            ))
        } else if loss >= (-2.0 * config.extinction_m_inv * range.unwrap_or(c.max_range_m)).exp() {
            result.rays.push(PlanarRayOutcome::Unavailable);
            None
        } else if let Some((index, point)) = surface {
            Some((
                point.scaled(1.0 / range.unwrap()),
                range.unwrap(),
                ReturnSource::Surface { input: index },
            ))
        } else {
            result.rays.push(PlanarRayOutcome::Clear);
            None
        };
        if let Some((direction, distance, source)) = chosen {
            // Bounds include the original noisy sample: zero added jitter
            // cannot change a valid baseline merely by clipping its base noise.
            let observed = (distance + jitter * config.range_jitter_m).clamp(
                c.min_range_m.min(distance).max(1e-9),
                c.max_range_m.max(distance),
            );
            result
                .rays
                .push(PlanarRayOutcome::Return(result.scan.points.len()));
            result.scan.points.push(direction.scaled(observed));
            result.sources.push(source);
        }
    }
    if clear {
        // Preserve original floating-point values and certified ray ownership.
        result.scan = input.clone();
        result.rays = sweep.rays.clone();
        result.sources = (0..input.points.len())
            .map(|input| ReturnSource::Surface { input })
            .collect();
    }
    Ok(result)
}

#[cfg(test)]
mod tests;
