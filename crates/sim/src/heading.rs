//! Synthetic acquisition only; no runtime truth crosses the sensor boundary.
use crate::{Scenario, gnss_noise::uniform, sensor_timing::SensorFailureWindow};
use rustdriving_core::{AbsoluteHeading, wrap_angle};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HeadingBiasWindow {
    pub from: f64,
    pub until: f64,
    pub offset_rad: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HeadingSensor {
    pub period_ticks: usize,
    pub variance_rad2: f64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub dropout_windows: Vec<SensorFailureWindow>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bias_windows: Vec<HeadingBiasWindow>,
}
impl HeadingSensor {
    pub fn validate(&self, duration: f64) -> Result<(), String> {
        if !(1..=7).contains(&self.period_ticks)
            || !self.variance_rad2.is_finite()
            || self.variance_rad2 <= 0.
            || self.variance_rad2 > std::f64::consts::PI.powi(2)
            || self.dropout_windows.len() > 32
            || self.bias_windows.len() > 32
        {
            return Err("invalid heading acquisition period, variance or window count".into());
        }
        for windows in [
            self.dropout_windows
                .iter()
                .map(|w| (w.from, w.until))
                .collect::<Vec<_>>(),
            self.bias_windows
                .iter()
                .map(|w| (w.from, w.until))
                .collect::<Vec<_>>(),
        ] {
            let mut previous_end = 0.;
            for (from, until) in windows {
                if !from.is_finite()
                    || !until.is_finite()
                    || from < previous_end
                    || from >= duration
                    || until <= from
                    || until > duration
                {
                    return Err("invalid or overlapping heading fault window".into());
                }
                previous_end = until;
            }
        }
        if self
            .bias_windows
            .iter()
            .any(|w| !w.offset_rad.is_finite() || w.offset_rad.abs() > std::f64::consts::PI)
        {
            return Err("heading bias must be a finite wrapped angle".into());
        }
        Ok(())
    }
}
impl Scenario {
    pub fn heading_observation(
        &self,
        seed: u64,
        tick: usize,
        time: f64,
        body_yaw: f64,
    ) -> Option<AbsoluteHeading> {
        let sensor = self.heading_sensor.as_ref()?;
        if !tick.is_multiple_of(sensor.period_ticks)
            || sensor
                .dropout_windows
                .iter()
                .any(|w| time + 1e-9 >= w.from && time < w.until - 1e-9)
        {
            return None;
        }
        let bias = sensor
            .bias_windows
            .iter()
            .find(|w| time + 1e-9 >= w.from && time < w.until - 1e-9)
            .map_or(0., |w| w.offset_rad);
        let noise = (3. * sensor.variance_rad2).sqrt() * uniform(seed, tick, 3);
        Some(AbsoluteHeading {
            stamp: time,
            yaw_rad: wrap_angle(body_yaw + bias + noise),
            variance_rad2: sensor.variance_rad2,
        })
    }
}
