//! Synthetic acquisition only; no runtime truth crosses the sensor boundary.
use crate::{Scenario, gnss_noise::uniform, sensor_timing::SensorFailureWindow};
use rustdriving_core::{AbsoluteHeading, wrap_angle};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

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
    /// Simulator-only transport delay; acquisition values and stamps are immutable.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub delay_ticks: usize,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub dropout_windows: Vec<SensorFailureWindow>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bias_windows: Vec<HeadingBiasWindow>,
}
impl HeadingSensor {
    pub fn validate(&self, duration: f64) -> Result<(), String> {
        if !(1..=7).contains(&self.period_ticks)
            || self.delay_ticks > 20
            || !self.variance_rad2.is_finite()
            || self.variance_rad2 <= 0.
            || self.variance_rad2 > std::f64::consts::PI.powi(2)
            || self.dropout_windows.len() > 32
            || self.bias_windows.len() > 32
        {
            return Err(
                "invalid heading acquisition period, delay, variance or window count".into(),
            );
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

fn is_zero(value: &usize) -> bool {
    *value == 0
}

#[derive(Default)]
pub(crate) struct HeadingDelivery {
    pending: VecDeque<(usize, AbsoluteHeading)>,
}
impl HeadingDelivery {
    pub(crate) fn apply(
        &mut self,
        delay: usize,
        tick: usize,
        sample: &mut Option<AbsoluteHeading>,
    ) {
        if let Some(acquired) = sample.take() {
            self.pending.push_back((tick + delay, acquired));
        }
        while self
            .pending
            .front()
            .is_some_and(|(ready, _)| *ready <= tick)
        {
            *sample = self.pending.pop_front().map(|(_, acquired)| acquired);
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transport_retains_acquired_values_through_missing_samples_and_skipped_delivery() {
        let mut delivery = HeadingDelivery::default();
        let sample = AbsoluteHeading {
            stamp: 0.,
            yaw_rad: 0.3,
            variance_rad2: 0.001,
        };
        let mut input = Some(sample);
        delivery.apply(3, 0, &mut input);
        assert!(input.is_none());
        delivery.apply(3, 2, &mut input);
        assert!(input.is_none());
        delivery.apply(3, 4, &mut input);
        let acquired = input.unwrap();
        assert_eq!(acquired.stamp, sample.stamp);
        assert_eq!(acquired.yaw_rad, sample.yaw_rad);
        assert_eq!(acquired.variance_rad2, sample.variance_rad2);
        input = None;
        delivery.apply(3, 5, &mut input);
        assert!(input.is_none());
    }
}
