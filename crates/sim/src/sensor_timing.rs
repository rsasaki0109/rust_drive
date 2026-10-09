//! Simulator-only LiDAR transport. Acquisition stamps and body-frame points are immutable.
use rustdriving_core::{Lidar3dScan, LidarScan, MultiHeightLidarScan};
use rustdriving_pipeline::SensorFrame;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// An immediate sensor-error observation during [from, until), in simulation seconds.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorFailureWindow {
    pub from: f64,
    pub until: f64,
}

/// Both adapters acquire at 10 Hz; selecting every Nth existing scan reduces the cadence.
/// Ticks are the common 0.05-second simulator steps, not wall-clock time.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorTiming {
    pub lidar_period_ticks: usize,
    pub lidar_delay_ticks: usize,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lidar_failure_windows: Vec<SensorFailureWindow>,
}
impl SensorTiming {
    pub fn validate(&self, duration: f64) -> Result<(), String> {
        if !(2..=10).contains(&self.lidar_period_ticks)
            || !self.lidar_period_ticks.is_multiple_of(2)
            || self.lidar_delay_ticks > 6
        {
            return Err(
                "LiDAR timing requires an even period of 2..=10 ticks and delay 0..=6 ticks".into(),
            );
        }
        if self.lidar_failure_windows.len() > 32 {
            return Err("at most 32 LiDAR failure windows are supported".into());
        }
        let mut previous_end = 0.0;
        for window in &self.lidar_failure_windows {
            if !window.from.is_finite()
                || !window.until.is_finite()
                || window.from < previous_end
                || window.from >= duration
                || window.until <= window.from
                || window.until > duration
            {
                return Err("invalid or overlapping LiDAR failure window".into());
            }
            previous_end = window.until;
        }
        Ok(())
    }
}

#[derive(Default)]
pub(crate) struct SensorDelivery {
    pending: VecDeque<PendingScan>,
}
struct PendingScan {
    ready_tick: usize,
    lidar: Option<LidarScan>,
    layered: Option<MultiHeightLidarScan>,
    cloud: Option<Lidar3dScan>,
}
impl SensorDelivery {
    pub(crate) fn apply(&mut self, timing: &SensorTiming, tick: usize, input: &mut SensorFrame) {
        let failure = input.lidar_failed
            || timing
                .lidar_failure_windows
                .iter()
                .any(|window| input.time + 1e-9 >= window.from && input.time < window.until - 1e-9);
        if failure {
            input.lidar_failed = true;
            input.lidar = None;
            input.multi_height_lidar = None;
            input.lidar3d = None;
            // A failure invalidates queued acquisitions. Recovery must acquire a new scan.
            self.pending.clear();
            return;
        }
        let scan = input.lidar.take();
        let layered = input.multi_height_lidar.take();
        let cloud = input.lidar3d.take();
        if (scan.is_some() || layered.is_some() || cloud.is_some())
            && tick.is_multiple_of(timing.lidar_period_ticks)
        {
            self.pending.push_back(PendingScan {
                ready_tick: tick + timing.lidar_delay_ticks,
                lidar: scan,
                layered,
                cloud,
            });
        }
        while self
            .pending
            .front()
            .is_some_and(|sample| sample.ready_tick <= tick)
        {
            // If a caller skipped delivery ticks, consume old eligible scans and emit the latest.
            if let Some(sample) = self.pending.pop_front() {
                input.lidar = sample.lidar;
                input.multi_height_lidar = sample.layered;
                input.lidar3d = sample.cloud;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Scenario, pipeline_config, simulate};
    use rustdriving_core::Vec2;

    fn timing(period: usize, delay: usize) -> SensorTiming {
        SensorTiming {
            lidar_period_ticks: period,
            lidar_delay_ticks: delay,
            lidar_failure_windows: vec![],
        }
    }
    fn frame(tick: usize) -> SensorFrame {
        let stamp = tick as f64 * 0.05;
        SensorFrame {
            time: stamp,
            odometry: None,
            gnss: None,
            lidar: tick.is_multiple_of(2).then(|| LidarScan {
                stamp,
                points: vec![Vec2::new(stamp, -1.0)],
            }),
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        }
    }

    #[test]
    fn delayed_delivery_preserves_acquisition_stamp_points_and_cadence() {
        let timing = timing(4, 3);
        let mut delivery = SensorDelivery::default();
        for tick in 0..=15 {
            let mut input = frame(tick);
            delivery.apply(&timing, tick, &mut input);
            if tick >= 3 && (tick - 3).is_multiple_of(4) {
                let scan = input.lidar.unwrap();
                let acquired = (tick - 3) as f64 * 0.05;
                assert_eq!(scan.stamp, acquired);
                assert_eq!(scan.points, vec![Vec2::new(acquired, -1.0)]);
                assert!(scan.stamp < input.time);
            } else {
                assert!(input.lidar.is_none());
            }
            assert!(!input.lidar_failed);
        }
    }

    fn layered_frame(tick: usize) -> SensorFrame {
        let mut input = frame(tick);
        input.multi_height_lidar = input.lidar.take().map(|scan| MultiHeightLidarScan {
            stamp: scan.stamp,
            planes: [0.6, 0.15, 3.7]
                .into_iter()
                .map(|height_m| rustdriving_core::LidarPlane {
                    height_m,
                    points: scan.points.clone(),
                })
                .collect(),
        });
        input
    }

    #[test]
    fn layered_delivery_preserves_every_plane_as_one_delayed_acquisition() {
        let mut delivery = SensorDelivery::default();
        let timing = timing(4, 3);
        for tick in 0..=15 {
            let mut input = layered_frame(tick);
            delivery.apply(&timing, tick, &mut input);
            assert!(input.lidar.is_none());
            if tick >= 3 && (tick - 3).is_multiple_of(4) {
                let scan = input.multi_height_lidar.unwrap();
                let acquired = (tick - 3) as f64 * 0.05;
                assert_eq!(scan.stamp, acquired);
                assert_eq!(scan.planes.len(), 3);
                for (plane, height) in scan.planes.iter().zip([0.6, 0.15, 3.7]) {
                    assert_eq!(plane.height_m, height);
                    assert_eq!(plane.points, vec![Vec2::new(acquired, -1.0)]);
                }
            } else {
                assert!(input.multi_height_lidar.is_none());
            }
        }
    }

    #[test]
    fn partial_or_scheduled_failure_flushes_all_layers_before_fresh_recovery() {
        for explicit in [false, true] {
            let mut timing = timing(2, 3);
            if !explicit {
                timing.lidar_failure_windows.push(SensorFailureWindow {
                    from: 0.05,
                    until: 0.1,
                });
            }
            let mut delivery = SensorDelivery::default();
            for tick in 0..=5 {
                let mut input = layered_frame(tick);
                input.lidar_failed = explicit && tick == 1;
                delivery.apply(&timing, tick, &mut input);
                assert_eq!(input.lidar_failed, tick == 1);
                assert!(input.lidar.is_none());
                if tick == 5 {
                    let scan = input.multi_height_lidar.unwrap();
                    assert_eq!(scan.stamp, 0.1);
                    assert_eq!(scan.planes.len(), 3);
                } else {
                    assert!(input.multi_height_lidar.is_none());
                }
            }
        }
    }

    #[test]
    fn layered_queue_emits_latest_eligible_bundle_once_after_skipped_ticks() {
        let mut delivery = SensorDelivery::default();
        let timing = timing(2, 3);
        for tick in [0, 2] {
            let mut input = layered_frame(tick);
            delivery.apply(&timing, tick, &mut input);
            assert!(input.multi_height_lidar.is_none());
        }
        let mut skipped = layered_frame(7);
        delivery.apply(&timing, 7, &mut skipped);
        assert_eq!(skipped.multi_height_lidar.unwrap().stamp, 0.1);
        let mut next = layered_frame(9);
        delivery.apply(&timing, 9, &mut next);
        assert!(next.multi_height_lidar.is_none());
    }

    fn cloud_frame(tick: usize) -> SensorFrame {
        let mut input = frame(tick);
        input.lidar3d = input.lidar.take().map(|scan| {
            let range = 12.0 + scan.stamp;
            let azimuth = std::f64::consts::PI / 719.0;
            let elevation = std::f64::consts::PI / 60.0;
            Lidar3dScan {
                stamp: scan.stamp,
                returns: vec![rustdriving_core::Lidar3dReturn {
                    ray_index: 360 * 16 + 9,
                    point: rustdriving_core::Vec3::new(
                        range * elevation.cos() * azimuth.cos(),
                        -range * elevation.cos() * azimuth.sin(),
                        0.6 + range * elevation.sin(),
                    ),
                }],
            }
        });
        input
    }

    #[test]
    fn tilted_cloud_delay_preserves_xyz_indices_and_acquisition_stamp_atomically() {
        let timing = timing(4, 3);
        let mut delivery = SensorDelivery::default();
        for tick in 0..=15 {
            let mut input = cloud_frame(tick);
            delivery.apply(&timing, tick, &mut input);
            assert!(input.lidar.is_none() && input.multi_height_lidar.is_none());
            if tick >= 3 && (tick - 3).is_multiple_of(4) {
                let scan = input.lidar3d.unwrap();
                let acquired = cloud_frame(tick - 3).lidar3d.unwrap();
                assert_eq!(
                    serde_json::to_value(scan).unwrap(),
                    serde_json::to_value(acquired).unwrap()
                );
            } else {
                assert!(input.lidar3d.is_none());
            }
        }
    }

    #[test]
    fn tilted_cloud_partial_and_scheduled_failures_flush_before_new_acquisition() {
        for explicit in [false, true] {
            let mut timing = timing(2, 3);
            if !explicit {
                timing.lidar_failure_windows.push(SensorFailureWindow {
                    from: 0.05,
                    until: 0.1,
                });
            }
            let mut delivery = SensorDelivery::default();
            for tick in 0..=5 {
                let mut input = cloud_frame(tick);
                input.lidar_failed = explicit && tick == 1;
                delivery.apply(&timing, tick, &mut input);
                assert_eq!(input.lidar_failed, tick == 1);
                if tick == 5 {
                    assert_eq!(input.lidar3d.unwrap().stamp, 0.1);
                } else {
                    assert!(input.lidar3d.is_none());
                }
            }
        }
    }

    #[test]
    fn tilted_cloud_skipped_delivery_emits_latest_eligible_acquisition_only_once() {
        let timing = timing(2, 3);
        let mut delivery = SensorDelivery::default();
        for tick in [0, 2] {
            let mut input = cloud_frame(tick);
            delivery.apply(&timing, tick, &mut input);
            assert!(input.lidar3d.is_none());
        }
        let mut skipped = cloud_frame(7);
        delivery.apply(&timing, 7, &mut skipped);
        assert_eq!(skipped.lidar3d.unwrap().stamp, 0.1);
        let mut next = cloud_frame(9);
        delivery.apply(&timing, 9, &mut next);
        assert!(next.lidar3d.is_none());
    }

    #[test]
    fn latest_eligible_scan_is_delivered_once_without_future_data() {
        let mut delivery = SensorDelivery::default();
        let timing = timing(2, 2);
        for tick in [0, 2] {
            let mut input = frame(tick);
            delivery.apply(&timing, tick, &mut input);
            if tick == 0 {
                assert!(input.lidar.is_none());
            } else {
                assert_eq!(input.lidar.unwrap().stamp, 0.0);
            }
        }
        let mut skipped = frame(7);
        delivery.apply(&timing, 7, &mut skipped);
        assert_eq!(skipped.lidar.unwrap().stamp, 0.1);
        let mut next = frame(9);
        delivery.apply(&timing, 9, &mut next);
        assert!(next.lidar.is_none());
    }

    #[test]
    fn explicit_and_scheduled_failure_flush_old_acquisitions_before_recovery() {
        for observed_failure in [false, true] {
            let mut timing = timing(2, 3);
            if !observed_failure {
                timing.lidar_failure_windows.push(SensorFailureWindow {
                    from: 0.05,
                    until: 0.1,
                });
            }
            let mut delivery = SensorDelivery::default();
            for tick in 0..=5 {
                let mut input = frame(tick);
                input.lidar_failed = observed_failure && tick == 1;
                delivery.apply(&timing, tick, &mut input);
                assert_eq!(input.lidar_failed, tick == 1);
                if tick == 5 {
                    assert_eq!(input.lidar.unwrap().stamp, 0.1);
                } else {
                    assert!(input.lidar.is_none());
                }
            }
        }
    }

    #[test]
    fn calibration_rejects_unsupported_clocks_and_invalid_failure_windows() {
        for (period, delay) in [(0, 0), (1, 0), (3, 0), (12, 0), (2, 7)] {
            assert!(timing(period, delay).validate(5.0).is_err());
        }
        for (from, until) in [
            (-0.1, 1.0),
            (2.0, 1.0),
            (2.0, 2.0),
            (4.0, 5.1),
            (f64::NAN, 1.0),
        ] {
            let mut invalid = timing(2, 1);
            invalid
                .lidar_failure_windows
                .push(SensorFailureWindow { from, until });
            assert!(invalid.validate(5.0).is_err());
        }
        let mut overlapping = timing(4, 0);
        overlapping.lidar_failure_windows = vec![
            SensorFailureWindow {
                from: 1.0,
                until: 2.0,
            },
            SensorFailureWindow {
                from: 1.5,
                until: 3.0,
            },
        ];
        assert!(overlapping.validate(5.0).is_err());
        assert!(timing(10, 6).validate(5.0).is_ok());
    }

    #[test]
    fn default_transport_is_identical_and_schedules_never_enter_replay_header() {
        let mut scenario: Scenario =
            serde_json::from_str(r#"{"name":"LiDAR transport test","duration":45,"road_length":100,"half_width":2.1,"expected":"goal","objects":[]}"#).unwrap();
        let baseline = simulate(scenario.clone(), 7).unwrap();
        let config_before = serde_json::to_value(pipeline_config(&scenario)).unwrap();
        scenario.sensor_timing = Some(timing(2, 0));
        assert_eq!(
            config_before,
            serde_json::to_value(pipeline_config(&scenario)).unwrap()
        );
        let transported = simulate(scenario, 7).unwrap();
        assert_eq!(
            {
                let mut bytes = Vec::new();
                baseline.sensor_log.unwrap().write(&mut bytes).unwrap();
                bytes
            },
            {
                let mut bytes = Vec::new();
                transported.sensor_log.unwrap().write(&mut bytes).unwrap();
                bytes
            }
        );
        assert_eq!(
            serde_json::to_value(baseline.summary).unwrap(),
            serde_json::to_value(transported.summary).unwrap()
        );
    }

    #[test]
    fn transient_sensor_failure_brakes_then_recovers_after_new_acquisition() {
        let mut scenario: Scenario =
            serde_json::from_str(r#"{"name":"LiDAR transport test","duration":45,"road_length":100,"half_width":2.1,"expected":"goal","objects":[]}"#).unwrap();
        scenario.sensor_timing = Some(SensorTiming {
            lidar_period_ticks: 2,
            lidar_delay_ticks: 2,
            lidar_failure_windows: vec![SensorFailureWindow {
                from: 3.0,
                until: 4.0,
            }],
        });
        let run = simulate(scenario, 7).unwrap();
        assert!(run.summary.passed, "{:?}", run.summary.failures);
        assert!(run.summary.emergency_steps >= 20);
        assert!(run.summary.reached_goal);
        let log = run.sensor_log.unwrap();
        let serialized = serde_json::to_value(&log.header).unwrap();
        for hidden in [
            "sensor_timing",
            "lidar_failure_windows",
            "lidar_delay_ticks",
        ] {
            assert!(!serialized["config"].to_string().contains(hidden));
        }
        for tick in &log.ticks {
            if (3.0..4.0).contains(&tick.input.time) {
                assert!(tick.input.lidar_failed);
                assert!(tick.input.lidar.is_none());
                assert!(tick.expected.emergency);
            }
            if let Some(scan) = &tick.input.lidar {
                assert!((tick.input.time - scan.stamp - 0.1).abs() < 1e-9);
                assert!(!(3.0..4.0).contains(&scan.stamp));
            }
        }
        let mut bytes = Vec::new();
        log.write(&mut bytes).unwrap();
        let replay =
            rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap();
        assert!(replay.verified);
        assert_eq!(replay.ticks, run.summary.steps);
    }
}
