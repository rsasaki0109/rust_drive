//! Optional local scan registration to a supplied surveyed world-frame map.
use rustdriving_core::{LidarScan, Pose, Vec2};
use rustdriving_localization::{
    Ekf,
    registration::{RegistrationConfig, match_scan},
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MapLocalizationConfig {
    pub points: Vec<Vec2>,
    pub max_gnss_outage_s: f64,
    pub max_correspondence_m: f64,
    pub max_rms_m: f64,
    pub min_pairs: usize,
    pub min_overlap: f64,
    pub max_translation_jump_m: f64,
    pub max_yaw_jump_rad: f64,
}
impl Default for MapLocalizationConfig {
    fn default() -> Self {
        let registration = RegistrationConfig::default();
        Self {
            points: vec![],
            max_gnss_outage_s: 10.0,
            max_correspondence_m: registration.max_correspondence_m,
            max_rms_m: registration.max_rms_m,
            min_pairs: registration.min_pairs,
            min_overlap: registration.min_overlap,
            max_translation_jump_m: registration.max_translation_jump_m,
            max_yaw_jump_rad: registration.max_yaw_jump_rad,
        }
    }
}
impl MapLocalizationConfig {
    fn registration(&self) -> RegistrationConfig {
        RegistrationConfig {
            max_correspondence_m: self.max_correspondence_m,
            max_rms_m: self.max_rms_m,
            min_pairs: self.min_pairs,
            min_overlap: self.min_overlap,
            max_translation_jump_m: self.max_translation_jump_m,
            max_yaw_jump_rad: self.max_yaw_jump_rad,
            ..RegistrationConfig::default()
        }
    }
    pub(crate) fn validate(&self) -> Result<(), String> {
        self.registration().validate()?;
        if !self.max_gnss_outage_s.is_finite()
            || !(0.75..=10.0).contains(&self.max_gnss_outage_s)
            || self.points.len() < self.min_pairs
            || self.points.len() > 100_000
            || self
                .points
                .iter()
                .any(|p| !p.finite() || p.x.abs() > 1e6 || p.y.abs() > 1e6)
        {
            return Err("invalid bounded surveyed localization map".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MapLocalizationDecision {
    Missing,
    Accepted,
    Rejected,
    Stale,
    UnsupportedTiming,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapLocalizationDiagnostics {
    pub last_observed_stamp: Option<f64>,
    pub last_accepted_stamp: Option<f64>,
    pub decision: MapLocalizationDecision,
    pub rejection: Option<String>,
    pub matched_pose: Option<Pose>,
    pub rms_m: Option<f64>,
    pub inlier_fraction: Option<f64>,
    pub inlier_count: Option<usize>,
    pub geometry_ratio: Option<f64>,
    pub condition_number: Option<f64>,
    pub neighbor_checks: Option<usize>,
    pub ambiguity_probes: Option<usize>,
    pub registration_covariance: Option<[[f64; 3]; 3]>,
    /// Actual EKF measurement covariance, after conservative floor scaling.
    pub covariance: Option<[[f64; 3]; 3]>,
    pub gnss_outage_covered: bool,
}
impl Default for MapLocalizationDiagnostics {
    fn default() -> Self {
        Self {
            last_observed_stamp: None,
            last_accepted_stamp: None,
            decision: MapLocalizationDecision::Missing,
            rejection: None,
            matched_pose: None,
            rms_m: None,
            inlier_fraction: None,
            inlier_count: None,
            geometry_ratio: None,
            condition_number: None,
            neighbor_checks: None,
            ambiguity_probes: None,
            registration_covariance: None,
            covariance: None,
            gnss_outage_covered: false,
        }
    }
}

pub(crate) struct MapLocalization {
    config: MapLocalizationConfig,
    diagnostic: MapLocalizationDiagnostics,
}
impl MapLocalization {
    pub(crate) fn new(config: MapLocalizationConfig) -> Self {
        Self {
            config,
            diagnostic: MapLocalizationDiagnostics::default(),
        }
    }
    pub(crate) fn update(
        &mut self,
        scan: Option<&LidarScan>,
        now: f64,
        eligible: bool,
        ekf: &mut Ekf,
    ) {
        if let Some(scan) = scan {
            if scan.stamp != now {
                self.diagnostic.decision = MapLocalizationDecision::UnsupportedTiming;
                self.diagnostic.rejection =
                    Some("map matching requires synchronous acquisition".into());
            } else if !eligible {
                self.diagnostic.decision = MapLocalizationDecision::Rejected;
                self.diagnostic.rejection = Some("invalid or repeated map observation".into());
            } else {
                let previous_accepted = self.diagnostic.last_accepted_stamp;
                self.diagnostic = MapLocalizationDiagnostics {
                    last_observed_stamp: Some(scan.stamp),
                    last_accepted_stamp: previous_accepted,
                    ..MapLocalizationDiagnostics::default()
                };
                match match_scan(
                    &scan.points,
                    &self.config.points,
                    ekf.state().pose,
                    &self.config.registration(),
                ) {
                    Ok(result) => {
                        let mut covariance = result.covariance;
                        // Congruence scaling preserves correlations and positive
                        // definiteness while preventing an overconfident IID fit.
                        let floor = [0.01, 0.01, 1e-4];
                        let scale = std::array::from_fn::<_, 3, _>(|i| {
                            (floor[i] / covariance[i][i]).max(1.0).sqrt()
                        });
                        for i in 0..3 {
                            for j in 0..3 {
                                covariance[i][j] *= scale[i] * scale[j];
                            }
                            // Repair a possible downward rounding at the floor;
                            // adding a nonnegative diagonal preserves SPD.
                            covariance[i][i] = covariance[i][i].max(floor[i]);
                        }
                        self.diagnostic.matched_pose = Some(result.pose);
                        self.diagnostic.rms_m = Some(result.rms_m);
                        self.diagnostic.inlier_fraction = Some(result.inlier_fraction);
                        self.diagnostic.inlier_count = Some(result.inlier_count);
                        self.diagnostic.geometry_ratio = Some(result.conditioning.geometry_ratio);
                        self.diagnostic.condition_number =
                            Some(result.conditioning.condition_number);
                        self.diagnostic.neighbor_checks = Some(result.conditioning.neighbor_checks);
                        self.diagnostic.ambiguity_probes =
                            Some(result.conditioning.ambiguity_probes);
                        self.diagnostic.registration_covariance = Some(result.covariance);
                        self.diagnostic.covariance = Some(covariance);
                        if ekf.correct_map_pose(result.pose, covariance) {
                            self.diagnostic.last_accepted_stamp = Some(scan.stamp);
                            self.diagnostic.decision = MapLocalizationDecision::Accepted;
                        } else {
                            self.diagnostic.decision = MapLocalizationDecision::Rejected;
                            self.diagnostic.rejection =
                                Some("EKF rejected map pose correction".into());
                        }
                    }
                    Err(error) => {
                        self.diagnostic.decision = MapLocalizationDecision::Rejected;
                        self.diagnostic.rejection = Some(error);
                    }
                }
            }
        } else if !eligible {
            self.diagnostic.decision = MapLocalizationDecision::Rejected;
            self.diagnostic.rejection = Some("map acquisition failed".into());
        }
        if self.diagnostic.decision == MapLocalizationDecision::Accepted
            && self
                .diagnostic
                .last_accepted_stamp
                .is_none_or(|t| now - t > 0.2 + 1e-9)
        {
            self.diagnostic.decision = MapLocalizationDecision::Stale;
            self.diagnostic.rejection = Some("accepted map localization became stale".into());
        }
        self.diagnostic.gnss_outage_covered = ekf.last_gnss.is_finite()
            && now - ekf.last_gnss > 0.75 + 1e-9
            && now - ekf.last_gnss <= self.config.max_gnss_outage_s + 1e-9
            && self.diagnostic.decision == MapLocalizationDecision::Accepted
            && self
                .diagnostic
                .last_accepted_stamp
                .is_some_and(|t| now - t <= 0.2 + 1e-9);
    }
    pub(crate) fn effective_fix_stamp(&self, actual_gnss: f64) -> f64 {
        if self.diagnostic.gnss_outage_covered {
            self.diagnostic.last_accepted_stamp.unwrap()
        } else {
            actual_gnss
        }
    }
    pub(crate) fn diagnostics(&self) -> MapLocalizationDiagnostics {
        self.diagnostic.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame};
    use rustdriving_core::{Gnss, Odometry, Route, VehicleConfig};

    fn surveyed_points() -> Vec<Vec2> {
        (0..36)
            .map(|i| {
                Vec2::new(
                    4.0 + 0.7 * i as f64,
                    4.0 * (1.7 * i as f64).sin() + 0.03 * i as f64,
                )
            })
            .collect()
    }
    fn pipeline(points: Vec<Vec2>, max_outage: f64) -> DrivingPipeline {
        let mut config = PipelineConfig::new(
            Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
            Pose::default(),
            VehicleConfig::default(),
        );
        config.localization_map = Some(MapLocalizationConfig {
            points,
            max_gnss_outage_s: max_outage,
            ..MapLocalizationConfig::default()
        });
        DrivingPipeline::new(config).unwrap()
    }
    fn frame(time: f64, points: &[Vec2], initial_gnss: bool) -> SensorFrame {
        SensorFrame {
            time,
            odometry: Some(Odometry {
                stamp: time,
                speed: 0.0,
                yaw_rate: 0.0,
            }),
            gnss: initial_gnss.then_some(Gnss {
                stamp: time,
                position: Vec2::default(),
                variance: 0.02,
            }),
            lidar: Some(LidarScan {
                stamp: time,
                points: points.to_vec(),
            }),
            lidar_failed: false,
            multi_height_lidar: None,
            lidar3d: None,
            navigation_update: None,
            traffic_signal: None,
        }
    }
    #[test]
    fn accepted_map_bridges_only_a_bounded_actual_gnss_outage() {
        let points = surveyed_points();
        let mut p = pipeline(points.clone(), 1.0);
        for i in 0..=20 {
            let out = p.step(&frame(i as f64 * 0.05, &points, i == 0)).unwrap();
            assert!(
                !out.health.contains(&HealthIssue::StaleGnss),
                "{:?}",
                out.map_localization
            );
            let map = out.map_localization.unwrap();
            assert_eq!(map.decision, MapLocalizationDecision::Accepted);
            assert_eq!(map.gnss_outage_covered, i > 15);
            let covariance = map.covariance.unwrap();
            assert!(
                covariance[0][0] >= 0.01 && covariance[1][1] >= 0.01 && covariance[2][2] >= 1e-4
            );
            let gnss = out.localization.unwrap();
            assert_eq!(gnss.last_accepted_stamp, Some(0.0));
            assert_eq!(gnss.accepted_fixes, 1);
            assert_eq!(gnss.last_observed_stamp, Some(0.0));
        }
        let expired = p.step(&frame(1.05, &points, false)).unwrap();
        assert!(expired.health.contains(&HealthIssue::StaleGnss));
        assert_eq!(expired.command.acceleration, -6.0);
        assert!(!expired.map_localization.unwrap().gnss_outage_covered);
        assert!(
            p.step(&frame(1.1, &points, true))
                .unwrap()
                .health
                .is_empty()
        );
    }
    #[test]
    fn a_map_correction_cannot_invent_initial_gnss_acceptance() {
        let points = surveyed_points();
        let mut p = pipeline(points.clone(), 10.0);
        let out = p.step(&frame(0.0, &points, false)).unwrap();
        assert_eq!(
            out.map_localization.as_ref().unwrap().decision,
            MapLocalizationDecision::Accepted
        );
        assert!(out.health.contains(&HealthIssue::StaleGnss));
        assert_eq!(out.localization.unwrap().accepted_fixes, 0);
        assert!(!out.map_localization.unwrap().gnss_outage_covered);
    }
    #[test]
    fn rejected_and_degenerate_matches_do_not_cover_untrusted_pose_age() {
        let points = surveyed_points();
        let collinear: Vec<_> = (0..36).map(|i| Vec2::new(3.0 + i as f64, 0.0)).collect();
        for degenerate in [false, true] {
            let map = if degenerate { &collinear } else { &points };
            let mut p = pipeline(map.clone(), 10.0);
            for i in 0..=15 {
                let out = p.step(&frame(i as f64 * 0.05, map, i == 0)).unwrap();
                // Fresh real GNSS can support operation despite rejected map geometry.
                assert!(!out.health.contains(&HealthIssue::StaleGnss));
            }
            let observed = if degenerate {
                collinear.clone()
            } else {
                vec![]
            };
            let out = p.step(&frame(0.8, &observed, false)).unwrap();
            assert!(out.health.contains(&HealthIssue::StaleGnss));
            let map = out.map_localization.unwrap();
            assert_eq!(map.decision, MapLocalizationDecision::Rejected);
            assert!(map.rejection.is_some());
            assert!(!map.gnss_outage_covered);
        }
    }
    #[test]
    fn missing_and_delayed_scans_cannot_refresh_map_localization() {
        let points = surveyed_points();
        for delayed in [false, true] {
            let mut p = pipeline(points.clone(), 10.0);
            for i in 0..=16 {
                p.step(&frame(i as f64 * 0.05, &points, i == 0)).unwrap();
            }
            let mut next = frame(0.85, &points, false);
            if delayed {
                next.lidar.as_mut().unwrap().stamp = 0.8;
                let out = p.step(&next).unwrap();
                assert!(out.health.contains(&HealthIssue::StaleGnss));
                assert_eq!(
                    out.map_localization.unwrap().decision,
                    MapLocalizationDecision::UnsupportedTiming
                );
            } else {
                next.lidar = None;
                let out = p.step(&next).unwrap();
                assert!(out.map_localization.unwrap().gnss_outage_covered);
                for i in 18..=21 {
                    let mut missing = frame(i as f64 * 0.05, &points, false);
                    missing.lidar = None;
                    next = missing;
                    let out = p.step(&next).unwrap();
                    if i == 21 {
                        assert!(out.health.contains(&HealthIssue::StaleGnss));
                        assert_eq!(
                            out.map_localization.unwrap().decision,
                            MapLocalizationDecision::Stale
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn omitted_map_fields_preserve_the_default_sensor_contract() {
        let config = PipelineConfig::new(
            Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
            Pose::default(),
            VehicleConfig::default(),
        );
        let encoded = serde_json::to_value(&config).unwrap();
        assert!(encoded.get("localization_map").is_none());
        let mut p = DrivingPipeline::new(config).unwrap();
        let out = serde_json::to_value(p.step(&frame(0.0, &[], true)).unwrap()).unwrap();
        assert!(out.get("map_localization").is_none());
        let mut bad = MapLocalizationConfig {
            points: surveyed_points(),
            ..MapLocalizationConfig::default()
        };
        bad.max_gnss_outage_s = 10.1;
        assert!(bad.validate().is_err());
        bad.max_gnss_outage_s = 10.0;
        bad.points[0].x = f64::NAN;
        assert!(bad.validate().is_err());
    }

    #[test]
    fn clock_and_invalid_observations_cannot_refresh_map_acceptance() {
        let points = surveyed_points();
        let mut p = pipeline(points.clone(), 10.0);
        p.step(&frame(0.0, &points, true)).unwrap();
        assert!(p.step(&frame(f64::NAN, &points, false)).is_err());
        assert_eq!(
            p.map_localization
                .as_ref()
                .unwrap()
                .diagnostics()
                .last_accepted_stamp,
            Some(0.0)
        );
        let mut future = frame(0.05, &points, false);
        future.lidar.as_mut().unwrap().stamp = 0.1;
        let out = p.step(&future).unwrap();
        assert!(out.health.contains(&HealthIssue::InvalidLidar));
        assert_eq!(out.map_localization.unwrap().last_accepted_stamp, Some(0.0));
        let mut malformed = frame(0.1, &points, false);
        malformed.lidar.as_mut().unwrap().points[0].x = f64::NAN;
        let out = p.step(&malformed).unwrap();
        assert!(out.health.contains(&HealthIssue::InvalidLidar));
        assert_eq!(out.map_localization.unwrap().last_accepted_stamp, Some(0.0));
        let out = p.step(&frame(0.15, &points, false)).unwrap();
        assert!(out.health.is_empty());
        assert_eq!(
            out.map_localization.unwrap().last_accepted_stamp,
            Some(0.15)
        );
    }

    #[test]
    fn accepted_map_does_not_clear_the_gnss_innovation_hold() {
        let points = surveyed_points();
        let mut p = pipeline(points.clone(), 10.0);
        p.step(&frame(0.0, &points, true)).unwrap();
        for i in 1..=16 {
            let mut input = frame(i as f64 * 0.05, &points, i <= 2);
            if let Some(fix) = &mut input.gnss {
                fix.position = Vec2::new(30.0, -25.0);
            }
            let out = p.step(&input).unwrap();
            assert_eq!(
                out.map_localization.as_ref().unwrap().decision,
                MapLocalizationDecision::Accepted
            );
            if i >= 2 {
                assert!(out.health.contains(&HealthIssue::GnssInnovationHold));
                assert_eq!(out.command.acceleration, -6.0);
            }
            assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(0.0));
        }
        assert!(
            p.step(&frame(0.85, &points, true))
                .unwrap()
                .health
                .is_empty()
        );
    }

    #[test]
    fn map_localization_replays_actual_sensors_and_rejects_forged_acceptance() {
        use crate::replay::{SensorLog, verify};
        let points = surveyed_points();
        let mut p = pipeline(points.clone(), 10.0);
        let mut log = SensorLog::new("surveyed-map-sensor-replay", p.config.clone());
        for i in 0..=20 {
            let input = frame(i as f64 * 0.05, &points, i == 0);
            let output = p.step(&input).unwrap();
            log.record(input, output);
        }
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        assert_eq!(
            verify(std::io::Cursor::new(&bytes), std::io::sink())
                .unwrap()
                .ticks,
            21
        );
        let mut records: Vec<serde_json::Value> = String::from_utf8(bytes)
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        records[17]["tick"]["expected"]["map_localization"]["last_accepted_stamp"] =
            serde_json::json!(0.0);
        let forged: String = records
            .iter()
            .map(|record| serde_json::to_string(record).unwrap() + "\n")
            .collect();
        assert!(
            verify(std::io::Cursor::new(forged), std::io::sink())
                .unwrap_err()
                .contains("mismatch")
        );
    }
}
