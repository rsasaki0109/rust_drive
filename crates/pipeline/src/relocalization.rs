//! Measured-map proposals after loss. No scene labels or evaluation pose enter.
use crate::{
    MapLocalizationConfig, MapLocalizationDecision, MapLocalizationDiagnostics,
    MotionDiscontinuity, SensorFrame,
};
use rustdriving_core::{Pose, wrap_angle};
use rustdriving_localization::registration::regional::{RegionalSearchConfig, search_region};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RelocalizationConfig {
    pub position_half_extent_m: f64,
    pub yaw_half_extent_rad: f64,
    pub max_neighbor_checks: usize,
    pub max_attempts: usize,
    pub confirmations: usize,
    pub retry_interval_s: f64,
    /// Optional absolute map-only window after a measured epoch reset (seconds).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub map_only_grace_s: Option<f64>,
    /// Arm the configured GNSS recovery policy before any post-reset fix commits.
    #[serde(skip_serializing_if = "is_false")]
    pub confirm_gnss_handoff: bool,
    /// Preserve the live navigation mission and recheck it at the measured pose.
    #[serde(skip_serializing_if = "is_false")]
    pub preserve_navigation: bool,
    /// Retain served fixed-route stops and restart unfinished stop confirmations.
    #[serde(skip_serializing_if = "is_false")]
    pub preserve_stop_signs: bool,
    /// Preserve fixed-route signal duties, requiring new post-reset permission.
    #[serde(skip_serializing_if = "is_false")]
    pub preserve_traffic_signals: bool,
    /// Retain owned yield entries and restart uncommitted clear-scan confirmation.
    #[serde(skip_serializing_if = "is_false")]
    pub preserve_yield_intersections: bool,
}
fn is_false(value: &bool) -> bool {
    !*value
}
impl Default for RelocalizationConfig {
    fn default() -> Self {
        Self {
            position_half_extent_m: 4.,
            yaw_half_extent_rad: 0.3,
            max_neighbor_checks: 20_000_000,
            max_attempts: 8,
            confirmations: 3,
            retry_interval_s: 0.5,
            map_only_grace_s: None,
            confirm_gnss_handoff: false,
            preserve_navigation: false,
            preserve_stop_signs: false,
            preserve_traffic_signals: false,
            preserve_yield_intersections: false,
        }
    }
}
impl RelocalizationConfig {
    fn search(&self) -> RegionalSearchConfig {
        RegionalSearchConfig {
            position_half_extent_m: self.position_half_extent_m,
            yaw_half_extent_rad: self.yaw_half_extent_rad,
            max_neighbor_checks: self.max_neighbor_checks,
        }
    }
    pub(crate) fn validate(&self) -> Result<(), String> {
        self.search().validate()?;
        if !(1..=16).contains(&self.max_attempts)
            || !(2..=5).contains(&self.confirmations)
            || self.confirmations > self.max_attempts
            || !self.retry_interval_s.is_finite()
            || !(0.5..=0.6).contains(&self.retry_interval_s)
            || self
                .map_only_grace_s
                .is_some_and(|s| !s.is_finite() || !(0.5..=5.).contains(&s))
        {
            return Err("invalid bounded relocalization confirmation policy".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum RelocalizationPhase {
    Idle,
    WaitingForStillness,
    WaitingForScan,
    Confirming,
    Rejected,
    Exhausted,
    Reinitialized,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum MapOnlySupportStatus {
    WaitingForMap,
    Active,
    Expired,
    Interrupted,
    GnssAccepted,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapOnlySupportDiagnostics {
    pub reset_stamp: f64,
    pub expires_at: f64,
    pub last_accepted_map_stamp: Option<f64>,
    pub status: MapOnlySupportStatus,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RelocalizationCandidate {
    pub seed_index: usize,
    pub pose: Pose,
    pub rms_m: f64,
    pub inlier_count: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegionalSearchDiagnostics {
    pub center: Pose,
    pub seeds_tested: usize,
    pub neighbor_checks: usize,
    pub correlation_comparisons: usize,
    pub scan_points_used: usize,
    pub hypotheses: Vec<RelocalizationSeed>,
    pub error: Option<String>,
    pub candidates: Vec<RelocalizationCandidate>,
    pub proposed_pose: Option<Pose>,
    pub proposed_covariance: Option<[[f64; 3]; 3]>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RelocalizationSeed {
    pub pose: Pose,
    pub votes: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RelocalizationDiagnostics {
    pub phase: RelocalizationPhase,
    pub attempts: usize,
    pub stopped_samples: usize,
    pub confirmed_scans: usize,
    pub last_attempt_stamp: Option<f64>,
    pub last_reinitialized_stamp: Option<f64>,
    pub last_gap: Option<MotionDiscontinuity>,
    pub rejection: Option<String>,
    /// Full finite-hypothesis evidence appears only on an actual search frame.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub search: Option<RegionalSearchDiagnostics>,
    /// Separate from GNSS freshness: never changes a GNSS clock or counter.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub map_only_support: Option<MapOnlySupportDiagnostics>,
}
impl Default for RelocalizationDiagnostics {
    fn default() -> Self {
        Self {
            phase: RelocalizationPhase::Idle,
            attempts: 0,
            stopped_samples: 0,
            confirmed_scans: 0,
            last_attempt_stamp: None,
            last_reinitialized_stamp: None,
            last_gap: None,
            rejection: None,
            search: None,
            map_only_support: None,
        }
    }
}
pub(crate) struct Relocalizer {
    config: RelocalizationConfig,
    diagnostic: RelocalizationDiagnostics,
    last_motion: Option<f64>,
    confirmation: Option<(f64, Pose)>,
}
impl Relocalizer {
    pub fn new(config: RelocalizationConfig) -> Self {
        Self {
            config,
            diagnostic: RelocalizationDiagnostics::default(),
            last_motion: None,
            confirmation: None,
        }
    }
    pub fn diagnostics(&self) -> RelocalizationDiagnostics {
        self.diagnostic.clone()
    }
    pub fn begin_frame(&mut self) {
        self.diagnostic.search = None;
    }
    fn interrupt(&mut self, phase: RelocalizationPhase, reason: &str) {
        self.diagnostic.phase = phase;
        self.diagnostic.confirmed_scans = 0;
        self.diagnostic.rejection = Some(reason.into());
        self.confirmation = None;
    }
    /// Only a new post-reset accepted map correction can support a fresh epoch.
    /// Neither later scans nor a second call can extend the absolute deadline.
    pub fn map_only_support(
        &mut self,
        now: f64,
        map: Option<&MapLocalizationDiagnostics>,
        gnss_accepted: bool,
        interrupted: bool,
        healthy: bool,
    ) -> bool {
        let Some(support) = &mut self.diagnostic.map_only_support else {
            return false;
        };
        support.last_accepted_map_stamp = map.and_then(|m| m.last_accepted_stamp);
        if gnss_accepted {
            support.status = MapOnlySupportStatus::GnssAccepted;
        } else if matches!(
            support.status,
            MapOnlySupportStatus::Expired
                | MapOnlySupportStatus::Interrupted
                | MapOnlySupportStatus::GnssAccepted
        ) {
            return false;
        } else if now > support.expires_at + 1e-9 {
            support.status = MapOnlySupportStatus::Expired;
        } else if interrupted {
            support.status = MapOnlySupportStatus::Interrupted;
        } else if healthy
            && map.is_some_and(|m| {
                m.decision == MapLocalizationDecision::Accepted
                    && m.last_accepted_stamp.is_some_and(|stamp| {
                        stamp > support.reset_stamp && now - stamp <= 0.2 + 1e-9
                    })
            })
        {
            support.status = MapOnlySupportStatus::Active;
        } else {
            support.status = MapOnlySupportStatus::WaitingForMap;
        }
        support.status == MapOnlySupportStatus::Active
    }
    pub fn reinitialized(&mut self, now: f64) {
        self.diagnostic.phase = RelocalizationPhase::Reinitialized;
        self.diagnostic.last_reinitialized_stamp = Some(now);
        self.diagnostic.rejection = None;
        self.last_motion = None;
        self.confirmation = None;
        self.diagnostic.map_only_support =
            self.config
                .map_only_grace_s
                .map(|grace| MapOnlySupportDiagnostics {
                    reset_stamp: now,
                    expires_at: now + grace,
                    last_accepted_map_stamp: None,
                    status: MapOnlySupportStatus::WaitingForMap,
                });
    }
    pub fn observe(
        &mut self,
        input: &SensorFrame,
        gap: &MotionDiscontinuity,
        center: Pose,
        map: &MapLocalizationConfig,
    ) -> Option<(Pose, [[f64; 3]; 3])> {
        if self
            .diagnostic
            .last_gap
            .as_ref()
            .is_none_or(|old| old.detected_at != gap.detected_at)
        {
            let previous_reset = self.diagnostic.last_reinitialized_stamp;
            let previous_support = self.diagnostic.map_only_support.take();
            self.diagnostic = RelocalizationDiagnostics {
                last_gap: Some(gap.clone()),
                last_reinitialized_stamp: previous_reset,
                map_only_support: previous_support,
                ..RelocalizationDiagnostics::default()
            };
            self.last_motion = None;
            self.confirmation = None;
        }
        if self.diagnostic.phase == RelocalizationPhase::Exhausted {
            return None;
        }
        let stopped = input.odometry.filter(|motion| {
            motion.stamp.is_finite()
                && (motion.stamp - input.time).abs() <= 1e-9
                && motion.stamp > gap.detected_at
                && motion.speed.is_finite()
                && motion.speed.abs() <= 0.05
                && motion.yaw_rate.is_finite()
                && motion.yaw_rate.abs() <= 0.01
                && self.last_motion.is_none_or(|last| motion.stamp > last)
        });
        let Some(motion) = stopped else {
            self.diagnostic.stopped_samples = 0;
            self.interrupt(
                RelocalizationPhase::WaitingForStillness,
                "fresh synchronous stopped wheel/gyro samples required",
            );
            return None;
        };
        if self
            .last_motion
            .is_some_and(|last| motion.stamp - last > 0.075 + 1e-9)
        {
            self.diagnostic.stopped_samples = 0;
            self.interrupt(
                RelocalizationPhase::WaitingForStillness,
                "uncovered standstill interval",
            );
        }
        self.last_motion = Some(motion.stamp);
        self.diagnostic.stopped_samples = (self.diagnostic.stopped_samples + 1).min(3);
        if self.diagnostic.stopped_samples < 3 {
            self.diagnostic.phase = RelocalizationPhase::WaitingForStillness;
            return None;
        }
        if self
            .confirmation
            .is_some_and(|(last, _)| input.time - last > 0.75 + 1e-9)
        {
            self.interrupt(
                RelocalizationPhase::WaitingForScan,
                "confirmation acquisition expired",
            );
        }
        if input.lidar_failed {
            self.interrupt(
                RelocalizationPhase::WaitingForScan,
                "LiDAR acquisition failed",
            );
            return None;
        }
        let Some(scan) = &input.lidar else {
            self.diagnostic.phase = RelocalizationPhase::WaitingForScan;
            return None;
        };
        if !scan.stamp.is_finite()
            || (scan.stamp - input.time).abs() > 1e-9
            || scan.stamp <= gap.detected_at
            || scan.points.len() > 20_000
            || scan
                .points
                .iter()
                .any(|p| !p.finite() || p.x.hypot(p.y) > 200.)
        {
            self.interrupt(
                RelocalizationPhase::WaitingForScan,
                "current valid body-XY scan required after lost motion",
            );
            return None;
        }
        if self
            .diagnostic
            .last_attempt_stamp
            .is_some_and(|last| input.time - last < self.config.retry_interval_s - 1e-9)
        {
            return None;
        }
        self.diagnostic.attempts += 1;
        self.diagnostic.last_attempt_stamp = Some(scan.stamp);
        let result = search_region(
            &scan.points,
            &map.points,
            center,
            &map.registration(),
            &self.config.search(),
        );
        let mut search = RegionalSearchDiagnostics {
            center,
            seeds_tested: result.seeds_tested,
            neighbor_checks: result.neighbor_checks,
            correlation_comparisons: result.correlation_comparisons,
            scan_points_used: result.scan_points_used,
            hypotheses: result
                .hypotheses
                .iter()
                .map(|seed| RelocalizationSeed {
                    pose: seed.pose,
                    votes: seed.votes,
                })
                .collect(),
            error: result.error.clone(),
            candidates: result
                .candidates
                .iter()
                .map(|c| RelocalizationCandidate {
                    seed_index: c.seed_index,
                    pose: c.fit.pose,
                    rms_m: c.fit.rms_m,
                    inlier_count: c.fit.inlier_count,
                })
                .collect(),
            proposed_pose: None,
            proposed_covariance: None,
        };
        if let Some(error) = result.error {
            self.interrupt(RelocalizationPhase::Rejected, &error);
        } else if let Some(best) = result.best {
            let mut covariance = best.covariance;
            let floors = [0.01, 0.01, 1e-4];
            let scales: [f64; 3] =
                std::array::from_fn(|i| (floors[i] / covariance[i][i]).max(1.).sqrt());
            for i in 0..3 {
                for j in 0..3 {
                    covariance[i][j] *= scales[i] * scales[j];
                }
                covariance[i][i] = covariance[i][i].max(floors[i]);
            }
            search.proposed_pose = Some(best.pose);
            search.proposed_covariance = Some(covariance);
            let consistent = self.confirmation.is_some_and(|(_, old)| {
                old.position.distance(best.pose.position) <= 0.15
                    && wrap_angle(old.yaw - best.pose.yaw).abs() <= 0.02
            });
            self.diagnostic.confirmed_scans = if consistent {
                self.diagnostic.confirmed_scans + 1
            } else {
                1
            };
            self.confirmation = Some((scan.stamp, best.pose));
            self.diagnostic.phase = RelocalizationPhase::Confirming;
            self.diagnostic.rejection = None;
            if self.diagnostic.confirmed_scans >= self.config.confirmations {
                self.diagnostic.search = Some(search);
                return Some((best.pose, covariance));
            }
        }
        self.diagnostic.search = Some(search);
        if self.diagnostic.attempts >= self.config.max_attempts {
            self.interrupt(
                RelocalizationPhase::Exhausted,
                "relocalization attempt budget exhausted",
            );
        }
        None
    }
}
