//! Delivery-owned recovery policy; candidate corrections never confirm themselves.
use rustdriving_core::Pose;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GnssRecoveryConfig {
    pub max_position_correction_m: f64,
    pub max_yaw_correction_rad: f64,
    pub confirmation_fixes: u8,
    pub max_hold_s: f64,
}
impl GnssRecoveryConfig {
    pub fn validate(self) -> Result<(), String> {
        if !self.max_position_correction_m.is_finite()
            || self.max_position_correction_m <= 0.
            || self.max_position_correction_m > 1.
            || !self.max_yaw_correction_rad.is_finite()
            || self.max_yaw_correction_rad <= 0.
            || self.max_yaw_correction_rad > 0.2
            || !(2..=8).contains(&self.confirmation_fixes)
            || !self.max_hold_s.is_finite()
            || self.max_hold_s <= 0.
            || self.max_hold_s > 10.
        {
            return Err("GNSS recovery needs finite positive position/yaw bounds <= 1 m / 0.2 rad, 2..=8 fixes and a hold budget <= 10 s".into());
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecoveryPhase {
    #[default]
    Ready,
    Holding,
    Confirming,
    Expired,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GnssRecoveryDiagnostics {
    pub phase: RecoveryPhase,
    pub hold_started_at: Option<f64>,
    pub consecutive_candidates: u8,
    pub last_candidate_stamp: Option<f64>,
    pub last_position_correction_m: Option<f64>,
    pub last_yaw_correction_rad: Option<f64>,
    pub held_fixes: u64,
    pub completed_recoveries: u64,
}
pub(super) struct Recovery {
    config: GnssRecoveryConfig,
    pub diagnostics: GnssRecoveryDiagnostics,
}
impl Recovery {
    pub fn new(config: GnssRecoveryConfig) -> Self {
        Self {
            config,
            diagnostics: GnssRecoveryDiagnostics::default(),
        }
    }
    pub fn active(&self) -> bool {
        self.diagnostics.phase != RecoveryPhase::Ready
    }
    pub fn expire(&mut self) {
        if self.active() {
            self.diagnostics.phase = RecoveryPhase::Expired;
            self.diagnostics.consecutive_candidates = 0;
        }
    }
    pub fn advance(&mut self, now: f64) {
        if self
            .diagnostics
            .hold_started_at
            .is_some_and(|start| now - start > self.config.max_hold_s + 1e-9)
        {
            self.diagnostics.phase = RecoveryPhase::Expired;
            self.diagnostics.consecutive_candidates = 0;
        } else if self.diagnostics.phase == RecoveryPhase::Confirming
            && self
                .diagnostics
                .last_candidate_stamp
                .is_some_and(|stamp| now - stamp > super::MAX_GNSS_AGE_S + 1e-9)
        {
            self.interrupt();
        }
    }
    pub fn interrupt(&mut self) {
        self.diagnostics.consecutive_candidates = 0;
        if self.diagnostics.phase == RecoveryPhase::Confirming {
            self.diagnostics.phase = RecoveryPhase::Holding;
        }
    }
    pub fn rejected(&mut self, now: f64, streak: u8) {
        self.interrupt();
        if streak >= 2 && !self.active() {
            self.diagnostics.phase = RecoveryPhase::Holding;
            self.diagnostics.hold_started_at = Some(now);
        }
    }
    /// Compare complete current-time proposals, including delayed event replay.
    /// A rejected proposal must be discarded by the caller, including history.
    pub fn consider(&mut self, now: f64, stamp: f64, before: Pose, proposed: Pose) -> bool {
        self.advance(now);
        let distance = before.position.distance(proposed.position);
        let yaw = rustdriving_core::wrap_angle(proposed.yaw - before.yaw).abs();
        let fresh = self
            .diagnostics
            .last_candidate_stamp
            .is_none_or(|last| stamp > last);
        self.diagnostics.last_position_correction_m = distance.is_finite().then_some(distance);
        self.diagnostics.last_yaw_correction_rad = yaw.is_finite().then_some(yaw);
        self.diagnostics.last_candidate_stamp = Some(stamp);
        if self.diagnostics.phase == RecoveryPhase::Expired
            || !fresh
            || !distance.is_finite()
            || distance > self.config.max_position_correction_m
            || !yaw.is_finite()
            || yaw > self.config.max_yaw_correction_rad
        {
            self.interrupt();
        } else {
            self.diagnostics.consecutive_candidates += 1;
            self.diagnostics.phase = RecoveryPhase::Confirming;
            if self.diagnostics.consecutive_candidates >= self.config.confirmation_fixes {
                self.diagnostics.phase = RecoveryPhase::Ready;
                self.diagnostics.hold_started_at = None;
                self.diagnostics.consecutive_candidates = 0;
                self.diagnostics.completed_recoveries =
                    self.diagnostics.completed_recoveries.saturating_add(1);
                return true;
            }
        }
        self.diagnostics.held_fixes = self.diagnostics.held_fixes.saturating_add(1);
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::Vec2;
    fn monitor() -> Recovery {
        Recovery::new(GnssRecoveryConfig {
            max_position_correction_m: 0.4,
            max_yaw_correction_rad: 0.1,
            confirmation_fixes: 3,
            max_hold_s: 5.,
        })
    }
    #[test]
    fn candidates_need_new_stamps_and_rejections_break_the_confirmation_sequence() {
        let mut r = monitor();
        r.rejected(1., 1);
        assert!(!r.active());
        r.rejected(1.2, 2);
        let pose = Pose::default();
        let proposed = Pose {
            position: Vec2::new(0.1, 0.),
            yaw: 0.02,
        };
        assert!(!r.consider(1.4, 1.4, pose, proposed));
        assert!(!r.consider(1.5, 1.4, pose, proposed));
        assert_eq!(r.diagnostics.consecutive_candidates, 0);
        assert!(!r.consider(1.6, 1.6, pose, proposed));
        r.rejected(1.7, 4);
        assert!(!r.consider(1.8, 1.8, pose, proposed));
        assert!(!r.consider(2., 2., pose, proposed));
        assert!(r.consider(2.2, 2.2, pose, proposed));
        assert!(!r.active());
        assert_eq!(r.diagnostics.completed_recoveries, 1);
    }
    #[test]
    fn yaw_limits_and_missing_confirmations_hold_while_expiry_is_irreversible() {
        let mut r = monitor();
        r.rejected(1., 2);
        let p = Pose::default();
        assert!(!r.consider(1.2, 1.2, p, Pose { yaw: 0.11, ..p }));
        assert!(!r.consider(1.4, 1.4, p, p));
        r.advance(2.2);
        assert_eq!(r.diagnostics.phase, RecoveryPhase::Holding);
        assert_eq!(r.diagnostics.consecutive_candidates, 0);
        r.advance(6.01);
        assert_eq!(r.diagnostics.phase, RecoveryPhase::Expired);
        for i in 0..5 {
            assert!(!r.consider(6.2 + i as f64 * 0.2, 6.2 + i as f64 * 0.2, p, p));
        }
        assert_eq!(r.diagnostics.completed_recoveries, 0);
        let mut r = monitor();
        r.rejected(1., 2);
        r.expire();
        assert!(!r.consider(1.2, 1.2, p, p));
        assert_eq!(r.diagnostics.phase, RecoveryPhase::Expired);
    }
}
