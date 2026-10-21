//! A new stopping commitment needs new scene evidence, never cached control dwell.
use rustdriving_core::{CheckedApproachDiagnostics, Vec2};

#[derive(Clone)]
struct Support {
    first: f64,
    last: f64,
    count: u8,
    position: Vec2,
    progress: f64,
    endpoint: Vec2,
    target: f64,
    ids: Vec<u64>,
}

#[derive(Clone)]
pub(super) struct Proposal {
    pub endpoint: Vec2,
    pub end_progress: f64,
    pub target: f64,
    pub ids: Vec<u64>,
}

#[derive(Clone, Default)]
pub(super) struct Confirmation {
    consumed: Option<f64>,
    last_control: Option<f64>,
    support: Option<Support>,
    pub diagnostics: CheckedApproachDiagnostics,
}

impl Confirmation {
    pub fn reset(&mut self) {
        self.support = None;
        self.diagnostics = CheckedApproachDiagnostics::default();
    }

    pub fn observe(
        &mut self,
        clock: Option<(f64, Option<f64>)>,
        position: Vec2,
        progress: f64,
        proposal: Option<Proposal>,
    ) -> bool {
        self.diagnostics = CheckedApproachDiagnostics::default();
        let Some((now, scene)) = clock else {
            self.reset();
            return false;
        };
        if !now.is_finite() || now < 0. || self.last_control.is_some_and(|old| now < old) {
            self.reset();
            return false;
        }
        self.last_control = Some(now);
        let valid = scene.filter(|stamp| {
            stamp.is_finite()
                && *stamp >= 0.
                && *stamp <= now
                && now - stamp <= 0.35 + 1e-9
                && self.consumed.is_none_or(|old| *stamp >= old)
        });
        let fresh = valid.is_some_and(|stamp| self.consumed.is_none_or(|old| stamp > old));
        if valid.is_none() {
            self.consumed = Some(self.consumed.map_or(now, |old| old.max(now)));
            self.reset();
            return false;
        }
        if fresh {
            self.consumed = valid;
        }
        let Some(proposal) = proposal.filter(|p| {
            position.finite()
                && progress.is_finite()
                && p.endpoint.finite()
                && p.end_progress.is_finite()
                && p.end_progress - progress >= 2.
                && p.target.is_finite()
                && !p.ids.is_empty()
                && p.ids.len() <= 9
                && p.ids.windows(2).all(|pair| pair[0] < pair[1])
        }) else {
            self.reset();
            return false;
        };
        let stamp = valid.expect("validated acquisition");
        let same = self.support.as_ref().is_some_and(|s| {
            stamp - s.last <= 0.35 + 1e-9
                && position.distance(s.position) <= 0.2
                && (progress - s.progress).abs() <= 0.2
                && proposal.endpoint.distance(s.endpoint) <= 1.
                && (proposal.target - s.target).abs() <= 0.01
                && proposal.ids == s.ids
        });
        if same {
            if fresh {
                let support = self.support.as_mut().expect("matching support");
                support.last = stamp;
                support.count = support.count.saturating_add(1).min(3);
            }
        } else if fresh {
            self.support = Some(Support {
                first: stamp,
                last: stamp,
                count: 1,
                position,
                progress,
                endpoint: proposal.endpoint,
                target: proposal.target,
                ids: proposal.ids.clone(),
            });
        } else {
            self.reset();
            return false;
        }
        let support = self.support.as_ref().expect("supported proposal");
        let adopted = fresh && support.count >= 3 && support.last - support.first >= 0.2 - 1e-9;
        self.diagnostics = CheckedApproachDiagnostics {
            supporting_acquisitions: support.count,
            first_acquisition_s: Some(support.first),
            last_acquisition_s: Some(support.last),
            proposal_end_progress_m: Some(proposal.end_progress),
            proposal_endpoint: Some(proposal.endpoint),
            proposal_lateral_target_m: Some(proposal.target),
            blocking_prediction_ids: proposal.ids,
            adopted,
        };
        if adopted {
            self.support = None;
        }
        adopted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn proposal() -> Proposal {
        Proposal {
            endpoint: Vec2::new(8., 3.),
            end_progress: 8.,
            target: 3.5,
            ids: vec![7, 9],
        }
    }
    fn sample(state: &mut Confirmation, now: f64, scene: Option<f64>, p: Option<Proposal>) -> bool {
        state.observe(Some((now, scene)), Vec2::default(), 0., p)
    }

    #[test]
    fn three_new_acquisitions_and_two_tenths_are_both_required() {
        let mut s = Confirmation::default();
        assert!(!sample(&mut s, 0., Some(0.), Some(proposal())));
        for i in 1..20 {
            assert!(!sample(
                &mut s,
                i as f64 * 0.005,
                Some(0.),
                Some(proposal())
            ));
        }
        assert_eq!(s.diagnostics.supporting_acquisitions, 1);
        assert!(!sample(&mut s, 0.1, Some(0.1), Some(proposal())));
        assert!(!sample(&mut s, 0.15, Some(0.15), Some(proposal())));
        assert_eq!(s.diagnostics.supporting_acquisitions, 3);
        assert!(!sample(&mut s, 0.2, Some(0.15), Some(proposal())));
        assert!(sample(&mut s, 0.21, Some(0.21), Some(proposal())));
        assert_eq!(s.diagnostics.first_acquisition_s, Some(0.));
        assert!(!sample(&mut s, 0.22, Some(0.21), Some(proposal())));
        assert_eq!(s.diagnostics.supporting_acquisitions, 0);
    }

    #[test]
    fn faults_consume_the_control_clock_and_cached_scenes_cannot_seed() {
        let mut s = Confirmation::default();
        sample(&mut s, 0., Some(0.), Some(proposal()));
        sample(&mut s, 0.1, Some(0.1), Some(proposal()));
        assert!(!sample(&mut s, 0.15, None, Some(proposal())));
        assert!(!sample(&mut s, 0.2, Some(0.1), Some(proposal())));
        assert_eq!(s.diagnostics.supporting_acquisitions, 0);
        assert!(!sample(&mut s, 0.3, Some(0.3), Some(proposal())));
        assert_eq!(s.diagnostics.first_acquisition_s, Some(0.3));
        assert!(!sample(&mut s, 0.8, Some(0.8), Some(proposal())));
        assert_eq!(s.diagnostics.supporting_acquisitions, 1);
    }

    #[test]
    fn target_endpoint_and_blocker_changes_cannot_inherit_credit() {
        for which in 0..4 {
            let mut s = Confirmation::default();
            sample(&mut s, 0., Some(0.), Some(proposal()));
            sample(&mut s, 0.1, Some(0.1), Some(proposal()));
            let mut p = proposal();
            match which {
                0 => p.target = -3.5,
                1 => p.endpoint.x += 1.1,
                2 => p.ids = vec![9],
                _ => p.ids = vec![9, 7],
            }
            assert!(!sample(&mut s, 0.2, Some(0.2), Some(p)));
            assert!(s.diagnostics.supporting_acquisitions <= 1);
        }
    }

    #[test]
    fn chained_small_ego_and_endpoint_moves_remain_anchored() {
        let mut s = Confirmation::default();
        sample(&mut s, 0., Some(0.), Some(proposal()));
        for i in 1..=3 {
            let x = i as f64 * 0.08;
            assert!(!s.observe(
                Some((i as f64 * 0.05, Some(i as f64 * 0.05))),
                Vec2::new(x, 0.),
                x,
                Some(proposal())
            ));
        }
        assert_eq!(s.diagnostics.first_acquisition_s, Some(3. * 0.05));
        let mut s = Confirmation::default();
        sample(&mut s, 0., Some(0.), Some(proposal()));
        for i in 1..=3 {
            let mut p = proposal();
            p.endpoint.y += i as f64 * 0.4;
            assert!(!sample(
                &mut s,
                i as f64 * 0.05,
                Some(i as f64 * 0.05),
                Some(p)
            ));
        }
        assert_eq!(s.diagnostics.first_acquisition_s, Some(3. * 0.05));
    }

    #[test]
    fn invalid_or_small_proposals_are_atomic_non_authorizations() {
        for which in 0..7 {
            let mut s = Confirmation::default();
            sample(&mut s, 0., Some(0.), Some(proposal()));
            let mut p = proposal();
            match which {
                0 => p.end_progress = 1.99,
                1 => p.end_progress = f64::NAN,
                2 => p.endpoint.x = f64::INFINITY,
                3 => p.target = f64::NAN,
                4 => p.ids.clear(),
                5 => p.ids = vec![1; 10],
                _ => p.ids = vec![1, 1],
            }
            assert!(!sample(&mut s, 0.1, Some(0.1), Some(p)));
            assert_eq!(s.diagnostics, CheckedApproachDiagnostics::default());
            assert!(!sample(&mut s, 0.15, Some(0.1), Some(proposal())));
            assert_eq!(s.diagnostics.supporting_acquisitions, 0);
        }
    }

    #[test]
    fn invalid_future_old_and_regressed_control_clocks_withdraw_support() {
        for (now, scene) in [
            (f64::NAN, Some(0.)),
            (-1., Some(0.)),
            (0.05, Some(0.05)),
            (0.2, Some(0.3)),
            (0.8, Some(0.1)),
            (0.2, Some(f64::INFINITY)),
            (0.2, Some(-1.)),
        ] {
            let mut s = Confirmation::default();
            sample(&mut s, 0.1, Some(0.1), Some(proposal()));
            assert!(!sample(&mut s, now, scene, Some(proposal())));
            assert_eq!(s.diagnostics.supporting_acquisitions, 0);
        }
    }

    #[test]
    fn reset_preserves_consumption_and_missing_proposals_withdraw_cached_support() {
        let mut s = Confirmation::default();
        sample(&mut s, 0., Some(0.), Some(proposal()));
        s.reset();
        sample(&mut s, 0.05, Some(0.), Some(proposal()));
        assert_eq!(s.diagnostics.supporting_acquisitions, 0);
        sample(&mut s, 0.1, Some(0.1), Some(proposal()));
        sample(&mut s, 0.15, Some(0.1), None);
        sample(&mut s, 0.2, Some(0.1), Some(proposal()));
        assert_eq!(s.diagnostics.supporting_acquisitions, 0);
    }
}
