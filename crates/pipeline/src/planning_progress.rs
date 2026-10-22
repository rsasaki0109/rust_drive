//! Sensor-clock evidence that the current lattice keeps a stationary checked stop.
//! This observer never changes planning, control, stop ownership or health.
use rustdriving_core::{CandidateDisposition, DrivingMode, PlanningDiagnostics, Trajectory, Vec2};
use serde::{Deserialize, Serialize};

const DWELL_S: f64 = 2.0;
const MAX_GAP_S: f64 = 0.35;
const POSITION_TOLERANCE_M: f64 = 0.2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlanningProgressState {
    Observing,
    /// Every enumerated complete path is unavailable and a checked stop remains selected.
    /// This is evidence about this lattice, not a proof that no safe route exists.
    StationaryForecastBlocked,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanningProgress {
    pub state: PlanningProgressState,
    pub first_supporting_acquisition_s: Option<f64>,
    pub last_supporting_acquisition_s: Option<f64>,
    pub supporting_acquisitions: u64,
    /// Elapsed acquired evidence, never time since the last cached control tick.
    pub supported_duration_s: f64,
    /// Sorted unique identities responsible for nominal path conflicts (at most nine).
    pub blocking_prediction_ids: Vec<u64>,
}

struct Support {
    first: f64,
    last: f64,
    count: u64,
    position: Vec2,
    progress: f64,
    ids: Vec<u64>,
}

#[derive(Default)]
pub(super) struct ProgressMonitor {
    // Retain this watermark through resets: a cached scene cannot start new support.
    last_acquisition: Option<f64>,
    support: Option<Support>,
}

pub(super) struct Observation<'a> {
    pub now: f64,
    pub acquisition: Option<f64>,
    pub position: Vec2,
    pub speed: f64,
    pub route_remaining: f64,
    pub healthy_unrestricted: bool,
    pub braking: bool,
    pub trajectory: &'a Trajectory,
    pub decisions: Option<&'a PlanningDiagnostics>,
}

impl ProgressMonitor {
    pub fn reset(&mut self) {
        self.support = None;
    }

    pub fn observe(&mut self, input: Observation<'_>) -> PlanningProgress {
        let scene = input.acquisition;
        let valid_scene = scene.filter(|t| {
            t.is_finite()
                && *t >= 0.0
                && input.now.is_finite()
                && *t <= input.now + 1e-9
                && input.now - *t <= MAX_GAP_S + 1e-9
                && self.last_acquisition.is_none_or(|old| *t >= old)
        });
        let fresh = valid_scene.is_some_and(|t| self.last_acquisition.is_none_or(|old| t > old));
        if fresh {
            self.last_acquisition = valid_scene;
        }
        let witness = input.decisions.and_then(|d| {
            let chosen = d
                .selected_candidate
                .and_then(|index| d.candidates.get(index as usize))?;
            let progress = d.route_progress_m.filter(|s| s.is_finite())?;
            if d.failure.is_some()
                || d.scene_acquisition_s != valid_scene
                || d.candidates.is_empty()
                || d.candidates.len() > 9
                || chosen.disposition != CandidateDisposition::CheckedStop
                || d.candidates
                    .iter()
                    .any(|c| c.disposition == CandidateDisposition::CompletePath)
            {
                return None;
            }
            let mut ids: Vec<_> = d
                .candidates
                .iter()
                .filter_map(|c| {
                    c.nominal_forecast_conflict
                        .as_ref()
                        .map(|f| f.prediction_id)
                })
                .collect();
            ids.sort_unstable();
            ids.dedup();
            (!ids.is_empty()).then_some((progress, ids))
        });
        let stopped = input.position.finite()
            && input.speed.is_finite()
            && input.speed.abs() <= 0.2
            && input.route_remaining.is_finite()
            && input.route_remaining > 2.0
            && input.trajectory.mode == DrivingMode::Yield
            && input.trajectory.points.len() >= 2
            && input.trajectory.points.iter().all(|p| {
                p.speed.is_finite()
                    && (0.0..=0.2).contains(&p.speed)
                    && p.position.distance(input.position) <= POSITION_TOLERANCE_M
            });
        match (valid_scene, witness) {
            (Some(stamp), Some((progress, ids)))
                if input.healthy_unrestricted && input.braking && stopped =>
            {
                if let Some(support) = self.support.as_mut().filter(|s| {
                    stamp - s.last <= MAX_GAP_S + 1e-9
                        && input.position.distance(s.position) <= POSITION_TOLERANCE_M
                        && (progress - s.progress).abs() <= POSITION_TOLERANCE_M
                        && ids == s.ids
                }) {
                    if fresh {
                        support.last = stamp;
                        support.count = support.count.saturating_add(1);
                    }
                } else if fresh {
                    self.support = Some(Support {
                        first: stamp,
                        last: stamp,
                        count: 1,
                        position: input.position,
                        progress,
                        ids,
                    });
                } else {
                    self.reset();
                }
            }
            _ => self.reset(),
        }
        let support = self.support.as_ref();
        let duration = support.map_or(0.0, |s| s.last - s.first);
        PlanningProgress {
            state: if support.is_some_and(|s| s.count >= 3 && duration >= DWELL_S - 1e-9) {
                PlanningProgressState::StationaryForecastBlocked
            } else {
                PlanningProgressState::Observing
            },
            first_supporting_acquisition_s: support.map(|s| s.first),
            last_supporting_acquisition_s: support.map(|s| s.last),
            supporting_acquisitions: support.map_or(0, |s| s.count),
            supported_duration_s: duration,
            blocking_prediction_ids: support.map_or_else(Vec::new, |s| s.ids.clone()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::{
        PlanningCandidateDiagnostic, PlanningForecastConflict, TrajectoryPoint,
    };

    fn decisions(stamp: f64) -> PlanningDiagnostics {
        PlanningDiagnostics {
            scene_acquisition_s: Some(stamp),
            route_progress_m: Some(0.),
            selected_candidate: Some(0),
            candidates: [11, 7, 11]
                .into_iter()
                .enumerate()
                .map(|(i, id)| PlanningCandidateDiagnostic {
                    index: i as u8,
                    lateral_target_m: 0.,
                    disposition: CandidateDisposition::CheckedStop,
                    nominal_forecast_conflict: Some(PlanningForecastConflict {
                        prediction_id: id,
                        relative_time_s: 2.,
                        forecast_radius_m: 2.,
                        forecast_horizon_s: 8.,
                        minimum_clearance_floor_m: 0.,
                    }),
                    profile: None,
                })
                .collect(),
            ..PlanningDiagnostics::default()
        }
    }
    fn hold() -> Trajectory {
        Trajectory {
            mode: DrivingMode::Yield,
            lateral_target: 0.,
            points: [0., 8.]
                .into_iter()
                .map(|time| TrajectoryPoint {
                    time,
                    position: Vec2::default(),
                    speed: 0.,
                })
                .collect(),
        }
    }
    fn input<'a>(now: f64, path: &'a Trajectory, d: &'a PlanningDiagnostics) -> Observation<'a> {
        Observation {
            now,
            acquisition: d.scene_acquisition_s,
            position: Vec2::default(),
            speed: 0.,
            route_remaining: 100.,
            healthy_unrestricted: true,
            braking: true,
            trajectory: path,
            decisions: Some(d),
        }
    }
    fn mature(m: &mut ProgressMonitor) -> PlanningProgress {
        let path = hold();
        let mut result = m.observe(input(0., &path, &decisions(0.)));
        for i in 1..=20 {
            let stamp = i as f64 * 0.1;
            result = m.observe(input(stamp, &path, &decisions(stamp)));
        }
        result
    }

    #[test]
    fn dwell_counts_actual_acquisitions_and_publishes_sorted_bounded_blockers() {
        let mut m = ProgressMonitor::default();
        let report = mature(&mut m);
        assert_eq!(
            report.state,
            PlanningProgressState::StationaryForecastBlocked
        );
        assert_eq!(report.supporting_acquisitions, 21);
        assert_eq!(report.supported_duration_s, 2.);
        assert_eq!(report.blocking_prediction_ids, vec![7, 11]);
        let cached = m.observe(input(2.05, &hold(), &decisions(2.)));
        assert_eq!(cached, report);
        let stale = m.observe(input(2.36, &hold(), &decisions(2.)));
        assert_eq!(stale.supporting_acquisitions, 0);
        assert_eq!(stale.state, PlanningProgressState::Observing);
    }

    #[test]
    fn oversampling_and_cache_reuse_cannot_replace_elapsed_evidence() {
        let mut m = ProgressMonitor::default();
        for i in 0..1000 {
            let stamp = i as f64 * 1e-5;
            let r = m.observe(input(stamp, &hold(), &decisions(stamp)));
            assert_eq!(r.state, PlanningProgressState::Observing);
        }
        for i in 1..=6 {
            let r = m.observe(input(
                0.00999 + i as f64 * 0.05,
                &hold(),
                &decisions(0.00999),
            ));
            assert_eq!(r.supporting_acquisitions, 1000);
            assert!(r.supported_duration_s < 0.01);
        }
    }

    #[test]
    fn gaps_regressions_future_stamps_and_resets_need_new_support() {
        let mut m = ProgressMonitor::default();
        mature(&mut m);
        let r = m.observe(input(2.4, &hold(), &decisions(2.4)));
        assert_eq!(r.supporting_acquisitions, 1);
        for (now, stamp) in [(2.45, 2.3), (2.5, 2.6)] {
            assert_eq!(
                m.observe(input(now, &hold(), &decisions(stamp)))
                    .supporting_acquisitions,
                0
            );
        }
        assert_eq!(
            m.observe(input(2.55, &hold(), &decisions(2.4)))
                .supporting_acquisitions,
            0
        );
        assert_eq!(
            m.observe(input(2.6, &hold(), &decisions(2.6)))
                .supporting_acquisitions,
            1
        );
        m.reset();
        assert_eq!(
            m.observe(input(2.65, &hold(), &decisions(2.6)))
                .supporting_acquisitions,
            0
        );
    }

    #[test]
    fn faults_regulatory_stops_goals_acceleration_and_departure_clear_support() {
        for variant in 0..7 {
            let mut m = ProgressMonitor::default();
            mature(&mut m);
            let path = hold();
            let d = decisions(2.1);
            let mut next = input(2.1, &path, &d);
            match variant {
                0 => next.healthy_unrestricted = false,
                1 => next.route_remaining = 2.,
                2 => next.braking = false,
                3 => next.speed = 0.21,
                4 => next.decisions = None,
                5 => next.position.x = f64::NAN,
                _ => next.route_remaining = f64::NAN,
            }
            assert_eq!(m.observe(next).supporting_acquisitions, 0);
            assert_eq!(m.observe(input(2.15, &path, &d)).supporting_acquisitions, 0);
        }
    }

    #[test]
    fn a_complete_candidate_or_different_blockers_withdraw_the_old_diagnosis() {
        let mut m = ProgressMonitor::default();
        mature(&mut m);
        let mut d = decisions(2.1);
        d.candidates[1]
            .nominal_forecast_conflict
            .as_mut()
            .unwrap()
            .prediction_id = 12;
        let r = m.observe(input(2.1, &hold(), &d));
        assert_eq!(r.supporting_acquisitions, 1);
        assert_eq!(r.blocking_prediction_ids, vec![11, 12]);
        d.scene_acquisition_s = Some(2.2);
        d.candidates[2].disposition = CandidateDisposition::CompletePath;
        assert_eq!(
            m.observe(input(2.2, &hold(), &d)).supporting_acquisitions,
            0
        );
    }

    #[test]
    fn position_and_route_progress_must_remain_near_the_original_anchor() {
        for position_change in [false, true] {
            let mut m = ProgressMonitor::default();
            mature(&mut m);
            let path = hold();
            let mut d = decisions(2.1);
            if !position_change {
                d.route_progress_m = Some(0.21);
            }
            let mut next = input(2.1, &path, &d);
            if position_change {
                next.position.x = 0.15;
                next.position.y = 0.15;
                let mut moved = hold();
                for p in &mut moved.points {
                    p.position = next.position;
                }
                next.trajectory = &moved;
                assert_eq!(m.observe(next).supporting_acquisitions, 1);
            } else {
                assert_eq!(m.observe(next).supporting_acquisitions, 1);
            }
        }
    }

    #[test]
    fn cached_pose_or_blocker_changes_withdraw_support_without_starting_a_new_prefix() {
        for changed_identity in [false, true] {
            let mut m = ProgressMonitor::default();
            mature(&mut m);
            let mut d = decisions(2.);
            let path = hold();
            if changed_identity {
                d.candidates[1]
                    .nominal_forecast_conflict
                    .as_mut()
                    .unwrap()
                    .prediction_id = 12;
            } else {
                d.route_progress_m = Some(0.21);
            }
            let report = m.observe(input(2.05, &path, &d));
            assert_eq!(report.state, PlanningProgressState::Observing);
            assert_eq!(report.supporting_acquisitions, 0);
        }
    }
}
