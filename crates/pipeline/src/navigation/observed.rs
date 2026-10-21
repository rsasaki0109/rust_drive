//! Mission-local route preferences from acquired, persistent lattice obstructions.
//! They do not assert a physical/legal road closure or authorize motion.
use super::{NavigationPhase, NavigationUpdate};
use rustdriving_core::{CandidateDisposition, EgoState, PlanningDiagnostics, Route, Track, Vec2};
use rustdriving_routing::{RoadNetworkSpec, RoutePlan};
use serde::{Deserialize, Serialize};

pub(super) const MAX_RETAINED_EDGES: usize = 8;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservedEdgeEvidence {
    pub edge: String,
    pub prediction_id: u64,
    pub first_acquisition_s: f64,
    pub last_acquisition_s: f64,
    pub acquisition_count: u64,
    pub anchor_position: Vec2,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservedEdgeAvoidance {
    pub evidence: ObservedEdgeEvidence,
    pub requested_at_control_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservedAvoidanceDecision {
    DetourRequested,
    NoAlternative,
    UnreachableDivergence,
    CapacityReached,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservedObstructionStatus {
    pub retained_edges: Vec<ObservedEdgeAvoidance>,
    pub candidate: Option<ObservedEdgeEvidence>,
    pub last_decision: Option<ObservedAvoidanceDecision>,
    pub last_decision_control_s: Option<f64>,
}

#[derive(Clone, Default)]
pub(super) struct ObservedObstructions {
    pub report: ObservedObstructionStatus,
    last_acquisition: Option<f64>,
}

pub(super) struct Observation<'a> {
    pub now: f64,
    pub acquisition: Option<f64>,
    pub healthy: bool,
    pub phase: NavigationPhase,
    pub newer_external_update: bool,
    pub ego: EgoState,
    pub vehicle_radius: f64,
    pub tracks: &'a [Track],
    pub decisions: Option<&'a PlanningDiagnostics>,
    pub active: &'a RoutePlan,
    pub network: &'a RoadNetworkSpec,
}

pub(crate) struct ObservedObstructionInput<'a> {
    pub now: f64,
    pub acquisition: Option<f64>,
    pub healthy: bool,
    pub ego: EgoState,
    pub tracks: &'a [Track],
    pub decisions: Option<&'a PlanningDiagnostics>,
    pub update: Option<&'a NavigationUpdate>,
    pub vehicle_radius: f64,
    pub deceleration: f64,
}

impl ObservedObstructions {
    pub fn reset_probe(&mut self, now: f64) {
        self.report.candidate = None;
        self.last_acquisition = Some(self.last_acquisition.map_or(now, |t| t.max(now)));
    }

    pub fn decision(&mut self, now: f64, decision: ObservedAvoidanceDecision) {
        self.report.last_decision = Some(decision);
        self.report.last_decision_control_s = Some(now);
    }

    pub fn observe(&mut self, input: Observation<'_>) -> Option<ObservedEdgeEvidence> {
        let acquisition = input.acquisition.filter(|t| {
            t.is_finite()
                && *t >= 0.
                && *t <= input.now + 1e-9
                && input.now - *t <= 0.35 + 1e-9
                && self.last_acquisition.is_none_or(|old| *t >= old)
        });
        let fresh = acquisition.is_some_and(|t| self.last_acquisition.is_none_or(|old| t > old));
        if fresh {
            self.last_acquisition = acquisition;
        }
        let witness = acquisition.and_then(|stamp| {
            if !input.healthy
                || input.phase != NavigationPhase::Following
                || input.newer_external_update
            {
                return None;
            }
            let d = input.decisions?;
            let scene = d.scene_acquisition_s?;
            if !scene.is_finite()
                || scene > stamp
                || input.now - scene > 0.15 + 1e-9
                || d.candidates.is_empty()
                || d.candidates.len() > 9
                || d.candidates
                    .iter()
                    .any(|c| c.disposition == CandidateDisposition::CompletePath)
            {
                return None;
            }
            let id = d.candidates[0]
                .nominal_forecast_conflict
                .as_ref()?
                .prediction_id;
            if !d.candidates.iter().all(|c| {
                c.nominal_forecast_conflict
                    .as_ref()
                    .is_some_and(|f| f.prediction_id == id)
            }) {
                return None;
            }
            let track = input.tracks.iter().find(|t| {
                t.id == id
                    && t.last_seen == stamp
                    && t.position.finite()
                    && t.velocity.finite()
                    && t.velocity.x.hypot(t.velocity.y) <= 0.2
                    && t.radius.is_finite()
                    && t.radius >= 0.4
            })?;
            let progress = input.active.route.project(track.position).0;
            if progress
                <= input.active.route.project(input.ego.pose.position).0 + input.vehicle_radius + 2.
            {
                return None;
            }
            let mut matches = Vec::new();
            for (i, id) in input.active.edge_ids.iter().enumerate() {
                if self
                    .report
                    .retained_edges
                    .iter()
                    .any(|e| &e.evidence.edge == id)
                {
                    continue;
                }
                let edge = input.network.edges.iter().find(|e| &e.id == id)?;
                let route = Route::new(edge.points.clone(), edge.half_width).ok()?;
                let s = route.project(track.position).0;
                let start = input
                    .active
                    .traversals
                    .get(i)
                    .map_or(0., |span| span.start_s_m);
                let end = input
                    .active
                    .traversals
                    .get(i)
                    .map_or(route.length(), |span| span.end_s_m);
                if s > start + track.radius + 1.
                    && s < end - track.radius - 1.
                    && route.sample(s, 0.).0.distance(track.position) + track.radius
                        <= edge.half_width
                {
                    matches.push(id.clone());
                }
            }
            (matches.len() == 1).then(|| (matches.remove(0), id, track.position))
        });
        match (acquisition, witness) {
            (Some(stamp), Some((edge, id, position))) => {
                if let Some(support) = self.report.candidate.as_mut().filter(|s| {
                    s.edge == edge
                        && s.prediction_id == id
                        && stamp - s.last_acquisition_s <= 0.35 + 1e-9
                        && position.distance(s.anchor_position) <= 0.15
                }) {
                    if fresh {
                        support.last_acquisition_s = stamp;
                        support.acquisition_count = support.acquisition_count.saturating_add(1);
                    }
                } else if fresh {
                    self.report.candidate = Some(ObservedEdgeEvidence {
                        edge,
                        prediction_id: id,
                        first_acquisition_s: stamp,
                        last_acquisition_s: stamp,
                        acquisition_count: 1,
                        anchor_position: position,
                    });
                } else {
                    self.report.candidate = None;
                }
            }
            _ => self.report.candidate = None,
        }
        self.report
            .candidate
            .as_ref()
            .filter(|s| {
                fresh
                    && s.acquisition_count >= 3
                    && s.last_acquisition_s - s.first_acquisition_s >= 0.2 - 1e-9
            })
            .cloned()
    }
}
