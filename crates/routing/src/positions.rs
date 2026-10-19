//! Directed partial-edge requests; no implicit snapping or reversed geometry.
use crate::{ArrivalQueueEntry, ArrivalState, RoadNetwork, RoutePlan, SpeedRoutePlan};
use rustdriving_core::{Route, RouteSpeedZone};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};

/// Arc distance along the canonical directed edge, in world ENU meters.
/// Strict interior only: use a node ID for a junction without arrival context.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EdgePosition {
    pub edge: String,
    pub s_m: f64,
}

/// Existing JSON node strings remain byte-compatible. An edge position is an
/// explicit direction commitment, rather than a nearest-road guess.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(untagged)]
pub enum RouteEndpoint {
    Node(String),
    Edge(EdgePosition),
}
impl From<&str> for RouteEndpoint {
    fn from(value: &str) -> Self {
        Self::Node(value.into())
    }
}
impl From<String> for RouteEndpoint {
    fn from(value: String) -> Self {
        Self::Node(value)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EdgeTraversal {
    pub edge: String,
    pub start_s_m: f64,
    pub end_s_m: f64,
}

impl RoadNetwork {
    fn endpoint(&self, endpoint: &RouteEndpoint) -> Result<(), String> {
        match endpoint {
            RouteEndpoint::Node(id) if self.nodes.contains_key(id) => Ok(()),
            RouteEndpoint::Node(_) => Err("unknown start or goal node".into()),
            RouteEndpoint::Edge(position) => {
                let (_, length) = self
                    .edges
                    .get(&position.edge)
                    .ok_or("unknown endpoint edge")?;
                if !position.s_m.is_finite() || position.s_m <= 0. || position.s_m >= *length {
                    return Err(
                        "edge position must be finite and strictly inside its directed edge".into(),
                    );
                }
                Ok(())
            }
        }
    }

    fn turn_allowed(&self, incoming: Option<&String>, outgoing: &str) -> bool {
        incoming
            .and_then(|id| self.turn_rules.get(id))
            .is_none_or(|rule| {
                !rule.prohibited.contains(outgoing)
                    && rule.only.as_ref().is_none_or(|only| only == outgoing)
            })
    }

    fn full_traversal(&self, id: &str) -> EdgeTraversal {
        EdgeTraversal {
            edge: id.into(),
            start_s_m: 0.,
            end_s_m: self.edges[id].1,
        }
    }

    /// Shortest legal route, preserving a partial origin's incoming-edge state
    /// and checking the turn into a partial destination. No closure fallback.
    pub fn shortest_route_between(
        &self,
        start: &RouteEndpoint,
        goal: &RouteEndpoint,
        closed_edges: &[String],
    ) -> Result<RoutePlan, String> {
        if let (RouteEndpoint::Node(start), RouteEndpoint::Node(goal)) = (start, goal) {
            return self.shortest_route(start, goal, closed_edges);
        }
        self.endpoint(start)?;
        self.endpoint(goal)?;
        let closed: BTreeSet<_> = closed_edges.iter().collect();
        if closed.iter().any(|id| !self.edges.contains_key(*id)) {
            return Err("closure references an unknown edge".into());
        }
        for endpoint in [start, goal] {
            if let RouteEndpoint::Edge(position) = endpoint
                && closed.contains(&position.edge)
            {
                return Err("endpoint lies on a closed directed edge".into());
            }
        }
        if let (RouteEndpoint::Edge(a), RouteEndpoint::Edge(b)) = (start, goal)
            && a.edge == b.edge
            && b.s_m > a.s_m
        {
            return self.compile_traversals(
                start,
                vec![EdgeTraversal {
                    edge: a.edge.clone(),
                    start_s_m: a.s_m,
                    end_s_m: b.s_m,
                }],
            );
        }
        if start == goal {
            return Err("start and goal must differ for a driving route".into());
        }
        let (initial, offset) = match start {
            RouteEndpoint::Node(id) => ((id.clone(), None), 0.),
            RouteEndpoint::Edge(p) => {
                let (edge, length) = &self.edges[&p.edge];
                ((edge.to.clone(), Some(p.edge.clone())), length - p.s_m)
            }
        };
        // At most one state per incoming edge, plus a node-origin state.
        let mut distances = BTreeMap::from([(initial.clone(), offset)]);
        let mut previous: BTreeMap<ArrivalState, ArrivalState> = BTreeMap::new();
        let mut queue = BinaryHeap::from([ArrivalQueueEntry {
            distance: offset,
            state: initial.clone(),
        }]);
        let mut reached = None;
        while let Some(ArrivalQueueEntry { distance, state }) = queue.pop() {
            if distance > distances[&state] {
                continue;
            }
            let terminal = match goal {
                RouteEndpoint::Node(id) => &state.0 == id,
                RouteEndpoint::Edge(p) => {
                    self.edges[&p.edge].0.from == state.0
                        && self.turn_allowed(state.1.as_ref(), &p.edge)
                }
            };
            // The extra destination prefix has the same fixed cost in every
            // terminal state, so the first terminal popped is optimal.
            if terminal {
                reached = Some(state);
                break;
            }
            for id in self.outgoing.get(&state.0).into_iter().flatten() {
                if closed.contains(id) || !self.turn_allowed(state.1.as_ref(), id) {
                    continue;
                }
                let (edge, length) = &self.edges[id];
                let candidate = distance + length;
                if !candidate.is_finite() {
                    return Err("route distance overflow".into());
                }
                let next = (edge.to.clone(), Some(id.clone()));
                if candidate < *distances.get(&next).unwrap_or(&f64::INFINITY) {
                    distances.insert(next.clone(), candidate);
                    previous.insert(next.clone(), state.clone());
                    queue.push(ArrivalQueueEntry {
                        distance: candidate,
                        state: next,
                    });
                }
            }
        }
        let mut state =
            reached.ok_or("goal is unreachable with these closures and turn restrictions")?;
        let mut traversals = vec![];
        while state != initial {
            traversals.push(self.full_traversal(state.1.as_ref().ok_or("missing arrival edge")?));
            state = previous
                .get(&state)
                .ok_or("missing route predecessor")?
                .clone();
        }
        if let RouteEndpoint::Edge(p) = start {
            traversals.push(EdgeTraversal {
                start_s_m: p.s_m,
                ..self.full_traversal(&p.edge)
            });
        }
        traversals.reverse();
        if let RouteEndpoint::Edge(p) = goal {
            traversals.push(EdgeTraversal {
                end_s_m: p.s_m,
                ..self.full_traversal(&p.edge)
            });
        }
        self.compile_traversals(start, traversals)
    }

    fn compile_traversals(
        &self,
        start: &RouteEndpoint,
        traversals: Vec<EdgeTraversal>,
    ) -> Result<RoutePlan, String> {
        let mut points = vec![];
        let mut node_ids = match start {
            RouteEndpoint::Node(id) => vec![id.clone()],
            _ => vec![],
        };
        let mut half_width = f64::INFINITY;
        for span in &traversals {
            let (edge, length) = &self.edges[&span.edge];
            let geometry = Route::new(edge.points.clone(), edge.half_width)?;
            let first = geometry.sample(span.start_s_m, 0.).0;
            if points.is_empty() {
                points.push(first);
            }
            points.extend(
                geometry
                    .points
                    .iter()
                    .zip(&geometry.lengths)
                    .filter(|(_, s)| **s > span.start_s_m && **s < span.end_s_m)
                    .map(|(p, _)| *p),
            );
            points.push(geometry.sample(span.end_s_m, 0.).0);
            if span.end_s_m == *length {
                node_ids.push(edge.to.clone());
            }
            half_width = half_width.min(edge.half_width);
        }
        let route = Route::new(points, half_width)?;
        if !route.length().is_finite() {
            return Err("route distance overflow".into());
        }
        Ok(RoutePlan {
            node_ids,
            edge_ids: traversals.iter().map(|s| s.edge.clone()).collect(),
            distance_m: route.length(),
            route,
            traversals,
        })
    }

    /// Compile only the traversed portion of each map speed cap. Adjacent equal
    /// caps merge and the original 128-interval / 1500 m policy budget applies.
    pub fn shortest_route_between_with_speed_zones(
        &self,
        start: &RouteEndpoint,
        goal: &RouteEndpoint,
        closed_edges: &[String],
    ) -> Result<SpeedRoutePlan, String> {
        if let (RouteEndpoint::Node(start), RouteEndpoint::Node(goal)) = (start, goal) {
            return self.shortest_route_with_speed_zones(start, goal, closed_edges);
        }
        let plan = self.shortest_route_between(start, goal, closed_edges)?;
        let mut distance = 0.;
        let mut speed_zones: Vec<RouteSpeedZone> = vec![];
        for span in &plan.traversals {
            let start = distance;
            distance += span.end_s_m - span.start_s_m;
            if let Some(speed) = self.edges[&span.edge].0.speed_limit_m_s {
                let zone = RouteSpeedZone {
                    start_s_m: start,
                    end_s_m: distance,
                    speed_m_s: speed.min(8.),
                };
                if let Some(last) = speed_zones
                    .last_mut()
                    .filter(|last| last.end_s_m == start && last.speed_m_s == zone.speed_m_s)
                {
                    last.end_s_m = distance;
                } else {
                    speed_zones.push(zone);
                }
            }
        }
        if speed_zones.len() > 128 || (!speed_zones.is_empty() && plan.route.length() > 1500.) {
            return Err("mapped speeds require at most 128 intervals over at most 1500 m".into());
        }
        Ok(SpeedRoutePlan { plan, speed_zones })
    }
}

#[cfg(test)]
mod tests;
