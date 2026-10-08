//! Directed map routing in world ENU meters, independent of sensing and simulation.
use rustdrive_core::{Route, Vec2};
use serde::{Deserialize, Serialize};
use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet, BinaryHeap},
};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadNode {
    pub id: String,
    pub position: Vec2,
}

/// A directed, traversable centerline. Cost is its geometric length in meters.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadEdge {
    pub id: String,
    pub from: String,
    pub to: String,
    pub points: Vec<Vec2>,
    pub half_width: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadNetworkSpec {
    pub nodes: Vec<RoadNode>,
    pub edges: Vec<RoadEdge>,
}

/// Selected topology and its local-planner route; width is the narrowest edge.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoutePlan {
    pub node_ids: Vec<String>,
    pub edge_ids: Vec<String>,
    pub distance_m: f64,
    pub route: Route,
}

/// Validated immutable map. No simulator state or obstacle labels enter routing.
#[derive(Clone, Debug)]
pub struct RoadNetwork {
    nodes: BTreeMap<String, Vec2>,
    edges: BTreeMap<String, (RoadEdge, f64)>,
    outgoing: BTreeMap<String, Vec<String>>,
}

#[derive(Debug)]
struct QueueEntry {
    distance: f64,
    node: String,
}
impl PartialEq for QueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.distance == other.distance && self.node == other.node
    }
}
impl Eq for QueueEntry {}
impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance
            .total_cmp(&self.distance)
            .then_with(|| other.node.cmp(&self.node))
    }
}

impl RoadNetwork {
    pub fn new(spec: RoadNetworkSpec) -> Result<Self, String> {
        let mut nodes = BTreeMap::new();
        for node in spec.nodes {
            if node.id.is_empty()
                || !node.position.finite()
                || nodes.insert(node.id, node.position).is_some()
            {
                return Err("empty/duplicate node ID or non-finite node position".into());
            }
        }
        if nodes.is_empty() {
            return Err("road network has no nodes".into());
        }
        let mut edges = BTreeMap::new();
        for edge in spec.edges {
            let from = nodes
                .get(&edge.from)
                .ok_or("edge references unknown origin")?;
            let to = nodes
                .get(&edge.to)
                .ok_or("edge references unknown destination")?;
            let geometry = Route::new(edge.points.clone(), edge.half_width)?;
            if edge.id.is_empty()
                || edge.from == edge.to
                || geometry.points[0].distance(*from) > 1e-6
                || geometry.points.last().unwrap().distance(*to) > 1e-6
                || !geometry.length().is_finite()
            {
                return Err("invalid edge ID, endpoints or length".into());
            }
            // Canonicalize tolerated endpoint roundoff, so connected edges join exactly.
            let mut edge = edge;
            edge.points[0] = *from;
            *edge.points.last_mut().unwrap() = *to;
            let length = Route::new(edge.points.clone(), edge.half_width)?.length();
            if !length.is_finite() || edges.insert(edge.id.clone(), (edge, length)).is_some() {
                return Err("duplicate edge ID or non-finite length".into());
            }
        }
        let mut outgoing: BTreeMap<String, Vec<String>> = BTreeMap::new();
        // Sorted IDs make equal-cost choices independent of input order.
        for (id, (edge, _)) in &edges {
            outgoing
                .entry(edge.from.clone())
                .or_default()
                .push(id.clone());
        }
        Ok(Self {
            nodes,
            edges,
            outgoing,
        })
    }

    /// Recompute a shortest route with externally supplied directed edge closures.
    /// An unreachable goal is an error, never a fallback through a closed edge.
    pub fn shortest_route(
        &self,
        start: &str,
        goal: &str,
        closed_edges: &[String],
    ) -> Result<RoutePlan, String> {
        if !self.nodes.contains_key(start) || !self.nodes.contains_key(goal) {
            return Err("unknown start or goal node".into());
        }
        if start == goal {
            return Err("start and goal must differ for a driving route".into());
        }
        let closed: BTreeSet<_> = closed_edges.iter().collect();
        if closed.iter().any(|id| !self.edges.contains_key(*id)) {
            return Err("closure references an unknown edge".into());
        }
        let mut distances = BTreeMap::from([(start.to_owned(), 0.0)]);
        let mut previous: BTreeMap<String, String> = BTreeMap::new();
        let mut queue = BinaryHeap::from([QueueEntry {
            distance: 0.0,
            node: start.to_owned(),
        }]);
        while let Some(QueueEntry { distance, node }) = queue.pop() {
            if distance > distances[&node] {
                continue;
            }
            if node == goal {
                break;
            }
            for id in self.outgoing.get(&node).into_iter().flatten() {
                if closed.contains(id) {
                    continue;
                }
                let (edge, length) = &self.edges[id];
                let candidate = distance + length;
                if !candidate.is_finite() {
                    return Err("route distance overflow".into());
                }
                if candidate < *distances.get(&edge.to).unwrap_or(&f64::INFINITY) {
                    distances.insert(edge.to.clone(), candidate);
                    previous.insert(edge.to.clone(), id.clone());
                    queue.push(QueueEntry {
                        distance: candidate,
                        node: edge.to.clone(),
                    });
                }
            }
        }
        let distance_m = *distances
            .get(goal)
            .ok_or("goal is unreachable with these closures")?;
        let mut edge_ids = Vec::new();
        let mut node = goal;
        while node != start {
            let id = previous.get(node).ok_or("missing route predecessor")?;
            edge_ids.push(id.clone());
            node = &self.edges[id].0.from;
        }
        edge_ids.reverse();
        let mut node_ids = vec![start.to_owned()];
        let mut points = Vec::new();
        let mut half_width = f64::INFINITY;
        for id in &edge_ids {
            let edge = &self.edges[id].0;
            node_ids.push(edge.to.clone());
            points.extend(
                edge.points
                    .iter()
                    .skip(usize::from(!points.is_empty()))
                    .copied(),
            );
            half_width = half_width.min(edge.half_width);
        }
        let route = Route::new(points, half_width)?;
        Ok(RoutePlan {
            node_ids,
            edge_ids,
            distance_m,
            route,
        })
    }
}

#[cfg(test)]
mod tests;
