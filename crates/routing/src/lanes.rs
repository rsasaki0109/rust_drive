//! Authored directed lane sections in world ENU meters. Point order is travel direction.
//! Neighbor permissions describe map legality; they do not authorize a traffic maneuver.
use super::{QueueEntry, RoadEdge, RoadNetworkSpec, RoadNode};
use rustdriving_core::{Route, Vec2};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};

mod change;
pub use change::{LaneChangeManeuver, LaneChangeRequest, LaneChangeRoutePlan};
mod speeds;
pub use speeds::LaneSpeedRoutePlan;

pub const MAX_LANES: usize = 4096;
pub const MAX_LANE_POINTS: usize = 2048;
pub const MAX_MAP_POINTS: usize = 65_536;
pub const MAX_SUCCESSORS: usize = 32;
pub const MAX_MAP_CONNECTIONS: usize = 32_768;
const GEOMETRY_TOLERANCE_M: f64 = 1e-6;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneMapSpec {
    pub schema_version: u32,
    pub lanes: Vec<Lane>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lane {
    pub id: String,
    pub centerline: Vec<Vec2>,
    pub width_m: f64,
    pub speed_limit_m_s: f64,
    #[serde(default)]
    pub successors: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub left: Option<LaneNeighbor>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub right: Option<LaneNeighbor>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneNeighbor {
    pub lane_id: String,
    pub change_allowed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LaneSide {
    Left,
    Right,
}

#[derive(Clone, Debug)]
pub struct LaneNetwork {
    lanes: BTreeMap<String, (Lane, f64)>,
}

/// A resolved lane itinerary for existing route consumers. `following_route` traverses
/// whole lanes; `lane_change_route` replaces the first two sections with a connector.
#[derive(Clone, Debug, Serialize)]
pub struct LaneRoutePlan {
    pub lane_ids: Vec<String>,
    pub distance_m: f64,
    pub speed_limit_m_s: f64,
    pub route: Route,
    pub network: RoadNetworkSpec,
    pub start_node: String,
    pub goal_node: String,
}

impl LaneNetwork {
    pub fn new(spec: LaneMapSpec) -> Result<Self, String> {
        if spec.schema_version != 1 || spec.lanes.is_empty() || spec.lanes.len() > MAX_LANES {
            return Err("lane map requires schema 1 and 1..4096 lanes".into());
        }
        let mut lanes = BTreeMap::new();
        let mut point_count = 0;
        let mut connection_count = 0;
        for mut lane in spec.lanes {
            if lane.id.trim().is_empty()
                || lane.id.trim() != lane.id
                || lane.id.len() > 128
                || lane.id.chars().any(char::is_control)
                || !(2..=MAX_LANE_POINTS).contains(&lane.centerline.len())
                || !lane.width_m.is_finite()
                || !(0.5..=12.0).contains(&lane.width_m)
                || !lane.speed_limit_m_s.is_finite()
                || !(0.1..=20.0).contains(&lane.speed_limit_m_s)
                || lane.successors.len() > MAX_SUCCESSORS
                || lane
                    .centerline
                    .iter()
                    .any(|p| !p.finite() || p.x.abs() > 1_000_000.0 || p.y.abs() > 1_000_000.0)
            {
                return Err("invalid lane ID, geometry, width, speed or successor count".into());
            }
            point_count += lane.centerline.len();
            connection_count += lane.successors.len();
            if point_count > MAX_MAP_POINTS || connection_count > MAX_MAP_CONNECTIONS {
                return Err("lane map exceeds total geometry or connection budget".into());
            }
            let geometry = Route::new(lane.centerline.clone(), lane.width_m / 2.0)?;
            if !geometry.length().is_finite() || geometry.length() < 0.001 {
                return Err("lane section must have finite length of at least 1 mm".into());
            }
            lane.successors.sort();
            if lane.successors.windows(2).any(|ids| ids[0] == ids[1]) {
                return Err("duplicate lane successor".into());
            }
            if lanes
                .insert(lane.id.clone(), (lane, geometry.length()))
                .is_some()
            {
                return Err("duplicate lane ID".into());
            }
        }
        for (lane, _) in lanes.values() {
            for id in &lane.successors {
                let target = &lanes.get(id).ok_or("unknown lane successor")?.0;
                // Explicit shared endpoints avoid fabricated lateral/longitudinal connectors.
                if id == &lane.id || lane.centerline.last() != target.centerline.first() {
                    return Err(
                        "lane successors require distinct IDs and exactly shared endpoints".into(),
                    );
                }
            }
            for (neighbor, side) in [(&lane.left, LaneSide::Left), (&lane.right, LaneSide::Right)] {
                if let Some(neighbor) = neighbor {
                    let target = &lanes
                        .get(&neighbor.lane_id)
                        .ok_or("unknown neighboring lane")?
                        .0;
                    let reciprocal = match side {
                        LaneSide::Left => &target.right,
                        LaneSide::Right => &target.left,
                    };
                    if target.id == lane.id
                        || reciprocal
                            .as_ref()
                            .is_none_or(|other| other.lane_id != lane.id)
                    {
                        return Err(
                            "lane neighbors must be distinct and reciprocal on opposite sides"
                                .into(),
                        );
                    }
                    validate_neighbor(lane, target, side)?;
                }
            }
        }
        Ok(Self { lanes })
    }

    pub fn lane(&self, id: &str) -> Option<&Lane> {
        self.lanes.get(id).map(|(lane, _)| lane)
    }

    fn closures<'a>(&self, closed: &'a [String]) -> Result<BTreeSet<&'a str>, String> {
        if closed.len() > self.lanes.len() {
            return Err("too many lane closures".into());
        }
        let ids: BTreeSet<_> = closed.iter().map(String::as_str).collect();
        if ids.len() != closed.len() || ids.iter().any(|id| !self.lanes.contains_key(*id)) {
            return Err("unknown or duplicate lane closure".into());
        }
        Ok(ids)
    }

    /// Map permission only: gaps, traffic, timing and steering feasibility are separate.
    pub fn lane_change_target(
        &self,
        from: &str,
        side: LaneSide,
        closed_lanes: &[String],
    ) -> Result<Option<&Lane>, String> {
        let closed = self.closures(closed_lanes)?;
        let lane = self.lane(from).ok_or("unknown source lane")?;
        let neighbor = match side {
            LaneSide::Left => &lane.left,
            LaneSide::Right => &lane.right,
        };
        Ok(neighbor.as_ref().and_then(|neighbor| {
            (neighbor.change_allowed
                && !closed.contains(from)
                && !closed.contains(neighbor.lane_id.as_str()))
            .then(|| self.lane(&neighbor.lane_id).unwrap())
        }))
    }

    /// Start and goal name whole lanes. No adjacency is converted into a route edge.
    pub fn following_route(
        &self,
        start_lane: &str,
        goal_lane: &str,
        closed_lanes: &[String],
    ) -> Result<LaneRoutePlan, String> {
        let closed = self.closures(closed_lanes)?;
        if !self.lanes.contains_key(start_lane) || !self.lanes.contains_key(goal_lane) {
            return Err("unknown start or goal lane".into());
        }
        if closed.contains(start_lane) || closed.contains(goal_lane) {
            return Err("start or goal lane is closed".into());
        }
        let initial = self.lanes[start_lane].1;
        let mut distances = BTreeMap::from([(start_lane.to_owned(), initial)]);
        let mut previous = BTreeMap::new();
        let mut queue = BinaryHeap::from([QueueEntry {
            distance: initial,
            node: start_lane.into(),
        }]);
        while let Some(QueueEntry { distance, node }) = queue.pop() {
            if distance > distances[&node] {
                continue;
            }
            if node == goal_lane {
                break;
            }
            for next in &self.lanes[&node].0.successors {
                if closed.contains(next.as_str()) {
                    continue;
                }
                let candidate = distance + self.lanes[next].1;
                if !candidate.is_finite() {
                    return Err("lane route distance overflow".into());
                }
                if candidate < *distances.get(next).unwrap_or(&f64::INFINITY) {
                    distances.insert(next.clone(), candidate);
                    previous.insert(next.clone(), node.clone());
                    queue.push(QueueEntry {
                        distance: candidate,
                        node: next.clone(),
                    });
                }
            }
        }
        let distance_m = *distances
            .get(goal_lane)
            .ok_or("goal lane is unreachable with these closures")?;
        let mut lane_ids = vec![goal_lane.to_owned()];
        while lane_ids.last().unwrap() != start_lane {
            lane_ids.push(
                previous
                    .get(lane_ids.last().unwrap())
                    .ok_or("missing lane predecessor")?
                    .clone(),
            );
        }
        lane_ids.reverse();
        let mut points = Vec::new();
        let mut nodes = vec![RoadNode {
            id: "lane-step-0".into(),
            position: self.lanes[start_lane].0.centerline[0],
        }];
        let mut edges = Vec::new();
        let mut half_width = f64::INFINITY;
        let mut speed_limit_m_s = f64::INFINITY;
        for (index, id) in lane_ids.iter().enumerate() {
            let lane = &self.lanes[id].0;
            let from = format!("lane-step-{index}");
            let to = format!("lane-step-{}", index + 1);
            nodes.push(RoadNode {
                id: to.clone(),
                position: *lane.centerline.last().unwrap(),
            });
            edges.push(RoadEdge {
                id: id.clone(),
                from,
                to,
                points: lane.centerline.clone(),
                half_width: lane.width_m / 2.0,
            });
            points.extend(
                lane.centerline
                    .iter()
                    .skip(usize::from(!points.is_empty()))
                    .copied(),
            );
            half_width = half_width.min(lane.width_m / 2.0);
            speed_limit_m_s = speed_limit_m_s.min(lane.speed_limit_m_s);
        }
        let goal_node = nodes.last().unwrap().id.clone();
        Ok(LaneRoutePlan {
            lane_ids,
            distance_m,
            speed_limit_m_s,
            route: Route::new(points, half_width)?,
            network: RoadNetworkSpec {
                nodes,
                edges,
                turn_restrictions: vec![],
            },
            start_node: "lane-step-0".into(),
            goal_node,
        })
    }
}

fn straight_axis(lane: &Lane) -> Result<(Vec2, f64), String> {
    let start = lane.centerline[0];
    let end = *lane.centerline.last().unwrap();
    let length = start.distance(end);
    if length < 0.001 {
        return Err("neighbor sections must be straight and coextensive".into());
    }
    let axis = Vec2::new((end.x - start.x) / length, (end.y - start.y) / length);
    let mut previous = 0.0;
    for point in &lane.centerline {
        let dx = point.x - start.x;
        let dy = point.y - start.y;
        let along = dx * axis.x + dy * axis.y;
        if (axis.x * dy - axis.y * dx).abs() > GEOMETRY_TOLERANCE_M
            || along < previous - GEOMETRY_TOLERANCE_M
            || along > length + GEOMETRY_TOLERANCE_M
        {
            return Err("neighbor sections must be straight without backtracking".into());
        }
        previous = along;
    }
    Ok((axis, length))
}

fn validate_neighbor(lane: &Lane, target: &Lane, side: LaneSide) -> Result<(), String> {
    let (axis, length) = straight_axis(lane)?;
    let (other_axis, other_length) = straight_axis(target)?;
    let dx = target.centerline[0].x - lane.centerline[0].x;
    let dy = target.centerline[0].y - lane.centerline[0].y;
    let sign = if side == LaneSide::Left { 1.0 } else { -1.0 };
    let lateral = sign * (axis.x * dy - axis.y * dx);
    let end_dx = target.centerline.last().unwrap().x - lane.centerline.last().unwrap().x;
    let end_dy = target.centerline.last().unwrap().y - lane.centerline.last().unwrap().y;
    if axis.x * other_axis.x + axis.y * other_axis.y < 1.0 - 1e-10
        || (length - other_length).abs() > GEOMETRY_TOLERANCE_M
        || (dx * axis.x + dy * axis.y).abs() > GEOMETRY_TOLERANCE_M
        || (end_dx * axis.x + end_dy * axis.y).abs() > GEOMETRY_TOLERANCE_M
        || (sign * (axis.x * end_dy - axis.y * end_dx) - lateral).abs() > GEOMETRY_TOLERANCE_M
        || (lateral - (lane.width_m + target.width_m) / 2.0).abs() > GEOMETRY_TOLERANCE_M
    {
        return Err(
            "neighbors must share a boundary, direction, side and longitudinal extent".into(),
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests;
