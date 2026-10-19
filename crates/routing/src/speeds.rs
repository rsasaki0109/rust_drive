//! Whole-edge map limits compiled onto the exact selected route arc.
use crate::{RoadNetwork, RoutePlan};
use rustdriving_core::RouteSpeedZone;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct SpeedRoutePlan {
    pub plan: RoutePlan,
    pub speed_zones: Vec<RouteSpeedZone>,
}

impl RoadNetwork {
    /// Preserve distance/turn-rule routing, then compile its surveyed limits.
    pub fn shortest_route_with_speed_zones(
        &self,
        start: &str,
        goal: &str,
        closed_edges: &[String],
    ) -> Result<SpeedRoutePlan, String> {
        let plan = self.shortest_route(start, goal, closed_edges)?;
        let mut speed_zones: Vec<RouteSpeedZone> = vec![];
        let mut first = 0;
        for id in &plan.edge_ids {
            let edge = &self.edges[id].0;
            let last = first + edge.points.len() - 1;
            if let Some(limit) = edge.speed_limit_m_s {
                let zone = RouteSpeedZone {
                    start_s_m: plan.route.lengths[first],
                    end_s_m: plan.route.lengths[last],
                    speed_m_s: limit.min(8.),
                };
                if let Some(previous) = speed_zones.last_mut().filter(|previous| {
                    previous.end_s_m == zone.start_s_m && previous.speed_m_s == zone.speed_m_s
                }) {
                    previous.end_s_m = zone.end_s_m;
                } else {
                    speed_zones.push(zone);
                }
            }
            first = last;
        }
        if speed_zones.len() > 128 || (!speed_zones.is_empty() && plan.route.length() > 1500.) {
            return Err("mapped speeds require at most 128 intervals over at most 1500 m".into());
        }
        Ok(SpeedRoutePlan { plan, speed_zones })
    }
}

#[cfg(test)]
mod tests;
