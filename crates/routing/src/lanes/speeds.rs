//! Compile selected whole-lane limits onto the exact resolved route origin.
use super::{LaneNetwork, LaneRoutePlan};
use rustdriving_core::RouteSpeedZone;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct LaneSpeedRoutePlan {
    pub plan: LaneRoutePlan,
    /// Effective caps for the bounded fixed-route consumer, in SI units.
    pub speed_zones: Vec<RouteSpeedZone>,
}

impl LaneNetwork {
    /// Preserve whole-lane routing and compile local speed caps for a fixed route.
    /// Limits above the consumer's 8 m/s ceiling become 8 m/s; width still uses
    /// the narrowest lane. No lateral connector or live remapping is supplied.
    pub fn following_route_with_speed_zones(
        &self,
        start_lane: &str,
        goal_lane: &str,
        closed_lanes: &[String],
    ) -> Result<LaneSpeedRoutePlan, String> {
        let plan = self.following_route(start_lane, goal_lane, closed_lanes)?;
        if plan.route.length() > 1500.0 {
            return Err("local speed limits require a fixed route of at most 1500 m".into());
        }
        let mut speed_zones: Vec<RouteSpeedZone> = Vec::new();
        let mut first = 0;
        for id in &plan.lane_ids {
            let lane = &self.lanes[id].0;
            let last = first + lane.centerline.len() - 1;
            // Index the concatenated route, avoiding accumulated per-lane
            // rounding errors that could leave a gap or exceed its endpoint.
            let zone = RouteSpeedZone {
                start_s_m: plan.route.lengths[first],
                end_s_m: plan.route.lengths[last],
                speed_m_s: lane.speed_limit_m_s.min(8.0),
            };
            if let Some(previous) = speed_zones
                .last_mut()
                .filter(|previous| previous.speed_m_s == zone.speed_m_s)
            {
                previous.end_s_m = zone.end_s_m;
            } else {
                speed_zones.push(zone);
            }
            if speed_zones.len() > 128 {
                return Err("local speed limits exceed 128 distinct adjacent intervals".into());
            }
            first = last;
        }
        Ok(LaneSpeedRoutePlan { plan, speed_zones })
    }
}

#[cfg(test)]
mod tests;
