//! Speed mapping for one fixed connector, without alternate-route returns.
use super::{LaneChangeRequest, LaneChangeRoutePlan, LaneNetwork};
use rustdriving_core::{RouteSpeedZone, VehicleConfig};
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct LaneSpeedChangeRoutePlan {
    pub change: LaneChangeRoutePlan,
    pub speed_zones: Vec<RouteSpeedZone>,
}

impl LaneNetwork {
    /// Source limit before changing, minimum source/target limit through the
    /// entire connector, target limit after it, then each selected successor.
    /// Boundaries use actual sampled-route arc length, not source longitudinal distance.
    pub fn lane_change_route_with_speed_zones(
        &self,
        start_lane: &str,
        goal_lane: &str,
        closed_lanes: &[String],
        request: LaneChangeRequest,
        vehicle: VehicleConfig,
    ) -> Result<LaneSpeedChangeRoutePlan, String> {
        let (change, [prefix, connector, target_end]) =
            self.lane_change_route_parts(start_lane, goal_lane, closed_lanes, request, vehicle)?;
        let route = &change.plan.route;
        if route.length() > 1500. {
            return Err("local speed limits require a fixed route of at most 1500 m".into());
        }
        let source = self.lane(&change.maneuver.from_lane).unwrap();
        let target = self.lane(&change.maneuver.to_lane).unwrap();
        let mut speed_zones: Vec<RouteSpeedZone> = vec![];
        let mut add = |first: usize, last: usize, speed: f64| -> Result<(), String> {
            if first == last {
                return Ok(());
            }
            let zone = RouteSpeedZone {
                start_s_m: route.lengths[first],
                end_s_m: route.lengths[last],
                speed_m_s: speed.min(8.),
            };
            if let Some(previous) = speed_zones
                .last_mut()
                .filter(|p| p.speed_m_s == zone.speed_m_s)
            {
                previous.end_s_m = zone.end_s_m;
            } else {
                speed_zones.push(zone);
            }
            if speed_zones.len() > 128 {
                return Err("local speed limits exceed 128 distinct adjacent intervals".into());
            }
            Ok(())
        };
        add(0, prefix, source.speed_limit_m_s)?;
        add(
            prefix,
            connector,
            source.speed_limit_m_s.min(target.speed_limit_m_s),
        )?;
        add(connector, target_end, target.speed_limit_m_s)?;
        let mut cursor = target_end;
        for id in change.plan.lane_ids.iter().skip(2) {
            let lane = self.lane(id).unwrap();
            let end = cursor + lane.centerline.len() - 1;
            add(cursor, end, lane.speed_limit_m_s)?;
            cursor = end;
        }
        Ok(LaneSpeedChangeRoutePlan {
            change,
            speed_zones,
        })
    }
}

#[cfg(test)]
mod tests;
