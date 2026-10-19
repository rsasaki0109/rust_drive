//! One pre-departure, map-authorized lane change. No traffic-gap decision is made here.
use super::{LaneNetwork, LaneRoutePlan, LaneSide, straight_axis};
use crate::{RoadEdge, RoadNetworkSpec, RoadNode};
use rustdriving_core::{Route, Vec2, VehicleConfig};
use serde::{Deserialize, Serialize};

const SAMPLE_STEP_M: f64 = 0.25;
const MAX_CHANGE_SEGMENTS: usize = 4096;
const TRACKING_RESERVE_M: f64 = 0.05;

/// Distances follow the source lane from its beginning, in meters, not elapsed time.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeRequest {
    pub side: LaneSide,
    pub start_m: f64,
    pub length_m: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct LaneChangeManeuver {
    pub from_lane: String,
    pub to_lane: String,
    pub request: LaneChangeRequest,
    pub vehicle: VehicleConfig,
    /// Conservative bound over the entire continuous curve, before polyline sampling.
    pub curvature_bound_m_inv: f64,
    pub steering_curvature_limit_m_inv: f64,
    pub corridor_tracking_margin_m: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct LaneChangeRoutePlan {
    pub maneuver: LaneChangeManeuver,
    pub plan: LaneRoutePlan,
}

impl LaneNetwork {
    /// Change once from the start lane, then follow explicit successors to the goal.
    /// The vehicle is a circular planar envelope, as in the reference simulator.
    /// This checks geometry and map permission; it neither observes nor accepts traffic gaps.
    pub fn lane_change_route(
        &self,
        start_lane: &str,
        goal_lane: &str,
        closed_lanes: &[String],
        request: LaneChangeRequest,
        vehicle: VehicleConfig,
    ) -> Result<LaneChangeRoutePlan, String> {
        self.lane_change_route_parts(start_lane, goal_lane, closed_lanes, request, vehicle)
            .map(|(change, _)| change)
    }

    /// Internal boundary indices: source-prefix end, connector end, target-section end.
    pub(super) fn lane_change_route_parts(
        &self,
        start_lane: &str,
        goal_lane: &str,
        closed_lanes: &[String],
        request: LaneChangeRequest,
        vehicle: VehicleConfig,
    ) -> Result<(LaneChangeRoutePlan, [usize; 3]), String> {
        if !request.start_m.is_finite()
            || request.start_m < 0.0
            || !request.length_m.is_finite()
            || request.length_m <= 0.0
            || !vehicle.wheelbase.is_finite()
            || vehicle.wheelbase <= 0.0
            || !vehicle.radius.is_finite()
            || vehicle.radius <= 0.0
            || !vehicle.max_steer.is_finite()
            || vehicle.max_steer <= 0.0
            || vehicle.max_steer >= std::f64::consts::FRAC_PI_2
        {
            return Err(
                "lane change requires finite positive geometry and vehicle calibration".into(),
            );
        }
        let target = self
            .lane_change_target(start_lane, request.side, closed_lanes)?
            .ok_or("lane change is forbidden, absent or closed")?;
        let source = self.lane(start_lane).unwrap();
        let (axis, section_length) = straight_axis(source)?;
        let end_m = request.start_m + request.length_m;
        let segments = (request.length_m / SAMPLE_STEP_M).ceil();
        if !end_m.is_finite() || end_m > section_length || segments > MAX_CHANGE_SEGMENTS as f64 {
            return Err("lane change exceeds the shared section or 4096-segment budget".into());
        }
        let mut tail = self.following_route(&target.id, goal_lane, closed_lanes)?;
        let half_width = tail.route.half_width.min(source.width_m / 2.0);
        let margin = half_width - vehicle.radius;
        if margin < TRACKING_RESERVE_M {
            return Err("lane change corridor cannot fit the vehicle and 0.05 m reserve".into());
        }

        let source_start = source.centerline[0];
        let source_end = *source.centerline.last().unwrap();
        let target_start = target.centerline[0];
        let target_end = *target.centerline.last().unwrap();
        let offset = target_start.minus(source_start);
        let end_offset = target_end.minus(source_end);
        let offset_rate = end_offset.minus(offset).scaled(1.0 / section_length);
        let dot = |p: Vec2| p.x * axis.x + p.y * axis.y;
        // f(u)=10u^3-15u^4+6u^5: max f'=15/8, max |f''|=10/sqrt(3).
        // Include the map's micrometer tolerance in parallelism/extent rather than
        // assuming perfectly translated lanes. |r'| >= its forward component;
        // curvature <= |r''| / |r'|^2 for every u, not just sampled points.
        let minimum_forward_derivative = 1.0
            - 1.875 * dot(offset).abs().max(dot(end_offset).abs()) / request.length_m
            - dot(offset_rate).abs();
        let second_derivative_bound = 10.0 / 3.0_f64.sqrt()
            * offset
                .x
                .hypot(offset.y)
                .max(end_offset.x.hypot(end_offset.y))
            / request.length_m.powi(2)
            + 3.75 * offset_rate.x.hypot(offset_rate.y) / request.length_m;
        let curvature_bound = second_derivative_bound / minimum_forward_derivative.powi(2);
        // Match the existing local-corner geometry's 0.5% steering-radius reserve.
        let curvature_limit = vehicle.max_steer.tan() / vehicle.wheelbase / 1.005;
        if minimum_forward_derivative <= 0.0
            || !curvature_bound.is_finite()
            || !curvature_limit.is_finite()
            || curvature_limit <= 0.0
            || curvature_bound > curvature_limit
        {
            return Err("lane change exceeds the calibrated steering curvature limit".into());
        }
        let source_vector = source_end.minus(source_start);
        let target_vector = target_end.minus(target_start);
        let lane_point =
            |start: Vec2, vector: Vec2, s: f64| start.plus(vector.scaled(s / section_length));
        let mut points = vec![source_start];
        if request.start_m > 0.0 {
            points.push(lane_point(source_start, source_vector, request.start_m));
        }
        let prefix_end = points.len() - 1;
        let segments = segments as usize;
        for i in 1..=segments {
            let u = i as f64 / segments as f64;
            let s = request.start_m + request.length_m * u;
            let f = u.powi(3) * (10.0 + u * (-15.0 + 6.0 * u));
            let from = lane_point(source_start, source_vector, s);
            let to = lane_point(target_start, target_vector, s);
            points.push(if i == segments {
                to
            } else {
                from.plus(to.minus(from).scaled(f))
            });
        }
        let connector_end = points.len() - 1;
        if end_m < section_length {
            points.push(target_end);
        } else {
            *points.last_mut().unwrap() = target_end;
        }
        let target_section_end = points.len() - 1;
        // Replace the whole target section with source prefix + transition + target suffix.
        points.extend(
            tail.route
                .points
                .iter()
                .skip(target.centerline.len())
                .copied(),
        );
        let route = Route::new(points, half_width)?;
        if !route.length().is_finite() {
            return Err("lane change route distance overflow".into());
        }
        let start_node = "lane-change-start".to_owned();
        let goal_node = "lane-change-goal".to_owned();
        let network = RoadNetworkSpec {
            nodes: vec![
                RoadNode {
                    id: start_node.clone(),
                    position: route.points[0],
                },
                RoadNode {
                    id: goal_node.clone(),
                    position: *route.points.last().unwrap(),
                },
            ],
            edges: vec![RoadEdge {
                id: "lane-change-route".into(),
                from: start_node.clone(),
                to: goal_node.clone(),
                points: route.points.clone(),
                half_width,
                speed_limit_m_s: None,
            }],
            turn_restrictions: vec![],
        };
        tail.lane_ids.insert(0, source.id.clone());
        Ok((
            LaneChangeRoutePlan {
                maneuver: LaneChangeManeuver {
                    from_lane: source.id.clone(),
                    to_lane: target.id.clone(),
                    request,
                    vehicle,
                    curvature_bound_m_inv: curvature_bound,
                    steering_curvature_limit_m_inv: curvature_limit,
                    corridor_tracking_margin_m: margin,
                },
                plan: LaneRoutePlan {
                    lane_ids: tail.lane_ids,
                    distance_m: route.length(),
                    speed_limit_m_s: source.speed_limit_m_s.min(tail.speed_limit_m_s),
                    route,
                    network,
                    start_node,
                    goal_node,
                },
            },
            [prefix_end, connector_end, target_section_end],
        ))
    }
}

#[cfg(test)]
mod tests;
