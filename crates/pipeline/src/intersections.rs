//! Fixed-route yielding from mapped conflict geometry and observed motion forecasts.
//! No simulator traffic schedule or physical-object identity enters this module.
use crate::traffic_controls::{StopLine, planning_prefix};
use rustdrive_core::{EgoState, Prediction, Route, Vec2, wrap_angle};
use serde::{Deserialize, Serialize};

pub const FORECAST_HORIZON_S: f64 = 8.0;
pub const FORECAST_MARGIN_M: f64 = 0.5;
pub const CLEAR_CONFIRM_S: f64 = 1.0;
pub const MAX_SCAN_AGE_S: f64 = 0.15;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConflictBounds {
    pub min: Vec2,
    pub max: Vec2,
}
impl ConflictBounds {
    pub fn overlaps_circle(&self, position: Vec2, radius: f64) -> bool {
        let nearest = Vec2::new(
            position.x.clamp(self.min.x, self.max.x),
            position.y.clamp(self.min.y, self.max.y),
        );
        position.distance(nearest) <= radius
    }
    fn intersects_segment(&self, a: Vec2, b: Vec2, inflation: f64) -> bool {
        let mut from: f64 = 0.0;
        let mut until: f64 = 1.0;
        for (start, delta, lo, hi) in [
            (
                a.x,
                b.x - a.x,
                self.min.x - inflation,
                self.max.x + inflation,
            ),
            (
                a.y,
                b.y - a.y,
                self.min.y - inflation,
                self.max.y + inflation,
            ),
        ] {
            if delta.abs() < 1e-12 {
                if start < lo || start > hi {
                    return false;
                }
            } else {
                let left = (lo - start) / delta;
                let right = (hi - start) / delta;
                from = from.max(left.min(right));
                until = until.min(left.max(right));
                if from > until {
                    return false;
                }
            }
        }
        true
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct YieldIntersection {
    pub stop_line: StopLine,
    pub conflict_bounds: ConflictBounds,
    /// Far conflict boundary arc length; rear must clear it before Passed.
    pub exit_s_m: f64,
}

pub fn validate(zones: &[YieldIntersection], route: &Route, radius: f64) -> Result<(), String> {
    if zones.len() > 16 {
        return Err("at most 16 yield intersections are supported".into());
    }
    if zones.is_empty() {
        return Ok(());
    }
    let direction = route.points.last().unwrap().minus(route.points[0]);
    let yaw = direction.y.atan2(direction.x);
    if route.points.windows(2).any(|p| {
        let d = p[1].minus(p[0]);
        wrap_angle(d.y.atan2(d.x) - yaw).abs() > 1e-6
    }) {
        return Err("yield intersections currently require a straight forward route".into());
    }
    for zone in zones {
        let b = zone.conflict_bounds;
        if !b.min.finite()
            || !b.max.finite()
            || !(1.0..=50.0).contains(&(b.max.x - b.min.x))
            || !(1.0..=50.0).contains(&(b.max.y - b.min.y))
            || !zone.exit_s_m.is_finite()
            || zone.exit_s_m <= zone.stop_line.route_s_m + 5.0
            || zone.exit_s_m >= route.length() - radius - 3.0
            || b.overlaps_circle(route.sample(zone.stop_line.route_s_m, 0.0).0, radius + 2.0)
            || !b.overlaps_circle(route.sample(zone.exit_s_m, 0.0).0, 0.01)
            || b.overlaps_circle(route.sample(zone.exit_s_m + 0.1, 0.0).0, 0.01)
        {
            return Err("invalid yield conflict bounds, approach margin or route exit".into());
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IntersectionPhase {
    Waiting,
    Proceeding,
    Passed,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlledIntersection {
    pub id: String,
    pub phase: IntersectionPhase,
    pub blocking_tracks: Vec<u64>,
    pub clear_since: Option<f64>,
    pub committed: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IntersectionStatus {
    pub zones: Vec<ControlledIntersection>,
    pub stop_s_m: Option<f64>,
}

fn forecast_blocks(bounds: &ConflictBounds, prediction: &Prediction) -> bool {
    if !prediction.dt.is_finite()
        || prediction.dt <= 0.0
        || !prediction.radius.is_finite()
        || prediction.radius < 0.0
        || prediction.positions.is_empty()
        || prediction.positions.iter().any(|p| !p.finite())
    {
        return true;
    }
    let inflation = prediction.radius + FORECAST_MARGIN_M;
    if bounds.intersects_segment(prediction.positions[0], prediction.positions[0], inflation) {
        return true;
    }
    for (index, points) in prediction.positions.windows(2).enumerate() {
        let start = index as f64 * prediction.dt;
        if start >= FORECAST_HORIZON_S {
            break;
        }
        let fraction = ((FORECAST_HORIZON_S - start) / prediction.dt).min(1.0);
        let end = points[0].plus(points[1].minus(points[0]).scaled(fraction));
        if bounds.intersects_segment(points[0], end, inflation) {
            return true;
        }
    }
    false
}

pub(crate) struct YieldIntersections {
    zones: Vec<YieldIntersection>,
    route: Route,
    clear_samples: Vec<usize>,
    last_scan: Option<f64>,
    status: IntersectionStatus,
}
impl YieldIntersections {
    pub fn new(zones: Vec<YieldIntersection>, route: Route) -> Self {
        Self {
            route,
            clear_samples: vec![0; zones.len()],
            last_scan: None,
            status: IntersectionStatus {
                zones: zones
                    .iter()
                    .map(|z| ControlledIntersection {
                        id: z.stop_line.id.clone(),
                        phase: IntersectionPhase::Waiting,
                        blocking_tracks: vec![],
                        clear_since: None,
                        committed: false,
                    })
                    .collect(),
                stop_s_m: None,
            },
            zones,
        }
    }
    pub fn step(
        &mut self,
        now: f64,
        ego: EgoState,
        radius: f64,
        predictions: &[Prediction],
        scan_stamp: Option<f64>,
        healthy: bool,
    ) {
        let route = &self.route;
        let (progress, lateral) = route.project(ego.pose.position);
        let aligned = wrap_angle(ego.pose.yaw - route.sample(progress, 0.0).1).abs() < 0.2
            && lateral.abs() + radius <= route.half_width;
        let fresh = healthy
            && aligned
            && scan_stamp.is_some_and(|s| now - s <= MAX_SCAN_AGE_S + 1e-9 && s <= now);
        let new_scan = scan_stamp.is_some_and(|s| self.last_scan.is_none_or(|old| s > old));
        if new_scan {
            self.last_scan = scan_stamp;
        }
        self.status.stop_s_m = None;
        for ((zone, state), samples) in self
            .zones
            .iter()
            .zip(&mut self.status.zones)
            .zip(&mut self.clear_samples)
        {
            if state.phase == IntersectionPhase::Passed {
                continue;
            }
            state.blocking_tracks = predictions
                .iter()
                .filter(|p| forecast_blocks(&zone.conflict_bounds, p))
                .map(|p| p.id)
                .collect();
            if !state.committed {
                if !fresh || !state.blocking_tracks.is_empty() {
                    state.phase = IntersectionPhase::Waiting;
                    state.clear_since = None;
                    *samples = 0;
                } else if new_scan {
                    let stamp = scan_stamp.unwrap();
                    let start = *state.clear_since.get_or_insert(stamp);
                    *samples += 1;
                    if *samples >= 3 && stamp - start + 1e-9 >= CLEAR_CONFIRM_S {
                        state.phase = IntersectionPhase::Proceeding;
                    }
                }
                if state.phase == IntersectionPhase::Proceeding
                    && fresh
                    && progress + radius >= zone.stop_line.route_s_m
                {
                    state.committed = true;
                }
            }
            if state.committed && healthy && aligned && progress - radius >= zone.exit_s_m {
                state.phase = IntersectionPhase::Passed;
            }
            if state.phase == IntersectionPhase::Waiting {
                let end = zone.stop_line.route_s_m - radius - 1.0;
                self.status.stop_s_m = Some(self.status.stop_s_m.map_or(end, |s| s.min(end)));
            }
        }
    }
    pub fn status(&self) -> IntersectionStatus {
        self.status.clone()
    }
    pub fn endpoint(&self) -> Option<f64> {
        self.status.stop_s_m
    }
    pub fn planning_route(&self, route: &Route) -> Option<Route> {
        planning_prefix(route, self.endpoint()?)
    }
    pub fn holding_brake(&self, route: &Route, ego: EgoState) -> bool {
        self.endpoint().is_some_and(|end| {
            end + 1.0 - route.project(ego.pose.position).0 <= 3.5 && ego.speed.abs() <= 0.2
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdrive_core::Pose;
    fn zone() -> YieldIntersection {
        YieldIntersection {
            stop_line: StopLine {
                id: "junction".into(),
                route_s_m: 35.0,
            },
            conflict_bounds: ConflictBounds {
                min: Vec2::new(42.0, -2.1),
                max: Vec2::new(48.0, 2.1),
            },
            exit_s_m: 48.0,
        }
    }
    fn route() -> Route {
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap()
    }
    fn ego(front: f64) -> EgoState {
        EgoState {
            pose: Pose {
                position: Vec2::new(front - 1.25, 0.0),
                yaw: 0.0,
            },
            speed: 0.0,
        }
    }
    #[test]
    fn fast_crossing_segments_block_even_when_no_prediction_sample_is_inside() {
        let b = zone().conflict_bounds;
        let p = Prediction {
            id: 1,
            positions: vec![Vec2::new(45.0, 10.0), Vec2::new(45.0, -10.0)],
            radius: 1.0,
            dt: 0.2,
        };
        assert!(forecast_blocks(&b, &p));
        let departing = Prediction {
            positions: vec![Vec2::new(45.0, -5.0), Vec2::new(45.0, -15.0)],
            ..p
        };
        assert!(!forecast_blocks(&b, &departing));
    }
    #[test]
    fn duplicate_stale_or_unhealthy_scans_cannot_release_the_conflict_gate() {
        let mut y = YieldIntersections::new(vec![zone()], route());
        for i in 0..40 {
            y.step(i as f64 * 0.05, ego(33.0), 1.25, &[], Some(0.0), true);
        }
        assert_eq!(y.status().zones[0].phase, IntersectionPhase::Waiting);
        for i in 40..61 {
            let t = i as f64 * 0.05;
            y.step(t, ego(33.0), 1.25, &[], Some(t), true);
        }
        assert_eq!(y.status().zones[0].phase, IntersectionPhase::Proceeding);
        y.step(3.05, ego(33.0), 1.25, &[], Some(3.05), false);
        assert_eq!(y.status().zones[0].phase, IntersectionPhase::Waiting);
        assert!(y.status().zones[0].clear_since.is_none());
    }
    #[test]
    fn new_conflict_reblocks_before_commit_but_passed_zone_is_not_reversed() {
        let mut y = YieldIntersections::new(vec![zone()], route());
        for i in 0..21 {
            let t = i as f64 * 0.05;
            y.step(t, ego(33.0), 1.25, &[], Some(t), true);
        }
        let p = Prediction {
            id: 7,
            positions: vec![Vec2::new(45.0, 0.0)],
            radius: 1.0,
            dt: 0.2,
        };
        y.step(
            1.05,
            ego(33.0),
            1.25,
            std::slice::from_ref(&p),
            Some(1.05),
            true,
        );
        assert_eq!(y.status().zones[0].phase, IntersectionPhase::Waiting);
        for i in 22..43 {
            let t = i as f64 * 0.05;
            y.step(t, ego(33.0), 1.25, &[], Some(t), true);
        }
        y.step(2.15, ego(36.0), 1.25, &[], Some(2.15), true);
        assert!(y.status().zones[0].committed);
        y.step(
            2.2,
            ego(51.0),
            1.25,
            std::slice::from_ref(&p),
            Some(2.2),
            true,
        );
        assert_eq!(y.status().zones[0].phase, IntersectionPhase::Passed);
        assert!(y.endpoint().is_none());
    }
    #[test]
    fn malformed_geometry_and_curved_routes_are_rejected() {
        assert!(validate(&[zone()], &route(), 1.25).is_ok());
        let mut z = zone();
        z.exit_s_m = 60.0;
        assert!(validate(&[z], &route(), 1.25).is_err());
        let r = Route::new(
            vec![Vec2::default(), Vec2::new(50.0, 2.0), Vec2::new(100.0, 0.0)],
            2.1,
        )
        .unwrap();
        assert!(validate(&[zone()], &r, 1.25).is_err());
    }
}
