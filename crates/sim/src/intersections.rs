//! Physical conflict-zone occupancy scoring; no driver diagnostics are consulted.
use crate::WorldObject;
use rustdrive_core::Vec2;
use rustdrive_pipeline::intersections::YieldIntersection;
use std::collections::BTreeMap;

const REQUIRED_GAP_S: f64 = 2.0;

#[derive(Clone, Copy, Debug)]
struct Interval {
    start: f64,
    end: f64,
}
#[derive(Default)]
struct Occupancy {
    start: Option<f64>,
    intervals: Vec<Interval>,
}
impl Occupancy {
    fn observe(&mut self, now: f64, inside: bool) {
        if inside {
            self.start.get_or_insert(now);
        } else {
            self.close(now);
        }
    }
    fn close(&mut self, now: f64) {
        if let Some(start) = self.start.take() {
            self.intervals.push(Interval { start, end: now });
        }
    }
}
#[derive(Default)]
struct ZoneOccupancy {
    ego: Occupancy,
    priority: BTreeMap<u64, Occupancy>,
}

/// Stores actual circular-body occupancy over the complete episode. Entry starts
/// at the first inside sample and exit ends at the first outside sample. Open
/// intervals remain occupied through the final episode timestamp.
pub struct IntersectionRuleEvaluator {
    zones: Vec<ZoneOccupancy>,
    pub violations: usize,
    /// Signed temporal separation: negative denotes overlapping occupancy.
    pub minimum_gap_s: Option<f64>,
}
impl IntersectionRuleEvaluator {
    pub fn new(count: usize) -> Self {
        Self {
            zones: (0..count).map(|_| ZoneOccupancy::default()).collect(),
            violations: 0,
            minimum_gap_s: None,
        }
    }
    pub fn observe(
        &mut self,
        intersections: &[YieldIntersection],
        now: f64,
        ego_position: Vec2,
        ego_radius: f64,
        objects: &[WorldObject],
    ) {
        for (intersection, zone) in intersections.iter().zip(&mut self.zones) {
            zone.ego.observe(
                now,
                intersection
                    .conflict_bounds
                    .overlaps_circle(ego_position, ego_radius),
            );
            for (id, occupancy) in &mut zone.priority {
                if !objects.iter().any(|object| object.id == *id) {
                    occupancy.close(now);
                }
            }
            for object in objects {
                zone.priority.entry(object.id).or_default().observe(
                    now,
                    intersection
                        .conflict_bounds
                        .overlaps_circle(object.position, object.radius),
                );
            }
        }
    }
    pub fn finish(&mut self, now: f64) {
        self.violations = 0;
        self.minimum_gap_s = None;
        for zone in &mut self.zones {
            zone.ego.close(now);
            for priority in zone.priority.values_mut() {
                priority.close(now);
            }
            let mut violated = false;
            for ego in &zone.ego.intervals {
                for priority in zone.priority.values().flat_map(|p| &p.intervals) {
                    let gap = if ego.end <= priority.start {
                        priority.start - ego.end
                    } else if priority.end <= ego.start {
                        ego.start - priority.end
                    } else {
                        -(ego.end.min(priority.end) - ego.start.max(priority.start))
                    };
                    self.minimum_gap_s = Some(self.minimum_gap_s.map_or(gap, |old| old.min(gap)));
                    violated |= gap + 1e-9 < REQUIRED_GAP_S;
                }
            }
            self.violations += usize::from(violated);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Scenario;
    use rustdrive_pipeline::intersections::ConflictBounds;
    use rustdrive_pipeline::traffic_controls::StopLine;

    fn intersection() -> YieldIntersection {
        YieldIntersection {
            stop_line: StopLine {
                id: "crossing".into(),
                route_s_m: 30.0,
            },
            conflict_bounds: ConflictBounds {
                min: Vec2::new(35.0, -2.0),
                max: Vec2::new(40.0, 2.0),
            },
            exit_s_m: 40.0,
        }
    }
    fn sample(
        e: &mut IntersectionRuleEvaluator,
        now: f64,
        ego_inside: bool,
        priority_inside: bool,
    ) {
        e.observe(
            &[intersection()],
            now,
            Vec2::new(if ego_inside { 37.0 } else { 20.0 }, 0.0),
            1.0,
            &if priority_inside {
                vec![WorldObject {
                    id: 11,
                    position: Vec2::new(37.0, 0.0),
                    radius: 1.0,
                }]
            } else {
                vec![]
            },
        );
    }
    #[test]
    fn overlap_and_insufficient_separation_are_rejected_without_driver_status() {
        for ego_entry in [0.5, 1.5, 2.95] {
            let mut e = IntersectionRuleEvaluator::new(1);
            sample(&mut e, 0.0, false, true);
            if ego_entry < 1.0 {
                sample(&mut e, ego_entry, true, true);
                sample(&mut e, 1.0, true, false);
            } else {
                sample(&mut e, 1.0, false, false);
                sample(&mut e, ego_entry, true, false);
            }
            sample(&mut e, 4.0, false, false);
            e.finish(5.0);
            assert_eq!(e.violations, 1);
            assert!(e.minimum_gap_s.unwrap() < 2.0);
        }
    }
    #[test]
    fn exact_two_second_separation_passes_with_conservative_exit_sample() {
        let mut e = IntersectionRuleEvaluator::new(1);
        sample(&mut e, 0.0, false, true);
        sample(&mut e, 1.0, false, false);
        sample(&mut e, 3.0, true, false);
        sample(&mut e, 4.0, false, false);
        e.finish(5.0);
        assert_eq!(e.violations, 0);
        assert_eq!(e.minimum_gap_s, Some(2.0));
    }
    #[test]
    fn later_priority_arrival_and_open_final_intervals_are_also_scored() {
        let mut e = IntersectionRuleEvaluator::new(1);
        sample(&mut e, 0.0, true, false);
        sample(&mut e, 1.0, false, false);
        sample(&mut e, 2.0, false, true);
        e.finish(5.0);
        assert_eq!(e.violations, 1);
        assert_eq!(e.minimum_gap_s, Some(1.0));

        let mut e = IntersectionRuleEvaluator::new(1);
        sample(&mut e, 0.0, false, true);
        sample(&mut e, 2.0, true, true);
        e.finish(5.0);
        assert_eq!(e.violations, 1);
        assert_eq!(e.minimum_gap_s, Some(-3.0));
    }
    #[test]
    fn repeated_occupancies_count_once_per_zone_and_absent_pairs_have_no_gap() {
        let mut e = IntersectionRuleEvaluator::new(1);
        for now in 0..5 {
            sample(&mut e, now as f64, now % 2 == 0, true);
        }
        e.finish(5.0);
        assert_eq!(e.violations, 1);

        for ego_present in [false, true] {
            let mut e = IntersectionRuleEvaluator::new(1);
            sample(&mut e, 0.0, ego_present, !ego_present);
            e.finish(5.0);
            assert_eq!(e.violations, 0);
            assert_eq!(e.minimum_gap_s, None);
        }
    }
    #[test]
    fn body_radius_and_disappearance_define_physical_occupancy_endpoints() {
        let mut e = IntersectionRuleEvaluator::new(1);
        // The center is outside, but its circular physical body overlaps the box.
        e.observe(
            &[intersection()],
            0.0,
            Vec2::new(20.0, 0.0),
            1.0,
            &[WorldObject {
                id: 19,
                position: Vec2::new(34.5, 0.0),
                radius: 1.0,
            }],
        );
        // A missing body ends its interval at this first sampled absence.
        sample(&mut e, 1.0, false, false);
        sample(&mut e, 2.0, true, false);
        e.finish(4.0);
        assert_eq!(e.violations, 1);
        assert_eq!(e.minimum_gap_s, Some(1.0));
    }
    #[test]
    fn controller_ignoring_backend_cannot_hide_a_physical_priority_violation() {
        use crate::{ReferenceBackend, SimulationBackend, pipeline_config, simulate_with_backend};
        use rustdrive_core::{ControlCommand, EgoState};
        use rustdrive_pipeline::SensorFrame;
        struct IgnoreControl(ReferenceBackend);
        impl SimulationBackend for IgnoreControl {
            fn state(&self) -> EgoState {
                self.0.state()
            }
            fn objects(&self, time: f64) -> Vec<WorldObject> {
                self.0.objects(time)
            }
            fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String> {
                self.0.observe(time, tick)
            }
            fn advance(&mut self, _command: ControlCommand, dt: f64) -> Result<(), String> {
                self.0.advance(
                    ControlCommand {
                        acceleration: 2.0,
                        steering: 0.0,
                    },
                    dt,
                )
            }
        }
        let mut scenario: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/stop-sign-single.json")).unwrap();
        scenario.stop_signs.clear();
        scenario.yield_intersections = vec![intersection()];
        scenario.objects = vec![crate::ObjectSpec {
            s: 37.0,
            lateral: 10.0,
            radius: 1.0,
            speed: 0.0,
            lateral_speed: -1.5,
            active_from: 0.0,
            moving_from: 0.0,
            following: None,
        }];
        let config = pipeline_config(&scenario);
        let backend = IgnoreControl(ReferenceBackend {
            scenario: scenario.clone(),
            traffic: crate::traffic::TrafficWorld::new(scenario.clone(), config.route.clone()),
            truth: EgoState {
                pose: config.initial_pose,
                speed: 0.0,
            },
            vehicle: config.vehicle,
            rng: crate::Rng::new(7),
            command: ControlCommand::default(),
        });
        let run = simulate_with_backend(scenario, 7, backend, config, "negative-priority-backend")
            .unwrap();
        assert!(!run.summary.passed);
        assert_eq!(run.summary.intersection_violations, 1);
        assert!(run.summary.intersection_min_gap_s.unwrap() < 2.0);
        assert!(
            run.summary
                .failures
                .iter()
                .any(|s| s.contains("physical intersection priority"))
        );
    }
}
