//! Piecewise-linear, synchronized circular sweeps for candidate trajectory validation.
use rustdriving_core::{Prediction, TrajectoryPoint, Vec2};

/// Earliest contact under the supplied time parameterization. Inputs are validated by
/// the planner. Prediction knots are linearly interpolated, then held at the last point.
/// Each subinterval uses its end-time uncertainty margin, conservatively covering the
/// increasing envelope. This does not validate the vehicle's longitudinal feasibility.
pub(super) fn first_contact_time(
    path: &[TrajectoryPoint],
    object: &Prediction,
    vehicle_radius: f64,
    held_heading: Vec2,
) -> Option<f64> {
    with_minimum_clearance(path, object, vehicle_radius, held_heading, 0.0)
}

#[derive(Clone, Copy)]
struct Safety {
    vehicle_radius: f64,
    minimum_clearance: f64,
}

pub(super) fn with_minimum_clearance(
    path: &[TrajectoryPoint],
    object: &Prediction,
    vehicle_radius: f64,
    held_heading: Vec2,
    minimum_clearance: f64,
) -> Option<f64> {
    let safety = Safety {
        vehicle_radius,
        minimum_clearance,
    };
    let first = path.first()?;
    let initial_tangent = path
        .windows(2)
        .map(|pair| pair[1].position.minus(pair[0].position))
        .find(|tangent| tangent.x.hypot(tangent.y) > 1e-6)
        .unwrap_or(held_heading);
    if first.position.distance(position_at(object, first.time))
        <= vehicle_radius
            + object.radius_at(first.time)
            + minimum_clearance.max(margin(
                first.time,
                velocity_at(object, first.time),
                initial_tangent,
            ))
    {
        return Some(first.time);
    }
    for pair in path.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let mut start = a.time;
        let first_knot = ((start / object.dt).floor() as usize).saturating_add(1);
        // A forecast may turn inside a trajectory segment: sweeping only its endpoint
        // chord would miss that turn. Visit each intervening forecast knot as well.
        for index in first_knot..object.positions.len() {
            let end = index as f64 * object.dt;
            if end >= b.time {
                break;
            }
            if end > start {
                if let Some(time) = segment_contact(a, b, object, safety, start, end, held_heading)
                {
                    return Some(time);
                }
                start = end;
            }
        }
        if let Some(time) = segment_contact(a, b, object, safety, start, b.time, held_heading) {
            return Some(time);
        }
    }
    None
}

fn margin(time: f64, velocity: Vec2, tangent: Vec2) -> f64 {
    // Moving across the candidate needs more empirical clearance than static or
    // parallel traffic. This reserve is not a certified prediction-error bound.
    // The transverse reserve follows direction, rather than vanishing with a
    // slow crossing's speed. Perpendicular motion retains 1 m at time zero.
    let fraction = transverse_fraction(velocity, tangent);
    let transverse_speed = velocity.x.hypot(velocity.y) * fraction;
    let motion_reserve = (0.40 * transverse_speed.min(1.)).max(0.70 * fraction);
    0.30 + 0.06 * time.min(5.0) + motion_reserve
}

fn transverse_fraction(velocity: Vec2, tangent: Vec2) -> f64 {
    let speed = velocity.x.hypot(velocity.y);
    if speed == 0. {
        return 0.;
    }
    let length = tangent.x.hypot(tangent.y);
    if length > 1e-6 {
        ((velocity.x / speed) * (tangent.y / length) - (velocity.y / speed) * (tangent.x / length))
            .abs()
            .min(1.)
    } else {
        // A stationary hold has no segment heading. Reserve for any observed
        // motion rather than choosing an arbitrary world-frame axis.
        1.
    }
}

fn velocity_at(object: &Prediction, time: f64) -> Vec2 {
    let index = (time / object.dt).floor() as usize;
    if index >= object.positions.len() - 1 {
        Vec2::default()
    } else {
        object.positions[index + 1]
            .minus(object.positions[index])
            .scaled(1.0 / object.dt)
    }
}

fn position_at(object: &Prediction, time: f64) -> Vec2 {
    let step = time / object.dt;
    let index = (step.floor() as usize).min(object.positions.len() - 1);
    let next = (index + 1).min(object.positions.len() - 1);
    object.positions[index].plus(
        object.positions[next]
            .minus(object.positions[index])
            .scaled((step - index as f64).clamp(0.0, 1.0)),
    )
}

fn segment_contact(
    a: &TrajectoryPoint,
    b: &TrajectoryPoint,
    object: &Prediction,
    safety: Safety,
    start: f64,
    end: f64,
    held_heading: Vec2,
) -> Option<f64> {
    let ego_at = |time| {
        let fraction = if b.time > a.time {
            (time - a.time) / (b.time - a.time)
        } else {
            0.0
        };
        a.position
            .plus(b.position.minus(a.position).scaled(fraction))
    };
    let relative_start = ego_at(start).minus(position_at(object, start));
    let relative_end = ego_at(end).minus(position_at(object, end));
    let displacement = b.position.minus(a.position);
    let tangent = if displacement.x.hypot(displacement.y) > 1e-6 {
        displacement
    } else {
        held_heading
    };
    contact_fraction(
        relative_start,
        relative_end,
        // Constant acceleration along a segment differs from its temporal chord
        // by at most |delta_v| * duration / 8. Inflate the circle by this bound.
        safety.vehicle_radius
            + object.radius_at(start).max(object.radius_at(end))
            + safety.minimum_clearance.max(margin(
                end,
                velocity_at(object, 0.5 * start + 0.5 * end),
                tangent,
            ))
            + (b.speed - a.speed).abs() * (b.time - a.time) / 8.0,
    )
    .map(|fraction| start + (end - start) * fraction)
}

/// Solve |start + u * delta| <= radius for the earliest u in [0, 1].
fn contact_fraction(start: Vec2, end: Vec2, radius: f64) -> Option<f64> {
    let c = start.x * start.x + start.y * start.y - radius * radius;
    if c <= 0.0 {
        return Some(0.0);
    }
    let delta = end.minus(start);
    let a = delta.x * delta.x + delta.y * delta.y;
    let b = start.x * delta.x + start.y * delta.y;
    if a == 0.0 || b >= 0.0 {
        return None;
    }
    let discriminant = b * b - a * c;
    if discriminant < 0.0 {
        return None;
    }
    // Equivalent to (-b - sqrt(discriminant)) / a, avoiding cancellation
    // when the start is close to the envelope boundary.
    let fraction = c / (-b + discriminant.sqrt());
    (0.0..=1.0).contains(&fraction).then_some(fraction)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_explicit_floor_rejects_a_legacy_near_miss_and_keeps_larger_motion_reserves() {
        let path = [
            TrajectoryPoint {
                position: Vec2::new(0., 0.),
                speed: 5.,
                time: 0.,
            },
            TrajectoryPoint {
                position: Vec2::new(20., 0.),
                speed: 5.,
                time: 4.,
            },
        ];
        let object = Prediction {
            id: 1,
            positions: vec![Vec2::new(10., 2.1); 41],
            radius: 0.5,
            additional_radius_m: vec![],
            dt: 0.2,
        };
        assert!(first_contact_time(&path, &object, 1., Vec2::new(1., 0.)).is_none());
        assert!(with_minimum_clearance(&path, &object, 1., Vec2::new(1., 0.), 1.).is_some());
        let hold = [
            TrajectoryPoint {
                position: Vec2::default(),
                speed: 0.,
                time: 0.,
            },
            TrajectoryPoint {
                position: Vec2::default(),
                speed: 0.,
                time: 8.,
            },
        ];
        let crossing = Prediction {
            positions: (0..=40)
                .map(|i| Vec2::new(2.6, 4. - i as f64 * 0.2))
                .collect(),
            ..object
        };
        assert!(with_minimum_clearance(&hold, &crossing, 1., Vec2::new(1., 0.), 1.).is_some());
    }

    #[test]
    fn the_clearance_floor_covers_an_interior_radius_peak_in_the_full_hold() {
        let hold = path(Vec2::default(), Vec2::default(), [0., 8.]);
        let object = Prediction {
            id: 1,
            positions: vec![Vec2::new(4., 0.); 3],
            radius: 0.5,
            additional_radius_m: vec![0., 2., 0.],
            dt: 4.,
        };
        assert!(with_minimum_clearance(&hold, &object, 1., Vec2::new(1., 0.), 1.).is_some());
    }

    fn path(start: Vec2, end: Vec2, times: [f64; 2]) -> [TrajectoryPoint; 2] {
        [
            TrajectoryPoint {
                position: start,
                speed: 0.0,
                time: times[0],
            },
            TrajectoryPoint {
                position: end,
                speed: 0.0,
                time: times[1],
            },
        ]
    }

    #[test]
    fn radius_growth_and_an_interior_peak_block_a_stationary_hold() {
        let hold = path(Vec2::default(), Vec2::default(), [0., 8.]);
        let mut object = Prediction {
            id: 9,
            positions: vec![Vec2::new(5., 0.); 3],
            radius: 0.5,
            additional_radius_m: vec![],
            dt: 4.,
        };
        assert_eq!(
            first_contact_time(&hold, &object, 1., Vec2::new(1., 0.)),
            None
        );
        for envelope in [vec![0., 2., 4.], vec![0., 4., 0.]] {
            object.additional_radius_m = envelope;
            assert!(first_contact_time(&hold, &object, 1., Vec2::new(1., 0.)).is_some());
        }
    }

    #[test]
    fn analytic_entry_tangency_overlap_and_miss() {
        assert_eq!(
            contact_fraction(Vec2::new(-5.0, 0.0), Vec2::new(5.0, 0.0), 1.0),
            Some(0.4)
        );
        assert_eq!(
            contact_fraction(Vec2::new(-5.0, 1.0), Vec2::new(5.0, 1.0), 1.0),
            Some(0.5)
        );
        assert_eq!(
            contact_fraction(Vec2::default(), Vec2::new(5.0, 0.0), 1.0),
            Some(0.0)
        );
        assert_eq!(
            contact_fraction(Vec2::new(-5.0, 2.0), Vec2::new(5.0, 2.0), 1.0),
            None
        );
        assert_eq!(
            contact_fraction(Vec2::new(2.0, 0.0), Vec2::new(3.0, 0.0), 1.0),
            None
        );
    }

    #[test]
    fn synchronized_crossing_detected_between_clear_endpoints() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [0.0, 1.0]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, -5.0)],
            radius: 0.5,
            dt: 1.0,
        };
        assert!(ego[0].position.distance(object.positions[0]) > 2.06);
        assert!(ego[1].position.distance(object.positions[1]) > 2.06);
        let time = first_contact_time(&ego, &object, 0.5, Vec2::default()).unwrap();
        assert!((time - (0.5 - 2.06 / 200.0_f64.sqrt())).abs() < 1e-12);
    }

    #[test]
    fn parallel_and_static_traffic_keep_clearance_in_rotated_frames() {
        for rotated in [false, true] {
            let rotate = |p: Vec2| {
                if rotated { Vec2::new(-p.y, p.x) } else { p }
            };
            let ego = path(
                rotate(Vec2::default()),
                rotate(Vec2::new(10.0, 0.0)),
                [0.0, 1.0],
            );
            let parallel = Prediction {
                additional_radius_m: vec![],
                id: 1,
                positions: vec![rotate(Vec2::new(0.0, 1.5)), rotate(Vec2::new(10.0, 1.5))],
                radius: 0.5,
                dt: 1.0,
            };
            assert!(first_contact_time(&ego, &parallel, 0.5, Vec2::default()).is_none());
            let stationary = Prediction {
                additional_radius_m: vec![],
                id: 2,
                positions: vec![rotate(Vec2::new(5.0, 1.5))],
                radius: 0.5,
                dt: 1.0,
            };
            assert!(first_contact_time(&ego, &stationary, 0.5, Vec2::default()).is_none());
        }
    }

    #[test]
    fn slow_crossing_motion_retains_a_meter_of_directional_reserve() {
        for angle in [0., 0.7, 2.4] {
            let ego = path(Vec2::default(), Vec2::default(), [0., 1.]);
            let heading = Vec2::new(1., 0.).rotated(angle);
            for speed in [0.1, 0.2, 0.5, 2.5] {
                let crossing = Prediction {
                    additional_radius_m: vec![],
                    id: 1,
                    positions: vec![
                        Vec2::new(1.8, 0.).rotated(angle),
                        Vec2::new(1.8, speed).rotated(angle),
                    ],
                    radius: 0.5,
                    dt: 1.,
                };
                // The physical gap is 0.8 m at time zero. Measured transverse
                // motion needs the same geometric reserve at either speed.
                assert_eq!(first_contact_time(&ego, &crossing, 0.5, heading), Some(0.));
                let parallel = Prediction {
                    positions: vec![
                        Vec2::new(0., 1.8).rotated(angle),
                        Vec2::new(speed, 1.8).rotated(angle),
                    ],
                    ..crossing
                };
                assert!(first_contact_time(&ego, &parallel, 0.5, heading).is_none());
            }
        }
    }

    #[test]
    fn fast_almost_parallel_motion_keeps_the_previous_transverse_speed_reserve() {
        let ego = path(Vec2::default(), Vec2::default(), [0., 1.]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(1.48, 0.), Vec2::new(9.48, 0.5)],
            radius: 0.5,
            dt: 1.,
        };
        // The earlier 0.5 m reserve already rejects this 0.48 m initial gap.
        // A small crossing angle cannot narrow that existing speed envelope.
        assert_eq!(
            first_contact_time(&ego, &object, 0.5, Vec2::new(1., 0.)),
            Some(0.)
        );
    }

    #[test]
    fn stationary_hold_reserves_for_motion_without_an_arbitrary_heading() {
        let ego = path(Vec2::default(), Vec2::default(), [0.0, 1.0]);
        let moving = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(1.5, 5.0), Vec2::new(1.5, -5.0)],
            radius: 0.5,
            dt: 1.0,
        };
        // Actual circle separation is 0.5 m at the closest point: the baseline
        // static reserve permits it, while a moving-object hold reserve rejects it.
        assert!(first_contact_time(&ego, &moving, 0.5, Vec2::default()).is_some());
        let stationary = Prediction {
            positions: vec![Vec2::new(1.5, 0.0)],
            ..moving
        };
        assert!(first_contact_time(&ego, &stationary, 0.5, Vec2::default()).is_none());
    }

    #[test]
    fn known_held_heading_preserves_parallel_clearance_and_crossing_reserve() {
        for rotated in [false, true] {
            let rotate = |p: Vec2| if rotated { Vec2::new(-p.y, p.x) } else { p };
            let ego = path(Vec2::default(), Vec2::default(), [0.0, 1.0]);
            let heading = rotate(Vec2::new(1.0, 0.0));
            let parallel = Prediction {
                additional_radius_m: vec![],
                id: 1,
                positions: vec![rotate(Vec2::new(-5.0, 1.5)), rotate(Vec2::new(5.0, 1.5))],
                radius: 0.5,
                dt: 1.0,
            };
            assert!(first_contact_time(&ego, &parallel, 0.5, heading).is_none());
            assert!(first_contact_time(&ego, &parallel, 0.5, Vec2::default()).is_some());
            let crossing = Prediction {
                positions: vec![rotate(Vec2::new(1.5, -5.0)), rotate(Vec2::new(1.5, 5.0))],
                ..parallel
            };
            assert!(first_contact_time(&ego, &crossing, 0.5, heading).is_some());
        }
    }

    #[test]
    fn geometric_crossing_at_different_times_is_clear() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [0.0, 1.0]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, 0.0)],
            radius: 0.5,
            dt: 1.0,
        };
        assert!(first_contact_time(&ego, &object, 0.5, Vec2::default()).is_none());
    }

    #[test]
    fn intermediate_prediction_turn_is_not_replaced_with_an_endpoint_chord() {
        let ego = path(Vec2::default(), Vec2::default(), [0.0, 1.0]);
        let object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(-5.0, 2.0), Vec2::default(), Vec2::new(-5.0, 2.0)],
            radius: 0.5,
            dt: 0.5,
        };
        assert!(first_contact_time(&ego, &object, 0.2, Vec2::default()).unwrap() < 0.5);
    }

    #[test]
    fn terminal_prediction_and_single_point_obstacles_remain_occupied() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [2.0, 3.0]);
        let mut object = Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, 0.0)],
            radius: 0.5,
            dt: 0.5,
        };
        assert!(first_contact_time(&ego, &object, 0.2, Vec2::default()).is_some());
        object.positions = vec![Vec2::new(5.0, 0.0)];
        assert!(first_contact_time(&ego, &object, 0.2, Vec2::default()).is_some());
    }
}
