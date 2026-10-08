//! Piecewise-linear, synchronized circular sweeps for candidate trajectory validation.
use rustdrive_core::{Prediction, TrajectoryPoint, Vec2};

/// Earliest contact under the supplied time parameterization. Inputs are validated by
/// the planner. Prediction knots are linearly interpolated, then held at the last point.
/// Each subinterval uses its end-time uncertainty margin, conservatively covering the
/// increasing envelope. This does not validate the vehicle's longitudinal feasibility.
pub(super) fn first_contact_time(
    path: &[TrajectoryPoint],
    object: &Prediction,
    vehicle_radius: f64,
) -> Option<f64> {
    let first = path.first()?;
    if first.position.distance(position_at(object, first.time))
        <= vehicle_radius + object.radius + margin(first.time)
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
                if let Some(time) = segment_contact(a, b, object, vehicle_radius, start, end) {
                    return Some(time);
                }
                start = end;
            }
        }
        if let Some(time) = segment_contact(a, b, object, vehicle_radius, start, b.time) {
            return Some(time);
        }
    }
    None
}

fn margin(time: f64) -> f64 {
    0.30 + 0.06 * time.min(5.0)
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
    vehicle_radius: f64,
    start: f64,
    end: f64,
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
    contact_fraction(
        relative_start,
        relative_end,
        // Constant acceleration along a segment differs from its temporal chord
        // by at most |delta_v| * duration / 8. Inflate the circle by this bound.
        vehicle_radius
            + object.radius
            + margin(end)
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
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, -5.0)],
            radius: 0.5,
            dt: 1.0,
        };
        assert!(ego[0].position.distance(object.positions[0]) > 1.36);
        assert!(ego[1].position.distance(object.positions[1]) > 1.36);
        let time = first_contact_time(&ego, &object, 0.5).unwrap();
        assert!((time - (0.5 - 1.36 / 200.0_f64.sqrt())).abs() < 1e-12);
    }

    #[test]
    fn geometric_crossing_at_different_times_is_clear() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [0.0, 1.0]);
        let object = Prediction {
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, 0.0)],
            radius: 0.5,
            dt: 1.0,
        };
        assert!(first_contact_time(&ego, &object, 0.5).is_none());
    }

    #[test]
    fn intermediate_prediction_turn_is_not_replaced_with_an_endpoint_chord() {
        let ego = path(Vec2::default(), Vec2::default(), [0.0, 1.0]);
        let object = Prediction {
            id: 1,
            positions: vec![Vec2::new(-5.0, 2.0), Vec2::default(), Vec2::new(-5.0, 2.0)],
            radius: 0.5,
            dt: 0.5,
        };
        assert!(first_contact_time(&ego, &object, 0.2).unwrap() < 0.5);
    }

    #[test]
    fn terminal_prediction_and_single_point_obstacles_remain_occupied() {
        let ego = path(Vec2::default(), Vec2::new(10.0, 0.0), [2.0, 3.0]);
        let mut object = Prediction {
            id: 1,
            positions: vec![Vec2::new(5.0, 5.0), Vec2::new(5.0, 0.0)],
            radius: 0.5,
            dt: 0.5,
        };
        assert!(first_contact_time(&ego, &object, 0.2).is_some());
        object.positions = vec![Vec2::new(5.0, 0.0)];
        assert!(first_contact_time(&ego, &object, 0.2).is_some());
    }
}
