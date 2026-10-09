//! Arc-length speed envelopes with constant-acceleration arrival times.
use rustdrive_core::{TrajectoryPoint, Vec2};

pub(super) struct Limits {
    pub acceleration: f64,
    pub deceleration: f64,
    pub lateral: Option<f64>,
    pub cruise: f64,
}

/// Build a reachable envelope. None means the measured initial speed cannot satisfy
/// a downstream bound using calibrated braking; callers must reject that candidate.
pub(super) fn profile(
    geometry: &[Vec2],
    initial_speed: f64,
    limits: &Limits,
    stop_distance: Option<f64>,
) -> Option<Vec<TrajectoryPoint>> {
    let mut positions = vec![*geometry.first()?];
    let mut distances = vec![0.0];
    for pair in geometry.windows(2) {
        let length = pair[0].distance(pair[1]);
        if length < 1e-8 {
            continue;
        }
        let start = *distances.last()?;
        if let Some(stop) = stop_distance.filter(|stop| *stop <= start + length) {
            if stop > start + 1e-8 {
                positions
                    .push(pair[0].plus(pair[1].minus(pair[0]).scaled((stop - start) / length)));
                distances.push(stop);
            }
            break;
        }
        positions.push(pair[1]);
        distances.push(start + length);
    }
    let mut hard_caps = vec![f64::INFINITY; positions.len()];
    if let Some(lateral) = limits.lateral {
        for (i, p) in positions.windows(3).enumerate() {
            let a = p[1].minus(p[0]);
            let b = p[2].minus(p[1]);
            let c = p[2].minus(p[0]);
            let product = a.x.hypot(a.y) * b.x.hypot(b.y) * c.x.hypot(c.y);
            if product > 1e-9 {
                let curvature = 2.0 * (a.x * b.y - a.y * b.x).abs() / product;
                if curvature > 1e-8 {
                    let cap = (lateral / curvature).sqrt();
                    // Cover the local bend at both ends of its adjoining intervals.
                    for speed in &mut hard_caps[i..=i + 2] {
                        *speed = speed.min(cap);
                    }
                }
            }
        }
    }
    if stop_distance.is_some() {
        *hard_caps.last_mut()? = 0.0;
    }
    // Hard feasibility uses the full supplied authority. Desired profiles reserve
    // 20% braking headroom for observation noise and replanning/tracking errors.
    let mut desired: Vec<f64> = hard_caps.iter().map(|cap| cap.min(limits.cruise)).collect();
    for i in (0..hard_caps.len() - 1).rev() {
        let ds = distances[i + 1] - distances[i];
        hard_caps[i] =
            hard_caps[i].min((hard_caps[i + 1].powi(2) + 2.0 * limits.deceleration * ds).sqrt());
        desired[i] =
            desired[i].min((desired[i + 1].powi(2) + 2.0 * 0.8 * limits.deceleration * ds).sqrt());
    }
    if initial_speed > hard_caps[0] + 1e-6 {
        return None;
    }
    let mut speeds = vec![initial_speed; positions.len()];
    for i in 1..speeds.len() {
        let ds = distances[i] - distances[i - 1];
        let lower = (speeds[i - 1].powi(2) - 2.0 * limits.deceleration * ds)
            .max(0.0)
            .sqrt();
        let upper = (speeds[i - 1].powi(2) + 2.0 * limits.acceleration * ds)
            .sqrt()
            .min(hard_caps[i]);
        if lower > upper + 1e-6 {
            return None;
        }
        speeds[i] = desired[i].min(upper).max(lower.min(upper));
    }
    let mut points = vec![TrajectoryPoint {
        position: positions[0],
        speed: speeds[0],
        time: 0.0,
    }];
    for i in 1..positions.len() {
        let sum = speeds[i - 1] + speeds[i];
        if sum < 1e-8 {
            break; // Never invent motion between two stationary states.
        }
        let time = points.last()?.time + 2.0 * (distances[i] - distances[i - 1]) / sum;
        points.push(TrajectoryPoint {
            position: positions[i],
            speed: speeds[i],
            time,
        });
    }
    if points.last()?.speed < 1e-8 {
        let last = *points.last()?;
        points.push(TrajectoryPoint {
            time: last.time + 8.0,
            ..last
        });
    }
    Some(points)
}

/// Arc distance at a time, using each segment's constant acceleration.
pub(super) fn distance_at(points: &[TrajectoryPoint], time: f64) -> f64 {
    let mut distance = 0.0;
    for pair in points.windows(2) {
        let length = pair[0].position.distance(pair[1].position);
        if time <= pair[1].time {
            let elapsed = (time - pair[0].time).max(0.0);
            let acceleration = (pair[1].speed - pair[0].speed) / (pair[1].time - pair[0].time);
            return distance
                + (pair[0].speed * elapsed + 0.5 * acceleration * elapsed.powi(2))
                    .clamp(0.0, length);
        }
        distance += length;
    }
    distance
}

#[cfg(test)]
mod tests {
    use super::*;
    fn line() -> Vec<Vec2> {
        (0..=80).map(|i| Vec2::new(i as f64 * 0.5, 0.0)).collect()
    }
    fn limits() -> Limits {
        Limits {
            acceleration: 2.0,
            deceleration: 2.5,
            lateral: None,
            cruise: 8.0,
        }
    }
    #[test]
    fn acceleration_from_rest_has_analytic_arrival_times() {
        let path = profile(&line(), 0.0, &limits(), None).unwrap();
        for point in &path[..=32] {
            assert!((point.speed - (4.0 * point.position.x).sqrt()).abs() < 1e-10);
            assert!((point.time - point.position.x.sqrt()).abs() < 1e-10);
        }
        assert!((path.last().unwrap().time - 7.0).abs() < 1e-10);
        assert!((distance_at(&path, 0.5) - 0.25).abs() < 1e-10);
    }
    #[test]
    fn stop_has_exact_location_finite_time_and_stationary_hold() {
        let path = profile(&line(), 6.0, &limits(), Some(12.3)).unwrap();
        let last = path.last().unwrap();
        assert!((last.position.x - 12.3).abs() < 1e-10);
        assert_eq!(last.speed, 0.0);
        for pair in path.windows(2) {
            assert!(pair[1].time.is_finite() && pair[1].time > pair[0].time);
            let ds = pair[1].position.distance(pair[0].position);
            if ds > 0.0 {
                let acceleration = (pair[1].speed.powi(2) - pair[0].speed.powi(2)) / (2.0 * ds);
                assert!((-2.5 - 1e-10..=2.0 + 1e-10).contains(&acceleration));
            }
        }
        assert_eq!(path[path.len() - 2].position, last.position);
    }
    #[test]
    fn unreachable_braking_bound_is_rejected_and_rest_does_not_move() {
        assert!(profile(&line(), 8.0, &limits(), Some(5.0)).is_none());
        let path = profile(&line(), 0.0, &limits(), Some(0.0)).unwrap();
        assert_eq!(path.len(), 2);
        assert_eq!(path[0].position, path[1].position);
        assert_eq!(path[1].time, 8.0);
    }
    #[test]
    fn nominal_stop_reserves_braking_authority_without_weakening_the_hard_bound() {
        let path = profile(&line(), 2.0, &limits(), Some(20.0)).unwrap();
        let peak_braking = path
            .windows(2)
            .map(|p| (p[0].speed - p[1].speed) / (p[1].time - p[0].time))
            .fold(0.0_f64, f64::max);
        assert!((peak_braking - 2.0).abs() < 1e-9);
        // A state above the comfortable stopping envelope is still physically
        // feasible: recover using full authority, preserving the measured speed.
        let path = profile(&line(), 7.0, &limits(), Some(10.0)).unwrap();
        assert_eq!(path[0].speed, 7.0);
        let first_braking = (path[0].speed - path[1].speed) / path[1].time;
        assert!((first_braking - 2.5).abs() < 1e-9);
        assert!(profile(&line(), 7.1, &limits(), Some(10.0)).is_none());
    }
    #[test]
    fn noisy_overspeed_recovers_without_an_instantaneous_speed_clamp() {
        let path = profile(&line(), 8.02, &limits(), None).unwrap();
        assert_eq!(path[0].speed, 8.02);
        assert_eq!(path[1].speed, 8.0);
        let acceleration = (path[1].speed - path[0].speed) / path[1].time;
        assert!((-2.5..=0.0).contains(&acceleration));
    }
    #[test]
    fn a_local_curve_cap_allows_acceleration_after_the_curve() {
        let geometry: Vec<_> = (0..=80)
            .map(|i| {
                let x = i as f64 * 0.5;
                Vec2::new(
                    x,
                    if x < 10.0 {
                        1.0 - (x * 0.3).cos()
                    } else {
                        1.0 - 3.0_f64.cos()
                    },
                )
            })
            .collect();
        let limits = Limits {
            lateral: Some(0.6),
            ..limits()
        };
        let path = profile(&geometry, 0.0, &limits, None).unwrap();
        assert!(path[4].speed < 2.9);
        assert!(path.last().unwrap().speed > 7.9);
    }
}
