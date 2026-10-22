//! Bounded alternatives to a rejected collision-retimed stopping position.
use crate::speed::{self, Limits};
use rustdriving_core::{TrajectoryPoint, Vec2};

/// Descending finite proposals beyond the heuristic stop. Each includes its
/// entire reachable braking path and stationary tail; moving-object feasibility
/// is not monotone in stopping distance. None preserves the ordinary fallback.
pub(super) fn later_profile(
    geometry: &[Vec2],
    initial_speed: f64,
    limits: &Limits<'_>,
    proposed_stop: f64,
    mut clear: impl FnMut(&[TrajectoryPoint]) -> bool,
) -> Option<Vec<TrajectoryPoint>> {
    if !(2..=81).contains(&geometry.len()) {
        return None;
    }
    let mut distance = 0.;
    let mut stops = Vec::with_capacity(geometry.len() - 1);
    for pair in geometry.windows(2) {
        distance += pair[0].distance(pair[1]);
        stops.push(distance);
    }
    if !proposed_stop.is_finite()
        || proposed_stop < 0.
        || !initial_speed.is_finite()
        || initial_speed < 0.
        || !distance.is_finite()
        || distance <= proposed_stop
    {
        return None;
    }
    for stop in stops.into_iter().rev() {
        if stop <= proposed_stop {
            break;
        }
        if let Some(path) = speed::profile(geometry, initial_speed, limits, Some(stop))
            && (path
                .windows(2)
                .map(|p| p[0].position.distance(p[1].position))
                .sum::<f64>()
                - stop)
                .abs()
                <= 1e-7 * stop.max(1.)
            && clear(&path)
        {
            return Some(path);
        }
    }
    None
}

/// Preserve the ordinary proposal first. Every alternative rebuilds the actual
/// reachable speed profile and sweeps its braking segments and eight-second
/// stationary tail. Moving traffic makes feasibility non-monotonic in distance;
/// no binary-search or endpoint-only clearance assumption is used.
pub(super) fn profile(
    geometry: &[Vec2],
    initial_speed: f64,
    limits: &Limits<'_>,
    proposed_stop: f64,
    search: bool,
    mut clear: impl FnMut(&[TrajectoryPoint]) -> bool,
) -> Option<(Vec<TrajectoryPoint>, bool)> {
    if let Some(path) = speed::profile(geometry, initial_speed, limits, Some(proposed_stop))
        && clear(&path)
    {
        return Some((path, false));
    }
    if !search {
        return None;
    }
    // Round slightly outward at positive speeds so the braking endpoint is
    // representable without an instantaneous speed clamp. Never extend the
    // original stopping proposal or change the supplied braking authority.
    let minimum = if initial_speed == 0.0 {
        0.0
    } else {
        (initial_speed.powi(2) / (2.0 * limits.deceleration) * (1.0 + 1e-12)).max(1e-7)
    };
    if !minimum.is_finite() || minimum >= proposed_stop {
        return None;
    }
    // Reuse the existing four-meter stopping heuristic after rejected retiming.
    // It proposes locations; only the unchanged complete sweep decides safety.
    let upper = (proposed_stop - 4.0).max(minimum);
    let fallback = Limits {
        zones: limits.zones,
        acceleration: limits.acceleration,
        deceleration: limits.deceleration,
        lateral: limits.lateral,
        cruise: if initial_speed > 0.2 {
            limits.cruise.min(initial_speed)
        } else {
            limits.cruise
        },
    };
    for index in 0..if upper == minimum { 1 } else { 16 } {
        let stop = upper + (minimum - upper) * index as f64 / 15.0;
        if let Some(path) = speed::profile(geometry, initial_speed, &fallback, Some(stop))
            && clear(&path)
        {
            return Some((path, true));
        }
    }
    None
}

/// Knots are uniform in route progress, rather than in actual candidate arc.
pub(super) fn distance_at_progress(geometry: &[Vec2], horizon: f64, progress: f64) -> f64 {
    if horizon <= 0.0 {
        return 0.0;
    }
    let step =
        (progress / horizon * (geometry.len() - 1) as f64).clamp(0.0, (geometry.len() - 1) as f64);
    geometry
        .windows(2)
        .enumerate()
        .map(|(i, p)| p[0].distance(p[1]) * (step - i as f64).clamp(0.0, 1.0))
        .sum()
}

pub(super) fn progress_at_distance(geometry: &[Vec2], horizon: f64, mut distance: f64) -> f64 {
    for (index, pair) in geometry.windows(2).enumerate() {
        let length = pair[0].distance(pair[1]);
        if distance <= length && length > 0.0 {
            return horizon * (index as f64 + distance / length) / (geometry.len() - 1) as f64;
        }
        distance -= length;
    }
    horizon
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> Limits<'static> {
        Limits {
            zones: None,
            acceleration: 2.,
            deceleration: 2.5,
            lateral: None,
            cruise: 8.,
        }
    }
    fn line() -> Vec<Vec2> {
        (0..=80).map(|i| Vec2::new(i as f64 * 0.5, 0.)).collect()
    }

    #[test]
    fn later_search_visits_existing_knots_in_descending_order_and_never_exceeds_eighty_sweeps() {
        let mut endpoints = vec![];
        assert!(
            later_profile(&line(), 1., &limits(), 0., |path| {
                let last = path.last().unwrap();
                let before = path[path.len() - 2];
                assert_eq!(last.speed, 0.);
                assert_eq!(last.position, before.position);
                assert!(last.time - before.time >= 8. - 1e-9);
                endpoints.push(last.position.x);
                false
            })
            .is_none()
        );
        assert_eq!(endpoints.len(), 80);
        assert_eq!(endpoints[0], 40.);
        assert_eq!(endpoints[79], 0.5);
        assert!(endpoints.windows(2).all(|p| p[0] > p[1]));
        let path = later_profile(&line(), 1., &limits(), 10., |p| {
            p.last().unwrap().position.x == 25.5
        })
        .unwrap();
        assert_eq!(path.last().unwrap().position.x, 25.5);
    }

    #[test]
    fn a_later_retimed_crossing_stop_can_be_safe_when_the_heuristic_is_unreachable() {
        use rustdriving_core::Prediction;
        let obstacle = Prediction {
            id: 7,
            positions: (0..=40)
                .map(|i| Vec2::new(20., -5. + i as f64 * 0.4))
                .collect(),
            radius: 1.,
            additional_radius_m: vec![],
            dt: 0.2,
        };
        let clear = |path: &[TrajectoryPoint]| {
            crate::collision::first_contact_time(path, &obstacle, 1.25, Vec2::new(1., 0.)).is_none()
        };
        let nominal = speed::profile(&line(), 8., &limits(), None).unwrap();
        assert!(!clear(&nominal));
        assert!(profile(&line(), 8., &limits(), 10., true, clear).is_none());
        let path = later_profile(&line(), 8., &limits(), 10., clear).unwrap();
        assert!(path.last().unwrap().position.x > 12.8);
        for pair in path.windows(2) {
            let a = pair[0];
            let b = pair[1];
            let duration = b.time - a.time;
            let acceleration = (b.speed - a.speed) / duration;
            assert!((-2.5 - 1e-9..=2. + 1e-9).contains(&acceleration));
            for i in 0..=100 {
                let dt = duration * i as f64 / 100.;
                let time = a.time + dt;
                let x = a.position.x + a.speed * dt + 0.5 * acceleration * dt * dt;
                let y = -5. + 2. * time.min(8.);
                let margin = 0.3 + 0.06 * time.min(5.) + if time < 8. { 0.7 } else { 0. };
                assert!(Vec2::new(x, 0.).distance(Vec2::new(20., y)) > 2.25 + margin);
            }
        }
    }

    #[test]
    fn malformed_unreachable_or_oversized_later_searches_never_invoke_clearance() {
        let bad = [Vec2::default(), Vec2::new(f64::NAN, 0.)];
        for (geometry, initial, stop) in [
            (&line()[..], f64::NAN, 1.),
            (&line()[..], -1., 1.),
            (&line()[..], 1., f64::NAN),
            (&line()[..], 1., -1.),
            (&bad[..], 1., 1.),
            (&[Vec2::default()][..], 1., 0.),
        ] {
            assert!(
                later_profile(geometry, initial, &limits(), stop, |_| panic!(
                    "invalid search reached clearance"
                ))
                .is_none()
            );
        }
        assert!(
            later_profile(&vec![Vec2::default(); 82], 1., &limits(), 0., |_| panic!(
                "oversized search"
            ))
            .is_none()
        );
        assert!(
            later_profile(&line()[..3], 8., &limits(), 0., |_| panic!(
                "unreachable braking"
            ))
            .is_none()
        );
    }

    #[test]
    fn a_stationary_prefix_cannot_be_reported_as_reaching_a_later_endpoint() {
        assert!(
            later_profile(
                &[Vec2::default(), Vec2::new(0.5, 0.)],
                0.,
                &limits(),
                0.,
                |_| panic!("stationary prefix passed as later stop")
            )
            .is_none()
        );
    }

    #[test]
    fn a_nonuniform_corrected_path_preserves_the_original_progress_anchor() {
        let geometry = [
            Vec2::default(),
            Vec2::new(3., 0.),
            Vec2::new(3., 4.),
            Vec2::new(9., 4.),
        ];
        assert_eq!(distance_at_progress(&geometry, 30., 15.), 5.);
        assert_eq!(progress_at_distance(&geometry, 30., 5.), 15.);
        assert_eq!(distance_at_progress(&geometry, 30., -1.), 0.);
        assert_eq!(distance_at_progress(&geometry, 30., 100.), 13.);
        for i in 0..=300 {
            let progress = i as f64 * 0.1;
            let roundtrip = progress_at_distance(
                &geometry,
                30.,
                distance_at_progress(&geometry, 30., progress),
            );
            assert!((roundtrip - progress).abs() < 1e-12);
        }
    }

    #[test]
    fn exhaustive_rejection_cannot_exceed_seventeen_complete_sweeps() {
        let geometry: Vec<_> = (0..=80).map(|i| Vec2::new(i as f64 * 0.5, 0.)).collect();
        let limits = Limits {
            zones: None,
            acceleration: 2.,
            deceleration: 2.5,
            lateral: None,
            cruise: 8.,
        };
        let mut calls = 0;
        assert!(
            profile(&geometry, 2., &limits, 30., true, |_| {
                calls += 1;
                false
            })
            .is_none()
        );
        assert_eq!(calls, 17);
    }
}
