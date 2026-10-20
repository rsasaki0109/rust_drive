//! Bounded alternatives to a rejected collision-retimed stopping position.
use crate::speed::{self, Limits};
use rustdriving_core::{TrajectoryPoint, Vec2};

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
