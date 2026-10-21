use rustdriving_core::{
    ForecastConsistencyFault as Fault, ForecastConsistencyState as State, Prediction, Track,
    TrackMeasurement, Vec2,
};
use rustdriving_prediction::ForecastConsistencyGuard;

fn sample(id: u64, stamp: f64, x: f64, radius: f64) -> (Track, TrackMeasurement, Prediction) {
    let position = Vec2::new(x, 0.);
    (
        Track {
            id,
            position,
            velocity: Vec2::default(),
            radius: 1.,
            last_seen: stamp,
        },
        TrackMeasurement {
            id,
            stamp,
            center: position,
            radius: 1.,
        },
        Prediction {
            id,
            positions: vec![position; 41],
            radius,
            additional_radius_m: vec![],
            dt: 0.2,
        },
    )
}
fn observe(g: &mut ForecastConsistencyGuard, now: f64, stamp: f64, x: f64, radius: f64) {
    let (t, m, p) = sample(9, stamp, x, radius);
    g.observe(now, Some(stamp), &[m], &[t], &[p], true);
}
fn breached() -> ForecastConsistencyGuard {
    let mut g = ForecastConsistencyGuard::default();
    observe(&mut g, 0., 0., 0., 1.25);
    observe(&mut g, 0.1, 0.1, 0.4, 1.25);
    g
}

#[test]
fn a_measured_disc_miss_brakes_until_three_new_consistent_acquisitions_span_two_tenths() {
    let mut g = breached();
    assert!(g.blocking());
    assert_eq!(g.diagnostics().fault, Some(Fault::MeasuredMiss));
    let miss = &g.diagnostics().misses[0];
    assert!((miss.shortfall_m - 0.15).abs() < 1e-12);
    assert_eq!(miss.reference_origin_s, 0.);
    assert_eq!(miss.measurement_acquisition_s, 0.1);
    observe(&mut g, 0.2, 0.2, 0.4, 1.25);
    assert_eq!(g.diagnostics().state, State::Confirming);
    assert_eq!(g.diagnostics().recovery_samples, 1);
    observe(&mut g, 0.25, 0.2, 0.4, 1.25);
    assert_eq!(g.diagnostics().recovery_samples, 1);
    observe(&mut g, 0.3, 0.3, 0.4, 1.25);
    assert!(g.blocking());
    observe(&mut g, 0.4, 0.4, 0.4, 1.25);
    assert!(!g.blocking());
    assert_eq!(g.diagnostics().state, State::Observing);
    assert!(g.diagnostics().held_identities.is_empty());
}

#[test]
fn delayed_measurements_use_an_eligible_old_origin_and_cannot_clear_before_the_fault_clock() {
    let mut g = ForecastConsistencyGuard::default();
    observe(&mut g, 0.2, 0., 0., 1.25);
    observe(&mut g, 0.3, 0.1, 0.4, 2.);
    assert!(!g.blocking());
    assert_eq!(g.diagnostics().unmonitored_measurements, 1);
    observe(&mut g, 0.4, 0.2, 0.8, 2.);
    assert!(g.blocking());
    let miss = &g.diagnostics().misses[0];
    assert_eq!(miss.reference_origin_s, 0.2);
    assert_eq!(miss.reference_acquisition_s, 0.);
    assert!((miss.shortfall_m - 0.55).abs() < 1e-12);
    for (now, stamp) in [(0.5, 0.3), (0.6, 0.4)] {
        observe(&mut g, now, stamp, 0.8, 2.);
        assert_eq!(g.diagnostics().recovery_samples, 0);
    }
    for (now, stamp) in [(0.7, 0.5), (0.8, 0.6), (0.9, 0.7)] {
        observe(&mut g, now, stamp, 0.8, 2.);
    }
    assert!(!g.blocking());
}

#[test]
fn interpolation_uses_the_complete_disc_and_never_extends_the_forecast_horizon() {
    for (measured_radius, blocked) in [(1.4, false), (1.6, true)] {
        let mut g = ForecastConsistencyGuard::default();
        let (t, m, mut p) = sample(9, 0., 0., 1.);
        p.positions = vec![Vec2::default(), Vec2::new(2., 0.)];
        p.additional_radius_m = vec![0., 1.];
        p.dt = 2.;
        g.observe(0., Some(0.), &[m], &[t], &[p], true);
        let (t, mut m, p) = sample(9, 1., 1., 2.);
        m.radius = measured_radius;
        g.observe(1., Some(1.), &[m], &[t], &[p], true);
        assert_eq!(g.blocking(), blocked);
        if blocked {
            assert!((g.diagnostics().misses[0].predicted_radius_m - 1.5).abs() < 1e-12);
        }
    }
    let mut g = ForecastConsistencyGuard::default();
    let (t, m, mut p) = sample(9, 0., 0., 1.25);
    p.positions.truncate(2);
    p.dt = 0.1;
    g.observe(0., Some(0.), &[m], &[t], &[p], true);
    observe(&mut g, 0.2, 0.2, 2., 1.25);
    assert!(!g.blocking());
    assert_eq!(g.diagnostics().unmonitored_measurements, 1);
}

#[test]
fn disappearance_or_another_identity_cannot_clear_the_responsible_track() {
    let mut g = breached();
    for stamp in [0.2, 0.3, 0.4] {
        g.observe(stamp, Some(stamp), &[], &[], &[], true);
    }
    for stamp in [0.5, 0.6, 0.7, 0.8] {
        let (t, m, p) = sample(10, stamp, 0.4, 1.25);
        g.observe(stamp, Some(stamp), &[m], &[t], &[p], true);
    }
    assert!(g.blocking());
    assert_eq!(g.diagnostics().held_identities, vec![9]);
    assert_eq!(g.diagnostics().recovery_samples, 0);
    assert!(!g.diagnostics().complete_scene_check);
}

#[test]
fn every_active_identity_must_supply_a_comparable_measurement_during_recovery() {
    let mut g = breached();
    let (t9, m9, p9) = sample(9, 0.2, 0.4, 1.25);
    let (t10, _, p10) = sample(10, 0.1, 5., 1.25);
    g.observe(0.2, Some(0.2), &[m9], &[t9, t10], &[p9, p10], true);
    assert!(g.blocking());
    assert_eq!(g.diagnostics().recovery_samples, 0);
}

#[test]
fn unhealthy_sensor_context_interrupts_votes_and_requires_later_acquisitions() {
    let mut g = breached();
    observe(&mut g, 0.2, 0.2, 0.4, 1.25);
    let (t, m, p) = sample(9, 0.2, 0.4, 1.25);
    g.observe(0.25, Some(0.2), &[m], &[t], &[p], false);
    assert_eq!(g.diagnostics().recovery_samples, 0);
    observe(&mut g, 0.3, 0.3, 0.4, 1.25);
    observe(&mut g, 0.4, 0.4, 0.4, 1.25);
    assert!(g.blocking());
    observe(&mut g, 0.5, 0.5, 0.4, 1.25);
    assert!(!g.blocking());
}

#[test]
fn a_long_confirmation_gap_restarts_support_instead_of_combining_old_votes() {
    let mut g = breached();
    observe(&mut g, 0.2, 0.2, 0.4, 1.25);
    observe(&mut g, 0.6, 0.6, 0.4, 1.25);
    assert_eq!(g.diagnostics().recovery_samples, 1);
    observe(&mut g, 0.7, 0.7, 0.4, 1.25);
    assert!(g.blocking());
    observe(&mut g, 0.8, 0.8, 0.4, 1.25);
    assert!(!g.blocking());
}

#[test]
fn motion_loss_and_clock_regression_require_a_new_epoch() {
    for motion in [true, false] {
        let mut g = breached();
        if motion {
            g.invalidate_motion_epoch(0.15);
        } else {
            observe(&mut g, 0., 0., 0., 1.25);
        }
        assert_eq!(g.diagnostics().state, State::ResetRequired);
        for stamp in [0.2, 0.3, 0.4] {
            observe(&mut g, stamp, stamp, 0.4, 1.25);
        }
        assert!(g.blocking());
    }
    assert!(!ForecastConsistencyGuard::default().blocking());
}

#[test]
fn invalid_measurements_do_not_enter_reference_history_and_capacity_is_bounded() {
    let mut g = ForecastConsistencyGuard::default();
    observe(&mut g, 0., 0., 0., 1.25);
    let (t, mut m, p) = sample(9, 0.1, 0., 1.25);
    m.center.x = f64::NAN;
    g.observe(0.1, Some(0.1), &[m], &[t], &[p], true);
    assert!(g.blocking());
    assert_eq!(g.diagnostics().retained_references, 1);
    let triples: Vec<_> = (0..129)
        .map(|id| sample(id, 0.2, id as f64, 1.25))
        .collect();
    let tracks: Vec<_> = triples.iter().map(|x| x.0.clone()).collect();
    let raw: Vec<_> = triples.iter().map(|x| x.1.clone()).collect();
    let predictions: Vec<_> = triples.iter().map(|x| x.2.clone()).collect();
    g.observe(0.2, Some(0.2), &raw, &tracks, &predictions, true);
    assert_eq!(g.diagnostics().state, State::ResetRequired);
    assert_eq!(g.diagnostics().fault, Some(Fault::Capacity));
}

#[test]
fn reference_history_has_a_fixed_bound_and_common_rigid_reprojection_preserves_misses() {
    let mut g = ForecastConsistencyGuard::default();
    for i in 0..100 {
        let stamp = i as f64 * 0.1;
        observe(&mut g, stamp, stamp, 0., 1.25);
    }
    assert_eq!(g.diagnostics().retained_references, 16);
    assert!(!g.blocking());
    for yaw in [0., 0.7, -2.1] {
        let mut transformed = ForecastConsistencyGuard::default();
        for (stamp, x) in [(0., 0.), (0.1, 0.4)] {
            let (mut t, mut m, mut p) = sample(9, stamp, x, 1.25);
            let convert = |v: Vec2| v.rotated(yaw).plus(Vec2::new(100., -50.));
            t.position = convert(t.position);
            m.center = convert(m.center);
            p.positions = p.positions.into_iter().map(convert).collect();
            transformed.observe(stamp, Some(stamp), &[m], &[t], &[p], true);
        }
        assert!((transformed.diagnostics().misses[0].shortfall_m - 0.15).abs() < 1e-12);
    }
}
