use rustdriving_core::*;
use rustdriving_prediction::{ForecastConsistencyAdmission, ForecastConsistencyGuard};

fn measured(g: &mut ForecastConsistencyGuard, t: f64, x: f64) {
    let p = Vec2::new(x, 0.);
    g.observe(
        t,
        Some(t),
        &[TrackMeasurement {
            id: 9,
            stamp: t,
            center: p,
            radius: 1.,
        }],
        &[Track {
            id: 9,
            position: p,
            velocity: Vec2::default(),
            radius: 1.,
            last_seen: t,
        }],
        &[Prediction {
            id: 9,
            positions: vec![p; 41],
            radius: 1.25,
            additional_radius_m: vec![],
            dt: 0.2,
        }],
        true,
    );
}
fn breached(enabled: bool) -> ForecastConsistencyGuard {
    let mut g = ForecastConsistencyGuard::default();
    if enabled {
        g.enable_clear_sweep_recovery();
    }
    measured(&mut g, 0., 0.);
    measured(&mut g, 0.1, 0.4);
    assert!(g.blocking());
    g
}
fn proof(t: f64) -> ClearSweepEvidence {
    let gap = std::f64::consts::TAU / 720.;
    ClearSweepEvidence {
        acquisition_s: t,
        checked_center_radius_m: 40.,
        minimum_resolved_disc_radius_m: 0.4,
        maximum_angular_gap_rad: gap,
        maximum_transverse_gap_m: 40. * (gap / 2.).sin(),
        ray_count: 720,
    }
}
fn empty(g: &mut ForecastConsistencyGuard, t: f64, healthy: bool) {
    g.observe_with_clear_sweep(
        t,
        Some(t),
        &[],
        &[],
        &[],
        ForecastConsistencyAdmission {
            upstream_healthy: healthy,
            clear_sweep: Some(proof(t)),
        },
    );
}

#[test]
fn a_cached_scan_and_a_long_gap_do_not_combine_distinct_clear_acquisitions() {
    let mut g = breached(true);
    empty(&mut g, 0.2, true);
    g.observe_with_clear_sweep(
        0.25,
        Some(0.2),
        &[],
        &[],
        &[],
        ForecastConsistencyAdmission {
            upstream_healthy: true,
            clear_sweep: None,
        },
    );
    assert_eq!(
        g.diagnostics()
            .clear_sweep
            .as_ref()
            .unwrap()
            .confirmation_samples,
        1
    );
    empty(&mut g, 0.3, true);
    empty(&mut g, 0.8, true);
    assert_eq!(
        g.diagnostics()
            .clear_sweep
            .as_ref()
            .unwrap()
            .first_acquisition_s,
        Some(0.8)
    );
    empty(&mut g, 0.9, true);
    assert!(g.blocking());
    empty(&mut g, 1., true);
    assert!(!g.blocking());
}

#[test]
fn independent_upstream_faults_reset_clear_votes_and_advance_the_control_cutoff() {
    let mut g = breached(true);
    empty(&mut g, 0.2, true);
    empty(&mut g, 0.3, true);
    empty(&mut g, 0.35, false);
    assert_eq!(
        g.diagnostics()
            .clear_sweep
            .as_ref()
            .unwrap()
            .confirmation_samples,
        0
    );
    assert_eq!(g.diagnostics().recovery_after_control_s, Some(0.35));
    for t in [0.4, 0.5, 0.6] {
        empty(&mut g, t, true);
    }
    let c = g.diagnostics().clear_sweep.as_ref().unwrap();
    assert!(!g.blocking());
    assert_eq!(c.last_release.as_ref().unwrap().identities, vec![9]);
    assert_eq!(c.last_release.as_ref().unwrap().first_acquisition_s, 0.4);
}

#[test]
fn positive_and_negative_confirmation_have_separate_support_windows() {
    let mut g = breached(true);
    measured(&mut g, 0.2, 0.4);
    // An unmatched but retained track keeps the old reference. Its stale
    // geometry cannot supply a positive observation during the empty sweep.
    let position = Vec2::new(0.4, 0.);
    g.observe_with_clear_sweep(
        0.3,
        Some(0.3),
        &[],
        &[Track {
            id: 9,
            position,
            velocity: Vec2::default(),
            radius: 1.,
            last_seen: 0.2,
        }],
        &[Prediction {
            id: 9,
            positions: vec![position; 41],
            radius: 1.25,
            additional_radius_m: vec![],
            dt: 0.2,
        }],
        ForecastConsistencyAdmission {
            upstream_healthy: true,
            clear_sweep: Some(proof(0.3)),
        },
    );
    assert_eq!(g.diagnostics().recovery_samples, 0);
    measured(&mut g, 0.4, 0.4);
    assert_eq!(g.diagnostics().recovery_samples, 1);
    assert_eq!(
        g.diagnostics()
            .clear_sweep
            .as_ref()
            .unwrap()
            .confirmation_samples,
        0
    );
    empty(&mut g, 0.5, true);
    empty(&mut g, 0.6, true);
    assert!(g.blocking());
    empty(&mut g, 0.7, true);
    assert!(!g.blocking());
}

#[test]
fn disabled_recovery_cannot_accept_absence_and_malformed_evidence_fails_closed() {
    let mut g = breached(false);
    for t in [0.2, 0.3, 0.4, 0.5] {
        empty(&mut g, t, true);
    }
    assert!(g.blocking());
    assert!(g.diagnostics().clear_sweep.is_none());
    let mut g = breached(true);
    let mut p = proof(0.2);
    p.acquisition_s = 0.1;
    g.observe_with_clear_sweep(
        0.2,
        Some(0.2),
        &[],
        &[],
        &[],
        ForecastConsistencyAdmission {
            upstream_healthy: true,
            clear_sweep: Some(p),
        },
    );
    assert!(g.blocking());
    assert_eq!(
        g.diagnostics().fault,
        Some(ForecastConsistencyFault::InvalidInput)
    );
    assert_eq!(
        g.diagnostics().clear_sweep.as_ref().unwrap().release_count,
        0
    );
}
