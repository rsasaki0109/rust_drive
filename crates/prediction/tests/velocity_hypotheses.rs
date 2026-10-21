use rustdriving_core::{ForecastEnvelopeConfig, Prediction, Track, Vec2};
use rustdriving_prediction::{ObservedBraking, RecentVelocityHypotheses};

fn bounds() -> ForecastEnvelopeConfig {
    ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    }
}
fn track(time: f64, velocity: Vec2) -> Track {
    Track {
        id: 1,
        position: Vec2::new(10., -7.),
        velocity,
        radius: 1.,
        last_seen: time,
    }
}
fn predict(t: &Track, now: f64) -> Vec<Prediction> {
    ObservedBraking::default()
        .with_envelope(bounds())
        .unwrap()
        .predict(std::slice::from_ref(t), now)
}
fn assert_same(a: &[Prediction], b: &[Prediction]) {
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(b) {
        assert_eq!(
            (a.id, &a.positions, a.radius, a.dt, &a.additional_radius_m),
            (b.id, &b.positions, b.radius, b.dt, &b.additional_radius_m)
        );
    }
}
fn observe(
    m: &mut RecentVelocityHypotheses,
    t: &Track,
    now: f64,
    supported: bool,
    failed: bool,
) -> Vec<Prediction> {
    let mut p = predict(t, now);
    m.apply(
        std::slice::from_ref(t),
        if supported { &[1] } else { &[] },
        Some(t.last_seen),
        now,
        failed,
        &mut p,
    )
    .unwrap();
    p
}

#[test]
fn every_recent_velocity_point_disc_fits_at_and_between_knots_in_any_frame() {
    for yaw in [0., 0.7, -1.8] {
        let velocities =
            [Vec2::new(-1., 0.), Vec2::new(0.2, 1.5), Vec2::new(1.3, 0.)].map(|v| v.rotated(yaw));
        let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
        for (i, v) in velocities.iter().enumerate() {
            observe(
                &mut m,
                &track(i as f64 * 0.1, *v),
                i as f64 * 0.1,
                true,
                false,
            );
        }
        let t = track(0.2, velocities[2]);
        let old = predict(&t, 0.25);
        let p = observe(&mut m, &t, 0.25, true, false);
        assert_eq!(p[0].positions, old[0].positions);
        assert_eq!(p[0].radius, old[0].radius);
        assert_eq!(m.diagnostics().tracks[0].sample_count, 3);
        for k in 0..=160 {
            let time = k as f64 * 0.05;
            let i = ((time / 0.2).floor() as usize).min(39);
            let fraction = time / 0.2 - i as f64;
            let center = p[0].positions[i]
                .scaled(1. - fraction)
                .plus(p[0].positions[i + 1].scaled(fraction));
            for velocity in velocities {
                for direction in 0..16 {
                    let a = std::f64::consts::TAU * direction as f64 / 16.;
                    let physical = t
                        .position
                        .plus(velocity.scaled(0.05 + time))
                        .plus(Vec2::new(a.cos(), a.sin()).scaled(0.2));
                    assert!(center.distance(physical) + t.radius <= p[0].radius_at(time) + 1e-8);
                }
            }
        }
    }
}

#[test]
fn already_enclosed_low_velocity_alternatives_preserve_every_forecast_value() {
    let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
    for (i, speed) in [0.02, -0.02, 0.01].into_iter().enumerate() {
        let t = track(i as f64 * 0.1, Vec2::new(speed, 0.));
        let old = predict(&t, t.last_seen);
        let p = observe(&mut m, &t, t.last_seen, true, false);
        assert_same(&p, &old);
    }
}

#[test]
fn cached_ticks_consume_the_window_and_never_add_samples() {
    let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
    for i in 0..=10 {
        let time = i as f64 * 0.1;
        observe(&mut m, &track(time, Vec2::new(1., 0.)), time, true, false);
    }
    assert_eq!(m.diagnostics().tracks[0].sample_count, 9);
    let t = track(1., Vec2::new(1., 0.));
    observe(&mut m, &t, 1.05, true, false);
    let report = &m.diagnostics().tracks[0];
    assert_eq!(report.sample_count, 8);
    assert_eq!(report.first_acquisition_s, Some(3. * 0.1));
    assert_eq!(report.last_acquisition_s, Some(1.));
    observe(&mut m, &t, 1.36, true, false);
    assert_eq!(m.diagnostics().tracks[0].sample_count, 0);
}

#[test]
fn unsupported_or_cached_velocity_cannot_seed_history() {
    let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
    let t = track(0., Vec2::new(2., 0.));
    observe(&mut m, &t, 0., false, false);
    observe(&mut m, &t, 0.05, true, false);
    assert_eq!(m.diagnostics().tracks[0].sample_count, 0);
    observe(&mut m, &track(0.1, Vec2::default()), 0.1, true, false);
    assert_eq!(m.diagnostics().tracks[0].sample_count, 1);
    let t = track(0.2, Vec2::default());
    observe(&mut m, &t, 0.2, false, false);
    observe(&mut m, &t, 0.25, true, false);
    assert_eq!(m.diagnostics().tracks[0].sample_count, 0);
    observe(&mut m, &track(0.3, Vec2::default()), 0.3, true, false);
    assert_eq!(m.diagnostics().tracks[0].first_acquisition_s, Some(0.3));
}

#[test]
fn faults_consume_old_scenes_and_gaps_discard_old_motion() {
    let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
    let t = track(0., Vec2::new(1., 0.));
    observe(&mut m, &t, 0., true, false);
    observe(&mut m, &t, 0.05, true, true);
    observe(&mut m, &t, 0.1, true, false);
    assert_eq!(m.diagnostics().tracks[0].sample_count, 0);
    observe(&mut m, &track(0.2, Vec2::default()), 0.2, true, false);
    observe(&mut m, &track(0.6, Vec2::default()), 0.6, true, false);
    assert_eq!(m.diagnostics().tracks[0].sample_count, 1);
    assert_eq!(m.diagnostics().tracks[0].first_acquisition_s, Some(0.6));
}

#[test]
fn high_rate_support_is_bounded_and_ids_do_not_share_or_retain_expired_history() {
    let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
    for i in 0..1000 {
        let time = i as f64 * 0.001;
        let a = track(time, Vec2::new(1., 0.));
        let mut b = a.clone();
        b.id = 2;
        b.velocity = Vec2::new(0., 1.);
        let mut p = vec![predict(&a, time).remove(0), predict(&b, time).remove(0)];
        m.apply(&[a, b], &[1, 2], Some(time), time, false, &mut p)
            .unwrap();
    }
    assert!(m.diagnostics().tracks.iter().all(|r| r.sample_count == 32));
    let mut t = track(1., Vec2::default());
    t.id = 2;
    let mut p = predict(&t, 1.);
    m.apply(&[t], &[], Some(1.), 1., false, &mut p).unwrap();
    assert_eq!(m.diagnostics().tracks.len(), 1);
    assert_eq!(m.diagnostics().tracks[0].id, 2);
    assert_eq!(m.diagnostics().tracks[0].sample_count, 0);
}

#[test]
fn invalid_support_clocks_tracks_and_forecasts_are_transactional() {
    let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
    observe(&mut m, &track(0., Vec2::default()), 0., true, false);
    for kind in 0..8 {
        let mut t = track(0.1, Vec2::new(1., 0.));
        let mut p = predict(&t, 0.1);
        let mut supported = vec![1];
        let mut now = 0.1;
        match kind {
            0 => supported.push(1),
            1 => supported[0] = 99,
            2 => now = -0.1,
            3 => t.last_seen = 0.2,
            4 => t.velocity.x = f64::NAN,
            5 => t.velocity.x = 16.,
            6 => p[0].id = 2,
            _ => p[0].dt = f64::MAX,
        }
        let before = m.diagnostics();
        let old = p.clone();
        assert!(
            m.apply(&[t], &supported, Some(0.1), now, false, &mut p)
                .is_err()
        );
        assert_eq!(m.diagnostics(), before);
        assert_same(&p, &old);
    }
}

#[test]
fn control_regressions_and_invalid_position_budgets_are_rejected() {
    let mut bad = bounds();
    bad.position_error_m = f64::NAN;
    assert!(RecentVelocityHypotheses::new(bad).is_err());
    let mut m = RecentVelocityHypotheses::new(bounds()).unwrap();
    observe(&mut m, &track(0.5, Vec2::default()), 0.5, true, false);
    let t = track(0.2, Vec2::default());
    let mut p = predict(&t, 0.4);
    assert!(m.apply(&[t], &[1], Some(0.2), 0.4, false, &mut p).is_err());
}
