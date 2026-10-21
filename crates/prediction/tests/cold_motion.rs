use rustdriving_core::{ForecastEnvelopeConfig, Prediction, Track, TrackMeasurement, Vec2};
use rustdriving_prediction::{ColdMotionEnvelope, ObservedBraking};

fn bounds() -> ForecastEnvelopeConfig {
    ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    }
}
fn sample(time: f64, speed: f64) -> (Track, TrackMeasurement) {
    let position = Vec2::new(10., 2.5 * time);
    (
        Track {
            id: 1,
            position,
            velocity: Vec2::new(0., speed),
            radius: 1.,
            last_seen: time,
        },
        TrackMeasurement {
            id: 1,
            stamp: time,
            center: position,
            radius: 1.,
        },
    )
}
fn predict(track: &Track, now: f64) -> Vec<Prediction> {
    ObservedBraking::default()
        .with_envelope(bounds())
        .unwrap()
        .predict(std::slice::from_ref(track), now)
}
fn observe(model: &mut ColdMotionEnvelope, time: f64, speed: f64) -> Vec<Prediction> {
    let (track, measurement) = sample(time, speed);
    let mut p = predict(&track, time);
    model
        .apply(&[track], &[measurement], Some(time), time, false, &mut p)
        .unwrap();
    p
}

#[test]
fn unknown_velocity_covers_the_whole_speed_disc_between_knots_and_consumes_age() {
    let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
    let (track, measurement) = sample(0., 2.5);
    for now in [0., 0.05, 0.1] {
        let mut p = predict(&track, now);
        model
            .apply(
                std::slice::from_ref(&track),
                std::slice::from_ref(&measurement),
                Some(0.),
                now,
                false,
                &mut p,
            )
            .unwrap();
        assert_eq!(model.diagnostics().tracks[0].acquisition_count, 1);
        for step in 0..161 {
            let time = step as f64 * 0.05;
            let f = time / p[0].dt;
            let i = (f.floor() as usize).min(39);
            let fraction = f - i as f64;
            let center = p[0].positions[i]
                .scaled(1. - fraction)
                .plus(p[0].positions[i + 1].scaled(fraction));
            for direction in 0..36 {
                let angle = direction as f64 * std::f64::consts::TAU / 36.;
                let physical = track
                    .position
                    .plus(Vec2::new(angle.cos(), angle.sin()).scaled(15. * (now + time)));
                assert!(
                    center.distance(physical) + track.radius + 0.2 <= p[0].radius_at(time) + 1e-9
                );
            }
        }
    }
}

#[test]
fn three_real_acquisitions_confirm_the_existing_velocity_without_changing_nominal_forecasts() {
    let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
    observe(&mut model, 0., 0.);
    observe(&mut model, 0.1, 0.3);
    assert!(!model.diagnostics().tracks[0].confirmed);
    let confirmed = observe(&mut model, 0.2, 2.5);
    let nominal = predict(&sample(0.2, 2.5).0, 0.2);
    assert_eq!(confirmed[0].positions, nominal[0].positions);
    assert_eq!(
        confirmed[0].additional_radius_m,
        nominal[0].additional_radius_m
    );
    let evidence = &model.diagnostics().tracks[0];
    assert!(evidence.confirmed);
    assert_eq!(evidence.confirmation_acquisition_s, Some(0.2));
    assert_eq!(evidence.acquisition_count, 3);
    assert!(evidence.velocity_residual_m_s.unwrap() < 1e-12);
    observe(&mut model, 0.3, 2.5);
    assert_eq!(model.diagnostics().tracks[0].last_acquisition_s, Some(0.2));
    assert_eq!(
        model.diagnostics().tracks[0].last_observed_acquisition_s,
        Some(0.3)
    );
}

#[test]
fn cached_ticks_cannot_vote_and_stale_support_rearms_the_unknown_motion_disc() {
    let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
    observe(&mut model, 0., 0.);
    let (track, measurement) = sample(0., 0.);
    for i in 1..=20 {
        let now = i as f64 * 0.01;
        let mut p = predict(&track, now);
        model
            .apply(
                std::slice::from_ref(&track),
                std::slice::from_ref(&measurement),
                Some(0.),
                now,
                false,
                &mut p,
            )
            .unwrap();
        assert!(!model.diagnostics().tracks[0].confirmed);
        assert_eq!(model.diagnostics().tracks[0].acquisition_count, 1);
    }
    observe(&mut model, 0.3, 2.5);
    observe(&mut model, 0.4, 2.5);
    let (track, _) = sample(0.4, 2.5);
    let mut p = predict(&track, 0.76);
    model
        .apply(&[track], &[], Some(0.4), 0.76, false, &mut p)
        .unwrap();
    assert!(!model.diagnostics().tracks[0].confirmed);
    assert_eq!(model.diagnostics().tracks[0].acquisition_count, 0);
    assert!(p[0].additional_radius_m[5] > 20.);
    observe(&mut model, 0.8, 2.5);
    assert_eq!(model.diagnostics().tracks[0].acquisition_count, 1);
}

#[test]
fn scan_failure_consumes_old_acquisitions_and_requires_a_new_prefix() {
    let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
    observe(&mut model, 0., 0.);
    observe(&mut model, 0.1, 2.5);
    let (track, measurement) = sample(0.1, 2.5);
    for (now, failed) in [(0.15, true), (0.18, false)] {
        let mut p = predict(&track, now);
        model
            .apply(
                std::slice::from_ref(&track),
                std::slice::from_ref(&measurement),
                Some(0.1),
                now,
                failed,
                &mut p,
            )
            .unwrap();
        assert_eq!(model.diagnostics().tracks[0].acquisition_count, 0);
    }
    observe(&mut model, 0.2, 2.5);
    observe(&mut model, 0.3, 2.5);
    assert!(!model.diagnostics().tracks[0].confirmed);
    observe(&mut model, 0.4, 2.5);
    assert_eq!(model.diagnostics().tracks[0].first_acquisition_s, Some(0.2));
    assert!(model.diagnostics().tracks[0].confirmed);
}

#[test]
fn shape_changes_or_velocity_disagreement_cannot_confirm_a_motion_model() {
    for changing_radius in [false, true] {
        let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
        for i in 0_usize..8 {
            let time = i as f64 * 0.1;
            let (track, mut measurement) = sample(time, if changing_radius { 2.5 } else { 0. });
            if changing_radius {
                measurement.radius += if i.is_multiple_of(2) { 0. } else { 0.2 };
            }
            let mut p = predict(&track, time);
            model
                .apply(&[track], &[measurement], Some(time), time, false, &mut p)
                .unwrap();
            assert!(!model.diagnostics().tracks[0].confirmed);
        }
    }
}

#[test]
fn high_rate_samples_are_bounded_without_false_elapsed_support() {
    let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
    for i in 0..1000 {
        observe(&mut model, i as f64 * 0.0001, 2.5);
        let r = &model.diagnostics().tracks[0];
        assert!(r.acquisition_count <= 32);
        assert!(!r.confirmed);
    }
}

#[test]
fn malformed_inputs_and_speed_bound_violations_are_transactional() {
    let mut model = ColdMotionEnvelope::new(1., bounds()).unwrap();
    observe(&mut model, 0., 0.);
    observe(&mut model, 0.1, 0.3);
    let before = model.diagnostics();
    let (track, measurement) = sample(0.2, 2.5);
    let mut p = predict(&track, 0.2);
    let nominal = p[0].additional_radius_m.clone();
    assert!(
        model
            .apply(&[track], &[measurement], Some(0.2), 0.2, false, &mut p)
            .is_err()
    );
    assert_eq!(model.diagnostics(), before);
    assert_eq!(p[0].additional_radius_m, nominal);
    let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
    let (track, mut measurement) = sample(0., 0.);
    measurement.stamp = 0.1;
    let mut p = predict(&track, 0.);
    assert!(
        model
            .apply(&[track], &[measurement], Some(0.), 0., false, &mut p)
            .is_err()
    );
    assert!(model.diagnostics().tracks.is_empty());
    for speed in [0., 0.6, 15.1, f64::NAN, f64::INFINITY] {
        assert!(ColdMotionEnvelope::new(speed, bounds()).is_err());
    }
}

#[test]
fn identities_do_not_share_confirmation_and_expired_tracks_release_memory() {
    let mut model = ColdMotionEnvelope::new(15., bounds()).unwrap();
    for time in [0., 0.1, 0.2] {
        observe(&mut model, time, 2.5);
    }
    let (track, measurement) = sample(0.3, 2.5);
    let mut second = track.clone();
    second.id = 2;
    let mut m2 = measurement.clone();
    m2.id = 2;
    let mut p = predict(&track, 0.3);
    p.push(predict(&second, 0.3).remove(0));
    model
        .apply(
            &[track, second],
            &[measurement, m2],
            Some(0.3),
            0.3,
            false,
            &mut p,
        )
        .unwrap();
    assert!(model.diagnostics().tracks[0].confirmed);
    assert!(!model.diagnostics().tracks[1].confirmed);
    model
        .apply(&[], &[], Some(0.4), 0.4, false, &mut [])
        .unwrap();
    assert!(model.diagnostics().tracks.is_empty());
}
