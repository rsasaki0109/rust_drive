use rustdriving_core::{ForecastEnvelopeConfig, Prediction, Track, Vec2};
use rustdriving_prediction::ObservedBraking;

fn bounds() -> ForecastEnvelopeConfig {
    ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    }
}
fn track(time: f64, speed: f64) -> Track {
    Track {
        id: 9,
        position: Vec2::new(20., -7.),
        velocity: Vec2::new(speed, 0.),
        radius: 0.8,
        last_seen: time,
    }
}
fn sample(p: &Prediction, time: f64) -> Vec2 {
    let step = time / p.dt;
    let i = (step.floor() as usize).min(p.positions.len() - 2);
    p.positions[i].plus(
        p.positions[i + 1]
            .minus(p.positions[i])
            .scaled(step - i as f64),
    )
}

#[test]
fn reachable_motion_with_rotated_error_and_changing_acceleration_is_contained_between_knots() {
    let b = bounds();
    for yaw in [0.0_f64, 0.7, -2.1] {
        let mut acquired = track(2., 3.);
        acquired.velocity = acquired.velocity.rotated(yaw);
        let forecast = ObservedBraking::default()
            .with_envelope(b)
            .unwrap()
            .predict(std::slice::from_ref(&acquired), 2.15)
            .remove(0);
        // Integrate a changing 2D acceleration independently at 1 ms. Start at
        // the edge of the declared position and velocity error discs.
        for sign in [-1., 1.] {
            let direction = Vec2::new(0., sign).rotated(yaw);
            let mut actual = acquired.position.plus(direction.scaled(b.position_error_m));
            let mut velocity = acquired
                .velocity
                .plus(direction.scaled(b.velocity_error_m_s));
            for j in 0..8150 {
                let elapsed = (j + 1) as f64 * 0.001;
                let acceleration = direction
                    .rotated(j as f64 * 0.003)
                    .scaled(b.acceleration_error_m_s2);
                actual = actual
                    .plus(velocity.scaled(0.001))
                    .plus(acceleration.scaled(0.0000005));
                velocity = velocity.plus(acceleration.scaled(0.001));
                if elapsed >= 0.15 {
                    let future = (elapsed - 0.15).min(8.);
                    assert!(
                        sample(&forecast, future).distance(actual) + acquired.radius
                            <= forecast.radius_at(future) + 1e-9
                    );
                }
            }
        }
    }
}

#[test]
fn fresh_braking_retains_both_nominal_and_no_further_braking_without_erasing_age() {
    let mut nominal = ObservedBraking::default();
    let mut enveloped = ObservedBraking::default().with_envelope(bounds()).unwrap();
    let mut latest = track(0., 4.);
    for i in 0..=6 {
        let t = i as f64 * 0.1;
        latest = track(t, 4. - 2. * t);
        nominal.predict(std::slice::from_ref(&latest), t);
        enveloped.predict(std::slice::from_ref(&latest), t);
    }
    let old = nominal
        .predict(std::slice::from_ref(&latest), 0.7)
        .remove(0);
    assert!(old.positions[40].x < latest.position.x + latest.velocity.x * 8.1 - 7.);
    let covered = enveloped
        .predict(std::slice::from_ref(&latest), 0.7)
        .remove(0);
    for j in 0..=800 {
        let time = j as f64 * 0.01;
        let center = sample(&covered, time);
        let cv = latest.position.plus(latest.velocity.scaled(time + 0.1));
        assert!(
            center.distance(sample(&old, time)) <= covered.radius_at(time) - latest.radius + 1e-9
        );
        let error = 0.2 + 0.3 * (time + 0.1) + 0.05 * (time + 0.1).powi(2);
        assert!(center.distance(cv) + error <= covered.radius_at(time) - latest.radius + 1e-9);
    }
    let again = enveloped
        .predict(std::slice::from_ref(&latest), 0.7)
        .remove(0);
    assert_eq!(covered.positions, again.positions);
    assert_eq!(covered.additional_radius_m, again.additional_radius_m);
    let later = enveloped
        .predict(std::slice::from_ref(&latest), 0.75)
        .remove(0);
    assert!(later.radius_at(0.) > covered.radius_at(0.));
}

#[test]
fn deadband_and_unconfirmed_slow_occupancy_are_both_preserved() {
    for speed in [0., 0.05, 0.5, 0.699] {
        for slow in [false, true] {
            let acquired = track(0., speed);
            let mut original = if slow {
                ObservedBraking::with_slow_motion()
            } else {
                ObservedBraking::default()
            };
            let nominal = original
                .predict(std::slice::from_ref(&acquired), 0.1)
                .remove(0);
            let mut policy = if slow {
                ObservedBraking::with_slow_motion()
            } else {
                ObservedBraking::default()
            };
            policy = policy.with_envelope(bounds()).unwrap();
            let p = policy
                .predict(std::slice::from_ref(&acquired), 0.1)
                .remove(0);
            for j in 0..=800 {
                let t = j as f64 * 0.01;
                assert!(
                    sample(&p, t).distance(sample(&nominal, t)) + nominal.radius_at(t)
                        <= p.radius_at(t) + 1e-9
                );
                assert!(
                    sample(&p, t)
                        .distance(acquired.position.plus(acquired.velocity.scaled(0.1 + t)))
                        + acquired.radius
                        <= p.radius_at(t) + 1e-9
                );
            }
        }
    }
}

#[test]
fn absent_policy_preserves_empty_radius_serialization_and_zero_budgets_still_cover_alternatives() {
    let t = track(0., 0.5);
    let legacy = ObservedBraking::default().predict(std::slice::from_ref(&t), 0.);
    assert!(legacy[0].additional_radius_m.is_empty());
    let zero = ForecastEnvelopeConfig {
        position_error_m: 0.,
        velocity_error_m_s: 0.,
        acceleration_error_m_s2: 0.,
    };
    let p = ObservedBraking::default()
        .with_envelope(zero)
        .unwrap()
        .predict(&[t], 0.);
    assert!((p[0].positions[40].x - 22.).abs() < 1e-12);
    assert!((p[0].additional_radius_m[40] - 2.).abs() < 1e-12);
}

#[test]
fn invalid_budgets_and_overflow_do_not_supply_a_valid_forecast() {
    for index in 0..3 {
        for value in [f64::NAN, f64::INFINITY, -0.1, 21.] {
            let mut b = bounds();
            match index {
                0 => b.position_error_m = value,
                1 => b.velocity_error_m_s = value,
                _ => b.acceleration_error_m_s2 = value,
            }
            assert!(ObservedBraking::default().with_envelope(b).is_err());
        }
    }
    let mut predictor = ObservedBraking::default().with_envelope(bounds()).unwrap();
    let acquired = track(0., 1.);
    assert!(
        predictor.predict(std::slice::from_ref(&acquired), f64::MAX)[0]
            .positions
            .is_empty()
    );
    let acquired = track(0., 0.05);
    assert!(
        predictor.predict(&[acquired], f64::MAX)[0]
            .positions
            .is_empty()
    );
    let future = track(1., 1.);
    assert!(predictor.predict(&[future], 0.)[0].positions.is_empty());
}
