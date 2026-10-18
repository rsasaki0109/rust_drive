use rustdriving_core::{Predictor, Track, Vec2};
use rustdriving_prediction::{ConstantVelocity, ObservedBraking};

fn sample(time: f64, speed: f64, direction: Vec2) -> Track {
    Track {
        id: 4,
        position: Vec2::new(12., 5.).plus(direction.scaled(speed * time)),
        velocity: direction.scaled(speed),
        radius: 0.7,
        last_seen: time,
    }
}

#[test]
fn persistent_low_speed_motion_forecasts_displacement_and_delivery_age() {
    for speed in [0.2, 0.5, 0.69] {
        for direction in [Vec2::new(0., -1.), Vec2::new(0.6, 0.8)] {
            let mut predictor = ObservedBraking::with_slow_motion();
            let mut established = ObservedBraking::default();
            for time in [0., 0.2, 0.4, 0.6] {
                let t = sample(time, speed, direction);
                let p = established.predict(std::slice::from_ref(&t), time);
                assert!(p[0].positions.iter().all(|p| *p == t.position));
            }
            for time in [0., 0.2, 0.4] {
                let track = sample(time, speed, direction);
                let p = predictor.predict(std::slice::from_ref(&track), time);
                assert!(p[0].positions.iter().all(|p| *p == track.position));
            }
            let track = sample(0.6, speed, direction);
            let p = predictor.predict(std::slice::from_ref(&track), 0.75);
            for (i, point) in p[0].positions.iter().enumerate() {
                let expected = track
                    .position
                    .plus(direction.scaled(speed * (0.15 + i as f64 * 0.2)));
                assert!(
                    point.distance(expected) < 1e-10,
                    "{speed}: {point:?} vs {expected:?}"
                );
            }
            // The stateless legacy baseline has no confirming observations.
            assert!(
                ConstantVelocity::default().predict(std::slice::from_ref(&track))[0]
                    .positions
                    .iter()
                    .all(|p| *p == track.position)
            );
        }
    }
}

#[test]
fn cached_calls_cannot_replace_distinct_acquisitions_or_change_the_forecast() {
    let mut baseline = ObservedBraking::with_slow_motion();
    let mut busy = ObservedBraking::with_slow_motion();
    for time in [0., 0.2, 0.4, 0.6] {
        let track = sample(time, 0.5, Vec2::new(1., 0.));
        let expected = baseline.predict(std::slice::from_ref(&track), time + 0.05);
        for _ in 0..100 {
            let actual = busy.predict(std::slice::from_ref(&track), time + 0.05);
            assert_eq!(actual[0].positions, expected[0].positions);
        }
    }
    let mut unconfirmed = ObservedBraking::with_slow_motion();
    let track = sample(0., 0.5, Vec2::new(1., 0.));
    for i in 0..100 {
        let p = unconfirmed.predict(std::slice::from_ref(&track), i as f64 * 0.01);
        assert!(p[0].positions.iter().all(|p| *p == track.position));
    }
}

#[test]
fn velocity_alone_contradictory_positions_and_missing_samples_cannot_confirm_motion() {
    for case in 0..4 {
        let mut predictor = ObservedBraking::with_slow_motion();
        let mut latest = None;
        for time in [0., 0.2, 0.4, 0.6] {
            if case == 3 && time == 0.2 {
                continue;
            }
            let mut t = sample(time, 0.5, Vec2::new(1., 0.));
            if case == 0 {
                t.position = Vec2::new(12., 5.);
            }
            if case == 1 && time == 0.4 {
                t.position.x -= 0.2;
            }
            if case == 2 && time == 0.4 {
                t.velocity = Vec2::new(0., 0.5);
            }
            let p = predictor.predict(std::slice::from_ref(&t), time);
            assert!(p[0].positions.iter().all(|p| *p == t.position));
            latest = Some(t);
        }
        assert!(latest.is_some());
    }
}

#[test]
fn gaps_absence_reordered_samples_and_stops_withdraw_low_speed_support() {
    for case in 0..4 {
        let mut predictor = ObservedBraking::with_slow_motion();
        for time in [0., 0.2, 0.4, 0.6] {
            let t = sample(time, 0.5, Vec2::new(1., 0.));
            predictor.predict(&[t], time);
        }
        let mut t = sample(0.8, 0.5, Vec2::new(1., 0.));
        match case {
            0 => {
                t = sample(1.2, 0.5, Vec2::new(1., 0.));
            }
            1 => {
                predictor.predict(&[], 0.7);
            }
            2 => {
                t = sample(0.5, 0.5, Vec2::new(1., 0.));
            }
            _ => {
                t.velocity = Vec2::default();
            }
        }
        let p = predictor.predict(std::slice::from_ref(&t), t.last_seen);
        assert!(p[0].positions.iter().all(|p| *p == t.position));
    }
}

#[test]
fn supported_braking_continues_through_slow_motion_and_withdraws_on_reacceleration() {
    let mut predictor = ObservedBraking::with_slow_motion();
    for time in [0., 0.2, 0.4, 0.6] {
        let mut t = sample(time, 0.8 - 0.5 * time, Vec2::new(1., 0.));
        t.position.x = 12. + 0.8 * time - 0.25 * time * time;
        let p = predictor.predict(std::slice::from_ref(&t), time + 0.1);
        if time == 0.6 {
            // Measured deceleration 0.5 supports 0.25 m/s² for the original
            // one-second budget: 0.375 m, then coasting at 0.25 m/s.
            for (i, point) in p[0].positions.iter().enumerate() {
                let elapsed = 0.1 + i as f64 * 0.2;
                let braking_time = elapsed.min(1.);
                let distance = 0.5 * braking_time - 0.125 * braking_time.powi(2)
                    + 0.25 * (elapsed - braking_time);
                assert!(point.distance(t.position.plus(Vec2::new(distance, 0.))) < 1e-10);
            }
        }
    }
    let mut accelerating = sample(0.8, 0.55, Vec2::new(1., 0.));
    accelerating.position.x = 12.5;
    let p = predictor.predict(std::slice::from_ref(&accelerating), 0.9);
    for (i, point) in p[0].positions.iter().enumerate() {
        let expected = accelerating
            .position
            .plus(Vec2::new(0.55 * (0.1 + i as f64 * 0.2), 0.));
        assert!(point.distance(expected) < 1e-10);
    }
}
