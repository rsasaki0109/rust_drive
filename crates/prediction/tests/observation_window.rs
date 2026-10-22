use rustdriving_core::{Prediction, Track, Vec2};
use rustdriving_prediction::ObservedBraking;

fn assert_constant_velocity(prediction: &Prediction, track: &Track, age: f64) {
    assert!(prediction.additional_radius_m.is_empty());
    assert_eq!(prediction.positions.len(), 41);
    for (i, point) in prediction.positions.iter().enumerate() {
        let expected = track
            .position
            .plus(track.velocity.scaled(age + i as f64 * 0.2));
        assert!(
            point.distance(expected) < 1e-10,
            "{i}: {point:?} vs {expected:?}"
        );
    }
}

#[test]
fn intervening_stops_turns_and_backward_steps_withdraw_slow_motion_until_they_leave_the_window() {
    for direction in [Vec2::new(1.0, 0.0), Vec2::new(0.6, 0.8)] {
        for contradiction in 0..3 {
            let mut predictor = ObservedBraking::with_slow_motion();
            for i in 0..=10 {
                let time = i as f64 * 0.1;
                let mut track = Track {
                    id: 17,
                    position: Vec2::new(12.0, -3.0).plus(direction.scaled(0.5 * time)),
                    velocity: direction.scaled(0.5),
                    radius: 0.7,
                    last_seen: time,
                };
                if i == 3 {
                    match contradiction {
                        0 => track.velocity = Vec2::default(),
                        1 => track.velocity = Vec2::new(-direction.y, direction.x).scaled(0.5),
                        _ => track.position = track.position.minus(direction.scaled(0.2)),
                    }
                }
                let prediction = predictor.predict(std::slice::from_ref(&track), time);
                // A backward interval has left the window at t=0.9, when its
                // earlier endpoint t=0.2 is excluded. A stopped/turned t=0.3
                // velocity remains contradictory at the earliest anchor itself.
                if (6..9).contains(&i) || (i == 9 && contradiction != 2) {
                    assert!(
                        !prediction[0].additional_radius_m.is_empty(),
                        "intervening contradiction {contradiction} was ignored at {time}"
                    );
                    // Cached calls cannot erase the contradictory acquisition.
                    let cached = predictor.predict(std::slice::from_ref(&track), time + 0.05);
                    assert!(!cached[0].additional_radius_m.is_empty());
                    for (j, center) in cached[0].positions.iter().enumerate() {
                        let continued = track
                            .position
                            .plus(track.velocity.scaled(0.05 + j as f64 * 0.2));
                        let extra = cached[0].additional_radius_m[j];
                        assert!(center.distance(track.position) <= extra + 1e-10);
                        assert!(center.distance(continued) <= extra + 1e-10);
                    }
                }
                if i == 10 {
                    assert_constant_velocity(&prediction[0], &track, 0.0);
                }
            }
        }
    }
}

fn braking_samples(base_speed: f64, direction: Vec2, turn: bool) -> Vec<Track> {
    let mut position = Vec2::new(10.0, 4.0);
    let mut previous_velocity = direction.scaled(base_speed);
    (0..=10)
        .map(|i| {
            let time = i as f64 * 0.1;
            let speed = base_speed - 0.5 * time;
            let velocity = if i == 3 {
                if turn {
                    Vec2::new(-direction.y, direction.x).scaled(speed)
                } else {
                    direction.scaled(base_speed - 0.1 + 0.05)
                }
            } else {
                direction.scaled(speed)
            };
            if i > 0 {
                position = position.plus(previous_velocity.plus(velocity).scaled(0.05));
            }
            previous_velocity = velocity;
            Track {
                id: 23,
                position,
                velocity,
                radius: 0.7,
                last_seen: time,
            }
        })
        .collect()
}

#[test]
fn intervening_reacceleration_or_turn_cannot_support_sustained_braking_in_either_mode() {
    for slow_mode in [false, true] {
        for direction in [Vec2::new(1.0, 0.0), Vec2::new(0.6, 0.8)] {
            for turn in [false, true] {
                let mut predictor = if slow_mode {
                    ObservedBraking::with_slow_motion()
                } else {
                    ObservedBraking::default()
                };
                for (i, track) in braking_samples(4.0, direction, turn).iter().enumerate() {
                    let prediction =
                        predictor.predict(std::slice::from_ref(track), track.last_seen + 0.1);
                    if i == 6 || i == 8 {
                        assert_constant_velocity(&prediction[0], track, 0.1);
                    }
                    if i == 10 {
                        // Once contradictory evidence has left the window, fresh
                        // sustained 0.5 m/s² braking supports half that rate again.
                        for (j, point) in prediction[0].positions.iter().enumerate() {
                            let time = 0.1 + j as f64 * 0.2;
                            let braking_time = time.min(1.0);
                            let speed = 3.5;
                            let distance = speed * braking_time - 0.125 * braking_time.powi(2)
                                + 3.25 * (time - braking_time);
                            assert!(
                                point.distance(track.position.plus(direction.scaled(distance)))
                                    < 1e-10
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn confirmed_slow_motion_stays_moving_but_withdraws_contradicted_braking() {
    let mut predictor = ObservedBraking::with_slow_motion();
    for (i, track) in braking_samples(0.9, Vec2::new(0.6, 0.8), false)
        .iter()
        .enumerate()
    {
        let prediction = predictor.predict(std::slice::from_ref(track), track.last_seen + 0.1);
        if i == 6 || i == 8 {
            assert_constant_velocity(&prediction[0], track, 0.1);
        }
    }
}
