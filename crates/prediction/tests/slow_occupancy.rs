use rustdriving_core::{Track, Vec2};
use rustdriving_prediction::ObservedBraking;

fn track() -> Track {
    Track {
        id: 3,
        position: Vec2::new(12.0, 5.0),
        velocity: Vec2::new(0.3, -0.4),
        radius: 0.7,
        last_seen: 1.0,
    }
}

#[test]
fn unconfirmed_slow_motion_does_not_collapse_to_a_static_forecast() {
    let observed = track();
    let predictions =
        ObservedBraking::with_slow_motion().predict(std::slice::from_ref(&observed), 1.0);
    assert!(predictions[0].positions[40].distance(observed.position) > 1.0);
}

#[test]
fn uncertain_slow_motion_accounts_for_acquisition_age() {
    let observed = track();
    let predictions =
        ObservedBraking::with_slow_motion().predict(std::slice::from_ref(&observed), 1.1);
    assert!(predictions[0].positions[0].distance(observed.position) > 0.01);
}

#[test]
fn uncertain_envelope_covers_stopping_and_every_intermediate_velocity_at_each_epoch() {
    for direction in [Vec2::new(1., 0.), Vec2::new(0., -1.), Vec2::new(0.6, 0.8)] {
        for speed in [0.1, 0.2, 0.5, 0.699] {
            for age in [0., 0.1, 0.3] {
                let mut observed = track();
                observed.velocity = direction.scaled(speed);
                let mut predictor = ObservedBraking::with_slow_motion();
                let forecast = predictor.predict(std::slice::from_ref(&observed), 1. + age);
                let p = &forecast[0];
                assert!(p.valid_radii());
                assert_eq!(p.additional_radius_m.len(), 41);
                // Dense independent occupancy sampling includes times between knots.
                for j in 0..=800 {
                    let t = j as f64 * 0.01;
                    let knot = ((t / p.dt).floor() as usize).min(39);
                    let f = ((t - knot as f64 * p.dt) / p.dt).min(1.);
                    let center = p.positions[knot]
                        .plus(p.positions[knot + 1].minus(p.positions[knot]).scaled(f));
                    for fraction in [0., 0.25, 0.5, 0.75, 1.] {
                        let possible = observed
                            .position
                            .plus(observed.velocity.scaled((age + t) * fraction));
                        assert!(
                            center.distance(possible) + observed.radius <= p.radius_at(t) + 1e-10
                        );
                    }
                }
                let cached = predictor.predict(std::slice::from_ref(&observed), 1. + age);
                assert_eq!(p.positions, cached[0].positions);
                assert_eq!(p.additional_radius_m, cached[0].additional_radius_m);
                let legacy =
                    ObservedBraking::default().predict(std::slice::from_ref(&observed), 1. + age);
                assert!(legacy[0].additional_radius_m.is_empty());
                assert!(legacy[0].positions.iter().all(|p| *p == observed.position));
            }
        }
    }
}
