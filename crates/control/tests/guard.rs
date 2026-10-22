use rustdriving_control::guard;
use rustdriving_core::ControlCommand;

#[test]
fn invalid_position_variance_cannot_authorize_motion() {
    let requested = ControlCommand {
        acceleration: 1.0,
        steering: 0.2,
    };
    for variance in [
        -0.01,
        -f64::MIN_POSITIVE,
        -f64::MAX,
        4.0_f64.next_up(),
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        let actual = guard(requested, 1.0, 1.0, 1.0, variance);
        assert!(actual.finite());
        assert_eq!(actual.acceleration, -6.0, "variance {variance}");
        assert_eq!(actual.steering, 0.0, "variance {variance}");
    }
}

#[test]
fn valid_variance_including_both_boundaries_preserves_motion() {
    let requested = ControlCommand {
        acceleration: 1.0,
        steering: -0.2,
    };
    for variance in [-0.0, 0.0, f64::MIN_POSITIVE, 0.1, 4.0] {
        let actual = guard(requested, 1.0, 1.0, 1.0, variance);
        assert_eq!(
            actual.acceleration.to_bits(),
            requested.acceleration.to_bits()
        );
        assert_eq!(actual.steering.to_bits(), requested.steering.to_bits());
    }
}
