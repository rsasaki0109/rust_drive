use rustdriving_control::PurePursuit;
use rustdriving_core::{
    ControlCommand, Controller, DrivingMode, EgoState, Trajectory, TrajectoryPoint, Vec2,
    VehicleConfig,
};

fn path(speed: f64, direction: f64) -> Trajectory {
    Trajectory {
        mode: DrivingMode::Cruise,
        lateral_target: 0.0,
        points: vec![
            TrajectoryPoint {
                position: Vec2::default(),
                speed,
                time: 0.0,
            },
            TrajectoryPoint {
                position: Vec2::new(3.0, 3.0 * direction),
                speed,
                time: 1.0,
            },
        ],
    }
}
fn stopped(command: ControlCommand) {
    assert_eq!(command.acceleration, -6.0);
    assert_eq!(command.steering, 0.0);
    assert!(command.finite());
}
#[test]
fn invalid_steering_limits_return_emergency_without_panicking() {
    for max_steer in [
        f64::NAN,
        -0.1,
        f64::NEG_INFINITY,
        f64::INFINITY,
        0.0,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
    ] {
        let vehicle = VehicleConfig {
            max_steer,
            ..VehicleConfig::default()
        };
        let mut controller = PurePursuit::with_vehicle(vehicle);
        stopped(controller.control(EgoState::default(), &path(0.0, 1.0), 0.05));
    }
}
#[test]
fn invalid_wheelbases_cannot_produce_a_normal_command() {
    for wheelbase in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0] {
        let vehicle = VehicleConfig {
            wheelbase,
            ..VehicleConfig::default()
        };
        let mut controller = PurePursuit::with_vehicle(vehicle);
        stopped(controller.control(EgoState::default(), &path(0.0, 1.0), 0.05));
    }
}
#[test]
fn configuration_mutation_is_checked_on_every_call_and_repair_starts_from_zero() {
    for invalid_steering in [false, true] {
        let mut controller = PurePursuit::default();
        let mut before = ControlCommand::default();
        for _ in 0..20 {
            before = controller.control(EgoState::default(), &path(4.0, 1.0), 0.05);
        }
        assert!(before.steering > 0.3);
        if invalid_steering {
            controller.vehicle.max_steer = -0.1;
        } else {
            controller.vehicle.wheelbase = 0.0;
        }
        stopped(controller.control(EgoState::default(), &path(4.0, 1.0), 0.05));
        controller.vehicle = VehicleConfig::default();
        let after = controller.control(EgoState::default(), &path(0.0, 1.0), 0.05);
        assert!(after.steering.abs() <= 0.7 * 0.05 + 1e-10);
        assert_eq!(after.acceleration, 0.0);
    }
}
#[test]
fn valid_calibrations_follow_known_geometry_without_pipeline_specific_limits() {
    for (wheelbase, max_steer) in [(0.25, 0.8), (2.7, 0.2)] {
        for direction in [-1.0, 1.0] {
            let vehicle = VehicleConfig {
                wheelbase,
                max_steer,
                radius: 0.1,
            };
            let mut controller = PurePursuit::with_vehicle(vehicle);
            let mut command = ControlCommand::default();
            for _ in 0..30 {
                command = controller.control(EgoState::default(), &path(0.0, direction), 0.05);
            }
            // The 45-degree ray meets the default 3 m preview circle at distance3.
            let expected = (2.0 * wheelbase * std::f64::consts::FRAC_1_SQRT_2 / 3.0)
                .atan()
                .min(max_steer);
            assert!((command.steering - direction * expected).abs() < 1e-12);
            assert!(command.finite());
        }
    }
}
#[test]
fn nonfinite_computed_steering_brakes_even_when_all_configuration_fields_are_finite() {
    let mut controller = PurePursuit::with_vehicle(VehicleConfig {
        wheelbase: f64::MAX,
        ..VehicleConfig::default()
    });
    stopped(controller.control(EgoState::default(), &path(0.0, 0.0), 0.05));
    controller.vehicle = VehicleConfig::default();
    let after = controller.control(EgoState::default(), &path(0.0, 1.0), 0.05);
    assert!(after.finite());
    assert!(after.steering.abs() <= 0.7 * 0.05 + 1e-10);
}

#[test]
fn a_reduced_limit_outside_the_last_command_brakes_then_recovers_within_the_new_limit() {
    for direction in [-1.0, 1.0] {
        let mut controller = PurePursuit::default();
        let mut before = ControlCommand::default();
        for _ in 0..20 {
            before = controller.control(EgoState::default(), &path(4.0, direction), 0.05);
        }
        assert!(before.steering.abs() > 0.3);
        controller.vehicle.max_steer = 0.1;
        stopped(controller.control(EgoState::default(), &path(4.0, direction), 0.05));
        let mut previous = 0.0;
        for _ in 0..4 {
            let command = controller.control(EgoState::default(), &path(0.0, direction), 0.05);
            assert!(command.steering.abs() <= 0.1);
            assert!((command.steering - previous).abs() <= 0.7 * 0.05 + 1e-10);
            assert_eq!(command.acceleration, 0.0);
            previous = command.steering;
        }
        assert!(previous * direction > 0.0);
    }
}

#[test]
fn a_smaller_limit_that_contains_the_last_command_keeps_feedback_and_rate_continuity() {
    for direction in [-1.0, 1.0] {
        let mut controller = PurePursuit::default();
        let mut before = ControlCommand::default();
        for _ in 0..3 {
            before = controller.control(EgoState::default(), &path(4.0, direction), 0.05);
        }
        assert!(before.steering.abs() < 0.2);
        controller.vehicle.max_steer = 0.2;
        let after = controller.control(EgoState::default(), &path(0.0, direction), 0.05);
        assert!((after.steering - before.steering).abs() <= 0.7 * 0.05 + 1e-10);
        assert!(after.steering * direction > before.steering * direction);
        assert!(after.steering.abs() <= 0.2);
        assert!((after.acceleration - 0.09).abs() < 1e-12);
    }
}

#[test]
fn increasing_the_limit_preserves_feedback_and_the_last_emitted_steering() {
    for direction in [-1.0, 1.0] {
        let mut controller = PurePursuit::with_vehicle(VehicleConfig {
            max_steer: 0.1,
            ..VehicleConfig::default()
        });
        let mut before = ControlCommand::default();
        for _ in 0..4 {
            before = controller.control(EgoState::default(), &path(4.0, direction), 0.05);
        }
        controller.vehicle.max_steer = 0.55;
        let after = controller.control(EgoState::default(), &path(0.0, direction), 0.05);
        assert!((after.steering - before.steering).abs() <= 0.7 * 0.05 + 1e-10);
        assert!(after.steering * direction > before.steering * direction);
        assert!((after.acceleration - 0.12).abs() < 1e-12);
    }
}

#[test]
fn direction_changes_at_saturation_do_not_round_beyond_the_configured_limit() {
    for direction in [-1.0, 1.0] {
        let mut controller = PurePursuit::with_vehicle(VehicleConfig {
            max_steer: 0.1,
            ..VehicleConfig::default()
        });
        let before = controller.control(EgoState::default(), &path(0.0, -direction), 0.05 / 0.7);
        assert!((before.steering + direction * 0.05).abs() < 1e-12);
        for _ in 0..2 {
            let command = controller.control(EgoState::default(), &path(0.0, direction), 0.25);
            assert!(command.finite());
            assert!(command.steering.abs() <= controller.vehicle.max_steer);
            assert_eq!(command.acceleration, 0.0);
        }
    }
}
