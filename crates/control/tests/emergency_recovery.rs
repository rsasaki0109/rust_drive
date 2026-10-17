use rustdriving_control::PurePursuit;
use rustdriving_core::{
    ControlCommand, Controller, DrivingMode, EgoState, Trajectory, TrajectoryPoint, Vec2,
};

fn path(initial: f64, next: f64, duration: f64) -> Trajectory {
    Trajectory {
        mode: DrivingMode::Cruise,
        lateral_target: 0.0,
        points: vec![
            TrajectoryPoint {
                position: Vec2::default(),
                speed: initial,
                time: 0.0,
            },
            TrajectoryPoint {
                position: Vec2::new(3.0, 3.0),
                speed: next,
                time: duration,
            },
        ],
    }
}
fn warmed() -> PurePursuit {
    let mut controller = PurePursuit::default();
    let mut command = ControlCommand::default();
    for _ in 0..20 {
        command = controller.control(EgoState::default(), &path(4.0, 4.0, 1.0), 0.05);
    }
    assert!(command.steering.abs() > 0.3);
    controller
}
fn stopped(command: ControlCommand) {
    assert_eq!(command.acceleration, -6.0);
    assert_eq!(command.steering, 0.0);
    assert!(command.finite());
}
fn recovered(controller: &mut PurePursuit) {
    let command = controller.control(EgoState::default(), &path(0.0, 0.0, 1.0), 0.05);
    assert!(command.steering.abs() <= 0.7 * 0.05 + 1e-10);
    assert_eq!(command.acceleration, 0.0);
}
#[test]
fn invalid_segment_duration_clears_the_last_emitted_steering_and_speed_feedback() {
    for duration in [0.0, -1.0] {
        let mut controller = warmed();
        stopped(controller.control(EgoState::default(), &path(4.0, 4.0, duration), 0.05));
        recovered(&mut controller);
    }
}
#[test]
fn a_moving_single_point_path_clears_feedback_before_recovery() {
    let mut controller = warmed();
    let mut missing_horizon = path(4.0, 4.0, 1.0);
    missing_horizon.points.truncate(1);
    stopped(controller.control(EgoState::default(), &missing_horizon, 0.05));
    recovered(&mut controller);
}
#[test]
fn finite_speed_inputs_with_overflowing_feedforward_brake_and_recover() {
    let mut controller = warmed();
    stopped(controller.control(
        EgoState::default(),
        &path(1.0, f64::MAX, f64::MIN_POSITIVE),
        0.05,
    ));
    recovered(&mut controller);
}
#[test]
fn overflowing_feedback_cannot_be_hidden_by_actuator_or_integral_clamps() {
    for (initial, next, dt) in [
        (f64::MAX, f64::MAX, 0.05),
        (4.0, 4.0, f64::MAX),
        (0.6 * f64::MAX, f64::MAX, 0.05),
    ] {
        let mut controller = warmed();
        stopped(controller.control(EgoState::default(), &path(initial, next, 1.0), dt));
        recovered(&mut controller);
    }
}
#[test]
fn finite_saturation_keeps_normal_feedback_and_the_single_point_stop_remains_valid() {
    let mut controller = PurePursuit::default();
    let command = controller.control(EgoState::default(), &path(100.0, 100.0, 1.0), 0.05);
    assert_eq!(command.acceleration, 2.0);
    let memory = controller.control(EgoState::default(), &path(0.0, 0.0, 1.0), 0.05);
    assert_eq!(memory.acceleration, 0.3);
    let mut hold = path(0.0, 0.0, 1.0);
    hold.points.truncate(1);
    assert_eq!(
        controller
            .control(EgoState::default(), &hold, 0.05)
            .acceleration,
        -3.5
    );
}
