use rustdriving_control::PurePursuit;
use rustdriving_core::{
    ControlCommand, Controller, DrivingMode, EgoState, Trajectory, TrajectoryPoint, Vec2,
};

fn path(times: &[f64], speed: f64) -> Trajectory {
    Trajectory {
        mode: DrivingMode::Cruise,
        lateral_target: 0.0,
        points: times
            .iter()
            .enumerate()
            .map(|(i, &time)| TrajectoryPoint {
                position: Vec2::new(3.0 * i as f64, 3.0 * i as f64),
                speed,
                time,
            })
            .collect(),
    }
}
fn stopped(command: ControlCommand) {
    assert!(command.finite());
    assert_eq!(command.acceleration, -6.0);
    assert_eq!(command.steering, 0.0);
}
#[test]
fn duplicate_and_reversed_late_times_brake_and_clear_feedback() {
    for last_time in [1.0, 0.75] {
        let mut controller = PurePursuit::default();
        let valid = path(&[0.0, 0.5, 1.0, 2.0], 4.0);
        let mut before = ControlCommand::default();
        for _ in 0..20 {
            before = controller.control(EgoState::default(), &valid, 0.05);
        }
        assert!(before.steering > 0.3);
        // The first two intervals are valid, and the preview ends before the bad tail.
        stopped(controller.control(
            EgoState::default(),
            &path(&[0.0, 0.5, 1.0, last_time], 4.0),
            0.05,
        ));
        let recovered = controller.control(EgoState::default(), &path(&[0.0, 1.0], 0.0), 0.05);
        assert!(recovered.steering.abs() <= 0.7 * 0.05 + 1e-10);
        assert_eq!(recovered.acceleration, 0.0);
    }
}
#[test]
fn negative_relative_times_cannot_schedule_past_motion_or_a_past_hold() {
    for times in [&[-2.0, -1.0, 0.0][..], &[-1.0, 0.0, 1.0][..], &[-0.1][..]] {
        let mut controller = PurePursuit::default();
        stopped(controller.control(EgoState::default(), &path(times, 0.0), 0.05));
    }
}
#[test]
fn valid_nonuniform_tail_times_keep_the_original_prefix_commands() {
    let mut short = PurePursuit::default();
    let mut extended = PurePursuit::default();
    for _ in 0..20 {
        let a = short.control(EgoState::default(), &path(&[0.0, 0.01], 4.0), 0.05);
        let b = extended.control(
            EgoState::default(),
            &path(&[0.0, 0.01, 0.5, 10.0], 4.0),
            0.05,
        );
        assert_eq!(a.acceleration.to_bits(), b.acceleration.to_bits());
        assert_eq!(a.steering.to_bits(), b.steering.to_bits());
        assert_eq!(b.acceleration, 2.0);
        assert!(b.finite());
    }
}
#[test]
fn stationary_holds_allow_repeated_positions_with_advancing_time() {
    let mut controller = PurePursuit::default();
    let mut hold = path(&[0.0, 0.25, 8.0], 0.0);
    hold.mode = DrivingMode::Goal;
    for point in &mut hold.points {
        point.position = Vec2::default();
    }
    let command = controller.control(EgoState::default(), &hold, 0.05);
    assert_eq!(command.acceleration, 0.0);
    assert_eq!(command.steering, 0.0);
    hold.points.truncate(1);
    assert_eq!(
        controller
            .control(EgoState::default(), &hold, 0.05)
            .acceleration,
        -3.5
    );
}
