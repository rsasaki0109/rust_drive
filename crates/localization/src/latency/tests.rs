use super::*;
use rustdriving_core::{Pose, Vec2};

fn odometry(stamp: f64) -> Odometry {
    Odometry {
        stamp,
        speed: 6.,
        yaw_rate: 0.2,
    }
}

fn fix(stamp: f64) -> Gnss {
    Gnss {
        stamp,
        position: Vec2::new(6. * stamp, 0.),
        variance: 0.04,
    }
}

fn same(a: &Ekf, b: &Ekf) {
    assert!(a.state.pose.position.distance(b.state.pose.position) < 1e-12);
    assert!((a.state.pose.yaw - b.state.pose.yaw).abs() < 1e-12);
    assert!((a.state.speed - b.state.speed).abs() < 1e-12);
    for i in 0..3 {
        for j in 0..3 {
            assert!((a.covariance[i][j] - b.covariance[i][j]).abs() < 1e-12);
        }
    }
    assert_eq!(a.last_gnss, b.last_gnss);
    assert_eq!(a.diagnostics.accepted_fixes, b.diagnostics.accepted_fixes);
}

#[test]
fn delayed_correction_matches_chronological_filter_for_both_chassis_models() {
    for offset in [0., 1.3] {
        for acquired in [0.2, 0.225] {
            let mut chronological = Ekf::new(Pose::default());
            let mut delayed = Ekf::new(Pose::default());
            let mut history = GnssHistory::default();
            history.reset(0.);
            for tick in 1..=10 {
                let start = (tick - 1) as f64 * 0.05;
                let end = tick as f64 * 0.05;
                let motion = odometry(end);
                if start < acquired && acquired <= end + 1e-12 {
                    advance(&mut chronological, motion, acquired - start, offset);
                    assert!(chronological.update(fix(acquired)));
                    if end - acquired > 1e-12 {
                        advance(&mut chronological, motion, end - acquired, offset);
                    }
                } else {
                    advance(&mut chronological, motion, end - start, offset);
                }
                history
                    .predict(&mut delayed, motion, start, end, offset)
                    .unwrap();
                if tick == 8 {
                    assert_eq!(
                        history.correct(&mut delayed, fix(acquired), end),
                        Ok(GnssDecision::Accepted)
                    );
                    same(&chronological, &delayed);
                    assert_eq!(delayed.diagnostics.last_accepted_stamp, Some(acquired));
                }
                if tick > 8 {
                    same(&chronological, &delayed);
                }
            }
        }
    }
}

#[test]
fn innovation_is_gated_at_acquisition_and_rejection_keeps_current_pose_and_covariance() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::default();
    history.reset(0.);
    assert_eq!(
        history.correct(&mut filter, fix(0.), 0.),
        Ok(GnssDecision::Accepted)
    );
    for tick in 1..=8 {
        let end = tick as f64 * 0.05;
        history
            .predict(
                &mut filter,
                Odometry {
                    yaw_rate: 0.,
                    ..odometry(end)
                },
                (tick - 1) as f64 * 0.05,
                end,
                0.,
            )
            .unwrap();
    }
    assert_eq!(
        history.correct(&mut filter, fix(0.2), 0.4),
        Ok(GnssDecision::Accepted)
    );
    assert!(filter.diagnostics.last_nis.unwrap() < 1e-20);
    assert!((filter.state.pose.position.x - 2.4).abs() < 1e-12);
    let before = filter.clone();
    assert_eq!(
        history.correct(
            &mut filter,
            Gnss {
                position: Vec2::new(30., -25.),
                ..fix(0.3)
            },
            0.4
        ),
        Ok(GnssDecision::RejectedInnovation)
    );
    same(&filter, &before);
    assert_eq!(filter.diagnostics.last_observed_stamp, Some(0.3));
    assert_eq!(filter.diagnostics.last_accepted_stamp, Some(0.2));
    assert_eq!(filter.diagnostics.rejected_fixes, 1);
    assert_eq!(
        history.correct(&mut filter, fix(0.25), 0.4),
        Ok(GnssDecision::IgnoredTimestamp)
    );
    assert_eq!(filter.diagnostics.rejected_fixes, 1);
}

#[test]
fn uncovered_expired_and_invalid_motion_cannot_invent_history_or_mutate_the_filter() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::default();
    history.reset(1.);
    let before = filter.clone();
    assert_eq!(
        history.correct(&mut filter, fix(0.8), 1.),
        Err(HistoryError::Unavailable)
    );
    assert_eq!(
        history.predict(&mut filter, odometry(1.), 0.9, 1., 0.),
        Err(HistoryError::InvalidClockOrMotion)
    );
    assert_eq!(
        history.correct(&mut filter, fix(1.1), 1.),
        Err(HistoryError::InvalidClockOrMotion)
    );
    same(&filter, &before);
    for tick in 1..=1000 {
        let start = 1. + (tick - 1) as f64 * 0.0001;
        let end = 1. + tick as f64 * 0.0001;
        history
            .predict(&mut filter, odometry(end), start, end, 0.)
            .unwrap();
        assert!(history.len() <= MAX_PREDICTIONS);
    }
    let before = filter.clone();
    assert_eq!(
        history.correct(&mut filter, fix(1.), 1.1),
        Err(HistoryError::Unavailable)
    );
    same(&filter, &before);
    history.reset(2.);
    history
        .predict(&mut filter, odometry(2.1), 2., 2.1, 0.)
        .unwrap();
    assert_eq!(
        history.correct(&mut filter, fix(1.59), 2.1),
        Err(HistoryError::Unavailable)
    );
}

fn map_pose(stamp: f64) -> Pose {
    Pose {
        position: Vec2::new(30. * (0.2 * stamp).sin(), 30. * (1. - (0.2 * stamp).cos())),
        yaw: 0.2 * stamp,
    }
}
const MAP_COVARIANCE: [[f64; 3]; 3] = [
    [0.02, 0.002, 0.0001],
    [0.002, 0.03, -0.0002],
    [0.0001, -0.0002, 0.0004],
];

#[test]
fn repeated_delayed_fixes_replay_initial_boundary_and_intermediate_map_measurements() {
    for offset in [0., 1.3] {
        let mut delayed = Ekf::new(Pose::default());
        let mut history = GnssHistory::default();
        history.reset(0.);
        let mut delivered = Vec::new();
        for tick in 0..=12 {
            let now = tick as f64 * 0.05;
            if tick > 0 {
                history
                    .predict(&mut delayed, odometry(now), now - 0.05, now, offset)
                    .unwrap();
            }
            let acquired = match tick {
                8 => Some(0.),
                9 => Some(0.125),
                10 => Some(0.2),
                11 => Some(0.3),
                _ => None,
            };
            if let Some(acquired) = acquired {
                assert_eq!(
                    history.correct(&mut delayed, fix(acquired), now),
                    Ok(GnssDecision::Accepted)
                );
                delivered.push(acquired);
            }
            assert_eq!(
                history.correct_map_pose(&mut delayed, map_pose(now), MAP_COVARIANCE, now),
                Ok(true)
            );

            // Unbounded chronological reference; no producer snapshots.
            let mut expected = Ekf::new(Pose::default());
            for step in 0..=tick {
                let end = step as f64 * 0.05;
                let mut cursor = if step == 0 { 0. } else { end - 0.05 };
                for &stamp in &delivered {
                    if (step == 0 && stamp == 0.)
                        || (step > 0 && stamp > cursor + EPS && stamp <= end + EPS)
                    {
                        if stamp - cursor > EPS {
                            advance(&mut expected, odometry(end), stamp - cursor, offset);
                        }
                        assert!(expected.update(fix(stamp)));
                        cursor = stamp;
                    }
                }
                if end - cursor > EPS {
                    advance(&mut expected, odometry(end), end - cursor, offset);
                }
                assert!(expected.correct_map_pose(map_pose(end), MAP_COVARIANCE));
            }
            same(&expected, &delayed);
        }
        assert!(history.replayed_map_corrections() >= 20);
        assert_eq!(delayed.diagnostics.accepted_fixes, 4);
    }
}

#[test]
fn conflicting_replayed_map_correction_refuses_the_whole_gnss_transaction() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::default();
    history.reset(0.);
    assert_eq!(
        history.correct_map_pose(&mut filter, Pose::default(), MAP_COVARIANCE, 0.),
        Ok(true)
    );
    history
        .predict(&mut filter, odometry(0.1), 0., 0.1, 0.)
        .unwrap();
    let before = filter.clone();
    let gnss = Gnss {
        position: Vec2::new(5., 0.),
        variance: 0.001,
        ..fix(0.)
    };
    // The initial GNSS gate alone accepts this innovation; its conflict with
    // the already admitted map measurement must still reject the transaction.
    assert!(Ekf::new(Pose::default()).update(gnss));
    assert_eq!(
        history.correct(&mut filter, gnss, 0.1),
        Err(HistoryError::MapCorrectionConflict)
    );
    same(&before, &filter);
    assert_eq!(filter.diagnostics.last_observed_stamp, None);
    assert_eq!(history.map_corrections(), 1);
    assert_eq!(history.replayed_map_corrections(), 0);
    assert_eq!(
        history.correct(&mut filter, fix(0.), 0.1),
        Ok(GnssDecision::Accepted)
    );
    assert_eq!(history.replayed_map_corrections(), 1);
}

#[test]
fn rejected_duplicate_or_uncovered_map_observations_cannot_enter_bounded_history() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::default();
    history.reset(0.);
    let invalid = [[0.; 3]; 3];
    assert_eq!(
        history.correct_map_pose(&mut filter, Pose::default(), invalid, 0.),
        Ok(false)
    );
    assert_eq!(history.map_corrections(), 0);
    for tick in 0..=1000 {
        let now = tick as f64 * 0.0001;
        if tick > 0 {
            history
                .predict(
                    &mut filter,
                    Odometry {
                        speed: 0.,
                        ..odometry(now)
                    },
                    now - 0.0001,
                    now,
                    0.,
                )
                .unwrap();
        }
        assert_eq!(
            history.correct_map_pose(&mut filter, Pose::default(), MAP_COVARIANCE, now),
            Ok(true)
        );
        assert!(history.map_corrections() <= MAX_MAP_CORRECTIONS);
        assert!(history.len() <= MAX_PREDICTIONS);
    }
    let before = filter.clone();
    assert_eq!(
        history.correct_map_pose(&mut filter, Pose::default(), MAP_COVARIANCE, 0.1),
        Err(HistoryError::InvalidClockOrMotion)
    );
    assert_eq!(
        history.correct_map_pose(&mut filter, Pose::default(), MAP_COVARIANCE, 0.11),
        Err(HistoryError::InvalidClockOrMotion)
    );
    assert_eq!(
        history.correct(&mut filter, fix(0.), 0.1),
        Err(HistoryError::Unavailable)
    );
    same(&before, &filter);
}
