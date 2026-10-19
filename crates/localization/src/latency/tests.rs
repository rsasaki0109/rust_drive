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
