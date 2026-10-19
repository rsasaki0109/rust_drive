use super::*;
use rustdriving_core::Vec2;

const COVARIANCE: [[f64; 3]; 3] = [
    [0.02, 0.002, 0.0001],
    [0.002, 0.03, -0.0002],
    [0.0001, -0.0002, 0.0004],
];
fn motion(stamp: f64) -> Odometry {
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
fn map(stamp: f64) -> Pose {
    Pose {
        position: Vec2::new(30. * (0.2 * stamp).sin(), 30. * (1. - (0.2 * stamp).cos())),
        yaw: 0.2 * stamp,
    }
}
fn same(a: &Ekf, b: &Ekf) {
    assert!(
        a.state.pose.position.distance(b.state.pose.position) < 1e-12,
        "{:?} != {:?}",
        a.state.pose,
        b.state.pose
    );
    assert!((a.state.pose.yaw - b.state.pose.yaw).abs() < 1e-12);
    assert!((a.state.speed - b.state.speed).abs() < 1e-12);
    for i in 0..3 {
        for j in 0..3 {
            assert!((a.covariance[i][j] - b.covariance[i][j]).abs() < 1e-12);
        }
    }
}

// Full chronological reference independent of retained producer checkpoints.
fn chronological(fixes: &[Gnss], maps: &[(f64, Pose)], now: f64, offset: f64) -> Ekf {
    let mut events: Vec<_> = fixes
        .iter()
        .filter(|g| g.stamp <= now + EPS)
        .map(|g| (g.stamp, 0, Some(*g), None))
        .chain(
            maps.iter()
                .filter(|m| m.0 <= now + EPS)
                .map(|m| (m.0, 1, None, Some(m.1))),
        )
        .collect();
    events.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut result = Ekf::new(Pose::default());
    let mut index = 0;
    for tick in 0..=(now / 0.05).ceil() as usize {
        let end = (tick as f64 * 0.05).min(now);
        let mut cursor = if tick == 0 {
            0.
        } else {
            ((tick - 1) as f64 * 0.05).min(now)
        };
        while index < events.len() && events[index].0 <= end + EPS {
            let (stamp, _, gnss, pose) = events[index];
            if stamp - cursor > EPS {
                advance(&mut result, motion(end), stamp - cursor, offset);
            }
            if let Some(gnss) = gnss {
                assert!(result.update(gnss));
            }
            if let Some(pose) = pose {
                assert!(result.correct_map_pose(pose, COVARIANCE));
            }
            cursor = stamp;
            index += 1;
        }
        if end - cursor > EPS {
            advance(&mut result, motion(end), end - cursor, offset);
        }
    }
    result
}

#[test]
fn both_delivery_orders_match_full_chronology_and_acquisition_pose_for_both_chassis_models() {
    for gnss_delay in [0, 8] {
        for offset in [0., 1.3] {
            let mut filter = Ekf::new(Pose::default());
            let mut history = GnssHistory::with_delayed_maps();
            history.reset(0.);
            let (mut fixes, mut maps) = (vec![], vec![]);
            for tick in 0..=20 {
                let now = tick as f64 * 0.05;
                if tick > 0 {
                    history
                        .predict(
                            &mut filter,
                            motion(now),
                            (tick - 1) as f64 * 0.05,
                            now,
                            offset,
                        )
                        .unwrap();
                }
                if tick >= gnss_delay && (tick - gnss_delay) % 4 == 0 {
                    let observation = fix((tick - gnss_delay) as f64 * 0.05);
                    assert_eq!(
                        history.correct(&mut filter, observation, now),
                        Ok(GnssDecision::Accepted)
                    );
                    fixes.push(observation);
                }
                if tick >= 2 && tick % 2 == 0 {
                    let stamp = (tick - 2) as f64 * 0.05;
                    let prior = history.map_prior_pose(&filter, stamp, now).unwrap();
                    let expected = chronological(&fixes, &maps, stamp, offset);
                    assert!(prior.position.distance(expected.state.pose.position) < 1e-12);
                    assert!((prior.yaw - expected.state.pose.yaw).abs() < 1e-12);
                    assert_eq!(
                        history.correct_map_pose_at(
                            &mut filter,
                            map(stamp),
                            COVARIANCE,
                            stamp,
                            now
                        ),
                        Ok(true)
                    );
                    maps.push((stamp, map(stamp)));
                }
                same(&filter, &chronological(&fixes, &maps, now, offset));
                assert_eq!(filter.diagnostics.accepted_fixes, fixes.len() as u64);
                assert_eq!(
                    filter.diagnostics.last_accepted_stamp,
                    fixes.last().map(|g| g.stamp)
                );
            }
            if gnss_delay == 0 {
                assert!(history.replayed_gnss_corrections() > 0);
            } else {
                assert!(history.replayed_map_corrections() > 0);
            }
        }
    }
}

#[test]
fn fractional_map_time_splits_motion_before_a_later_accepted_gnss() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::with_delayed_maps();
    history.reset(0.);
    for tick in 1..=4 {
        let now = tick as f64 * 0.05;
        history
            .predict(&mut filter, motion(now), (tick - 1) as f64 * 0.05, now, 1.3)
            .unwrap();
        if tick == 2 {
            assert_eq!(
                history.correct(&mut filter, fix(0.1), now),
                Ok(GnssDecision::Accepted)
            );
        }
    }
    let prior = history.map_prior_pose(&filter, 0.075, 0.2).unwrap();
    let expected = chronological(&[], &[], 0.075, 1.3);
    assert!(prior.position.distance(expected.state.pose.position) < 1e-12);
    assert_eq!(
        history.correct_map_pose_at(&mut filter, map(0.075), COVARIANCE, 0.075, 0.2),
        Ok(true)
    );
    same(
        &filter,
        &chronological(&[fix(0.1)], &[(0.075, map(0.075))], 0.2, 1.3),
    );
    assert_eq!(history.replayed_gnss_corrections(), 1);
}

#[test]
fn inconsistent_map_is_atomic_and_cannot_invalidate_an_original_gnss_acceptance() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::with_delayed_maps();
    history.reset(0.);
    history
        .predict(
            &mut filter,
            Odometry {
                speed: 0.,
                yaw_rate: 0.,
                stamp: 0.1,
            },
            0.,
            0.1,
            0.,
        )
        .unwrap();
    assert_eq!(
        history.correct(
            &mut filter,
            Gnss {
                position: Vec2::default(),
                variance: 0.001,
                ..fix(0.1)
            },
            0.1
        ),
        Ok(GnssDecision::Accepted)
    );
    let before = filter.clone();
    let wrong = Pose {
        position: Vec2::new(5., 0.),
        ..Pose::default()
    };
    assert!(Ekf::new(Pose::default()).correct_map_pose(wrong, COVARIANCE));
    assert_eq!(
        history.correct_map_pose_at(&mut filter, wrong, COVARIANCE, 0., 0.1),
        Err(HistoryError::GnssCorrectionConflict)
    );
    same(&filter, &before);
    assert_eq!(filter.diagnostics.last_nis, before.diagnostics.last_nis);
    assert_eq!(filter.diagnostics.last_accepted_stamp, Some(0.1));
    assert_eq!(filter.diagnostics.accepted_fixes, 1);
    assert_eq!(history.map_corrections(), 0);
    assert_eq!(history.gnss_corrections(), 1);
    assert_eq!(history.replayed_gnss_corrections(), 0);
    assert_eq!(
        history.correct_map_pose_at(&mut filter, Pose::default(), COVARIANCE, 0., 0.1),
        Ok(true)
    );
}

#[test]
fn rejected_gnss_is_not_revived_and_original_gate_evidence_remains_owned() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::with_delayed_maps();
    history.reset(0.);
    history
        .predict(
            &mut filter,
            Odometry {
                speed: 0.,
                yaw_rate: 0.,
                stamp: 0.1,
            },
            0.,
            0.1,
            0.,
        )
        .unwrap();
    let rejected = Gnss {
        position: Vec2::new(7., 0.),
        variance: 0.02,
        ..fix(0.1)
    };
    assert_eq!(
        history.correct(&mut filter, rejected, 0.1),
        Ok(GnssDecision::RejectedInnovation)
    );
    let nis = filter.diagnostics.last_nis;
    assert_eq!(
        history.correct_map_pose_at(
            &mut filter,
            Pose {
                position: Vec2::new(6., 0.),
                ..Pose::default()
            },
            [[0.1, 0., 0.], [0., 0.1, 0.], [0., 0., 0.03]],
            0.,
            0.1
        ),
        Ok(true)
    );
    assert_eq!(filter.diagnostics.accepted_fixes, 0);
    assert_eq!(filter.diagnostics.rejected_fixes, 1);
    assert_eq!(filter.diagnostics.last_nis, nis);
    assert_eq!(filter.diagnostics.last_accepted_stamp, None);
    assert_eq!(
        history.correct(&mut filter, rejected, 0.1),
        Ok(GnssDecision::IgnoredTimestamp)
    );
    assert_eq!(history.gnss_corrections(), 0);
}

#[test]
fn expired_uncovered_duplicate_and_invalid_clock_observations_do_not_mutate_history() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::with_delayed_maps();
    history.reset(-1.);
    assert_eq!(
        history.map_prior_pose(&filter, -1., -1.).err(),
        Some(HistoryError::InvalidClockOrMotion)
    );
    history.reset(0.);
    for tick in 0..=200 {
        let now = tick as f64 * 0.0001;
        if tick > 0 {
            history
                .predict(
                    &mut filter,
                    Odometry {
                        stamp: now,
                        speed: 0.,
                        yaw_rate: 0.,
                    },
                    (tick - 1) as f64 * 0.0001,
                    now,
                    0.,
                )
                .unwrap();
        }
        assert_eq!(
            history.correct(
                &mut filter,
                Gnss {
                    position: Vec2::default(),
                    ..fix(now)
                },
                now
            ),
            Ok(GnssDecision::Accepted)
        );
        assert_eq!(
            history.correct_map_pose_at(&mut filter, Pose::default(), COVARIANCE, now, now),
            Ok(true)
        );
        assert!(
            history.len() <= MAX_PREDICTIONS
                && history.gnss_corrections() <= MAX_GNSS_CORRECTIONS
                && history.map_corrections() <= MAX_MAP_CORRECTIONS
        );
    }
    let before = filter.clone();
    assert_eq!(
        history.correct_map_pose_at(&mut filter, Pose::default(), COVARIANCE, 0., 0.02),
        Err(HistoryError::InvalidClockOrMotion)
    );
    assert_eq!(
        history.map_prior_pose(&filter, 0., 0.02).err(),
        Some(HistoryError::Unavailable)
    );
    assert_eq!(
        history.correct_map_pose_at(&mut filter, Pose::default(), COVARIANCE, 0.02, 0.02),
        Err(HistoryError::InvalidClockOrMotion)
    );
    history.reset(1.);
    assert_eq!(
        history.correct_map_pose_at(&mut filter, Pose::default(), COVARIANCE, 0.79, 1.),
        Err(HistoryError::Unavailable)
    );
    assert_eq!(
        history.correct_map_pose_at(&mut filter, Pose::default(), COVARIANCE, 0.9, 1.),
        Err(HistoryError::Unavailable)
    );
    same(&filter, &before);
}
