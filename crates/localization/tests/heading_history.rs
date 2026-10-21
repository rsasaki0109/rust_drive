use rustdriving_core::{
    AbsoluteHeading, Gnss, GnssDecision, HeadingDecision, Odometry, Pose, Vec2,
};
use rustdriving_localization::{
    Ekf,
    latency::{GnssHistory, HistoryError, MAX_HEADING_CORRECTIONS},
};

const COVARIANCE: [[f64; 3]; 3] = [[0.02, 0., 0.], [0., 0.02, 0.], [0., 0., 0.0004]];
fn motion(stamp: f64) -> Odometry {
    Odometry {
        stamp,
        speed: 6.,
        yaw_rate: 0.2,
    }
}
fn pose(stamp: f64) -> Pose {
    Pose {
        position: Vec2::new(30. * (0.2 * stamp).sin(), 30. * (1. - (0.2 * stamp).cos())),
        yaw: 0.2 * stamp,
    }
}
fn heading(stamp: f64) -> AbsoluteHeading {
    AbsoluteHeading {
        stamp,
        yaw_rad: 0.2 * stamp,
        variance_rad2: 0.0004,
    }
}
fn fix(stamp: f64) -> Gnss {
    Gnss {
        stamp,
        position: pose(stamp).position,
        variance: 0.04,
    }
}
fn same(a: &Ekf, b: &Ekf) {
    assert!(a.state().pose.position.distance(b.state().pose.position) < 1e-12);
    assert!((a.state().pose.yaw - b.state().pose.yaw).abs() < 1e-12);
    assert!((a.state().speed - b.state().speed).abs() < 1e-12);
    for (i, row) in a.pose_covariance().iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            assert!((value - b.pose_covariance()[i][j]).abs() < 1e-12);
        }
    }
}

// Full chronological reconstruction has no dependency on history checkpoints.
fn chronological(
    fixes: &[Gnss],
    headings: &[AbsoluteHeading],
    maps: &[f64],
    now: f64,
    offset: f64,
) -> Ekf {
    let mut events: Vec<_> = fixes
        .iter()
        .map(|g| (g.stamp, 1, Some(*g), None))
        .chain(headings.iter().map(|h| (h.stamp, 2, None, Some(*h))))
        .chain(maps.iter().map(|t| (*t, 3, None, None)))
        .collect();
    events.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut filter = Ekf::new(Pose::default());
    let mut index = 0;
    for tick in 0..=(now / 0.05).round() as usize {
        let end = tick as f64 * 0.05;
        let mut cursor = if tick == 0 {
            0.
        } else {
            (tick - 1) as f64 * 0.05
        };
        while index < events.len() && events[index].0 <= end + 1e-9 {
            let (stamp, kind, gnss, sample) = events[index];
            if stamp - cursor > 1e-9 {
                filter.predict_chassis(motion(end), stamp - cursor, offset);
            }
            match kind {
                1 => assert_eq!(filter.correct(gnss.unwrap()), GnssDecision::Accepted),
                2 => assert_eq!(
                    filter.correct_heading(sample.unwrap()),
                    HeadingDecision::Accepted
                ),
                _ => assert!(filter.correct_map_pose(pose(stamp), COVARIANCE)),
            }
            cursor = stamp;
            index += 1;
        }
        if end - cursor > 1e-9 {
            filter.predict_chassis(motion(end), end - cursor, offset);
        }
    }
    assert_eq!(index, events.len());
    filter
}

#[test]
fn fractional_gnss_and_both_map_arrival_orders_retain_every_heading_and_covariance() {
    for gnss_delay in [1, 4] {
        for map_delay in [0, 2] {
            for offset in [0., 1.3] {
                let mut filter = Ekf::new(Pose::default());
                let mut history = GnssHistory::with_retained_corrections();
                history.reset(0.);
                let (mut fixes, mut headings, mut maps) = (vec![], vec![], vec![]);
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
                        let sample = fix((tick - gnss_delay) as f64 * 0.05 + 0.025);
                        assert_eq!(
                            history.correct(&mut filter, sample, now),
                            Ok(GnssDecision::Accepted)
                        );
                        fixes.push(sample);
                    }
                    if tick % 2 == 0 {
                        assert_eq!(
                            history.correct_heading(&mut filter, heading(now), now),
                            Ok(HeadingDecision::Accepted)
                        );
                        headings.push(heading(now));
                    }
                    if tick >= map_delay && (tick - map_delay) % 2 == 0 {
                        let stamp = (tick - map_delay) as f64 * 0.05;
                        let prior = history.map_prior_pose(&filter, stamp, now).unwrap();
                        let expected = chronological(
                            &fixes
                                .iter()
                                .copied()
                                .filter(|g| g.stamp <= stamp)
                                .collect::<Vec<_>>(),
                            &headings
                                .iter()
                                .copied()
                                .filter(|h| h.stamp <= stamp)
                                .collect::<Vec<_>>(),
                            &maps
                                .iter()
                                .copied()
                                .filter(|t| *t < stamp)
                                .collect::<Vec<_>>(),
                            stamp,
                            offset,
                        );
                        assert!(prior.position.distance(expected.state().pose.position) < 1e-12);
                        assert!((prior.yaw - expected.state().pose.yaw).abs() < 1e-12);
                        assert_eq!(
                            history.correct_map_pose_at(
                                &mut filter,
                                pose(stamp),
                                COVARIANCE,
                                stamp,
                                now
                            ),
                            Ok(true)
                        );
                        maps.push(stamp);
                    }
                    same(
                        &filter,
                        &chronological(&fixes, &headings, &maps, now, offset),
                    );
                    assert_eq!(
                        filter.heading_diagnostics().accepted_samples,
                        headings.len() as u64
                    );
                    assert_eq!(
                        filter.heading_diagnostics().last_accepted_stamp,
                        headings.last().map(|h| h.stamp)
                    );
                    assert_eq!(filter.diagnostics().accepted_fixes, fixes.len() as u64);
                    assert_eq!(
                        filter.diagnostics().last_accepted_stamp,
                        fixes.last().map(|g| g.stamp)
                    );
                }
                if gnss_delay == 4 || map_delay == 2 {
                    assert!(history.replayed_heading_corrections() > 0);
                } else {
                    assert_eq!(history.replayed_heading_corrections(), 0);
                }
            }
        }
    }
}

#[test]
fn a_rewind_conflicting_with_an_accepted_heading_is_atomic() {
    let prior = [[0.1, 0., 0.], [0., 0.1, 0.04], [0., 0.04, 0.03]];
    let mut filter = Ekf::with_pose_covariance(Pose::default(), prior).unwrap();
    let mut history = GnssHistory::with_retained_corrections();
    history.reset(0.);
    let stopped = Odometry {
        speed: 0.,
        yaw_rate: 0.,
        stamp: 0.05,
    };
    history.predict(&mut filter, stopped, 0., 0.05, 0.).unwrap();
    assert_eq!(
        history.correct_heading(
            &mut filter,
            AbsoluteHeading {
                yaw_rad: 0.,
                variance_rad2: 1e-4,
                ..heading(0.05)
            },
            0.05
        ),
        Ok(HeadingDecision::Accepted)
    );
    let before = filter.clone();
    let counters = (
        history.len(),
        history.heading_corrections(),
        history.replayed_heading_corrections(),
    );
    assert_eq!(
        history.correct(
            &mut filter,
            Gnss {
                position: Vec2::new(0., 1.2),
                variance: 0.001,
                ..fix(0.)
            },
            0.05
        ),
        Err(HistoryError::HeadingCorrectionConflict)
    );
    same(&filter, &before);
    assert_eq!(
        format!("{:?}", filter.heading_diagnostics()),
        format!("{:?}", before.heading_diagnostics())
    );
    assert_eq!(
        format!("{:?}", filter.diagnostics()),
        format!("{:?}", before.diagnostics())
    );
    assert_eq!(
        counters,
        (
            history.len(),
            history.heading_corrections(),
            history.replayed_heading_corrections()
        )
    );
    assert_eq!(
        history.correct(&mut filter, fix(0.), 0.05),
        Ok(GnssDecision::Accepted)
    );
}

#[test]
fn replay_preserves_rejected_heading_evidence_and_never_renews_its_clock() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::with_retained_corrections();
    history.reset(0.);
    assert_eq!(
        history.correct_heading(&mut filter, heading(0.), 0.),
        Ok(HeadingDecision::Accepted)
    );
    history
        .predict(&mut filter, motion(0.05), 0., 0.05, 0.)
        .unwrap();
    assert_eq!(
        history.correct_heading(
            &mut filter,
            AbsoluteHeading {
                yaw_rad: 0.8,
                ..heading(0.05)
            },
            0.05
        ),
        Ok(HeadingDecision::RejectedInnovation)
    );
    let evidence = format!("{:?}", filter.heading_diagnostics());
    history
        .predict(&mut filter, motion(0.1), 0.05, 0.1, 0.)
        .unwrap();
    assert_eq!(
        history.correct(&mut filter, fix(0.025), 0.1),
        Ok(GnssDecision::Accepted)
    );
    assert_eq!(format!("{:?}", filter.heading_diagnostics()), evidence);
    assert_eq!(history.heading_corrections(), 1);
    assert_eq!(
        history.correct_heading(&mut filter, heading(0.05), 0.1),
        Ok(HeadingDecision::IgnoredTimestamp)
    );
    assert_eq!(filter.heading_diagnostics().last_accepted_stamp, Some(0.));
}

#[test]
fn invalid_or_unsynchronized_heading_cannot_change_state_and_history_stays_bounded() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::with_retained_corrections();
    history.reset(0.);
    for sample in [
        heading(0.01),
        AbsoluteHeading {
            variance_rad2: f64::NAN,
            ..heading(0.)
        },
    ] {
        let before = filter.clone();
        assert_ne!(
            history.correct_heading(&mut filter, sample, 0.),
            Ok(HeadingDecision::Accepted)
        );
        same(&filter, &before);
        assert_eq!(history.heading_corrections(), 0);
    }
    for i in 0..=150 {
        let now = i as f64 * 0.001;
        if i > 0 {
            history
                .predict(&mut filter, motion(now), (i - 1) as f64 * 0.001, now, 0.)
                .unwrap();
        }
        assert_eq!(
            history.correct_heading(&mut filter, heading(now), now),
            Ok(HeadingDecision::Accepted)
        );
        assert!(history.heading_corrections() <= MAX_HEADING_CORRECTIONS);
        assert!(history.len() <= 64);
    }
    // Count pruning folded this boundary heading into the first remaining
    // prediction's snapshot. An equal-time GNSS must precede that heading,
    // so the unavailable prior must be refused rather than corrected twice.
    let boundary = (150 - 64) as f64 * 0.001;
    let before = filter.clone();
    assert_eq!(
        history.correct(&mut filter, fix(boundary), 0.15),
        Err(HistoryError::Unavailable)
    );
    same(&filter, &before);
    assert_eq!(
        history.correct(&mut filter, fix(boundary + 0.0005), 0.15),
        Ok(GnssDecision::Accepted)
    );
    history.reset(1.);
    assert_eq!(history.heading_corrections(), 0);
    assert_eq!(
        history.correct(&mut filter, fix(0.9), 1.),
        Err(HistoryError::Unavailable)
    );
}

#[test]
fn delayed_heading_matches_full_chronology_across_gnss_map_order_and_fractional_motion() {
    for gnss_delay in [0, 6] {
        for heading_delay in [1, 4] {
            for fractional in [false, true] {
                for offset in [0., 1.3] {
                    let mut filter = Ekf::new(Pose::default());
                    let mut history = GnssHistory::with_retained_corrections();
                    history.reset(0.);
                    let (mut fixes, mut headings, mut maps) = (vec![], vec![], vec![]);
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
                            let sample = fix((tick - gnss_delay) as f64 * 0.05);
                            assert_eq!(
                                history.correct(&mut filter, sample, now),
                                Ok(GnssDecision::Accepted)
                            );
                            fixes.push(sample);
                        }
                        if tick >= heading_delay && (tick - heading_delay) % 2 == 0 {
                            let stamp = (tick - heading_delay) as f64 * 0.05
                                + if fractional { 0.025 } else { 0. };
                            let evidence = format!("{:?}", filter.diagnostics());
                            assert_eq!(
                                history.correct_heading_at(&mut filter, heading(stamp), now),
                                Ok(HeadingDecision::Accepted)
                            );
                            assert_eq!(format!("{:?}", filter.diagnostics()), evidence);
                            headings.push(heading(stamp));
                        }
                        if tick >= 2 && tick % 2 == 0 {
                            let stamp = (tick - 2) as f64 * 0.05;
                            assert_eq!(
                                history.correct_map_pose_at(
                                    &mut filter,
                                    pose(stamp),
                                    COVARIANCE,
                                    stamp,
                                    now
                                ),
                                Ok(true)
                            );
                            maps.push(stamp);
                        }
                        same(
                            &filter,
                            &chronological(&fixes, &headings, &maps, now, offset),
                        );
                        assert_eq!(
                            filter.heading_diagnostics().accepted_samples,
                            headings.len() as u64
                        );
                        assert_eq!(
                            filter.heading_diagnostics().last_accepted_stamp,
                            headings.last().map(|h| h.stamp)
                        );
                        assert!(history.len() <= 64);
                        assert!(history.heading_corrections() <= MAX_HEADING_CORRECTIONS);
                    }
                }
            }
        }
    }
}

#[test]
fn delayed_heading_conflicts_with_future_gnss_or_map_roll_back_the_entire_transaction() {
    for map in [false, true] {
        let prior = [[0.1, 0., 0.], [0., 0.055, 0.04], [0., 0.04, 0.03]];
        let mut filter = Ekf::with_pose_covariance(Pose::default(), prior).unwrap();
        let mut history = GnssHistory::with_retained_corrections();
        history.reset(0.);
        history
            .predict(
                &mut filter,
                Odometry {
                    stamp: 0.05,
                    speed: 0.,
                    yaw_rate: 0.,
                },
                0.,
                0.05,
                0.,
            )
            .unwrap();
        if map {
            assert_eq!(
                history.correct_map_pose(&mut filter, Pose::default(), COVARIANCE, 0.05),
                Ok(true)
            );
        } else {
            assert_eq!(
                history.correct(
                    &mut filter,
                    Gnss {
                        stamp: 0.05,
                        position: Vec2::default(),
                        variance: 0.001
                    },
                    0.05
                ),
                Ok(GnssDecision::Accepted)
            );
        }
        let before = filter.clone();
        let counters = (
            history.len(),
            history.heading_corrections(),
            history.replayed_gnss_corrections(),
            history.replayed_map_corrections(),
            history.replayed_heading_corrections(),
        );
        assert_eq!(
            history.correct_heading_at(
                &mut filter,
                AbsoluteHeading {
                    yaw_rad: 0.4,
                    variance_rad2: 1e-4,
                    ..heading(0.)
                },
                0.05
            ),
            Err(if map {
                HistoryError::MapCorrectionConflict
            } else {
                HistoryError::GnssCorrectionConflict
            })
        );
        same(&filter, &before);
        assert_eq!(
            format!("{:?}", filter.diagnostics()),
            format!("{:?}", before.diagnostics())
        );
        assert_eq!(
            format!("{:?}", filter.heading_diagnostics()),
            format!("{:?}", before.heading_diagnostics())
        );
        assert_eq!(
            counters,
            (
                history.len(),
                history.heading_corrections(),
                history.replayed_gnss_corrections(),
                history.replayed_map_corrections(),
                history.replayed_heading_corrections()
            )
        );
        assert_eq!(
            history.correct_heading_at(&mut filter, heading(0.), 0.05),
            Ok(HeadingDecision::Accepted)
        );
    }
}

#[test]
fn heading_delay_limit_future_input_and_uncovered_motion_preserve_live_state() {
    let mut filter = Ekf::new(Pose::default());
    let mut history = GnssHistory::with_retained_corrections();
    history.reset(0.);
    history
        .predict(&mut filter, motion(0.2), 0., 0.2, 0.)
        .unwrap();
    for sample in [
        heading(0.21),
        AbsoluteHeading {
            stamp: f64::NAN,
            ..heading(0.)
        },
    ] {
        let before = filter.clone();
        assert_ne!(
            history.correct_heading_at(&mut filter, sample, 0.2),
            Ok(HeadingDecision::Accepted)
        );
        same(&filter, &before);
        assert_eq!(history.heading_corrections(), 0);
    }
    assert_eq!(
        history.correct_heading_at(&mut filter, heading(0.), 0.2),
        Ok(HeadingDecision::Accepted)
    );
    history
        .predict(&mut filter, motion(0.45), 0.2, 0.45, 0.)
        .unwrap();
    let before = filter.clone();
    assert_eq!(
        history.correct_heading_at(&mut filter, heading(0.249), 0.45),
        Err(HistoryError::Unavailable)
    );
    same(&filter, &before);
    history.reset(1.);
    assert_eq!(
        history.correct_heading_at(&mut filter, heading(0.9), 1.),
        Err(HistoryError::Unavailable)
    );
    same(&filter, &before);
}
