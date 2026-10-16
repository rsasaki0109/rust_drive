use rustdriving_image_tracking::*;

fn detection(left: f64, score: f64) -> Detection {
    Detection {
        class_index: 2,
        confidence: score,
        bbox: [left, 10., left + 20., 30.],
    }
}
fn frame(time: f64, identity: u64, detections: Vec<Detection>) -> Frame {
    Frame {
        timestamp_s: time,
        acquisition: Acquisition::Detected {
            image_sha256: format!("{identity:064x}"),
            detections,
        },
    }
}
fn tracker() -> Tracker {
    Tracker::new(100, 100, Config::default()).unwrap()
}

#[test]
fn global_assignment_avoids_the_greedy_cardinality_trap() {
    let mut t = tracker();
    let d = |left, score| Detection {
        bbox: [left, 10., left + 10., 20.],
        ..detection(left, score)
    };
    t.step(&frame(0., 1, vec![d(10., 0.9), d(14., 0.8)]))
        .unwrap();
    let s = t
        .step(&frame(0.1, 2, vec![d(11., 0.9), d(7., 0.8)]))
        .unwrap();
    assert_eq!(
        s.tracks
            .iter()
            .map(|v| (v.track_id, v.detection_index))
            .collect::<Vec<_>>(),
        [(1, Some(1)), (2, Some(0))]
    );
    assert!(s.tracks.iter().all(|v| v.phase == Phase::Confirmed));
}

#[test]
fn velocity_retains_crossing_identities_through_short_occlusions() {
    let mut t = tracker();
    for i in 0..3 {
        t.step(&frame(
            i as f64 * 0.1,
            i + 1,
            vec![
                detection(20. + i as f64 * 5., 0.9),
                detection(70. - i as f64 * 5., 0.8),
            ],
        ))
        .unwrap();
    }
    let coast = t
        .step(&Frame {
            timestamp_s: 0.3,
            acquisition: Acquisition::Missing,
        })
        .unwrap();
    assert!(
        coast
            .tracks
            .iter()
            .all(|v| v.phase == Phase::Coasting && v.detection_index.is_none())
    );
    assert_eq!(coast.tracks[0].last_observed_timestamp_s, 0.2);
    t.step(&frame(
        0.4,
        4,
        vec![detection(50., 0.8), detection(40., 0.9)],
    ))
    .unwrap();
    t.step(&Frame {
        timestamp_s: 0.5,
        acquisition: Acquisition::Missing,
    })
    .unwrap();
    let s = t
        .step(&frame(
            0.6,
            5,
            vec![detection(40., 0.8), detection(50., 0.9)],
        ))
        .unwrap();
    assert_eq!(
        s.tracks
            .iter()
            .map(|v| (v.track_id, v.detection_index))
            .collect::<Vec<_>>(),
        [(1, Some(1)), (2, Some(0))]
    );
    assert!((s.tracks[0].velocity_px_s[0] - 50.).abs() < 1e-10);
    assert!((s.tracks[1].velocity_px_s[0] + 50.).abs() < 1e-10);
}

#[test]
fn coasting_never_refreshes_observations_and_expiry_allocates_a_new_id() {
    let mut t = tracker();
    t.step(&frame(0., 1, vec![detection(10., 0.9)])).unwrap();
    t.step(&frame(0.25, 2, vec![detection(10., 0.8)])).unwrap();
    let coast = t
        .step(&Frame {
            timestamp_s: 0.75,
            acquisition: Acquisition::Missing,
        })
        .unwrap();
    assert_eq!(coast.tracks[0].last_observed_timestamp_s, 0.25);
    assert_eq!(coast.tracks[0].last_observed_confidence, 0.8);
    assert_eq!(coast.tracks[0].observation_count, 2);
    assert_eq!(coast.tracks[0].phase, Phase::Coasting);
    let gone = t
        .step(&Frame {
            timestamp_s: 0.75001,
            acquisition: Acquisition::Missing,
        })
        .unwrap();
    assert!(gone.tracks.is_empty());
    assert_eq!(gone.retired[0].reason, RetirementReason::Expired);
    let new = t.step(&frame(0.8, 3, vec![detection(10., 0.9)])).unwrap();
    assert_eq!(
        (new.tracks[0].track_id, new.tracks[0].phase),
        (2, Phase::Tentative)
    );
}

#[test]
fn failed_acquisition_invalidates_tracks_and_does_not_reuse_ids_or_images() {
    let mut t = tracker();
    t.step(&frame(0., 1, vec![detection(10., 0.9)])).unwrap();
    t.step(&frame(0.1, 2, vec![detection(10., 0.9)])).unwrap();
    let failed = t
        .step(&Frame {
            timestamp_s: 0.2,
            acquisition: Acquisition::Failed,
        })
        .unwrap();
    assert_eq!(failed.health, Health::AcquisitionFailed);
    assert!(failed.tracks.is_empty());
    assert_eq!(
        failed.retired[0].reason,
        RetirementReason::AcquisitionFailed
    );
    assert!(t.step(&frame(0.3, 1, vec![])).is_err());
    let resumed = t.step(&frame(0.3, 3, vec![detection(10., 0.9)])).unwrap();
    assert_eq!(resumed.tracks[0].track_id, 2);
    assert_eq!(resumed.tracks[0].phase, Phase::Tentative);
}

#[test]
fn rejected_frames_are_atomic_including_clock_history_and_future_ids() {
    let mut t = tracker();
    t.step(&frame(1., 1, vec![detection(10., 0.9)])).unwrap();
    let before = t.clone();
    let mut invalid = vec![
        frame(1., 2, vec![]),
        frame(0.9, 2, vec![]),
        frame(f64::NAN, 2, vec![]),
        frame(f64::INFINITY, 2, vec![]),
        frame(1.1, 1, vec![]),
        frame(1.1, 2, vec![detection(10., f64::NAN)]),
        frame(1.1, 2, vec![detection(10., 1.1)]),
        frame(1.1, 2, vec![detection(90., 0.9)]),
        frame(1.1, 2, vec![detection(10., 0.9); MAX_DETECTIONS + 1]),
    ];
    let mut bad_class = detection(10., 0.9);
    bad_class.class_index = 80;
    invalid.push(frame(1.1, 2, vec![bad_class]));
    for input in invalid {
        assert!(t.step(&input).is_err());
        assert_eq!(t, before);
    }
    assert_eq!(
        t.step(&frame(1.1, 2, vec![detection(10., 0.9)]))
            .unwrap()
            .tracks[0]
            .track_id,
        1
    );
}

#[test]
fn one_hit_false_positives_retire_on_a_miss_and_classes_never_merge() {
    let mut t = tracker();
    t.step(&frame(0., 1, vec![detection(10., 0.9)])).unwrap();
    let miss = t.step(&frame(0.1, 2, vec![])).unwrap();
    assert_eq!(miss.health, Health::Observed);
    assert!(miss.tracks.is_empty());
    assert_eq!(miss.retired[0].reason, RetirementReason::TentativeMiss);
    t.step(&frame(0.2, 3, vec![detection(10., 0.9)])).unwrap();
    let mut other = detection(10., 0.9);
    other.class_index = 7;
    let changed = t.step(&frame(0.3, 4, vec![other])).unwrap();
    assert_eq!(changed.tracks.len(), 1);
    assert_eq!(
        (changed.tracks[0].track_id, changed.tracks[0].class_index),
        (3, 7)
    );
}

#[test]
fn capacity_preserves_coasting_tracks_and_reports_untracked_detections() {
    let mut t = Tracker::new(
        100,
        100,
        Config {
            max_tracks: 2,
            confirmation_hits: 1,
            ..Config::default()
        },
    )
    .unwrap();
    let first = t
        .step(&frame(
            0.,
            1,
            vec![detection(0., 0.8), detection(30., 0.7), detection(60., 0.9)],
        ))
        .unwrap();
    assert_eq!(first.untracked_detection_indices, [1]);
    assert_eq!(
        first
            .tracks
            .iter()
            .map(|v| v.detection_index)
            .collect::<Vec<_>>(),
        [Some(2), Some(0)]
    );
    let new = Detection {
        bbox: [40., 60., 60., 80.],
        ..detection(40., 1.)
    };
    let full = t.step(&frame(0.1, 2, vec![new])).unwrap();
    assert_eq!(full.untracked_detection_indices, [0]);
    assert!(full.tracks.iter().all(|v| v.phase == Phase::Coasting));
}

#[test]
fn dense_maximum_capacity_has_one_to_one_deterministic_bounded_assignment() {
    let mut t = tracker();
    t.step(&frame(0., 1, vec![detection(10., 0.9); 300]))
        .unwrap();
    let initial = t.clone();
    let input = frame(0.1, 2, vec![detection(10., 0.9); 300]);
    let actual = t.step(&input).unwrap();
    let repeated = initial.clone().step(&input).unwrap();
    assert_eq!(actual, repeated);
    assert!(actual.assignment_work <= 60_000_000);
    let mut indices: Vec<_> = actual
        .tracks
        .iter()
        .map(|v| v.detection_index.unwrap())
        .collect();
    indices.sort_unstable();
    assert_eq!(indices, (0..300).collect::<Vec<_>>());
}

#[test]
fn recent_image_identity_cache_is_explicitly_bounded() {
    let mut t = tracker();
    for i in 0..270 {
        t.step(&frame(i as f64, i + 1, vec![])).unwrap();
    }
    assert!(t.step(&frame(270., 270, vec![])).is_err());
    assert!(t.step(&frame(270., 1, vec![])).is_ok());
}

#[test]
fn invalid_geometry_and_unbounded_configuration_are_rejected() {
    assert!(Tracker::new(0, 100, Config::default()).is_err());
    assert!(Tracker::new(5000, 5000, Config::default()).is_err());
    for config in [
        Config {
            coast_seconds: f64::NAN,
            ..Config::default()
        },
        Config {
            max_tracks: 301,
            ..Config::default()
        },
        Config {
            min_iou: 0.,
            ..Config::default()
        },
        Config {
            velocity_gain: 2.,
            ..Config::default()
        },
    ] {
        assert!(Tracker::new(100, 100, config).is_err());
    }
}

#[test]
fn unix_epoch_seconds_are_accepted_without_a_session_specific_clock_origin() {
    let mut t = tracker();
    t.step(&frame(1_800_000_000., 1, vec![detection(10., 0.9)]))
        .unwrap();
    assert_eq!(
        t.step(&frame(1_800_000_000.125, 2, vec![detection(11., 0.9)]))
            .unwrap()
            .tracks[0]
            .track_id,
        1
    );
}
