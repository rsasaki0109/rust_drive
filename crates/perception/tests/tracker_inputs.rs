use rustdriving_core::{Detection, Track, Vec2};
use rustdriving_perception::{Tracker, TrackerUpdateError};

fn detection(x: f64) -> Detection {
    Detection {
        center: Vec2::new(x, 0.),
        radius: 1.,
    }
}

fn snapshot(tracks: &[Track]) -> Vec<(u64, Vec2, Vec2, f64, f64)> {
    tracks
        .iter()
        .map(|t| (t.id, t.position, t.velocity, t.radius, t.last_seen))
        .collect()
}

#[test]
fn duplicate_and_reversed_acquisitions_do_not_rewrite_tracks_or_the_next_correction() {
    for stamp in [0.1, 0.05] {
        let (mut actual, mut clean) = (Tracker::default(), Tracker::default());
        for t in [&mut actual, &mut clean] {
            t.update(&[detection(0.)], 0.);
            t.update(&[detection(0.2)], 0.1);
        }
        let before = clean.update(&[], 0.11);
        assert_eq!(
            snapshot(&actual.update(&[detection(1.)], stamp)),
            snapshot(&before)
        );
        assert_eq!(
            snapshot(&actual.update(&[detection(0.4)], 0.2)),
            snapshot(&clean.update(&[detection(0.4)], 0.2))
        );
    }
}

#[test]
fn invalid_clocks_cannot_expire_tracks_or_consume_ids() {
    for stamp in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.1] {
        let mut t = Tracker::default();
        let before = t.update(&[detection(0.)], 1.);
        assert_eq!(snapshot(&t.update(&[], stamp)), snapshot(&before));
        let next = t.update(&[detection(0.2), detection(20.)], 1.1);
        assert_eq!(next.len(), 2);
        assert_eq!((next[0].id, next[1].id), (1, 2));
        assert!((next[0].velocity.x - 0.24).abs() < 1e-12);
    }
}

#[test]
fn one_invalid_detection_rejects_the_whole_batch_without_spending_its_time_or_ids() {
    let mut invalid = Vec::new();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        invalid.push(Detection {
            center: Vec2::new(value, 0.),
            radius: 1.,
        });
        invalid.push(Detection {
            center: Vec2::new(20., 0.),
            radius: value,
        });
    }
    invalid.push(Detection {
        center: Vec2::new(20., 0.),
        radius: -1.,
    });
    for bad in invalid {
        let (mut actual, mut clean) = (Tracker::default(), Tracker::default());
        let before = actual.update(&[detection(0.)], 1.);
        clean.update(&[detection(0.)], 1.);
        assert_eq!(
            snapshot(&actual.update(&[detection(0.2), bad], 1.1)),
            snapshot(&before)
        );
        // Retry the same acquisition time with repaired measurements. No
        // partial correction or newly allocated identity may survive rejection.
        assert_eq!(
            snapshot(&actual.update(&[detection(0.2), detection(20.)], 1.1)),
            snapshot(&clean.update(&[detection(0.2), detection(20.)], 1.1))
        );
    }
}

#[test]
fn successful_empty_scans_advance_acceptance_time_without_renewing_tracks() {
    let mut t = Tracker::default();
    let before = t.update(&[detection(0.)], 0.);
    assert_eq!(snapshot(&t.update(&[], 0.3)), snapshot(&before));
    assert_eq!(
        snapshot(&t.update(&[detection(1.)], 0.2)),
        snapshot(&before)
    );
    let next = t.update(&[detection(1.)], 0.4);
    assert_eq!(next[0].id, 1);
    assert!((next[0].position.x - 0.75).abs() < 1e-12);
    assert!((next[0].velocity.x - 0.3).abs() < 1e-12);
}

#[test]
fn checked_api_reports_errors_and_repaired_inputs_can_retry_the_same_acquisition() {
    let mut t = Tracker::default();
    assert_eq!(
        t.try_update(&[], f64::NAN).unwrap_err(),
        TrackerUpdateError::InvalidTimestamp
    );
    let bad = Detection {
        center: Vec2::new(20., 0.),
        radius: -1.,
    };
    assert_eq!(
        t.try_update(std::slice::from_ref(&bad), 0.).unwrap_err(),
        TrackerUpdateError::InvalidDetection { index: 0 }
    );
    let before = t.try_update(&[detection(0.)], 0.).unwrap();
    // This invalid future-dated batch must not expire the old observation.
    assert_eq!(
        t.try_update(&[detection(0.2), bad], 20.).unwrap_err(),
        TrackerUpdateError::InvalidDetection { index: 1 }
    );
    assert_eq!(snapshot(&t.update(&[], 0.)), snapshot(&before));
    let good = t
        .try_update(&[detection(0.2), detection(20.)], 0.1)
        .unwrap();
    assert_eq!((good[0].id, good[1].id), (1, 2));
    assert!((good[0].velocity.x - 0.24).abs() < 1e-12);
    assert!(t.try_update(&[], 0.2).is_ok());
    assert_eq!(
        t.try_update(&[detection(0.3)], 0.15).unwrap_err(),
        TrackerUpdateError::InvalidTimestamp
    );
}

#[test]
fn inserting_rejected_scans_preserves_the_entire_accepted_track_sequence() {
    let (mut actual, mut clean) = (Tracker::default(), Tracker::default());
    let mut rejected = 0;
    for i in 0..100 {
        let stamp = i as f64 * 0.1;
        let mut detections = vec![detection(i as f64 * 0.15)];
        if i % 10 < 4 {
            detections.push(detection(30. + i as f64 * 0.2));
        }
        assert_eq!(
            snapshot(&actual.try_update(&detections, stamp).unwrap()),
            snapshot(&clean.try_update(&detections, stamp).unwrap())
        );
        for bad_stamp in [stamp, stamp - 0.05, f64::NAN, f64::INFINITY] {
            assert_eq!(
                actual
                    .try_update(&[detection(-20.)], bad_stamp)
                    .unwrap_err(),
                TrackerUpdateError::InvalidTimestamp
            );
            rejected += 1;
        }
        let mut bad = detection(30.);
        bad.center.y = f64::NAN;
        assert_eq!(
            actual
                .try_update(&[detection(-20.), bad], stamp + 10.)
                .unwrap_err(),
            TrackerUpdateError::InvalidDetection { index: 1 }
        );
        rejected += 1;
    }
    assert_eq!(rejected, 500);
}
