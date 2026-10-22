use rustdriving_core::{Detection, Track, Vec2};
use rustdriving_perception::{Tracker, TrackerUpdateError};

fn d(x: f64, y: f64) -> Detection {
    Detection {
        center: Vec2::new(x, y),
        radius: 0.5,
    }
}

fn fixture() -> Tracker {
    let mut tracker = Tracker::default();
    tracker.try_update(&[d(0., 0.), d(2., 0.)], 0.).unwrap();
    tracker
}

fn snapshot(tracks: &[Track]) -> Vec<(u64, Vec2, Vec2, f64)> {
    tracks
        .iter()
        .map(|t| (t.id, t.position, t.velocity, t.last_seen))
        .collect()
}

#[test]
fn matching_preserves_both_existing_tracks_when_a_greedy_choice_would_create_a_third() {
    for detections in [[d(0.9, 0.), d(-2.1, 0.)], [d(-2.1, 0.), d(0.9, 0.)]] {
        let tracks = fixture().try_update(&detections, 0.5).unwrap();
        assert_eq!(
            tracks.len(),
            2,
            "a feasible two-track assignment was lost: {tracks:?}"
        );
        assert_eq!((tracks[0].id, tracks[1].id), (1, 2));
        assert!((tracks[0].position.x + 1.575).abs() < 1e-12);
        assert!((tracks[1].position.x - 1.175).abs() < 1e-12);
        assert_eq!((tracks[0].last_seen, tracks[1].last_seen), (0.5, 0.5));
    }
}

#[test]
fn matching_minimizes_total_distance_after_preserving_the_number_of_matches() {
    for detections in [[d(1.1, 0.), d(2.9, 0.)], [d(2.9, 0.), d(1.1, 0.)]] {
        let tracks = fixture().try_update(&detections, 0.5).unwrap();
        assert_eq!(tracks.len(), 2);
        // Total acquired-to-predicted distance is 2.0 m; greedy yields 3.8 m.
        assert!((tracks[0].position.x - 0.825).abs() < 1e-12);
        assert!((tracks[1].position.x - 2.675).abs() < 1e-12);
    }
}

#[test]
fn equal_cost_established_track_associations_are_invariant_to_detection_order() {
    let a = fixture()
        .try_update(&[d(1., -0.25), d(1., 0.25)], 0.5)
        .unwrap();
    let b = fixture()
        .try_update(&[d(1., 0.25), d(1., -0.25)], 0.5)
        .unwrap();
    assert_eq!(snapshot(&a), snapshot(&b));
    assert_eq!(a.len(), 2);
    assert!(a.iter().all(|t| t.last_seen == 0.5));
}

#[test]
fn oversized_components_reject_atomically_and_allow_a_repaired_scan_at_the_same_time() {
    let mut t = Tracker::default();
    let before = t.try_update(&[d(0., 0.)], 0.).unwrap();
    let crowded = vec![d(0.2, 0.); 65];
    assert_eq!(
        t.try_update(&crowded, 0.1).unwrap_err(),
        TrackerUpdateError::AssociationLimit
    );
    assert_eq!(snapshot(&t.update(&crowded, 0.1)), snapshot(&before));
    let repaired = t.try_update(&[d(0.2, 0.)], 0.1).unwrap();
    assert_eq!(repaired.len(), 1);
    assert_eq!(repaired[0].id, 1);
    assert!((repaired[0].velocity.x - 0.24).abs() < 1e-12);

    let mut t = Tracker::default();
    let before = t.try_update(&vec![d(0., 0.); 65], 0.).unwrap();
    assert_eq!(
        t.try_update(&[d(0.2, 0.)], 0.1).unwrap_err(),
        TrackerUpdateError::AssociationLimit
    );
    assert_eq!(
        snapshot(&t.try_update(&[], 0.1).unwrap()),
        snapshot(&before)
    );
}

#[test]
fn pair_budget_rejection_preserves_tracks_and_acceptance_time() {
    let mut t = Tracker::default();
    let initial: Vec<_> = (0..1001).map(|i| d(i as f64 * 10., 0.)).collect();
    let before = t.try_update(&initial, 0.).unwrap();
    assert_eq!(
        t.try_update(&initial[..1000], 0.1).unwrap_err(),
        TrackerUpdateError::AssociationLimit
    );
    let repaired = t.try_update(&[d(0.2, 0.)], 0.1).unwrap();
    assert_eq!(repaired.len(), before.len());
    assert_eq!(snapshot(&repaired[1..]), snapshot(&before[1..]));
    assert_eq!(repaired[0].id, 1);
    assert!((repaired[0].velocity.x - 0.24).abs() < 1e-12);
}
