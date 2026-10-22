use rustdriving_core::{Detection, Track, Vec2};
use rustdriving_perception::Tracker;

fn detection(x: f64, y: f64) -> Detection {
    Detection {
        center: Vec2::new(x, y),
        radius: 1.,
    }
}

fn track(tracks: &[Track], id: u64) -> &Track {
    tracks.iter().find(|t| t.id == id).unwrap()
}

#[test]
fn missed_scans_and_other_objects_cannot_change_a_reacquired_tracks_velocity() {
    for distractor in [false, true] {
        let mut sparse = Tracker::default();
        let mut busy = Tracker::default();
        let first = detection(0., 0.);
        sparse.update(std::slice::from_ref(&first), 0.);
        busy.update(&[first], 0.);
        for stamp in [0.1, 0.2, 0.3, 0.4] {
            let other = if distractor {
                vec![detection(20., 10.)]
            } else {
                vec![]
            };
            busy.update(&other, stamp);
        }
        let next = detection(1., 0.5);
        let a = sparse.update(std::slice::from_ref(&next), 0.5);
        let b = busy.update(&[next], 0.5);
        let (a, b) = (track(&a, 1), track(&b, 1));
        assert_eq!(a.position, b.position);
        assert_eq!(a.velocity, b.velocity);
        assert_eq!(a.last_seen, b.last_seen);
        // Zero prior velocity and 0.5 s between observations: beta = 0.12
        // corrects 12% of the measured 2 m/s x, 1 m/s y displacement rate.
        assert!((b.velocity.x - 0.24).abs() < 1e-12);
        assert!((b.velocity.y - 0.12).abs() < 1e-12);
    }
}

#[test]
fn each_track_uses_its_own_interval_in_the_same_reacquisition_scan() {
    let mut t = Tracker::default();
    t.update(&[detection(0., 0.)], 0.);
    t.update(&[detection(20., 0.)], 0.3);
    let result = t.update(&[detection(1., 0.), detection(20.4, 0.)], 0.5);
    assert_eq!(result.len(), 2);
    assert!((track(&result, 1).velocity.x - 0.24).abs() < 1e-12);
    assert!((track(&result, 2).velocity.x - 0.24).abs() < 1e-12);
}

#[test]
fn long_empty_waits_expire_identity_and_never_create_a_velocity_on_reappearance() {
    let mut t = Tracker::default();
    t.update(&[detection(0., 0.)], 0.);
    for i in 1..=2000 {
        let stamp = i as f64 * 0.1;
        let tracks = t.update(&[], stamp);
        if i < 6 {
            assert_eq!(tracks.len(), 1);
            assert_eq!(tracks[0].last_seen, 0.);
        } else {
            assert!(tracks.is_empty());
        }
    }
    let result = t.update(&[detection(1., 0.)], 200.1);
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].id, 2);
    assert_eq!(result[0].velocity, Vec2::default());
    assert_eq!(result[0].last_seen, 200.1);
}
