use rustdriving_core::{Detection, Vec2};
use rustdriving_perception::{Tracker, TrackerUpdateError};

fn detection(x: f64, y: f64, radius: f64) -> Detection {
    Detection {
        center: Vec2::new(x, y),
        radius,
    }
}

#[test]
fn shifted_growing_disc_is_preserved_without_changing_motion() {
    let mut old = Tracker::default();
    let mut new = Tracker::default().with_measurement_enclosure();
    for (stamp, d) in [(0., detection(0., 0., 1.)), (0.1, detection(2., 0., 1.5))] {
        let a = old.try_update(std::slice::from_ref(&d), stamp).unwrap();
        let b = new.try_update(std::slice::from_ref(&d), stamp).unwrap();
        assert_eq!(a[0].position, b[0].position);
        assert_eq!(a[0].velocity, b[0].velocity);
        if stamp > 0. {
            assert_eq!(b[0].position, Vec2::new(1.5, 0.));
            assert_eq!(b[0].radius, 2.);
            assert!((a[0].radius - 1.1).abs() < 1e-12);
        }
    }
    assert_eq!(new.measurements()[0].center, Vec2::new(2., 0.));
}

#[test]
fn a_smaller_measurement_retains_only_the_existing_decay_floor() {
    let mut tracker = Tracker::default().with_measurement_enclosure();
    tracker.try_update(&[detection(0., 0., 3.)], 0.).unwrap();
    let next = tracker.try_update(&[detection(0., 0., 0.25)], 0.1).unwrap();
    assert!((next[0].radius - 2.45).abs() < 1e-12);
    let cached = tracker.try_update(&[], 0.2).unwrap();
    assert_eq!(next[0].radius, cached[0].radius);
    assert_eq!(cached[0].last_seen, 0.1);
    assert!(tracker.measurements().is_empty());
}

#[test]
fn every_boundary_is_contained_with_an_active_minimum_witness_in_any_frame() {
    for yaw in [0., 0.7, -2.] {
        let mut tracker = Tracker::default().with_measurement_enclosure();
        let mut old_radius = None;
        for i in 0..40 {
            let stamp = i as f64 * 0.1;
            let center = Vec2::new(0.1 * i as f64, (0.3 * i as f64).sin())
                .rotated(yaw)
                .plus(Vec2::new(300., -170.));
            let radius = 0.8 + 0.2 * (i as f64).cos();
            let d = Detection { center, radius };
            let track = tracker
                .try_update(std::slice::from_ref(&d), stamp)
                .unwrap()
                .remove(0);
            let floor = old_radius.map_or(radius, |r| 0.8 * r + 0.2 * radius);
            let mut farthest: f64 = 0.;
            let offset = center.minus(track.position);
            let direction = offset.y.atan2(offset.x);
            for j in 0..=64 {
                let angle = direction + j as f64 * std::f64::consts::TAU / 64.;
                let boundary = center.plus(Vec2::new(radius * angle.cos(), radius * angle.sin()));
                let distance = track.position.distance(boundary);
                assert!(distance <= track.radius + 1e-10);
                farthest = farthest.max(distance);
            }
            assert!((track.radius - farthest.max(floor)).abs() < 1e-10);
            old_radius = Some(track.radius);
        }
    }
}

#[test]
fn rejected_batches_preserve_radius_raw_acquisitions_and_the_retry() {
    let mut tracker = Tracker::default().with_measurement_enclosure();
    tracker.try_update(&[detection(0., 0., 1.)], 0.).unwrap();
    let mut control = tracker.clone();
    let before = tracker.measurements().to_vec();
    assert_eq!(
        tracker
            .try_update(&[detection(2., 0., 1.5), detection(f64::NAN, 0., 1.)], 0.1)
            .unwrap_err(),
        TrackerUpdateError::InvalidDetection { index: 1 }
    );
    assert_eq!(tracker.measurements(), before);
    let actual = tracker.try_update(&[detection(2., 0., 1.5)], 0.1).unwrap();
    let expected = control.try_update(&[detection(2., 0., 1.5)], 0.1).unwrap();
    assert_eq!(actual[0].radius, expected[0].radius);
    assert_eq!(actual[0].position, expected[0].position);
    assert_eq!(actual[0].velocity, expected[0].velocity);
    assert_eq!(actual[0].last_seen, expected[0].last_seen);
}

#[test]
fn identity_expiration_does_not_transfer_the_old_extent() {
    let mut tracker = Tracker::default().with_measurement_enclosure();
    tracker.try_update(&[detection(0., 0., 3.)], 0.).unwrap();
    let fresh = tracker.try_update(&[detection(0., 0., 0.5)], 0.6).unwrap();
    assert_eq!(fresh.len(), 1);
    assert_eq!(fresh[0].id, 2);
    assert_eq!(fresh[0].radius, 0.5);
    assert_eq!(fresh[0].velocity, Vec2::default());
}

#[test]
fn enclosure_cannot_change_the_raw_shape_support_for_velocity_confirmation() {
    let mut old = Tracker::with_confirmed_initialization();
    let mut new = Tracker::with_confirmed_initialization().with_measurement_enclosure();
    for i in 0..=4 {
        let stamp = i as f64 * 0.1;
        let d = [detection(
            20. + 2. * stamp,
            0.,
            if i == 0 { 0.85 } else { 1.12 },
        )];
        let a = old.try_update(&d, stamp).unwrap();
        let b = new.try_update(&d, stamp).unwrap();
        assert_eq!(a[0].position, b[0].position);
        assert_eq!(a[0].velocity, b[0].velocity);
        assert_eq!(old.measurements(), new.measurements());
        if i >= 3 {
            assert!(b[0].velocity.distance(Vec2::new(2., 0.)) < 1e-9);
        }
    }
}
