use rustdriving_core::{Detection, Track, Vec2};
use rustdriving_perception::{Tracker, TrackerUpdateError};

fn detection(x: f64, y: f64, radius: f64) -> Detection {
    Detection {
        center: Vec2::new(x, y),
        radius,
    }
}
fn same(a: &[Track], b: &[Track]) {
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(b) {
        assert_eq!(a.id, b.id);
        assert_eq!(a.position, b.position);
        assert_eq!(a.velocity, b.velocity);
        assert_eq!(a.radius, b.radius);
        assert_eq!(a.last_seen, b.last_seen);
    }
}

#[test]
fn irregular_acquisitions_fit_two_dimensional_motion_and_current_position() {
    let mut tracker = Tracker::with_confirmed_initialization();
    for stamp in [10., 10.07, 10.23] {
        let p = Vec2::new(4., -2.).plus(Vec2::new(2., -1.).scaled(stamp - 10.));
        let tracks = tracker
            .try_update(&[detection(p.x, p.y, 1.)], stamp)
            .unwrap();
        if stamp < 10.2 {
            assert_eq!(tracks[0].velocity, Vec2::default());
        } else {
            assert!(tracks[0].velocity.distance(Vec2::new(2., -1.)) < 1e-12);
            assert!(tracks[0].position.distance(p) < 1e-12);
        }
        assert_eq!(tracker.measurements()[0].stamp, stamp);
        assert_eq!(tracker.measurements()[0].center, p);
    }
    let tracks = tracker
        .try_update(&[detection(4.66, -2.33, 1.)], 10.33)
        .unwrap();
    assert!(tracks[0].velocity.distance(Vec2::new(2., -1.)) < 1e-12);
}

#[test]
fn shape_transition_does_not_manufacture_stationary_object_velocity() {
    let mut confirmed = Tracker::with_confirmed_initialization();
    let mut legacy = Tracker::default();
    for (i, (x, radius)) in [(9.2, 0.85), (10., 1.12), (10., 1.12), (10., 1.12)]
        .into_iter()
        .enumerate()
    {
        let detections = [detection(x, 0., radius)];
        let stamp = i as f64 * 0.1;
        let tracks = confirmed.try_update(&detections, stamp).unwrap();
        let previous = legacy.try_update(&detections, stamp).unwrap();
        assert_eq!(tracks[0].velocity, Vec2::default());
        if i == 3 {
            assert!(tracks[0].position.distance(Vec2::new(10., 0.)) < 1e-12);
            assert!(previous[0].velocity.x > 0.9);
        }
    }
    assert!(legacy.measurements().is_empty());
}

#[test]
fn empty_scans_and_other_objects_do_not_renew_or_resample_support() {
    let mut tracker = Tracker::with_confirmed_initialization();
    tracker.try_update(&[detection(0., 0., 1.)], 0.).unwrap();
    tracker.try_update(&[detection(20., 0., 1.)], 0.05).unwrap();
    tracker.try_update(&[], 0.1).unwrap();
    assert!(tracker.measurements().is_empty());
    let middle = tracker
        .try_update(&[detection(0.3, -0.15, 1.)], 0.15)
        .unwrap();
    assert_eq!(middle[0].last_seen, 0.15);
    assert_eq!(middle[0].velocity, Vec2::default());
    let end = tracker
        .try_update(&[detection(0.7, -0.35, 1.)], 0.35)
        .unwrap();
    assert!(end[0].velocity.distance(Vec2::new(2., -1.)) < 1e-12);
    assert_eq!(tracker.measurements().len(), 1);
    assert_eq!(tracker.measurements()[0].id, 1);
    tracker.try_update(&[], 0.96).unwrap();
    let renewed = tracker.try_update(&[detection(0.8, -0.4, 1.)], 1.).unwrap();
    assert_eq!(renewed[0].id, 3);
    assert_eq!(renewed[0].velocity, Vec2::default());
}

#[test]
fn rejected_scans_preserve_pending_fit_measurements_and_identity() {
    let mut a = Tracker::with_confirmed_initialization();
    a.try_update(&[detection(0., 0., 1.)], 0.).unwrap();
    a.try_update(&[detection(0.2, 0., 1.)], 0.1).unwrap();
    let mut b = a.clone();
    let measurements = a.measurements().to_vec();
    for stamp in [f64::NAN, f64::INFINITY, -0.1, 0., 0.1] {
        assert_eq!(
            a.try_update(&[], stamp).unwrap_err(),
            TrackerUpdateError::InvalidTimestamp
        );
    }
    assert_eq!(
        a.try_update(&[detection(1., 0., 1.), detection(f64::NAN, 0., 1.)], 0.2)
            .unwrap_err(),
        TrackerUpdateError::InvalidDetection { index: 1 }
    );
    assert_eq!(a.measurements(), measurements);
    let input = [detection(0.4, 0., 1.), detection(20., 0., 1.)];
    let actual = a.try_update(&input, 0.2).unwrap();
    same(&actual, &b.try_update(&input, 0.2).unwrap());
    assert!(actual[0].velocity.distance(Vec2::new(2., 0.)) < 1e-12);
}

#[test]
fn excessive_curvature_speed_or_unstable_shape_cannot_confirm() {
    for observations in [
        [(0., 1.), (1., 1.), (0., 1.)],
        [(0., 1.), (2., 1.), (4., 1.)],
        [(0., 1.), (0.2, 1.2), (0.4, 1.)],
    ] {
        let mut tracker = Tracker::with_confirmed_initialization();
        for (i, (x, radius)) in observations.into_iter().enumerate() {
            let tracks = tracker
                .try_update(&[detection(x, 0., radius)], i as f64 * 0.1)
                .unwrap();
            assert_eq!(tracks[0].velocity, Vec2::default());
        }
    }
}

#[test]
fn fit_is_invariant_to_coordinate_origin_and_yaw() {
    let offset = Vec2::new(1200., -500.);
    let yaw = 0.7;
    let mut tracker = Tracker::with_confirmed_initialization();
    for stamp in [0., 0.11, 0.31] {
        let p = Vec2::new(3. * stamp, -2. * stamp).rotated(yaw).plus(offset);
        let tracks = tracker
            .try_update(&[detection(p.x, p.y, 1.)], stamp)
            .unwrap();
        if stamp == 0.31 {
            assert!(tracks[0].velocity.distance(Vec2::new(3., -2.).rotated(yaw)) < 1e-9);
            assert!(tracks[0].position.distance(p) < 1e-9);
        }
    }
}
