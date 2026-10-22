use rustdriving_core::{Detection, Track, Vec2};
use rustdriving_perception::Tracker;

fn detection(position: Vec2, radius: f64) -> Detection {
    Detection {
        center: position,
        radius,
    }
}

fn equal(a: &[Track], b: &[Track]) {
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
fn recent_acquisitions_follow_acceleration_after_initial_confirmation() {
    let mut recent = Tracker::with_recent_motion_fit().with_measurement_enclosure();
    let mut legacy = Tracker::default().with_measurement_enclosure();
    for i in 0..=10 {
        let time = i as f64 * 0.1;
        let observations = [detection(Vec2::new(time * time, 0.), 1.)];
        let actual = recent.try_update(&observations, time).unwrap();
        let old = legacy.try_update(&observations, time).unwrap();
        if i >= 5 {
            // A line through a uniformly sampled quadratic estimates velocity
            // at the middle acquisition. The endpoint position bias is -a*h²/6.
            assert!((actual[0].velocity.x - (2. * time - 0.2)).abs() < 1e-10);
            assert!((actual[0].position.x - (time * time - 0.01 / 3.)).abs() < 1e-10);
            assert!(
                actual[0].position.distance(observations[0].center) + 1.
                    <= actual[0].radius + 1e-10
            );
        }
        if i == 10 {
            assert!((old[0].velocity.x - 2.).abs() > 0.5);
        }
        assert_eq!(actual[0].last_seen, time);
    }
}

#[test]
fn small_static_jitter_and_unconfirmed_slow_motion_preserve_ordinary_filtering() {
    for moving in [false, true] {
        let mut recent = Tracker::with_recent_motion_fit();
        let mut legacy = Tracker::default();
        for i in 0..100 {
            let time = i as f64 * 0.1;
            let x = if moving {
                0.6 * time
            } else if i % 2 == 0 {
                0.02
            } else {
                -0.02
            };
            let observations = [detection(Vec2::new(x, 0.), 1.)];
            equal(
                &recent.try_update(&observations, time).unwrap(),
                &legacy.try_update(&observations, time).unwrap(),
            );
        }
    }
}

#[test]
fn a_constant_velocity_fit_is_independent_of_scan_rate_and_coordinate_frame() {
    for times in [
        vec![0., 0.1, 0.2, 0.3],
        vec![0., 0.05, 0.1, 0.15, 0.2],
        vec![0., 0.07, 0.23, 0.4],
    ] {
        let mut tracker = Tracker::with_recent_motion_fit();
        let velocity = Vec2::new(3., -2.).rotated(0.7);
        let origin = Vec2::new(1200., -500.);
        for time in times {
            let position = origin.plus(velocity.scaled(time));
            let tracks = tracker
                .try_update(&[detection(position, 1.)], time)
                .unwrap();
            if time >= 0.2 - 1e-9 {
                assert!(tracks[0].position.distance(position) < 1e-9);
                assert!(tracks[0].velocity.distance(velocity) < 1e-9);
            }
        }
    }
}

#[test]
fn unstable_shape_curvature_and_excess_speed_keep_ordinary_updates() {
    for samples in [
        [(0., 1.), (0.2, 1.2), (0.4, 1.)],
        [(0., 1.), (1., 1.), (0., 1.)],
        [(0., 1.), (2., 1.), (4., 1.)],
    ] {
        let mut recent = Tracker::with_recent_motion_fit();
        let mut legacy = Tracker::default();
        for (i, (x, radius)) in samples.into_iter().enumerate() {
            let observations = [detection(Vec2::new(x, 0.), radius)];
            let time = i as f64 * 0.1;
            equal(
                &recent.try_update(&observations, time).unwrap(),
                &legacy.try_update(&observations, time).unwrap(),
            );
        }
    }
}

#[test]
fn rejected_and_empty_scans_cannot_resample_support() {
    let mut recent = Tracker::with_recent_motion_fit();
    recent
        .try_update(&[detection(Vec2::default(), 1.)], 0.)
        .unwrap();
    recent
        .try_update(&[detection(Vec2::new(0.4, 0.), 1.)], 0.1)
        .unwrap();
    let mut clean = recent.clone();
    assert!(recent.try_update(&[], 0.1).is_err());
    assert!(
        recent
            .try_update(&[detection(Vec2::new(f64::NAN, 0.), 1.)], 0.15)
            .is_err()
    );
    recent.try_update(&[], 0.15).unwrap();
    let observations = [detection(Vec2::new(0.8, 0.), 1.)];
    let actual = recent.try_update(&observations, 0.2).unwrap();
    equal(&actual, &clean.try_update(&observations, 0.2).unwrap());
    assert!((actual[0].velocity.x - 4.).abs() < 1e-10);
}

#[test]
fn an_uncovered_acquisition_gap_drops_old_fit_support_without_renewing_identity() {
    let mut tracker = Tracker::with_recent_motion_fit();
    for time in [0., 0.1, 0.2] {
        tracker
            .try_update(&[detection(Vec2::new(4. * time, 0.), 1.)], time)
            .unwrap();
    }
    let tracks = tracker
        .try_update(&[detection(Vec2::new(2., 0.), 1.)], 0.6)
        .unwrap();
    // The old track persists, but a new fit cannot bridge the 0.4 s gap.
    // Ordinary alpha-beta: p=2.4+0.75*(2-2.4), v=4+0.12*(2-2.4)/0.4.
    assert_eq!(tracks[0].id, 1);
    assert!((tracks[0].position.x - 2.1).abs() < 1e-10);
    assert!((tracks[0].velocity.x - 3.88).abs() < 1e-10);
    tracker.try_update(&[], 1.21).unwrap();
    let reborn = tracker
        .try_update(&[detection(Vec2::new(2., 0.), 1.)], 1.3)
        .unwrap();
    assert_eq!(reborn[0].id, 2);
    assert_eq!(reborn[0].velocity, Vec2::default());
}
