use rustdriving_core::{LidarScan, Perception, Pose, Vec2};
use rustdriving_perception::LidarClusters;

fn arc(center: Vec2, count: usize, facing: f64, half_span: f64) -> LidarScan {
    LidarScan {
        stamp: 0.,
        sweep: None,
        points: (0..count)
            .map(|i| {
                let a = facing - half_span + 2. * half_span * i as f64 / (count - 1) as f64;
                center.plus(Vec2::new(a.cos(), a.sin()))
            })
            .collect(),
    }
}

fn assert_legacy(scan: &LidarScan, pose: Pose) {
    let sparse = LidarClusters.detect_with_sparse_circles(scan, pose);
    let legacy = LidarClusters.detect(scan, pose);
    assert_eq!(sparse.len(), legacy.len());
    for (a, b) in sparse.iter().zip(legacy) {
        assert_eq!(a.center, b.center);
        assert_eq!(a.radius, b.radius);
    }
}

#[test]
fn sparse_visible_arcs_recover_the_center_instead_of_the_surface_centroid() {
    for count in [3, 4] {
        for bearing in [0_f64, 1.4, -2.8] {
            let center = Vec2::new(40., 0.).rotated(bearing);
            let scan = arc(center, count, std::f64::consts::PI + bearing, 0.5);
            let fitted = LidarClusters.detect_with_sparse_circles(&scan, Pose::default());
            assert_eq!(fitted.len(), 1);
            assert!(fitted[0].center.distance(center) < 1e-9);
            assert!((fitted[0].radius - 1.12).abs() < 1e-9);
            let legacy = LidarClusters.detect(&scan, Pose::default());
            assert!(legacy[0].center.distance(center) > 0.85);
        }
    }
}

#[test]
fn insufficient_curvature_and_the_far_surface_keep_the_existing_proxy() {
    let center = Vec2::new(40., 0.);
    let mut collinear = arc(center, 3, std::f64::consts::PI, 0.5);
    collinear.points = vec![Vec2::new(40., -0.5), center, Vec2::new(40., 0.5)];
    for scan in [
        arc(center, 3, std::f64::consts::PI, 0.2),
        arc(center, 4, 0., 0.5),
        collinear,
    ] {
        assert_legacy(&scan, Pose::default());
    }
}

#[test]
fn sparse_fit_uses_every_return_and_preserves_pose_and_input_order_invariance() {
    let center = Vec2::new(40., 0.);
    let mut scan = arc(center, 4, std::f64::consts::PI, 0.5);
    let pose = Pose {
        position: Vec2::new(3., -8.),
        yaw: 0.8,
    };
    let first = LidarClusters.detect_with_sparse_circles(&scan, pose);
    scan.points.reverse();
    let reversed = LidarClusters.detect_with_sparse_circles(&scan, pose);
    assert!(first[0].center.distance(pose.to_world(center)) < 1e-9);
    assert!(first[0].center.distance(reversed[0].center) < 1e-9);
    assert!((first[0].radius - reversed[0].radius).abs() < 1e-9);
    // One contradictory acquired return must not be dropped to fit a triplet.
    scan.points[1].x += 0.5;
    assert_legacy(&scan, pose);
}

#[test]
fn dense_fits_and_clusters_below_three_points_are_unchanged() {
    for count in [2, 5, 8, 20] {
        let scan = arc(Vec2::new(40., 0.), count, std::f64::consts::PI, 0.5);
        assert_legacy(&scan, Pose::default());
    }
}
