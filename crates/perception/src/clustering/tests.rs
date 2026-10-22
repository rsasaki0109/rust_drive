use super::*;
use rustdriving_core::{Detection, LidarScan, Perception, Pose};

mod legacy {
    use rustdriving_core::{Detection, LidarScan, Pose, Vec2};
    include!("../../tests/fixtures/legacy_cluster_detector.rs");

    pub(super) fn detect(scan: &LidarScan, pose: Pose, sparse: bool) -> Vec<Detection> {
        detect_clusters(scan, pose, sparse)
    }
}

// Materialize the complete directed graph before traversal. No spatial cells,
// mutable candidate buckets or discovery-time index removals enter this oracle.
fn graph_components(points: &[Vec2]) -> Vec<Vec<usize>> {
    let edges: Vec<Vec<usize>> = points
        .iter()
        .map(|p| {
            let tolerance = (0.28 + p.x.hypot(p.y) * 0.015).min(0.85);
            points
                .iter()
                .enumerate()
                .filter_map(|(i, q)| (p.distance(*q) < tolerance).then_some(i))
                .collect()
        })
        .collect();
    let mut seen = vec![false; points.len()];
    let mut result = Vec::new();
    for seed in 0..points.len() {
        if seen[seed] {
            continue;
        }
        seen[seed] = true;
        let mut component = vec![seed];
        let mut cursor = 0;
        while cursor < component.len() {
            for &neighbor in &edges[component[cursor]] {
                if !seen[neighbor] {
                    seen[neighbor] = true;
                    component.push(neighbor);
                }
            }
            cursor += 1;
        }
        result.push(component);
    }
    result
}

fn isolated(count: usize) -> Vec<Vec2> {
    (0..count)
        .map(|i| Vec2::new(100. + i as f64 * 3., -100.))
        .collect()
}

fn check(points: &[Vec2]) {
    assert_eq!(
        Components::new(points).collect::<Vec<_>>(),
        graph_components(points)
    );
}

#[test]
fn negative_cells_duplicates_and_strict_radius_boundaries_preserve_queue_order() {
    let edge = 50.85_f64;
    let mut points = vec![
        Vec2::new(-1e-12, 10.),
        Vec2::new(1e-12, 10.),
        Vec2::new(-1., 10.),
        Vec2::new(-1. + 1e-12, 10.),
        Vec2::new(50., 0.),
        Vec2::new(f64::from_bits(edge.to_bits() - 1), 0.),
        Vec2::new(edge, 0.),
        Vec2::new(f64::from_bits(edge.to_bits() + 1), 0.),
        Vec2::new(-1e-12, 10.),
        Vec2::new(-0., 10.),
    ];
    points.extend(isolated(70));
    for _ in 0..points.len() {
        check(&points);
        points.rotate_left(1);
    }
    points.reverse();
    check(&points);
}

#[test]
fn range_adaptive_links_remain_directed_instead_of_becoming_symmetric_components() {
    let mut points = vec![Vec2::new(5., 0.), Vec2::new(5.357, 0.)];
    points.extend(isolated(70));
    let groups = Components::new(&points).collect::<Vec<_>>();
    assert_eq!(groups[0], vec![0]);
    assert_eq!(groups[1], vec![1]);
    points.swap(0, 1);
    let groups = Components::new(&points).collect::<Vec<_>>();
    assert_eq!(groups[0], vec![0, 1]);
    check(&points);
}

#[test]
fn seeded_dense_sparse_and_permuted_clouds_match_the_full_directed_graph() {
    let mut state = 0x0026_a710_2245_u64;
    let mut random = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        (state >> 11) as f64 / (1_u64 << 53) as f64
    };
    for case in 0..512 {
        let mut points = Vec::new();
        for i in 0..64 + case % 129 {
            let p = if i > 0 && i % 11 == 0 {
                points[i - 1]
            } else {
                let group = i % (1 + case % 23);
                let scale = if case % 3 == 0 { 80. } else { 0.7 };
                Vec2::new(
                    (group % 7) as f64 * 1.3 - 4. + scale * (random() - 0.5),
                    (group / 7) as f64 * 1.3 - 3. + scale * (random() - 0.5),
                )
            };
            points.push(p);
        }
        if case % 2 == 0 {
            points.reverse();
        }
        check(&points);
    }
}

#[test]
fn small_scans_and_nonindexable_coordinates_keep_exhaustive_behavior() {
    for n in [0, 1, 64, 65, 80] {
        let mut points = isolated(n);
        assert_eq!(Components::new(&points).grid.is_some(), n > 64);
        check(&points);
        if n == 0 {
            continue;
        }
        for x in [(1_u64 << 52) as f64, -((1_u64 << 52) as f64)] {
            points.fill(Vec2::new(x, 0.));
            assert_eq!(Components::new(&points).grid.is_some(), n > 64);
            check(&points);
        }
        for x in [(1_u64 << 52) as f64 + 1., 1e200, -1e200] {
            points[0] = Vec2::new(x, 0.);
            assert!(Components::new(&points).grid.is_none());
            check(&points);
        }
    }
}

#[test]
fn separated_large_clouds_do_not_visit_unrelated_points() {
    let points = isolated(16384);
    let mut components = Components::new(&points);
    assert_eq!(components.by_ref().count(), points.len());
    assert_eq!(components.candidate_visits, 0);
    assert_eq!(components.distance_checks, 0);
}

#[test]
fn a_dense_single_cell_is_removed_once_instead_of_rescanned_for_every_member() {
    let points = vec![Vec2::new(4., -3.); 16384];
    let mut components = Components::new(&points);
    assert_eq!(
        components.next().unwrap(),
        (0..points.len()).collect::<Vec<_>>()
    );
    assert!(components.next().is_none());
    assert_eq!(components.candidate_visits, points.len() - 1);
    assert_eq!(components.distance_checks, points.len() - 1);
}

fn bits(detections: &[Detection]) -> Vec<[u64; 3]> {
    detections
        .iter()
        .map(|d| {
            [
                d.center.x.to_bits(),
                d.center.y.to_bits(),
                d.radius.to_bits(),
            ]
        })
        .collect()
}

#[test]
fn actual_detectors_match_frozen_fit_filter_and_transform_bits_in_both_modes() {
    let mut points = vec![
        Vec2::new(f64::NAN, 1.),
        Vec2::new(1., f64::INFINITY),
        Vec2::new(0.3, 0.),
        Vec2::new(-0., 0.),
    ];
    for i in 0..96 {
        let (group, sample) = if i < 48 {
            (i / 4, i % 4)
        } else {
            (12 + (i - 48) / 8, (i - 48) % 8)
        };
        let angle = 2.4 + sample as f64 * 0.4;
        points.push(Vec2::new(
            5. + (group % 6) as f64 * 5. + angle.cos(),
            -12. + (group / 6) as f64 * 5. + angle.sin(),
        ));
    }
    for yaw in [0., 0.7, -1.9] {
        let pose = Pose {
            position: Vec2::new(12., -7.),
            yaw,
        };
        for _ in 0..8 {
            for sparse in [false, true] {
                let scan = LidarScan {
                    stamp: 42.,
                    points: points.clone(),
                    sweep: None,
                };
                let expected = legacy::detect(&scan, pose, sparse);
                let actual = if sparse {
                    crate::LidarClusters.detect_with_sparse_circles(&scan, pose)
                } else {
                    crate::LidarClusters.detect(&scan, pose)
                };
                assert_eq!(bits(&actual), bits(&expected));
            }
            points.rotate_left(13);
        }
    }
}
