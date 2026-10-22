use super::*;

fn scan(points: Vec<Vec2>) -> LidarScan {
    LidarScan {
        stamp: 0.0,
        sweep: None,
        points,
    }
}

#[test]
fn marks_hit_and_clears_ray() {
    let mut g = OccupancyGrid::new(Vec2::new(-1.0, -1.0), 20, 10, 0.5);
    for _ in 0..4 {
        g.update(&scan(vec![Vec2::new(5.0, 0.0)]), Pose::default());
    }
    assert!(g.probability(Vec2::new(5.0, 0.0)).unwrap() > 0.9);
    assert!(g.probability(Vec2::new(2.0, 0.0)).unwrap() < 0.5);
    assert!(g.probability(Vec2::new(100.0, 0.0)).is_none());
}

#[test]
fn a_hit_cell_receives_only_occupied_evidence() {
    let mut g = OccupancyGrid::new(Vec2::default(), 4, 4, 1.0);
    g.update(
        &scan(vec![Vec2::new(2.625, 0.0)]),
        Pose {
            position: Vec2::new(0.25, 0.25),
            yaw: 0.0,
        },
    );
    assert_eq!(g.log_odds[2], 0.85_f32);
}

#[test]
fn a_diagonal_clears_each_crossed_cell_once() {
    let mut g = OccupancyGrid::new(Vec2::default(), 4, 4, 1.0);
    g.update(
        &scan(vec![Vec2::new(2.5, 2.5)]),
        Pose {
            position: Vec2::new(0.25, 0.25),
            yaw: 0.0,
        },
    );
    assert_eq!(g.log_odds[0], -0.15_f32);
    assert_eq!(g.log_odds[5], -0.15_f32);
    assert_eq!(g.log_odds[10], 0.85_f32);
    assert_eq!(g.log_odds.iter().filter(|&&v| v != 0.0).count(), 3);
}

#[test]
fn short_intersections_between_distance_samples_are_not_skipped() {
    let mut grid = OccupancyGrid::new(Vec2::default(), 4, 4, 1.0);
    grid.update(
        &scan(vec![Vec2::new(3.5, 2.5)]),
        Pose {
            position: Vec2::new(0.25, 0.25),
            yaw: 0.0,
        },
    );
    // The preceding distance sampler missed both of these positive intervals.
    assert_eq!(grid.log_odds[1], -0.15);
    assert_eq!(grid.log_odds[2 * 4 + 2], -0.15);
    assert_eq!(grid.log_odds, oracle([1, 1], [15, 11], 4, 4));
}

// The oracle tests every cell rectangle independently. Its exact rational
// comparisons use quarter-cell integer coordinates, without grid traversal,
// floating ray sampling, or production clipping/indexing helpers.
#[derive(Clone, Copy, Debug, Eq)]
struct Ratio(i128, i128);

impl Ratio {
    fn new(n: i64, d: i64) -> Self {
        if d < 0 {
            Self(-i128::from(n), -i128::from(d))
        } else {
            Self(i128::from(n), i128::from(d))
        }
    }
}
impl PartialEq for Ratio {
    fn eq(&self, rhs: &Self) -> bool {
        self.cmp(rhs).is_eq()
    }
}
impl PartialOrd for Ratio {
    fn partial_cmp(&self, rhs: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(rhs))
    }
}
impl Ord for Ratio {
    fn cmp(&self, rhs: &Self) -> std::cmp::Ordering {
        (self.0 * rhs.1).cmp(&(rhs.0 * self.1))
    }
}

fn oracle(start: [i64; 2], end: [i64; 2], width: usize, height: usize) -> Vec<f32> {
    let hit = (end[0] >= 0 && end[1] >= 0)
        .then(|| (end[0] as usize / 4, end[1] as usize / 4))
        .filter(|&(x, y)| x < width && y < height);
    let mut expected = vec![0.0; width * height];
    for y in 0..height {
        for x in 0..width {
            if Some((x, y)) == hit {
                expected[y * width + x] = 0.85;
                continue;
            }
            if start == end {
                continue;
            }
            let (mut low, mut high) = (Ratio(0, 1), Ratio(1, 1));
            let mut inside = true;
            for (axis, cell) in [x, y].into_iter().enumerate() {
                let a = 4 * cell as i64;
                let b = a + 4;
                let delta = end[axis] - start[axis];
                if delta == 0 {
                    inside &= a <= start[axis] && start[axis] < b;
                } else {
                    let p = Ratio::new(a - start[axis], delta);
                    let q = Ratio::new(b - start[axis], delta);
                    low = low.max(p.min(q));
                    high = high.min(p.max(q));
                }
            }
            if inside && low < high {
                expected[y * width + x] = -0.15;
            }
        }
    }
    expected
}

fn assert_ray(start: [i64; 2], end: [i64; 2], origin: Vec2, resolution: f64) {
    let position = origin.plus(Vec2::new(
        start[0] as f64 * resolution / 4.0,
        start[1] as f64 * resolution / 4.0,
    ));
    let point = Vec2::new(
        (end[0] - start[0]) as f64 * resolution / 4.0,
        (end[1] - start[1]) as f64 * resolution / 4.0,
    );
    let mut grid = OccupancyGrid::new(origin, 9, 7, resolution);
    grid.update(&scan(vec![point]), Pose { position, yaw: 0.0 });
    assert_eq!(
        grid.log_odds,
        oracle(start, end, 9, 7),
        "ray {start:?} -> {end:?}, origin {origin:?}, resolution {resolution}"
    );
}

#[test]
fn edges_corners_outside_entries_and_zero_length_match_exact_intersections() {
    let rays = [
        ([0, 0], [36, 28]),
        ([36, 28], [0, 0]),
        ([1, 1], [33, 25]),
        ([33, 25], [1, 1]),
        ([4, 4], [28, 28]),
        ([28, 28], [4, 4]),
        ([4, 0], [4, 32]),
        ([4, 32], [4, -4]),
        ([0, 4], [40, 4]),
        ([40, 4], [-4, 4]),
        ([36, -4], [36, 32]),
        ([-4, 28], [40, 28]),
        ([-4, -4], [0, 0]),
        ([0, 0], [-4, -4]),
        ([-4, 4], [4, -4]),
        ([40, 24], [32, 32]),
        ([3, 3], [3, 3]),
        ([-3, -3], [-3, -3]),
        ([1, 1], [3, 3]),
    ];
    for (start, end) in rays {
        for (origin, resolution) in [
            (Vec2::default(), 1.0),
            (Vec2::new(-17.5, 12.25), 0.5),
            (Vec2::new(256.0, -512.0), 2.0),
        ] {
            assert_ray(start, end, origin, resolution);
        }
    }
}

#[test]
fn randomized_rays_match_every_cell_of_the_exact_rectangle_oracle() {
    let mut state = 0x90ad_182e_7634_9ac1_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state % 89) as i64 - 24
    };
    for _ in 0..8192 {
        assert_ray(
            [next(), next()],
            [next(), next()],
            Vec2::new(-17.5, 12.25),
            0.5,
        );
    }
}

#[test]
fn rotated_body_returns_use_the_world_grid() {
    let mut grid = OccupancyGrid::new(Vec2::default(), 9, 7, 1.0);
    grid.update(
        &scan(vec![Vec2::new(2.5, 1.0)]),
        Pose {
            position: Vec2::new(4.25, 1.25),
            yaw: std::f64::consts::FRAC_PI_2,
        },
    );
    assert_eq!(grid.log_odds, oracle([17, 5], [13, 15], 9, 7));
}

#[test]
fn endpoint_queries_and_traversal_agree_at_nonbinary_resolution_boundaries() {
    let mut grid = OccupancyGrid::new(Vec2::new(-1.1, 0.2), 9, 7, 0.3);
    let pose = Pose {
        position: Vec2::new(-0.95, 0.35),
        yaw: 0.0,
    };
    let boundary: f64 = -1.1 + 4.0 * 0.3;
    for end_x in [boundary.next_down(), boundary, boundary.next_up()] {
        grid.log_odds.fill(0.0);
        let point = Vec2::new(end_x - pose.position.x, 0.0);
        let hit = grid.index(pose.to_world(point)).unwrap();
        grid.update(&scan(vec![point]), pose);
        assert_eq!(grid.log_odds[hit], 0.85);
        assert!(
            grid.log_odds
                .iter()
                .all(|&v| [0.0, -0.15, 0.85].contains(&v))
        );
    }
}

#[test]
fn remote_endpoints_have_grid_bounded_unique_traversal() {
    for (start, end) in [
        (Vec2::new(-1e12, 3.5), Vec2::new(1e12, 3.5)),
        (Vec2::new(1e12, 3.5), Vec2::new(-1e12, 3.5)),
        (Vec2::new(4.5, -1e12), Vec2::new(4.5, 1e12)),
        (Vec2::new(4.5, 1e12), Vec2::new(4.5, -1e12)),
        (Vec2::new(0.25, 3.5), Vec2::new(1e300, 3.5)),
    ] {
        let cells: Vec<_> = ray::Cells::new(start, end, 9, 7).unwrap().collect();
        assert!(cells.len() <= 9 + 7);
        let distinct: std::collections::BTreeSet<_> = cells.iter().copied().collect();
        assert_eq!(distinct.len(), cells.len());
        let expected = if start.x == end.x { 7 } else { 9 };
        assert_eq!(cells.len(), expected, "{start:?} -> {end:?}: {cells:?}");
    }
    let mut grid = OccupancyGrid::new(Vec2::default(), 9, 7, 1.0);
    grid.update(
        &scan(vec![Vec2::new(1e300, 0.0)]),
        Pose {
            position: Vec2::new(0.25, 3.5),
            yaw: 0.0,
        },
    );
    assert_eq!(grid.log_odds.iter().filter(|&&v| v == -0.15).count(), 9);
    assert!(grid.occupied_cells().is_empty());
}

#[test]
fn distinct_returns_accumulate_and_saturate_but_empty_scans_do_not_clear() {
    let mut grid = OccupancyGrid::new(Vec2::default(), 4, 4, 1.0);
    let pose = Pose {
        position: Vec2::new(0.25, 0.25),
        yaw: 0.0,
    };
    let input = scan(vec![Vec2::new(2.5, 2.5); 2]);
    grid.update(&input, pose);
    assert_eq!(grid.log_odds[0], -0.3);
    assert_eq!(grid.log_odds[5], -0.3);
    assert_eq!(grid.log_odds[10], 1.7);
    for _ in 0..100 {
        grid.update(&input, pose);
    }
    assert_eq!(grid.log_odds[0], -4.0);
    assert_eq!(grid.log_odds[5], -4.0);
    assert_eq!(grid.log_odds[10], 4.0);
    let before = grid.log_odds.clone();
    grid.update(&scan(vec![]), pose);
    assert_eq!(grid.log_odds, before);
}

#[test]
fn nonfinite_pose_points_and_unrepresentable_transforms_do_not_change_the_grid() {
    let mut grid = OccupancyGrid::new(Vec2::default(), 4, 4, 0.5);
    let valid = scan(vec![Vec2::new(1.0, 1.0)]);
    for pose in [
        Pose {
            position: Vec2::new(f64::NAN, 0.0),
            yaw: 0.0,
        },
        Pose {
            position: Vec2::new(0.0, f64::INFINITY),
            yaw: 0.0,
        },
        Pose {
            position: Vec2::default(),
            yaw: f64::NAN,
        },
        Pose {
            position: Vec2::new(f64::MAX, f64::MAX),
            yaw: 0.0,
        },
    ] {
        grid.update(&valid, pose);
    }
    grid.update(
        &scan(vec![
            Vec2::new(f64::NAN, 0.0),
            Vec2::new(0.0, f64::NEG_INFINITY),
            Vec2::new(f64::MAX, f64::MAX),
        ]),
        Pose::default(),
    );
    assert!(grid.log_odds.iter().all(|&v| v == 0.0));
}
