use super::*;
use rustdriving_core::Vec2;

fn track(id: u64, x: f64, y: f64) -> Track {
    Track {
        id,
        position: Vec2::new(x, y),
        velocity: Vec2::default(),
        radius: 0.5,
        last_seen: 0.,
    }
}
fn detection(x: f64, y: f64) -> Detection {
    Detection {
        center: Vec2::new(x, y),
        radius: 0.5,
    }
}

// Enumerate every partial injective mapping. No assignment solver, dummy cost,
// component decomposition or dual potentials are used by this oracle.
fn exhaustive(costs: &[Vec<f64>], row: usize, used: &mut [bool]) -> (usize, f64) {
    if row == costs.len() {
        return (0, 0.);
    }
    let mut best = exhaustive(costs, row + 1, used);
    for col in 0..used.len() {
        if used[col] || costs[row][col] >= 3. {
            continue;
        }
        used[col] = true;
        let (matches, cost) = exhaustive(costs, row + 1, used);
        used[col] = false;
        let candidate = (matches + 1, cost + costs[row][col]);
        if candidate.0 > best.0 || (candidate.0 == best.0 && candidate.1 < best.1) {
            best = candidate;
        }
    }
    best
}

fn assert_optimum(tracks: &[Track], detections: &[Detection]) {
    let costs: Vec<Vec<f64>> = detections
        .iter()
        .map(|d| {
            tracks
                .iter()
                .map(|t| {
                    t.position
                        .plus(t.velocity.scaled(0.5 - t.last_seen))
                        .distance(d.center)
                })
                .collect()
        })
        .collect();
    let expected = exhaustive(&costs, 0, &mut vec![false; tracks.len()]);
    let matches = associate(tracks, detections, 0.5).unwrap();
    let mut used = vec![false; tracks.len()];
    let mut total = (0, 0.);
    for (d, matched) in matches.iter().enumerate() {
        if let Some(t) = *matched {
            assert!(!used[t] && costs[d][t] < 3.);
            used[t] = true;
            total.0 += 1;
            total.1 += costs[d][t];
        }
    }
    assert_eq!(total.0, expected.0);
    assert!(
        (total.1 - expected.1).abs() < 1e-9,
        "{total:?} != {expected:?}"
    );
    let mut reversed = detections.to_vec();
    reversed.reverse();
    let mut reordered = associate(tracks, &reversed, 0.5).unwrap();
    reordered.reverse();
    assert_eq!(matches, reordered);
}

#[test]
fn small_assignments_agree_with_exhaustive_enumeration_and_detection_permutations() {
    assert_optimum(
        &[track(1, 0., 0.)],
        &[detection(3., 0.), detection(2.999999, 0.)],
    );
    for seed in 0..300 {
        let tracks: Vec<_> = (0..seed % 5)
            .map(|i| {
                let mut t = track(
                    i as u64 + 1,
                    ((seed * 17 + i * 11) % 23) as f64 * 0.4 - 4.,
                    ((seed * 13 + i * 7) % 19) as f64 * 0.3 - 3.,
                );
                t.velocity = Vec2::new(i as f64 * 0.2, -0.1);
                t.last_seen = (i % 2) as f64 * 0.1;
                t
            })
            .collect();
        let detections: Vec<_> = (0..(seed / 5) % 5)
            .map(|i| {
                detection(
                    ((seed * 7 + i * 13) % 23) as f64 * 0.4 - 4.,
                    ((seed * 11 + i * 5) % 19) as f64 * 0.3 - 3.,
                )
            })
            .collect();
        assert_optimum(&tracks, &detections);
    }
}

#[test]
fn many_independent_components_do_not_hit_the_local_component_size_limit() {
    let tracks: Vec<_> = (0..128).map(|i| track(i + 1, i as f64 * 10., 0.)).collect();
    let detections: Vec<_> = (0..128)
        .map(|i| detection(i as f64 * 10. + 0.2, 0.))
        .collect();
    assert_eq!(
        associate(&tracks, &detections, 0.1).unwrap(),
        (0..128).map(Some).collect::<Vec<_>>()
    );
}

#[test]
fn solver_work_exhaustion_is_an_explicit_failure() {
    let costs = vec![vec![0., 1., 9., 9.], vec![1., 0., 9., 9.]];
    assert_eq!(
        solve(&costs, &mut 0).unwrap_err(),
        TrackerUpdateError::AssociationLimit
    );
    assert_eq!(solve(&costs, &mut 100).unwrap(), vec![0, 1]);
}
