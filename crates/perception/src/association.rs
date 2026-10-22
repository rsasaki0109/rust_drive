//! Component-wise maximum-cardinality, minimum-distance gated assignment.
use crate::TrackerUpdateError;
use rustdriving_core::{Detection, Track};

const GATE_M: f64 = 3.;
const MAX_PAIRS: usize = 1_000_000;
const MAX_COMPONENT_SIDE: usize = 64;
const MAX_WORK: usize = 2_000_000;

/// One optional existing track index per detection. New identities are assigned
/// by the caller only after the whole established-track assignment is fixed.
pub(super) fn associate(
    tracks: &[Track],
    detections: &[Detection],
    stamp: f64,
) -> Result<Vec<Option<usize>>, TrackerUpdateError> {
    let mut result = vec![None; detections.len()];
    if tracks.is_empty() || detections.is_empty() {
        return Ok(result);
    }
    let pairs = tracks
        .len()
        .checked_mul(detections.len())
        .filter(|n| *n <= MAX_PAIRS)
        .ok_or(TrackerUpdateError::AssociationLimit)?;
    let mut work = MAX_WORK - pairs;
    let predicted: Vec<_> = tracks
        .iter()
        .map(|t| t.position.plus(t.velocity.scaled(stamp - t.last_seen)))
        .collect();
    let mut track_edges = vec![vec![]; tracks.len()];
    let mut detection_edges = vec![vec![]; detections.len()];
    for (t, position) in predicted.iter().enumerate() {
        for (d, detection) in detections.iter().enumerate() {
            if position.distance(detection.center) < GATE_M {
                track_edges[t].push(d);
                detection_edges[d].push(t);
            }
        }
    }
    let mut seen_tracks = vec![false; tracks.len()];
    let mut seen_detections = vec![false; detections.len()];
    for start in 0..tracks.len() {
        if seen_tracks[start] || track_edges[start].is_empty() {
            continue;
        }
        let mut ts = vec![start];
        let mut ds = vec![];
        seen_tracks[start] = true;
        let mut cursor = 0;
        while cursor < ts.len() {
            for &d in &track_edges[ts[cursor]] {
                if seen_detections[d] {
                    continue;
                }
                seen_detections[d] = true;
                ds.push(d);
                if ds.len() > MAX_COMPONENT_SIDE {
                    return Err(TrackerUpdateError::AssociationLimit);
                }
                for &t in &detection_edges[d] {
                    if !seen_tracks[t] {
                        seen_tracks[t] = true;
                        ts.push(t);
                        if ts.len() > MAX_COMPONENT_SIDE {
                            return Err(TrackerUpdateError::AssociationLimit);
                        }
                    }
                }
            }
            cursor += 1;
        }
        ts.sort_by_key(|&t| tracks[t].id);
        ds.sort_by(|&a, &b| {
            detections[a]
                .center
                .x
                .total_cmp(&detections[b].center.x)
                .then(detections[a].center.y.total_cmp(&detections[b].center.y))
                .then(detections[a].radius.total_cmp(&detections[b].radius))
        });
        // Every valid cost is <3 m. One unmatched penalty exceeds the sum
        // of all valid costs, so losing a feasible match can never be cheaper.
        let unmatched = GATE_M * (ds.len() as f64 + 1.);
        let mut costs = vec![vec![unmatched; ts.len() + ds.len()]; ds.len()];
        for (row, &d) in ds.iter().enumerate() {
            for (col, &t) in ts.iter().enumerate() {
                spend(&mut work)?;
                let distance = predicted[t].distance(detections[d].center);
                costs[row][col] = if distance < GATE_M {
                    distance
                } else {
                    2. * unmatched
                };
            }
        }
        for (row, col) in solve(&costs, &mut work)?.into_iter().enumerate() {
            if col < ts.len() && costs[row][col] < GATE_M {
                result[ds[row]] = Some(ts[col]);
            }
        }
    }
    Ok(result)
}

fn spend(work: &mut usize) -> Result<(), TrackerUpdateError> {
    *work = work
        .checked_sub(1)
        .ok_or(TrackerUpdateError::AssociationLimit)?;
    Ok(())
}

/// Rectangular Hungarian algorithm. Explicit dummy columns make every row
/// assignable without selecting an invalid gate edge. Columns >= rows.
fn solve(costs: &[Vec<f64>], work: &mut usize) -> Result<Vec<usize>, TrackerUpdateError> {
    let (n, m) = (costs.len(), costs[0].len());
    let (mut u, mut v) = (vec![0.; n + 1], vec![0.; m + 1]);
    let (mut p, mut way) = (vec![0; m + 1], vec![0; m + 1]);
    for i in 1..=n {
        p[0] = i;
        let mut j0 = 0;
        let mut minv = vec![f64::INFINITY; m + 1];
        let mut used = vec![false; m + 1];
        loop {
            used[j0] = true;
            let i0 = p[j0];
            let (mut delta, mut j1) = (f64::INFINITY, 0);
            for j in 1..=m {
                spend(work)?;
                if used[j] {
                    continue;
                }
                let reduced = costs[i0 - 1][j - 1] - u[i0] - v[j];
                if reduced < minv[j] {
                    minv[j] = reduced;
                    way[j] = j0;
                }
                if minv[j] < delta {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for j in 0..=m {
                if used[j] {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if p[j0] == 0 {
                break;
            }
        }
        loop {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if j0 == 0 {
                break;
            }
        }
    }
    let mut columns = vec![0; n];
    for j in 1..=m {
        if p[j] != 0 {
            columns[p[j] - 1] = j - 1;
        }
    }
    Ok(columns)
}

#[cfg(test)]
mod tests;
