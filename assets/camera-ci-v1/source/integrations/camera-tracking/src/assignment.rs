//! Rectangular Hungarian assignment with explicit unmatched columns.
use crate::Detection;

pub const MAX_WORK: usize = 60_000_000;

fn iou(a: [f64; 4], b: [f64; 4]) -> f64 {
    let area = |v: [f64; 4]| (v[2] - v[0]) * (v[3] - v[1]);
    let intersection =
        (a[2].min(b[2]) - a[0].max(b[0])).max(0.) * (a[3].min(b[3]) - a[1].max(b[1])).max(0.);
    intersection / (area(a) + area(b) - intersection)
}

pub(crate) fn associate(
    tracks: &[(usize, [f64; 4])],
    detections: &[Detection],
    gate: f64,
) -> Result<(Vec<Option<usize>>, usize), String> {
    let n = tracks.len();
    let d = detections.len();
    if n == 0 || d == 0 {
        return Ok((vec![None; n], 0));
    }
    let m = d + n;
    // Losing a match costs more than the sum of all valid IoU costs. Thus
    // cardinality is optimized first, then total IoU; no invalid edge is needed.
    let unmatched = n as f64 + 1.;
    let mut costs = vec![vec![unmatched; m]; n];
    for (i, &(kind, bbox)) in tracks.iter().enumerate() {
        for (j, detection) in detections.iter().enumerate() {
            let overlap = iou(bbox, detection.bbox);
            costs[i][j] = if kind == detection.class_index && overlap >= gate {
                1. - overlap
            } else {
                unmatched * 2.
            };
        }
    }
    let (mut u, mut v) = (vec![0.; n + 1], vec![0.; m + 1]);
    let (mut p, mut way) = (vec![0; m + 1], vec![0; m + 1]);
    let mut work = n * d;
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
                work += 1;
                if work > MAX_WORK {
                    return Err("assignment work budget exceeded".into());
                }
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
    let mut matches = vec![None; n];
    for j in 1..=d {
        if p[j] > 0 && costs[p[j] - 1][j - 1] < unmatched {
            matches[p[j] - 1] = Some(j - 1);
        }
    }
    Ok((matches, work))
}
