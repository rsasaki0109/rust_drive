// Frozen from 2da76ef2de409c829a13499d7a62593057bcd1fc:crates/perception/src/lib.rs; detector and fit bodies unchanged.
fn detect_clusters(scan: &LidarScan, pose: Pose, sparse_circles: bool) -> Vec<Detection> {
    let points: Vec<_> = scan
        .points
        .iter()
        .copied()
        .filter(|p| p.finite() && p.x.hypot(p.y) > 0.3)
        .collect();
    let mut visited = vec![false; points.len()];
    let mut result = Vec::new();
    for i in 0..points.len() {
        if visited[i] {
            continue;
        }
        visited[i] = true;
        let mut queue = vec![i];
        let mut cursor = 0;
        while cursor < queue.len() {
            let p = points[queue[cursor]];
            cursor += 1;
            let tolerance = (0.28 + p.x.hypot(p.y) * 0.015).min(0.85);
            for j in 0..points.len() {
                if !visited[j] && p.distance(points[j]) < tolerance {
                    visited[j] = true;
                    queue.push(j);
                }
            }
        }
        if queue.len() < 3 {
            continue;
        }
        let mut center = Vec2::default();
        for &j in &queue {
            center = center.plus(points[j]);
        }
        center = center.scaled(1.0 / queue.len() as f64);
        // Visible surfaces underestimate centers: conservative enclosing proxy, not a shape classifier.
        let extent = queue
            .iter()
            .map(|&j| center.distance(points[j]))
            .fold(0.0, f64::max);
        let cluster: Vec<_> = queue.iter().map(|&j| points[j]).collect();
        let fitted = fit_circle(&cluster).or_else(|| {
            sparse_circles
                .then(|| fit_sparse_circle(&cluster))
                .flatten()
        });
        let (center, radius) = fitted.unwrap_or((center, (extent + 0.65).clamp(0.85, 3.0)));
        result.push(Detection {
            center: pose.to_world(center),
            radius,
        });
    }
    result
}
/// Algebraic least-squares circle fit, with bounds and residual checks.
fn fit_circle(points: &[Vec2]) -> Option<(Vec2, f64)> {
    if points.len() < 5 {
        return None;
    }
    fit_circle_parameters(points)
}

/// Sparse arcs require both a baseline and curvature above the degeneracy floor.
/// All points must lie on the near visible branch of the fitted circle.
fn fit_sparse_circle(points: &[Vec2]) -> Option<(Vec2, f64)> {
    if !(3..=4).contains(&points.len()) {
        return None;
    }
    let (mut baseline, mut twice_area) = (0_f64, 0_f64);
    for (i, a) in points.iter().enumerate() {
        for (j, b) in points.iter().enumerate().skip(i + 1) {
            baseline = baseline.max(a.distance(*b));
            for c in points.iter().skip(j + 1) {
                let u = b.minus(*a);
                let v = c.minus(*a);
                twice_area = twice_area.max((u.x * v.y - u.y * v.x).abs());
            }
        }
    }
    if baseline < 0.5 || twice_area / baseline < 0.05 {
        return None;
    }
    let (center, radius) = fit_circle_parameters(points)?;
    if points.iter().any(|p| {
        let normal = p.minus(center);
        p.x * normal.x + p.y * normal.y > 0.12 * p.x.hypot(p.y)
    }) {
        return None;
    }
    Some((center, radius))
}

fn fit_circle_parameters(points: &[Vec2]) -> Option<(Vec2, f64)> {
    let origin = points
        .iter()
        .fold(Vec2::default(), |sum, p| sum.plus(*p))
        .scaled(1.0 / points.len() as f64);
    let mut augmented = [[0.0; 4]; 3];
    for point in points {
        let q = point.minus(origin);
        let a = [2.0 * q.x, 2.0 * q.y, 1.0];
        let b = q.x * q.x + q.y * q.y;
        for i in 0..3 {
            for j in 0..3 {
                augmented[i][j] += a[i] * a[j];
            }
            augmented[i][3] += a[i] * b;
        }
    }
    for col in 0..3 {
        let pivot = (col..3)
            .max_by(|&a, &b| augmented[a][col].abs().total_cmp(&augmented[b][col].abs()))?;
        if augmented[pivot][col].abs() < 1e-8 {
            return None;
        }
        augmented.swap(col, pivot);
        let divisor = augmented[col][col];
        for j in col..4 {
            augmented[col][j] /= divisor;
        }
        for i in 0..3 {
            if i == col {
                continue;
            }
            let factor = augmented[i][col];
            for j in col..4 {
                augmented[i][j] -= factor * augmented[col][j];
            }
        }
    }
    let local = Vec2::new(augmented[0][3], augmented[1][3]);
    let radius = (augmented[2][3] + local.x * local.x + local.y * local.y).sqrt();
    let center = origin.plus(local);
    if !radius.is_finite()
        || !(0.4..=2.5).contains(&radius)
        || points
            .iter()
            .any(|p| (p.distance(center) - radius).abs() > 0.12)
    {
        return None;
    }
    Some((center, radius + 0.12))
}
