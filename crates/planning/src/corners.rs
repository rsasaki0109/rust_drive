//! Optional local fillets; the supplied polyline remains the physical corridor.
use rustdrive_core::{Route, Vec2, VehicleConfig};

const STEP_M: f64 = 0.25;

#[derive(Clone, Copy)]
struct Corner {
    incoming: Vec2,
    outgoing: Vec2,
    angle: f64,
    cut: f64,
}

fn push(points: &mut Vec<Vec2>, point: Vec2) {
    if points.last().is_none_or(|last| last.distance(point) > 1e-6) {
        points.push(point);
    }
}

fn straight(points: &mut Vec<Vec2>, end: Vec2) {
    let start = *points.last().unwrap();
    let count = (start.distance(end) / STEP_M).ceil().max(1.0) as usize;
    for i in 1..=count {
        push(
            points,
            start.plus(end.minus(start).scaled(i as f64 / count as f64)),
        );
    }
}

pub(super) fn fit(route: &Route, vehicle: VehicleConfig) -> Result<Route, String> {
    let radius = vehicle.wheelbase / vehicle.max_steer.tan() * 1.005;
    if !radius.is_finite() || radius <= 0.0 || route.half_width <= vehicle.radius {
        return Err("local route geometry requires a finite steering radius and corridor".into());
    }
    let mut corners = vec![None; route.points.len()];
    for (i, triple) in route.points.windows(3).enumerate() {
        let before = triple[1].minus(triple[0]);
        let after = triple[2].minus(triple[1]);
        let incoming = before.scaled(1.0 / before.x.hypot(before.y));
        let outgoing = after.scaled(1.0 / after.x.hypot(after.y));
        let angle = (incoming.x * outgoing.y - incoming.y * outgoing.x)
            .atan2(incoming.x * outgoing.x + incoming.y * outgoing.y);
        if angle.abs() < 0.002 {
            continue;
        }
        let cut = radius * (0.5 * angle.abs()).tan();
        let deviation = radius * (1.0 - (0.5 * angle).cos());
        if !cut.is_finite()
            || angle.abs() >= std::f64::consts::PI - 0.01
            || deviation > route.half_width - vehicle.radius - 0.05
        {
            return Err(format!(
                "corner {} cannot fit the calibrated steering radius in the supplied corridor",
                i + 1
            ));
        }
        corners[i + 1] = Some(Corner {
            incoming,
            outgoing,
            angle,
            cut,
        });
    }
    for i in 0..route.points.len() - 1 {
        let used = corners[i].map_or(0.0, |c| c.cut) + corners[i + 1].map_or(0.0, |c| c.cut);
        if used >= route.lengths[i + 1] - route.lengths[i] - 1e-5 {
            return Err(format!(
                "adjacent calibrated corner fillets overlap on segment {i}"
            ));
        }
    }
    let mut points = vec![route.points[0]];
    for (i, corner) in corners
        .iter()
        .enumerate()
        .take(route.points.len() - 1)
        .skip(1)
    {
        let Some(corner) = corner else {
            straight(&mut points, route.points[i]);
            continue;
        };
        let entry = route.points[i].minus(corner.incoming.scaled(corner.cut));
        let exit = route.points[i].plus(corner.outgoing.scaled(corner.cut));
        straight(&mut points, entry);
        let normal = Vec2::new(-corner.incoming.y, corner.incoming.x).scaled(corner.angle.signum());
        let center = entry.plus(normal.scaled(radius));
        let radial = entry.minus(center);
        let start = radial.y.atan2(radial.x);
        let count = (radius * corner.angle.abs() / STEP_M).ceil().max(1.0) as usize;
        for j in 1..count {
            let a = start + corner.angle * j as f64 / count as f64;
            push(
                &mut points,
                center.plus(Vec2::new(a.cos(), a.sin()).scaled(radius)),
            );
        }
        push(&mut points, exit);
    }
    straight(&mut points, *route.points.last().unwrap());
    Route::new(points, route.half_width)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sparse_turn_remains_in_original_corridor_with_calibrated_curvature() {
        let original = Route::new(
            vec![Vec2::default(), Vec2::new(30.0, 0.0), Vec2::new(30.0, 30.0)],
            3.0,
        )
        .unwrap();
        let vehicle = VehicleConfig::default();
        let fitted = fit(&original, vehicle).unwrap();
        assert_eq!(fitted.points.first(), original.points.first());
        assert_eq!(fitted.points.last(), original.points.last());
        for point in &fitted.points {
            assert!(original.project(*point).1.abs() + vehicle.radius < original.half_width);
        }
        let max_curvature = fitted
            .points
            .windows(3)
            .map(|p| {
                let a = p[1].minus(p[0]);
                let b = p[2].minus(p[1]);
                let c = p[2].minus(p[0]);
                2.0 * (a.x * b.y - a.y * b.x).abs()
                    / (a.x.hypot(a.y) * b.x.hypot(b.y) * c.x.hypot(c.y))
            })
            .fold(0.0_f64, f64::max);
        assert!(max_curvature <= vehicle.max_steer.tan() / vehicle.wheelbase + 1e-9);
    }
    #[test]
    fn impossible_corner_or_overlapping_fillets_are_rejected() {
        let tight = Route::new(
            vec![Vec2::default(), Vec2::new(3.0, 0.0), Vec2::new(3.0, 3.0)],
            3.0,
        )
        .unwrap();
        assert!(fit(&tight, VehicleConfig::default()).is_err());
        let narrow = Route::new(
            vec![Vec2::default(), Vec2::new(30.0, 0.0), Vec2::new(30.0, 30.0)],
            2.0,
        )
        .unwrap();
        assert!(fit(&narrow, VehicleConfig::default()).is_err());
    }
}
