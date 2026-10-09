//! Smooth interpolation of supplied route geometry; the route corridor is unchanged.
use rustdriving_core::{Route, Vec2};

fn derivatives(route: &Route, i: usize) -> (Vec2, Vec2) {
    let last = route.points.len() - 1;
    if i == 0 || i == last {
        let a = i.saturating_sub(1);
        let b = (i + 1).min(last);
        return (
            route.points[b]
                .minus(route.points[a])
                .scaled(1.0 / (route.lengths[b] - route.lengths[a])),
            Vec2::default(),
        );
    }
    let before = route.lengths[i] - route.lengths[i - 1];
    let after = route.lengths[i + 1] - route.lengths[i];
    let incoming = route.points[i]
        .minus(route.points[i - 1])
        .scaled(1.0 / before);
    let outgoing = route.points[i + 1]
        .minus(route.points[i])
        .scaled(1.0 / after);
    (
        incoming
            .scaled(after)
            .plus(outgoing.scaled(before))
            .scaled(1.0 / (before + after)),
        outgoing.minus(incoming).scaled(2.0 / (before + after)),
    )
}

/// Quintic Hermite segments share position, first and second derivatives at
/// route knots. Normal offsets therefore join without polyline-normal jumps.
pub(super) fn sample(route: &Route, s: f64, lateral: f64) -> Vec2 {
    let s = s.clamp(0.0, route.length());
    let i = route
        .lengths
        .partition_point(|x| *x < s)
        .saturating_sub(1)
        .min(route.points.len() - 2);
    let length = route.lengths[i + 1] - route.lengths[i];
    let u = (s - route.lengths[i]) / length;
    let (v0, a0) = derivatives(route, i);
    let (v1, a1) = derivatives(route, i + 1);
    let c0 = route.points[i];
    let c1 = v0.scaled(length);
    let c2 = a0.scaled(0.5 * length * length);
    let r0 = route.points[i + 1].minus(c0.plus(c1).plus(c2));
    let r1 = v1.scaled(length).minus(c1.plus(c2.scaled(2.0)));
    let r2 = a1.scaled(length * length).minus(c2.scaled(2.0));
    let c3 = r0.scaled(10.0).minus(r1.scaled(4.0)).plus(r2.scaled(0.5));
    let c4 = r0.scaled(-15.0).plus(r1.scaled(7.0)).minus(r2);
    let c5 = r0.scaled(6.0).minus(r1.scaled(3.0)).plus(r2.scaled(0.5));
    let center = c0.plus(
        c1.plus(
            c2.plus(c3.plus(c4.plus(c5.scaled(u)).scaled(u)).scaled(u))
                .scaled(u),
        )
        .scaled(u),
    );
    let tangent = c1.plus(
        c2.scaled(2.0)
            .plus(
                c3.scaled(3.0)
                    .plus(c4.scaled(4.0).plus(c5.scaled(5.0 * u)).scaled(u))
                    .scaled(u),
            )
            .scaled(u),
    );
    let magnitude = tangent.x.hypot(tangent.y);
    let normal = if magnitude > 1e-9 {
        Vec2::new(-tangent.y / magnitude, tangent.x / magnitude)
    } else {
        Vec2::default()
    };
    center.plus(normal.scaled(lateral))
}

/// A position correction fades with zero endpoint slope and curvature. This
/// derivative correction has unit initial slope and zero value/slope/curvature
/// at the far endpoint, joining the path to the measured heading as well.
pub(super) fn heading_weight(distance: f64, transition: f64) -> f64 {
    let u = (distance / transition).clamp(0.0, 1.0);
    transition * (u - 6.0 * u.powi(3) + 8.0 * u.powi(4) - 3.0 * u.powi(5))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn straight_route_is_exact_even_with_unequal_segments() {
        let route = Route::new(
            vec![Vec2::default(), Vec2::new(2.0, 0.0), Vec2::new(11.0, 0.0)],
            5.5,
        )
        .unwrap();
        for s in [0.0, 1.0, 2.0, 2.0001, 7.0, 11.0] {
            assert!(sample(&route, s, 3.5).distance(Vec2::new(s, 3.5)) < 1e-10);
        }
    }
    #[test]
    fn offset_is_continuous_in_position_and_heading_at_a_route_knot() {
        let route = Route::new(
            vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 0.02),
                Vec2::new(2.0, 0.06),
                Vec2::new(3.0, 0.12),
            ],
            5.5,
        )
        .unwrap();
        let s = route.lengths[1];
        let epsilon = 1e-5;
        let p = sample(&route, s, 3.5);
        let left = p
            .minus(sample(&route, s - epsilon, 3.5))
            .scaled(1.0 / epsilon);
        let right = sample(&route, s + epsilon, 3.5)
            .minus(p)
            .scaled(1.0 / epsilon);
        assert!(left.distance(right) < 1e-4);
        assert!(sample(&route, s, 0.0).distance(route.points[1]) < 1e-10);
    }
}
