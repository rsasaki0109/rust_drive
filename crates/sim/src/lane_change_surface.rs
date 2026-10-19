//! Authored physical lane surfaces for the optional return mode; never driver inputs.
use rustdriving_core::{Route, Vec2};
use rustdriving_pipeline::lane_change::LaneChangeConfig;

pub(crate) fn circle_margin(
    route: &Route,
    gate: &LaneChangeConfig,
    point: Vec2,
    radius: f64,
) -> f64 {
    let source = gate
        .return_on_conflict
        .as_ref()
        .expect("enabled return surface");
    let vector = gate.target_end.minus(gate.target_start);
    let length = vector.x.hypot(vector.y);
    let axis = vector.scaled(1.0 / length);
    let coordinates = |p: Vec2| {
        let d = p.minus(gate.target_start);
        (d.x * axis.x + d.y * axis.y, axis.x * d.y - axis.y * d.x)
    };
    let (x, y) = coordinates(point);
    let source_y = coordinates(source.source_start).1;
    let s = route.project(point).0;
    let original_margin = route.half_width - radius - point.distance(route.sample(s, 0.0).0);
    if x >= radius && x <= length - radius {
        // Exact erosion of the joined straight lane strips away from their ends.
        let lower = (-gate.target_width_m / 2.0).min(source_y - source.source_width_m / 2.0);
        let upper = (gate.target_width_m / 2.0).max(source_y + source.source_width_m / 2.0);
        return (y - lower - radius)
            .min(upper - y - radius)
            .max(original_margin);
    }
    let along_error = x - x.clamp(0.0, length);
    let source_margin = source.source_width_m / 2.0 - radius - along_error.hypot(y - source_y);
    let target_margin = gate.target_width_m / 2.0 - radius - along_error.hypot(y);
    source_margin.max(target_margin).max(original_margin)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_pipeline::lane_change::LaneChangeReturnConfig;
    fn config() -> LaneChangeConfig {
        LaneChangeConfig {
            entry_s_m: 25.0,
            exit_s_m: 56.0,
            target_start: Vec2::new(0.0, 3.5),
            target_end: Vec2::new(80.0, 3.5),
            target_width_m: 3.5,
            return_on_conflict: Some(LaneChangeReturnConfig {
                source_start: Vec2::default(),
                source_end: Vec2::new(80.0, 0.0),
                source_width_m: 3.5,
                max_return_distance_m: 30.0,
                speed_limit_m_s: None,
                resume_after_return: None,
            }),
        }
    }
    #[test]
    fn physical_surface_is_the_authored_lane_union_with_unchanged_outer_boundaries() {
        let route = Route::new(
            vec![
                Vec2::default(),
                Vec2::new(25.0, 0.0),
                Vec2::new(55.0, 3.5),
                Vec2::new(160.0, 3.5),
            ],
            1.75,
        )
        .unwrap();
        let c = config();
        assert!(circle_margin(&route, &c, Vec2::new(60.0, 0.0), 1.25) >= 0.0);
        assert!(circle_margin(&route, &c, Vec2::new(40.0, 1.75), 1.25) >= 0.0);
        assert!(circle_margin(&route, &c, Vec2::new(40.0, -0.501), 1.25) < 0.0);
        assert!(circle_margin(&route, &c, Vec2::new(40.0, 4.001), 1.25) < 0.0);
        let on_surface = |p: Vec2| {
            ((0.0..=80.0).contains(&p.x) && (-1.75..=5.25).contains(&p.y))
                || p.distance(Vec2::new(p.x.clamp(0.0, 80.0), 0.0)) <= 1.75
                || p.distance(Vec2::new(p.x.clamp(0.0, 80.0), 3.5)) <= 1.75
                || route.points.windows(2).any(|pair| {
                    let d = pair[1].minus(pair[0]);
                    let q = p.minus(pair[0]);
                    let t = ((q.x * d.x + q.y * d.y) / (d.x * d.x + d.y * d.y)).clamp(0.0, 1.0);
                    p.distance(pair[0].plus(d.scaled(t))) <= 1.75 + 1e-10
                })
        };
        for x in [0.0, 0.2, 2.0, 40.0, 77.0, 79.9, 80.0, 85.0, 150.0] {
            for y in [-0.5, 0.0, 0.5, 1.75, 3.5, 4.0, 4.1] {
                let p = Vec2::new(x, y);
                if circle_margin(&route, &c, p, 1.25) >= 0.0 {
                    for i in 0..720 {
                        let a = i as f64 * std::f64::consts::TAU / 720.0;
                        assert!(
                            on_surface(p.plus(Vec2::new(a.cos(), a.sin()).scaled(1.25))),
                            "{x},{y}"
                        );
                    }
                }
            }
        }
    }
}
