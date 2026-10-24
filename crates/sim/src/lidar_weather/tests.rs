use super::*;
use rustdriving_perception::planar_sweep::validate_sweep;

fn baseline(empty: bool) -> LidarScan {
    let objects = if empty {
        vec![]
    } else {
        vec![
            WorldObject {
                id: 1,
                position: Vec2::new(8.0, 0.0),
                radius: 0.8,
            },
            WorldObject {
                id: 2,
                position: Vec2::new(22.0, 6.0),
                radius: 1.1,
            },
            WorldObject {
                id: 3,
                position: Vec2::new(38.0, -8.0),
                radius: 1.5,
            },
        ]
    };
    reference_scan(
        Pose {
            position: Vec2::default(),
            yaw: 0.0,
        },
        &objects,
        1.0,
        7,
    )
    .unwrap()
}
fn config() -> WeatherConfig {
    WeatherConfig {
        extinction_m_inv: 0.018,
        backscatter_rate_m_inv: 0.015,
        range_jitter_m: 0.3,
    }
}
#[test]
fn clear_is_bitwise_identity_even_when_ray_ownership_is_not_point_order() {
    let mut scan = baseline(false);
    scan.points.reverse();
    let count = scan.points.len();
    for ray in &mut scan.sweep.as_mut().unwrap().rays {
        if let PlanarRayOutcome::Return(i) = ray {
            *i = count - 1 - *i;
        }
    }
    let before = serde_json::to_vec(&scan).unwrap();
    let result = apply(&scan, WeatherConfig::CLEAR, 3).unwrap();
    assert_eq!(serde_json::to_vec(&result.scan).unwrap(), before);
    for (i, source) in result.sources.iter().enumerate() {
        assert!(matches!(source, ReturnSource::Surface { input } if *input == i));
    }
}
#[test]
fn common_draws_are_deterministic_and_input_is_unchanged() {
    let scan = baseline(false);
    let before = serde_json::to_vec(&scan).unwrap();
    let a = apply(&scan, config(), 4).unwrap();
    let b = apply(&scan, config(), 4).unwrap();
    assert_eq!(
        serde_json::to_vec(&a).unwrap(),
        serde_json::to_vec(&b).unwrap()
    );
    assert_eq!(serde_json::to_vec(&scan).unwrap(), before);
    assert_ne!(
        serde_json::to_vec(&a).unwrap(),
        serde_json::to_vec(&apply(&scan, config(), 5).unwrap()).unwrap()
    );
}
#[test]
fn weather_loss_is_unavailable_and_cannot_certify_an_empty_scene() {
    let scan = baseline(true);
    let result = apply(
        &scan,
        WeatherConfig {
            extinction_m_inv: 0.1,
            ..WeatherConfig::CLEAR
        },
        8,
    )
    .unwrap();
    assert!(result.scan.points.is_empty());
    assert!(
        result
            .rays
            .iter()
            .any(|r| matches!(r, PlanarRayOutcome::Unavailable))
    );
    assert!(result.scan.sweep.is_none());
    assert!(validate_sweep(&result.scan).is_err());
    let with_surfaces = baseline(false);
    let lost = apply(
        &with_surfaces,
        WeatherConfig {
            extinction_m_inv: 0.1,
            ..WeatherConfig::CLEAR
        },
        8,
    )
    .unwrap();
    for (before, after) in with_surfaces.sweep.unwrap().rays.iter().zip(lost.rays) {
        if matches!(before, PlanarRayOutcome::Return(_)) {
            assert!(!matches!(after, PlanarRayOutcome::Clear));
        }
    }
}
#[test]
fn particles_replace_only_farther_surfaces_and_have_exclusive_return_ownership() {
    let scan = baseline(false);
    let result = apply(&scan, config(), 11).unwrap();
    let mut seen = vec![false; result.scan.points.len()];
    for (beam, ray) in result.rays.iter().enumerate() {
        if let PlanarRayOutcome::Return(i) = *ray {
            assert!(!seen[i]);
            seen[i] = true;
            if let ReturnSource::Precipitation { beam: owner } = result.sources[i] {
                assert_eq!(owner, beam);
                assert!(
                    result.effects[beam].particle_range_m.unwrap()
                        < result.effects[beam].input_range_m.unwrap_or(45.0)
                );
            }
        }
    }
    assert!(seen.into_iter().all(|v| v));
    let empty = apply(&baseline(true), config(), 11).unwrap();
    assert!(!empty.scan.points.is_empty());
    assert!(
        empty
            .sources
            .iter()
            .all(|s| matches!(s, ReturnSource::Precipitation { .. }))
    );
}
#[test]
fn larger_extinction_never_restores_a_surface_with_common_draws() {
    let scan = baseline(false);
    let light = apply(
        &scan,
        WeatherConfig {
            extinction_m_inv: 0.002,
            ..WeatherConfig::CLEAR
        },
        3,
    )
    .unwrap();
    let heavy = apply(
        &scan,
        WeatherConfig {
            extinction_m_inv: 0.05,
            ..WeatherConfig::CLEAR
        },
        3,
    )
    .unwrap();
    for (a, b) in light.rays.iter().zip(&heavy.rays) {
        if matches!(a, PlanarRayOutcome::Unavailable) {
            assert!(matches!(b, PlanarRayOutcome::Unavailable));
        }
    }
}
#[test]
fn added_range_error_is_bounded_and_never_changes_the_ray_direction() {
    let scan = baseline(false);
    let cfg = WeatherConfig {
        range_jitter_m: 0.25,
        ..WeatherConfig::CLEAR
    };
    let result = apply(&scan, cfg, 21).unwrap();
    for (point, source) in result.scan.points.iter().zip(result.sources) {
        let ReturnSource::Surface { input } = source else {
            panic!("no particles configured");
        };
        let original = scan.points[input];
        assert!(
            (point.x.hypot(point.y) - original.x.hypot(original.y)).abs()
                <= cfg.range_jitter_m + 1e-12
        );
        assert!((point.x * original.y - point.y * original.x).abs() < 1e-10);
    }
}
#[test]
fn invalid_or_incomplete_input_is_rejected_before_weather_is_applied() {
    let mut scan = baseline(false);
    for value in [f64::NAN, f64::INFINITY, -0.1, 0.11] {
        assert!(
            apply(
                &scan,
                WeatherConfig {
                    extinction_m_inv: value,
                    ..WeatherConfig::CLEAR
                },
                1
            )
            .is_err()
        );
    }
    scan.sweep.as_mut().unwrap().rays[0] = PlanarRayOutcome::Unavailable;
    assert!(apply(&scan, config(), 1).is_err());
    scan.sweep = None;
    assert!(apply(&scan, config(), 1).is_err());
    assert!(
        reference_scan(
            Pose {
                position: Vec2::default(),
                yaw: f64::NAN
            },
            &[],
            0.0,
            1
        )
        .is_err()
    );
}
