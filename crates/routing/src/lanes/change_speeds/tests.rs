use super::*;
use crate::lanes::{Lane, LaneMapSpec, LaneNeighbor, LaneSide};
use rustdriving_core::Vec2;

fn map() -> LaneMapSpec {
    let lane = |id: &str, x0, x1, y, speed, next: &str| Lane {
        id: id.into(),
        centerline: vec![Vec2::new(x0, y), Vec2::new(x1, y)],
        width_m: 3.5,
        speed_limit_m_s: speed,
        successors: if next.is_empty() {
            vec![]
        } else {
            vec![next.into()]
        },
        left: None,
        right: None,
    };
    let mut source = lane("source", 0., 80., 0., 4., "source-exit");
    source.left = Some(LaneNeighbor {
        lane_id: "target".into(),
        change_allowed: true,
    });
    let mut target = lane("target", 0., 80., 3.5, 2., "target-exit");
    target.right = Some(LaneNeighbor {
        lane_id: "source".into(),
        change_allowed: true,
    });
    LaneMapSpec {
        schema_version: 1,
        lanes: vec![
            source,
            target,
            lane("source-exit", 80., 120., 0., 4., ""),
            lane("target-exit", 80., 120., 3.5, 4., ""),
        ],
    }
}
fn request(start_m: f64) -> LaneChangeRequest {
    LaneChangeRequest {
        side: LaneSide::Left,
        start_m,
        length_m: 30.,
    }
}
fn compile(spec: LaneMapSpec, start: f64) -> LaneSpeedChangeRoutePlan {
    LaneNetwork::new(spec)
        .unwrap()
        .lane_change_route_with_speed_zones(
            "source",
            "target-exit",
            &[],
            request(start),
            VehicleConfig::default(),
        )
        .unwrap()
}

#[test]
fn the_connector_uses_both_limits_and_true_arc_length_before_target_and_tail() {
    let result = compile(map(), 25.);
    assert_eq!(result.speed_zones.len(), 3);
    let zones = &result.speed_zones;
    assert_eq!(
        (zones[0].start_s_m, zones[0].end_s_m, zones[0].speed_m_s),
        (0., 25., 4.)
    );
    assert_eq!(zones[1].speed_m_s, 2.);
    assert!(zones[1].end_s_m > 80.2);
    assert_eq!(zones[1].end_s_m, zones[2].start_s_m);
    assert_eq!(zones[2].end_s_m, result.change.plan.route.length());
    assert_eq!(zones[2].speed_m_s, 4.);
    let mut inverse = map();
    inverse.lanes[0].speed_limit_m_s = 1.5;
    inverse.lanes[1].speed_limit_m_s = 4.;
    let inverse = compile(inverse, 25.);
    assert_eq!(inverse.speed_zones.len(), 2);
    assert_eq!(inverse.speed_zones[0].start_s_m, 0.);
    assert!(inverse.speed_zones[0].end_s_m > 55.2 && inverse.speed_zones[0].end_s_m < 56.);
    assert_eq!(inverse.speed_zones[0].speed_m_s, 1.5);
}

#[test]
fn zero_prefix_and_zero_target_suffix_emit_no_empty_speed_intervals() {
    for start in [0., 50.] {
        let result = compile(map(), start);
        assert!(result.speed_zones.iter().all(|z| z.valid()));
        assert_eq!(result.speed_zones[0].start_s_m, 0.);
        assert_eq!(
            result.speed_zones.last().unwrap().end_s_m,
            result.change.plan.route.length()
        );
    }
}

#[test]
fn successor_knots_and_mirrored_changes_keep_their_selected_policy() {
    for kind in 0..3 {
        let mut spec = map();
        if kind == 2 {
            let mut thin = spec.lanes[3].clone();
            thin.id = "target-thin".into();
            thin.centerline = vec![Vec2::new(80., 3.5), Vec2::new(80.01, 3.5)];
            thin.speed_limit_m_s = 1.5;
            thin.successors = vec!["target-exit".into()];
            spec.lanes[1].successors = vec![thin.id.clone()];
            spec.lanes[3].centerline[0].x = 80.01;
            spec.lanes.push(thin);
        }
        for lane in &mut spec.lanes {
            for p in &mut lane.centerline {
                if kind == 0 {
                    p.y = -p.y;
                }
                if kind == 1 {
                    *p = Vec2::new(-p.y, p.x);
                }
            }
            if kind == 0 {
                std::mem::swap(&mut lane.left, &mut lane.right);
            }
        }
        let side = if kind == 0 {
            LaneSide::Right
        } else {
            LaneSide::Left
        };
        let network = LaneNetwork::new(spec).unwrap();
        let result = network
            .lane_change_route_with_speed_zones(
                "source",
                "target-exit",
                &[],
                LaneChangeRequest {
                    side,
                    ..request(25.)
                },
                VehicleConfig::default(),
            )
            .unwrap();
        assert!(result.speed_zones.iter().all(|z| z.valid()));
        if kind == 2 {
            let thin = result
                .speed_zones
                .iter()
                .find(|z| z.speed_m_s == 1.5)
                .unwrap();
            assert!((thin.end_s_m - thin.start_s_m - 0.01).abs() < 1e-10);
        }
        assert!(
            network
                .lane_change_route_with_speed_zones(
                    "source",
                    "target-exit",
                    &["target".into()],
                    LaneChangeRequest {
                        side,
                        ..request(25.)
                    },
                    VehicleConfig::default()
                )
                .is_err()
        );
    }
}

#[test]
fn an_excessive_change_profile_is_rejected_without_dropping_tail_limits() {
    let mut spec = map();
    spec.lanes[1].successors = vec!["tail-0".into()];
    for i in 0..129 {
        spec.lanes.push(Lane {
            id: format!("tail-{i}"),
            centerline: vec![
                Vec2::new(80. + i as f64, 3.5),
                Vec2::new(81. + i as f64, 3.5),
            ],
            width_m: 3.5,
            speed_limit_m_s: 2. + (i % 2) as f64,
            successors: if i == 128 {
                vec![]
            } else {
                vec![format!("tail-{}", i + 1)]
            },
            left: None,
            right: None,
        });
    }
    let network = LaneNetwork::new(spec).unwrap();
    assert!(
        network
            .lane_change_route(
                "source",
                "tail-128",
                &[],
                request(25.),
                VehicleConfig::default()
            )
            .is_ok()
    );
    assert!(
        network
            .lane_change_route_with_speed_zones(
                "source",
                "tail-128",
                &[],
                request(25.),
                VehicleConfig::default()
            )
            .is_err()
    );
}
