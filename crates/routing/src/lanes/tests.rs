use super::*;
use crate::RoadNetwork;

fn lane(id: &str, points: &[(f64, f64)], successors: &[&str]) -> Lane {
    Lane {
        id: id.into(),
        centerline: points.iter().map(|&(x, y)| Vec2::new(x, y)).collect(),
        width_m: 3.5,
        speed_limit_m_s: 4.0,
        successors: successors.iter().map(|id| (*id).into()).collect(),
        left: None,
        right: None,
    }
}
fn map() -> LaneMapSpec {
    let mut a = lane("a", &[(0., 0.), (10., 0.)], &["b", "detour"]);
    a.left = Some(LaneNeighbor {
        lane_id: "left".into(),
        change_allowed: true,
    });
    let mut left = lane("left", &[(0., 3.5), (10., 3.5)], &[]);
    left.right = Some(LaneNeighbor {
        lane_id: "a".into(),
        change_allowed: false,
    });
    let mut b = lane("b", &[(10., 0.), (20., 0.)], &["c"]);
    b.width_m = 3.2;
    b.speed_limit_m_s = 3.0;
    let mut detour = lane("detour", &[(10., 0.), (15., -5.), (20., 0.)], &["c"]);
    detour.speed_limit_m_s = 1.5;
    LaneMapSpec {
        schema_version: 1,
        lanes: vec![a, b, detour, lane("c", &[(20., 0.), (30., 0.)], &[]), left],
    }
}

#[test]
fn directed_lane_routing_preserves_width_speed_closures_and_resolved_geometry() {
    let network = LaneNetwork::new(map()).unwrap();
    let plan = network.following_route("a", "c", &[]).unwrap();
    assert_eq!(plan.lane_ids, ["a", "b", "c"]);
    assert_eq!(plan.distance_m, 30.0);
    assert_eq!(plan.route.half_width, 1.6);
    assert_eq!(plan.speed_limit_m_s, 3.0);
    let resolved = RoadNetwork::new(plan.network)
        .unwrap()
        .shortest_route(&plan.start_node, &plan.goal_node, &[])
        .unwrap();
    assert_eq!(resolved.edge_ids, plan.lane_ids);
    assert_eq!(resolved.route.points, plan.route.points);
    let detour = network.following_route("a", "c", &["b".into()]).unwrap();
    assert_eq!(detour.lane_ids, ["a", "detour", "c"]);
    assert!((detour.distance_m - (20.0 + 200.0_f64.sqrt())).abs() < 1e-9);
    assert_eq!(detour.speed_limit_m_s, 1.5);
    assert_eq!(
        network.following_route("a", "c", &[]).unwrap().lane_ids,
        ["a", "b", "c"]
    );
    assert!(network.following_route("c", "a", &[]).is_err());
}

#[test]
fn neighbor_permissions_are_directional_and_never_implicit_following_connections() {
    let network = LaneNetwork::new(map()).unwrap();
    assert_eq!(
        network
            .lane_change_target("a", LaneSide::Left, &[])
            .unwrap()
            .unwrap()
            .id,
        "left"
    );
    assert!(
        network
            .lane_change_target("left", LaneSide::Right, &[])
            .unwrap()
            .is_none()
    );
    assert!(
        network
            .lane_change_target("a", LaneSide::Right, &[])
            .unwrap()
            .is_none()
    );
    assert!(
        network
            .lane_change_target("a", LaneSide::Left, &["left".into()])
            .unwrap()
            .is_none()
    );
    assert!(
        network
            .lane_change_target("a", LaneSide::Left, &["a".into()])
            .unwrap()
            .is_none()
    );
    assert!(network.following_route("a", "left", &[]).is_err());
    let mut disconnected = map();
    disconnected.lanes[0].successors.clear();
    assert!(
        LaneNetwork::new(disconnected)
            .unwrap()
            .following_route("a", "b", &[])
            .is_err()
    );
}

#[test]
fn left_and_right_are_relative_to_travel_direction_including_westbound_lanes() {
    let mut spec = map();
    for lane in &mut spec.lanes {
        for point in &mut lane.centerline {
            point.x = -point.x;
            point.y = -point.y;
        }
    }
    let network = LaneNetwork::new(spec).unwrap();
    let target = network
        .lane_change_target("a", LaneSide::Left, &[])
        .unwrap()
        .unwrap();
    assert_eq!(target.centerline[0], Vec2::new(0.0, -3.5));
    let plan = network.following_route("a", "c", &[]).unwrap();
    assert_eq!(*plan.route.points.last().unwrap(), Vec2::new(-30.0, 0.0));
}

#[test]
fn unknown_closed_and_duplicate_requests_fail_while_one_whole_lane_is_a_valid_route() {
    let network = LaneNetwork::new(map()).unwrap();
    for (start, goal, closed) in [
        ("missing", "c", vec![]),
        ("a", "missing", vec![]),
        ("a", "c", vec!["a".into()]),
        ("a", "c", vec!["c".into()]),
        ("a", "c", vec!["unknown".into()]),
        ("a", "c", vec!["b".into(), "b".into()]),
        ("a", "c", vec!["b".into(), "detour".into()]),
    ] {
        assert!(network.following_route(start, goal, &closed).is_err());
    }
    assert!(
        network
            .lane_change_target("missing", LaneSide::Left, &[])
            .is_err()
    );
    assert!(
        network
            .lane_change_target("a", LaneSide::Left, &["unknown".into()])
            .is_err()
    );
    let plan = network.following_route("a", "a", &[]).unwrap();
    assert_eq!(plan.lane_ids, ["a"]);
    assert_eq!(plan.distance_m, 10.0);
}

#[test]
fn malformed_topology_geometry_and_lane_limits_are_rejected_before_search() {
    for case in 0..20 {
        let mut spec = map();
        match case {
            0 => spec.schema_version = 2,
            1 => spec.lanes.clear(),
            2 => spec.lanes.push(spec.lanes[0].clone()),
            3 => spec.lanes[0].id = " ".into(),
            4 => spec.lanes[0].width_m = f64::NAN,
            5 => spec.lanes[0].speed_limit_m_s = f64::INFINITY,
            6 => spec.lanes[0].centerline[0].x = f64::NAN,
            7 => spec.lanes[0].centerline[0].x = 1_000_001.0,
            8 => spec.lanes[0].centerline[1] = spec.lanes[0].centerline[0],
            9 => spec.lanes[0].successors.push("unknown".into()),
            10 => spec.lanes[0].successors.push("b".into()),
            11 => spec.lanes[1].centerline[0].x += 1e-7,
            12 => spec.lanes[0].left.as_mut().unwrap().lane_id = "unknown".into(),
            13 => spec.lanes[4].right = None,
            14 => spec.lanes[4].centerline.reverse(),
            15 => spec.lanes[4].centerline.iter_mut().for_each(|p| p.y = -3.5),
            16 => spec.lanes[4].width_m = 4.0,
            17 => spec.lanes[4].centerline[1].x = 12.0,
            18 => spec.lanes[4].centerline.insert(1, Vec2::new(5.0, 3.6)),
            _ => spec.lanes[4].centerline[1].y += 1e-4,
        }
        assert!(LaneNetwork::new(spec).is_err(), "case {case}");
    }
}

#[test]
fn map_work_and_memory_are_bounded() {
    let one = lane("a", &[(0., 0.), (10., 0.)], &[]);
    assert!(
        LaneNetwork::new(LaneMapSpec {
            schema_version: 1,
            lanes: vec![one.clone(); MAX_LANES + 1]
        })
        .is_err()
    );
    let mut too_many_points = one;
    too_many_points.centerline = (0..=MAX_LANE_POINTS)
        .map(|i| Vec2::new(i as f64, 0.0))
        .collect();
    assert!(
        LaneNetwork::new(LaneMapSpec {
            schema_version: 1,
            lanes: vec![too_many_points]
        })
        .is_err()
    );
    let lanes = (0..33)
        .map(|index| {
            let mut section = lane(&format!("n{index}"), &[(0., 0.), (1., 0.)], &[]);
            section.centerline = (0..MAX_LANE_POINTS)
                .map(|i| Vec2::new(i as f64, 0.0))
                .collect();
            section
        })
        .collect();
    assert!(
        LaneNetwork::new(LaneMapSpec {
            schema_version: 1,
            lanes
        })
        .unwrap_err()
        .contains("budget")
    );
    let lanes = (0..1100)
        .map(|index| {
            let mut section = lane(&format!("n{index}"), &[(0., 0.), (1., 0.)], &[]);
            section.successors = (0..MAX_SUCCESSORS).map(|i| format!("n{i}")).collect();
            section
        })
        .collect();
    assert!(
        LaneNetwork::new(LaneMapSpec {
            schema_version: 1,
            lanes
        })
        .unwrap_err()
        .contains("budget")
    );
}

#[test]
fn equal_cost_routes_are_stable_under_lane_and_successor_permutations() {
    let mut spec = map();
    let mut parallel = spec.lanes[1].clone();
    parallel.id = "b2".into();
    spec.lanes[0].successors.push("b2".into());
    spec.lanes.push(parallel);
    let first = LaneNetwork::new(spec.clone())
        .unwrap()
        .following_route("a", "c", &[])
        .unwrap();
    spec.lanes.reverse();
    for lane in &mut spec.lanes {
        lane.successors.reverse();
    }
    let second = LaneNetwork::new(spec)
        .unwrap()
        .following_route("a", "c", &[])
        .unwrap();
    assert_eq!(first.lane_ids, ["a", "b", "c"]);
    assert_eq!(first.lane_ids, second.lane_ids);
}

fn exhaustive(
    spec: &LaneMapSpec,
    start: &str,
    goal: &str,
    closed: &[String],
    visited: &mut BTreeSet<String>,
) -> Option<f64> {
    if closed.iter().any(|id| id == start) || !visited.insert(start.into()) {
        return None;
    }
    let lane = spec.lanes.iter().find(|lane| lane.id == start).unwrap();
    let length: f64 = lane
        .centerline
        .windows(2)
        .map(|p| (p[1].x - p[0].x).hypot(p[1].y - p[0].y))
        .sum();
    let result = if start == goal {
        Some(length)
    } else {
        lane.successors
            .iter()
            .filter_map(|next| exhaustive(spec, next, goal, closed, visited))
            .map(|rest| rest + length)
            .min_by(f64::total_cmp)
    };
    visited.remove(start);
    result
}

#[test]
fn routing_matches_exhaustive_simple_paths_on_a_cyclic_lane_graph() {
    let mut spec = map();
    spec.lanes[1].successors.push("back".into());
    spec.lanes[2].successors.push("back".into());
    spec.lanes
        .push(lane("back", &[(20., 0.), (10., 0.)], &["b", "detour"]));
    let network = LaneNetwork::new(spec.clone()).unwrap();
    for start in &spec.lanes {
        for goal in &spec.lanes {
            for closed in [
                vec![],
                vec!["b".into()],
                vec!["detour".into()],
                vec!["back".into()],
                vec!["c".into()],
            ] {
                let expected =
                    exhaustive(&spec, &start.id, &goal.id, &closed, &mut BTreeSet::new());
                let actual = network.following_route(&start.id, &goal.id, &closed);
                match expected {
                    Some(distance) => assert!((actual.unwrap().distance_m - distance).abs() < 1e-9),
                    None => assert!(actual.is_err()),
                }
            }
        }
    }
}
