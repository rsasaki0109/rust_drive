use super::*;
use crate::{RoadNetworkSpec, TurnRestrictionKind};
use rustdriving_core::Vec2;

fn spec() -> RoadNetworkSpec {
    let nodes = [("a", 0., 0.), ("b", 6., 8.), ("c", 16., 8.), ("d", 6., 18.)]
        .into_iter()
        .map(|(id, x, y)| crate::RoadNode {
            id: id.into(),
            position: Vec2::new(x, y),
        })
        .collect::<Vec<_>>();
    let edges = [
        ("ab", 0, 1, 4., Some(3.)),
        ("bc", 1, 2, 2., Some(2.)),
        ("bd", 1, 3, 3., None),
        ("dc", 3, 2, 3., Some(5.)),
        ("ca", 2, 0, 4., None),
    ]
    .into_iter()
    .map(|(id, a, b, width, speed)| {
        let mut points = vec![nodes[a].position, nodes[b].position];
        if id == "ab" {
            points.insert(1, Vec2::new(3., 4.));
        }
        crate::RoadEdge {
            id: id.into(),
            from: nodes[a].id.clone(),
            to: nodes[b].id.clone(),
            points,
            half_width: width,
            speed_limit_m_s: speed,
        }
    })
    .collect();
    RoadNetworkSpec {
        nodes,
        edges,
        turn_restrictions: vec![],
    }
}
fn at(edge: &str, s_m: f64) -> RouteEndpoint {
    RouteEndpoint::Edge(EdgePosition {
        edge: edge.into(),
        s_m,
    })
}
fn restrict(map: &mut RoadNetworkSpec, from: &str, to: &str, kind: TurnRestrictionKind) {
    map.turn_restrictions.push(crate::TurnRestriction {
        id: "rule".into(),
        from_edge: from.into(),
        via_node: "b".into(),
        to_edge: to.into(),
        kind,
    });
}

#[test]
fn partial_polyline_has_exact_endpoints_knots_width_and_speed_arc() {
    let n = RoadNetwork::new(spec()).unwrap();
    let p = n
        .shortest_route_between_with_speed_zones(&at("ab", 2.5), &at("bc", 4.25), &[])
        .unwrap();
    assert_eq!(p.plan.edge_ids, ["ab", "bc"]);
    assert_eq!(p.plan.node_ids, ["b"]);
    assert_eq!(p.plan.distance_m, 11.75);
    assert_eq!(p.plan.route.half_width, 2.);
    assert_eq!(
        p.plan.route.points,
        [
            Vec2::new(1.5, 2.),
            Vec2::new(3., 4.),
            Vec2::new(6., 8.),
            Vec2::new(10.25, 8.)
        ]
    );
    assert_eq!(p.speed_zones.len(), 2);
    assert_eq!(p.speed_zones[0].start_s_m, 0.);
    assert_eq!(p.speed_zones[0].end_s_m, 7.5);
    assert_eq!(p.speed_zones[1].end_s_m, 11.75);
    let knot = n
        .shortest_route_between(&at("ab", 5.), &at("ab", 7.5), &[])
        .unwrap();
    assert_eq!(knot.route.points, [Vec2::new(3., 4.), Vec2::new(4.5, 6.)]);
    assert!(knot.node_ids.is_empty());
}

#[test]
fn origin_incoming_edge_and_destination_turn_obey_no_and_only_rules() {
    for kind in [TurnRestrictionKind::No, TurnRestrictionKind::Only] {
        let mut map = spec();
        restrict(
            &mut map,
            "ab",
            if kind == TurnRestrictionKind::No {
                "bc"
            } else {
                "bd"
            },
            kind,
        );
        let n = RoadNetwork::new(map.clone()).unwrap();
        let p = n
            .shortest_route_between(&at("ab", 7.), &"c".into(), &[])
            .unwrap();
        assert_eq!(p.edge_ids, ["ab", "bd", "dc"]);
        assert!(
            n.shortest_route_between(&at("ab", 7.), &"c".into(), &["bd".into()])
                .is_err()
        );
        // Reaching b does not authorize entering the forbidden goal prefix.
        assert!(
            n.shortest_route_between(&"a".into(), &at("bc", 2.), &[])
                .is_err()
        );
        let mut other = map.edges[0].clone();
        other.id = "other-arrival".into();
        other.points = vec![Vec2::new(0., 0.), Vec2::new(0., 8.), Vec2::new(6., 8.)];
        map.edges.push(other);
        let n = RoadNetwork::new(map).unwrap();
        let p = n
            .shortest_route_between(&"a".into(), &at("bc", 2.), &[])
            .unwrap();
        assert_eq!(p.edge_ids, ["other-arrival", "bc"]);
        assert_eq!(p.distance_m, 16.);
    }
}

#[test]
fn directed_backward_goal_requires_real_cycle_and_can_be_unreachable() {
    let n = RoadNetwork::new(spec()).unwrap();
    let forward = n
        .shortest_route_between(&at("ab", 2.), &at("ab", 8.), &[])
        .unwrap();
    assert_eq!(forward.edge_ids, ["ab"]);
    assert_eq!(forward.distance_m, 6.);
    let backward = n
        .shortest_route_between(&at("ab", 8.), &at("ab", 2.), &[])
        .unwrap();
    assert_eq!(backward.edge_ids, ["ab", "bc", "ca", "ab"]);
    assert_eq!(backward.node_ids, ["b", "c", "a"]);
    assert!((backward.distance_m - (14. + 16_f64.hypot(8.))).abs() < 1e-12);
    assert!(
        n.shortest_route_between(&at("ab", 8.), &at("ab", 2.), &["ca".into()])
            .is_err()
    );
    assert!(
        n.shortest_route_between(&at("ab", 2.), &at("ab", 8.), &["ab".into()])
            .is_err()
    );
}

#[test]
fn invalid_positions_closures_and_zero_routes_fail_without_clamping() {
    let n = RoadNetwork::new(spec()).unwrap();
    for s in [f64::NAN, f64::INFINITY, -1., 0., 10., 11.] {
        assert!(
            n.shortest_route_between(&at("ab", s), &"c".into(), &[])
                .is_err()
        );
        assert!(
            n.shortest_route_between(&"a".into(), &at("ab", s), &[])
                .is_err()
        );
    }
    assert!(
        n.shortest_route_between(&at("missing", 1.), &"c".into(), &[])
            .is_err()
    );
    assert!(
        n.shortest_route_between(&at("ab", 1.), &"missing".into(), &[])
            .is_err()
    );
    assert!(
        n.shortest_route_between(&at("ab", 1.), &at("ab", 1.), &[])
            .is_err()
    );
    assert!(
        n.shortest_route_between(&at("ab", 1.), &at("bc", 1.), &["missing".into()])
            .is_err()
    );
    assert!(
        n.shortest_route_between(&at("ab", 1.), &at("bc", 1.), &["bc".into()])
            .is_err()
    );
}

#[test]
fn node_requests_preserve_old_geometry_and_tie_choices() {
    let n = RoadNetwork::new(spec()).unwrap();
    let old = n.shortest_route_with_speed_zones("a", "c", &[]).unwrap();
    let new = n
        .shortest_route_between_with_speed_zones(&"a".into(), &"c".into(), &[])
        .unwrap();
    assert_eq!(old.plan.edge_ids, new.plan.edge_ids);
    assert_eq!(old.plan.route.points, new.plan.route.points);
    assert_eq!(old.plan.distance_m, new.plan.distance_m);
    let values = |zones: Vec<rustdriving_core::RouteSpeedZone>| {
        zones
            .into_iter()
            .map(|z| (z.start_s_m, z.end_s_m, z.speed_m_s))
            .collect::<Vec<_>>()
    };
    assert_eq!(values(old.speed_zones), values(new.speed_zones));
    assert!(new.plan.traversals.is_empty());
}

#[test]
fn adjacent_equal_caps_merge_and_uncapped_edges_remain_gaps() {
    let mut map = spec();
    map.edges[1].speed_limit_m_s = Some(3.);
    let n = RoadNetwork::new(map).unwrap();
    let p = n
        .shortest_route_between_with_speed_zones(&at("ab", 2.), &at("ca", 5.), &[])
        .unwrap();
    assert_eq!(p.speed_zones.len(), 1);
    assert_eq!(p.speed_zones[0].end_s_m, 18.);
    assert_eq!(p.plan.traversals.last().unwrap().end_s_m, 5.);
    let p = n
        .shortest_route_between_with_speed_zones(&at("ab", 2.), &at("dc", 5.), &[])
        .unwrap();
    assert_eq!(p.speed_zones.len(), 2);
    assert_eq!(p.speed_zones[1].start_s_m, 18.);
    assert_eq!(p.speed_zones[1].end_s_m, 23.);
}
