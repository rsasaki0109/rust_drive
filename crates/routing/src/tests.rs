use super::*;

fn map() -> RoadNetworkSpec {
    let nodes: Vec<_> = [
        ("s", 0., 0.),
        ("a", 10., 0.),
        ("b", 0., 20.),
        ("g", 20., 0.),
    ]
    .into_iter()
    .map(|(id, x, y)| RoadNode {
        id: id.into(),
        position: Vec2::new(x, y),
    })
    .collect();
    let edges = [
        ("sa", 0, 1),
        ("ag", 1, 3),
        ("sb", 0, 2),
        ("bg", 2, 3),
        ("as", 1, 0),
    ]
    .into_iter()
    .map(|(id, a, b)| RoadEdge {
        id: id.into(),
        from: nodes[a].id.clone(),
        to: nodes[b].id.clone(),
        points: vec![nodes[a].position, nodes[b].position],
        half_width: if id == "ag" { 2. } else { 4. },
    })
    .collect();
    RoadNetworkSpec {
        nodes,
        edges,
        turn_restrictions: vec![],
    }
}

#[test]
fn shortest_path_and_closure_detour_follow_direction_and_geometry() {
    let network = RoadNetwork::new(map()).unwrap();
    let direct = network.shortest_route("s", "g", &[]).unwrap();
    assert_eq!(direct.edge_ids, ["sa", "ag"]);
    assert_eq!(direct.node_ids, ["s", "a", "g"]);
    assert_eq!(direct.distance_m, 20.);
    assert_eq!(direct.route.half_width, 2.);
    assert_eq!(direct.route.points.len(), 3);
    let detour = network.shortest_route("s", "g", &["ag".into()]).unwrap();
    assert_eq!(detour.edge_ids, ["sb", "bg"]);
    assert!(detour.distance_m > direct.distance_m);
    assert!((detour.distance_m - detour.route.length()).abs() < 1e-10);
    assert!(network.shortest_route("g", "s", &[]).is_err());
    assert!(
        network
            .shortest_route("s", "g", &["ag".into(), "bg".into()])
            .is_err()
    );
    // Closures belong to a request: recomputing an open map restores the short route.
    assert_eq!(
        network.shortest_route("s", "g", &[]).unwrap().edge_ids,
        direct.edge_ids
    );
}

#[test]
fn unknown_inputs_and_stationary_requests_fail_explicitly() {
    let network = RoadNetwork::new(map()).unwrap();
    assert!(network.shortest_route("missing", "g", &[]).is_err());
    assert!(network.shortest_route("s", "missing", &[]).is_err());
    assert!(network.shortest_route("s", "s", &[]).is_err());
    assert!(network.shortest_route("s", "g", &["typo".into()]).is_err());
}

#[test]
fn equal_cost_parallel_edges_are_deterministic_under_input_permutation() {
    let mut spec = map();
    let mut edge = spec.edges[0].clone();
    edge.id = "00-preferred".into();
    spec.edges.push(edge);
    let a = RoadNetwork::new(spec.clone())
        .unwrap()
        .shortest_route("s", "g", &[])
        .unwrap();
    spec.nodes.reverse();
    spec.edges.reverse();
    let b = RoadNetwork::new(spec)
        .unwrap()
        .shortest_route("s", "g", &[])
        .unwrap();
    assert_eq!(a.edge_ids, b.edge_ids);
    assert_eq!(a.edge_ids[0], "00-preferred");
}

#[test]
fn malformed_maps_are_rejected_before_search() {
    for mutation in 0..9 {
        let mut spec = map();
        match mutation {
            0 => spec.nodes.push(spec.nodes[0].clone()),
            1 => spec.edges.push(spec.edges[0].clone()),
            2 => spec.edges[0].to = "unknown".into(),
            3 => spec.edges[0].points[0].x = 2.,
            4 => spec.edges[0].points[1] = spec.edges[0].points[0],
            5 => spec.edges[0].half_width = f64::NAN,
            6 => spec.nodes[0].position.x = f64::INFINITY,
            7 => spec.edges[0].points[1].x = f64::NAN,
            _ => spec.edges[0].id.clear(),
        }
        assert!(RoadNetwork::new(spec).is_err(), "mutation {mutation}");
    }
}

#[test]
fn tolerance_is_canonicalized_without_duplicate_junction_points() {
    let mut spec = map();
    spec.edges[0].points[1].x += 1e-7;
    let plan = RoadNetwork::new(spec)
        .unwrap()
        .shortest_route("s", "g", &[])
        .unwrap();
    assert_eq!(
        plan.route.points,
        vec![Vec2::new(0., 0.), Vec2::new(10., 0.), Vec2::new(20., 0.)]
    );
}

#[test]
fn search_matches_exhaustive_simple_paths_on_a_cyclic_graph() {
    let mut spec = map();
    for (id, a, b) in [("ab", 1, 2), ("ba", 2, 1), ("gs", 3, 0)] {
        spec.edges.push(RoadEdge {
            id: id.into(),
            from: spec.nodes[a].id.clone(),
            to: spec.nodes[b].id.clone(),
            points: vec![spec.nodes[a].position, spec.nodes[b].position],
            half_width: 3.,
        });
    }
    fn enumerate(
        spec: &RoadNetworkSpec,
        at: &str,
        goal: &str,
        closed: &[String],
        seen: &mut Vec<String>,
        cost: f64,
        best: &mut f64,
    ) {
        if at == goal {
            *best = best.min(cost);
            return;
        }
        for edge in &spec.edges {
            if edge.from != at || closed.contains(&edge.id) || seen.contains(&edge.to) {
                continue;
            }
            seen.push(edge.to.clone());
            let length: f64 = edge.points.windows(2).map(|p| p[0].distance(p[1])).sum();
            enumerate(spec, &edge.to, goal, closed, seen, cost + length, best);
            seen.pop();
        }
    }
    let network = RoadNetwork::new(spec.clone()).unwrap();
    for start in &spec.nodes {
        for goal in &spec.nodes {
            if start.id == goal.id {
                continue;
            }
            for mask in 0..(1 << spec.edges.len()) {
                let closed: Vec<_> = spec
                    .edges
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| mask & (1 << i) != 0)
                    .map(|(_, e)| e.id.clone())
                    .collect();
                let mut best = f64::INFINITY;
                enumerate(
                    &spec,
                    &start.id,
                    &goal.id,
                    &closed,
                    &mut vec![start.id.clone()],
                    0.,
                    &mut best,
                );
                let result = network.shortest_route(&start.id, &goal.id, &closed);
                if best.is_finite() {
                    assert!((result.unwrap().distance_m - best).abs() < 1e-10);
                } else {
                    assert!(result.is_err());
                }
            }
        }
    }
}

fn restriction(
    id: &str,
    from: &str,
    via: &str,
    to: &str,
    kind: TurnRestrictionKind,
) -> TurnRestriction {
    TurnRestriction {
        id: id.into(),
        from_edge: from.into(),
        via_node: via.into(),
        to_edge: to.into(),
        kind,
    }
}

fn add_edge(spec: &mut RoadNetworkSpec, id: &str, from: &str, to: &str) {
    let position = |id: &str| {
        spec.nodes
            .iter()
            .find(|node| node.id == id)
            .unwrap()
            .position
    };
    spec.edges.push(RoadEdge {
        id: id.into(),
        from: from.into(),
        to: to.into(),
        points: vec![position(from), position(to)],
        half_width: 3.,
    });
}

#[test]
fn restricted_search_retains_more_expensive_legal_arrival_at_same_node() {
    let mut spec = map();
    spec.edges.retain(|edge| edge.id != "bg");
    add_edge(&mut spec, "ba", "b", "a");
    spec.turn_restrictions.push(restriction(
        "no-short-arrival",
        "sa",
        "a",
        "ag",
        TurnRestrictionKind::No,
    ));
    let plan = RoadNetwork::new(spec)
        .unwrap()
        .shortest_route("s", "g", &[])
        .unwrap();
    // Node-only Dijkstra would discard the costlier b -> a arrival, although
    // that arrival is the only one legally able to continue to g.
    assert_eq!(plan.edge_ids, ["sb", "ba", "ag"]);
    assert_eq!(plan.node_ids, ["s", "b", "a", "g"]);
    assert!((plan.distance_m - (30. + 500_f64.sqrt())).abs() < 1e-10);
    assert_eq!(plan.route.half_width, 2.);
    assert_eq!(plan.route.points.len(), 4);
}

#[test]
fn only_turn_restrictions_and_closures_never_fall_back_to_prohibited_turn() {
    let mut spec = map();
    add_edge(&mut spec, "ab", "a", "b");
    spec.turn_restrictions = vec![
        restriction("only-detour", "sa", "a", "ab", TurnRestrictionKind::Only),
        restriction("no-return", "sa", "a", "as", TurnRestrictionKind::No),
    ];
    let network = RoadNetwork::new(spec).unwrap();
    let plan = network.shortest_route("s", "g", &["sb".into()]).unwrap();
    assert_eq!(plan.edge_ids, ["sa", "ab", "bg"]);
    assert!(
        network
            .shortest_route("s", "g", &["sb".into(), "ab".into()])
            .is_err()
    );
    // No fictitious arrival at the request's first node: its outgoing edge
    // remains legal even though a later arrival along sa has a restriction.
    assert_eq!(
        network.shortest_route("a", "g", &[]).unwrap().edge_ids,
        ["ag"]
    );
}

#[test]
fn u_turns_are_directed_transitions_and_repeated_nodes_can_be_required() {
    let mut spec = map();
    spec.edges
        .retain(|edge| matches!(edge.id.as_str(), "sa" | "ag"));
    add_edge(&mut spec, "ab", "a", "b");
    add_edge(&mut spec, "ba", "b", "a");
    spec.turn_restrictions = vec![
        restriction("no-direct", "sa", "a", "ag", TurnRestrictionKind::No),
        restriction("only-return", "ab", "b", "ba", TurnRestrictionKind::Only),
    ];
    let plan = RoadNetwork::new(spec.clone())
        .unwrap()
        .shortest_route("s", "g", &[])
        .unwrap();
    assert_eq!(plan.edge_ids, ["sa", "ab", "ba", "ag"]);
    assert_eq!(plan.node_ids, ["s", "a", "b", "a", "g"]);
    spec.turn_restrictions[1].kind = TurnRestrictionKind::No;
    assert!(
        RoadNetwork::new(spec)
            .unwrap()
            .shortest_route("s", "g", &[])
            .is_err()
    );
}

#[test]
fn restricted_equal_cost_choices_ignore_input_order() {
    let mut spec = map();
    let mut parallel = spec.edges[0].clone();
    parallel.id = "00-preferred".into();
    spec.edges.push(parallel);
    spec.turn_restrictions = vec![
        restriction("no-sa-return", "sa", "a", "as", TurnRestrictionKind::No),
        restriction(
            "no-parallel-return",
            "00-preferred",
            "a",
            "as",
            TurnRestrictionKind::No,
        ),
    ];
    let a = RoadNetwork::new(spec.clone())
        .unwrap()
        .shortest_route("s", "g", &[])
        .unwrap();
    spec.nodes.reverse();
    spec.edges.reverse();
    spec.turn_restrictions.reverse();
    let b = RoadNetwork::new(spec)
        .unwrap()
        .shortest_route("s", "g", &[])
        .unwrap();
    assert_eq!(a.edge_ids, ["00-preferred", "ag"]);
    assert_eq!(a.edge_ids, b.edge_ids);
    assert_eq!(a.distance_m, b.distance_m);
}

#[test]
fn invalid_and_conflicting_turn_restrictions_are_rejected_before_search() {
    for mutation in 0..11 {
        let mut spec = map();
        spec.turn_restrictions.push(restriction(
            "rule",
            "sa",
            "a",
            "ag",
            TurnRestrictionKind::No,
        ));
        match mutation {
            0 => spec.turn_restrictions[0].id.clear(),
            1 => spec.turn_restrictions[0].id = "  ".into(),
            2 => spec.turn_restrictions[0].from_edge = "missing".into(),
            3 => spec.turn_restrictions[0].to_edge = "missing".into(),
            4 => spec.turn_restrictions[0].via_node = "missing".into(),
            5 => spec.turn_restrictions[0].via_node = "s".into(),
            6 => spec.turn_restrictions[0].to_edge = "sb".into(),
            7 => spec.turn_restrictions.push(restriction(
                "rule",
                "sa",
                "a",
                "as",
                TurnRestrictionKind::No,
            )),
            8 => spec.turn_restrictions.push(restriction(
                "duplicate",
                "sa",
                "a",
                "ag",
                TurnRestrictionKind::No,
            )),
            9 => spec.turn_restrictions.push(restriction(
                "contradiction",
                "sa",
                "a",
                "ag",
                TurnRestrictionKind::Only,
            )),
            _ => {
                spec.turn_restrictions[0].kind = TurnRestrictionKind::Only;
                spec.turn_restrictions.push(restriction(
                    "conflicting-only",
                    "sa",
                    "a",
                    "as",
                    TurnRestrictionKind::Only,
                ));
            }
        }
        assert!(RoadNetwork::new(spec).is_err(), "mutation {mutation}");
    }
    // Contradictions and duplicate IDs must also fail in reversed input order.
    let mut spec = map();
    spec.turn_restrictions = vec![
        restriction("only", "sa", "a", "ag", TurnRestrictionKind::Only),
        restriction("no", "sa", "a", "ag", TurnRestrictionKind::No),
    ];
    assert!(RoadNetwork::new(spec).is_err());
}

#[test]
fn restriction_count_is_bounded() {
    let mut spec = map();
    spec.turn_restrictions = (0..10_001)
        .map(|i| {
            restriction(
                &format!("rule-{i}"),
                "sa",
                "a",
                "ag",
                TurnRestrictionKind::No,
            )
        })
        .collect();
    assert!(RoadNetwork::new(spec).unwrap_err().contains("10000"));
}

#[test]
fn restricted_search_matches_exhaustive_edge_paths_with_every_closure_subset() {
    let mut spec = map();
    add_edge(&mut spec, "ab", "a", "b");
    add_edge(&mut spec, "ba", "b", "a");
    spec.turn_restrictions = vec![
        restriction("no-direct", "sa", "a", "ag", TurnRestrictionKind::No),
        restriction("only-back", "ab", "b", "ba", TurnRestrictionKind::Only),
    ];
    fn enumerate(
        spec: &RoadNetworkSpec,
        at: &str,
        goal: &str,
        closed: &[String],
        path: &mut Vec<String>,
        cost: f64,
        best: &mut f64,
    ) {
        if at == goal {
            *best = best.min(cost);
            return;
        }
        // Positive lengths imply a shortest state path cannot traverse the
        // same directed edge twice. Nodes can repeat with different arrivals.
        for edge in &spec.edges {
            if edge.from != at || closed.contains(&edge.id) || path.contains(&edge.id) {
                continue;
            }
            let allowed = spec.turn_restrictions.iter().all(|rule| {
                if path.last() != Some(&rule.from_edge) {
                    return true;
                }
                match rule.kind {
                    TurnRestrictionKind::No => edge.id != rule.to_edge,
                    TurnRestrictionKind::Only => edge.id == rule.to_edge,
                }
            });
            if !allowed {
                continue;
            }
            path.push(edge.id.clone());
            let length: f64 = edge
                .points
                .windows(2)
                .map(|pair| pair[0].distance(pair[1]))
                .sum();
            enumerate(spec, &edge.to, goal, closed, path, cost + length, best);
            path.pop();
        }
    }
    let network = RoadNetwork::new(spec.clone()).unwrap();
    for start in &spec.nodes {
        for goal in &spec.nodes {
            if start.id == goal.id {
                continue;
            }
            for mask in 0..(1 << spec.edges.len()) {
                let closed: Vec<_> = spec
                    .edges
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| mask & (1 << i) != 0)
                    .map(|(_, edge)| edge.id.clone())
                    .collect();
                let mut best = f64::INFINITY;
                enumerate(
                    &spec,
                    &start.id,
                    &goal.id,
                    &closed,
                    &mut vec![],
                    0.,
                    &mut best,
                );
                let result = network.shortest_route(&start.id, &goal.id, &closed);
                if best.is_finite() {
                    assert!((result.unwrap().distance_m - best).abs() < 1e-10);
                } else {
                    assert!(result.is_err());
                }
            }
        }
    }
}
