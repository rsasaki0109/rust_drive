use super::*;

fn node(id: u64, x: f64, y: f64) -> OverpassElement {
    OverpassElement::Node {
        id,
        lat: y * 0.001,
        lon: x * 0.001,
        tags: BTreeMap::new(),
    }
}
fn way(id: u64, refs: &[u64], extra: &[(&str, &str)]) -> OverpassElement {
    let mut tags = BTreeMap::from([("highway".into(), "residential".into())]);
    tags.extend(extra.iter().map(|(k, v)| ((*k).into(), (*v).into())));
    OverpassElement::Way {
        id,
        nodes: refs.to_vec(),
        tags,
    }
}
fn options() -> ImportOptions {
    ImportOptions {
        origin_lat: 0.0,
        origin_lon: 0.0,
        default_half_width: 3.0,
    }
}
fn topology() -> OverpassDocument {
    OverpassDocument {
        elements: vec![
            node(1, 0., 0.),
            node(2, 1., 0.),
            node(3, 2., 0.),
            node(4, 1., 1.),
            node(5, 0.5, 0.),
            way(10, &[1, 5, 2, 3], &[]),
            way(20, &[2, 4], &[]),
            way(30, &[1, 4, 3], &[("oneway", "yes")]),
        ],
    }
}
#[test]
fn independent_wgs84_equatorial_distances_and_dateline_projection() {
    let e = project_enu(0., 0.001, 0., 0.).unwrap();
    assert!((e.x - 111.319490788).abs() < 1e-6);
    assert!(e.y.abs() < 1e-9);
    let n = project_enu(0.001, 0., 0., 0.).unwrap();
    assert!((n.y - 110.574275817).abs() < 1e-6);
    assert!(n.x.abs() < 1e-9);
    assert!(project_enu(0., -179.999, 0., 179.999).unwrap().x > 222.0);
    assert!(project_enu(0., 180., 0., 0.).is_err());
}
#[test]
fn shared_junctions_split_but_keep_intermediate_way_geometry() {
    let (map, report) = import(topology(), options()).unwrap();
    assert_eq!(report.imported_ways, 3);
    assert_eq!(map.nodes.len(), 4);
    assert_eq!(map.edges.len(), 8);
    let graph = RoadNetwork::new(map).unwrap();
    let plan = graph
        .shortest_route("osm-node-1", "osm-node-3", &[])
        .unwrap();
    assert_eq!(
        plan.edge_ids,
        ["osm-way-10-0-forward", "osm-way-10-1-forward"]
    );
    assert_eq!(plan.route.points.len(), 4);
    assert!((plan.distance_m - 222.63898154).abs() < 1e-5);
    let detour = graph
        .shortest_route("osm-node-1", "osm-node-3", &["osm-way-10-0-forward".into()])
        .unwrap();
    assert!(detour.distance_m > plan.distance_m);
    assert_eq!(
        detour.edge_ids,
        ["osm-way-30-0-forward", "osm-way-30-1-forward"]
    );
}
#[test]
fn yes_reverse_and_roundabout_direction_cannot_be_traversed_backwards() {
    for (tags, start, goal) in [
        (&[("oneway", "yes")][..], "osm-node-1", "osm-node-2"),
        (&[("oneway", "-1")][..], "osm-node-2", "osm-node-1"),
        (
            &[("junction", "roundabout")][..],
            "osm-node-1",
            "osm-node-2",
        ),
    ] {
        let doc = OverpassDocument {
            elements: vec![node(1, 0., 0.), node(2, 1., 0.), way(10, &[1, 2], tags)],
        };
        let (map, _) = import(doc, options()).unwrap();
        let graph = RoadNetwork::new(map).unwrap();
        assert!(graph.shortest_route(start, goal, &[]).is_ok());
        assert!(graph.shortest_route(goal, start, &[]).is_err());
    }
}
#[test]
fn reordered_elements_produce_identical_graph_and_route() {
    let a = topology();
    let mut b = a.clone();
    b.elements.reverse();
    let (a, _) = import(a, options()).unwrap();
    let (b, _) = import(b, options()).unwrap();
    assert_eq!(format!("{a:?}"), format!("{b:?}"));
}
#[test]
fn motor_access_precedence_and_filter_reporting_are_explicit() {
    let doc = OverpassDocument {
        elements: vec![
            node(1, 0., 0.),
            node(2, 1., 0.),
            way(10, &[1, 2], &[("access", "private")]),
            way(
                20,
                &[1, 2],
                &[("access", "no"), ("motorcar", "yes"), ("width", "6 m")],
            ),
            way(30, &[1, 2], &[("motor_vehicle", "no")]),
            way(40, &[1, 2], &[("highway", "footway")]),
            way(50, &[1, 2], &[("access:conditional", "no @ (Mo-Fr)")]),
            way(60, &[1], &[]),
            OverpassElement::Other,
        ],
    };
    let (map, r) = import(doc, options()).unwrap();
    assert_eq!(r.imported_ways, 1);
    assert_eq!(r.filtered_ways, 5);
    assert_eq!(r.clipped_short_ways, 1);
    assert_eq!(r.unsupported_elements, 1);
    assert_eq!(r.default_width_ways, 0);
    assert_eq!(map.edges[0].half_width, 3.0);
}
#[test]
fn restricted_nodes_and_physical_barriers_filter_connected_ways() {
    let mut restricted = node(2, 1., 0.);
    if let OverpassElement::Node { tags, .. } = &mut restricted {
        tags.insert("barrier".into(), "gate".into());
    }
    let doc = OverpassDocument {
        elements: vec![
            node(1, 0., 0.),
            restricted,
            node(3, 0., 1.),
            way(10, &[1, 2], &[]),
            way(20, &[1, 3], &[]),
        ],
    };
    let (map, report) = import(doc, options()).unwrap();
    assert_eq!(report.node_restricted_ways, 1);
    assert!(
        map.edges
            .iter()
            .all(|e| !e.to.ends_with("-2") && !e.from.ends_with("-2"))
    );
}
#[test]
fn a_closed_roundabout_keeps_forward_cycle_without_self_edges() {
    let doc = OverpassDocument {
        elements: vec![
            node(1, 0., 0.),
            node(2, 1., 0.),
            node(3, 1., 1.),
            node(4, 0., 1.),
            way(10, &[1, 2, 3, 4, 1], &[("junction", "roundabout")]),
        ],
    };
    let (map, _) = import(doc, options()).unwrap();
    assert_eq!(map.edges.len(), 2);
    assert!(map.edges.iter().all(|e| e.from != e.to));
    let graph = RoadNetwork::new(map).unwrap();
    assert!(
        graph
            .shortest_route("osm-node-1", "osm-node-3", &[])
            .is_ok()
    );
    assert!(
        graph
            .shortest_route("osm-node-3", "osm-node-1", &[])
            .is_ok()
    );
}
#[test]
fn bad_geometry_missing_nodes_duplicates_tags_and_bounds_fail() {
    let mut invalid = options();
    invalid.default_half_width = f64::NAN;
    assert!(import(topology(), invalid).is_err());
    for refs in [vec![1, 99], vec![1, 2, 1, 2], vec![], vec![1, 1]] {
        assert!(
            import(
                OverpassDocument {
                    elements: vec![node(1, 0., 0.), node(2, 1., 0.), way(10, &refs, &[])]
                },
                options()
            )
            .is_err()
        );
    }
    assert!(
        import(
            OverpassDocument {
                elements: vec![node(1, 0., 0.), node(2, 0., 0.), way(10, &[1, 2], &[])]
            },
            options()
        )
        .is_err()
    );
    assert!(
        import(
            OverpassDocument {
                elements: vec![node(1, 0., 0.), node(1, 1., 0.), way(10, &[1, 1], &[])]
            },
            options()
        )
        .is_err()
    );
    for tags in [
        &[("oneway", "reversible")][..],
        &[("width", "12 feet")][..],
        &[("width", "NaN")][..],
    ] {
        assert!(
            import(
                OverpassDocument {
                    elements: vec![node(1, 0., 0.), node(2, 1., 0.), way(10, &[1, 2], tags)]
                },
                options()
            )
            .is_err()
        );
    }
    assert!(project_enu(0., 0.03, 0., 0.).is_err());
    assert!(project_enu(90., 0., 0., 0.).is_err());
}

fn relation(id: u64, from: u64, via: u64, to: u64, extra: &[(&str, &str)]) -> OverpassElement {
    let mut tags = BTreeMap::from([("type".into(), "restriction".into())]);
    tags.extend(extra.iter().map(|(k, v)| ((*k).into(), (*v).into())));
    OverpassElement::Relation {
        id,
        members: vec![
            RelationMember {
                member_type: "way".into(),
                reference: from,
                role: "from".into(),
            },
            RelationMember {
                member_type: "node".into(),
                reference: via,
                role: "via".into(),
            },
            RelationMember {
                member_type: "way".into(),
                reference: to,
                role: "to".into(),
            },
        ],
        tags,
    }
}
fn restricted_junction(value: &str) -> OverpassDocument {
    OverpassDocument {
        elements: vec![
            node(1, 0., 0.),
            node(2, 1., 0.),
            node(3, 2., 0.),
            node(4, 1., 1.),
            way(10, &[1, 2], &[("oneway", "yes")]),
            way(20, &[2, 3], &[("oneway", "yes")]),
            way(30, &[2, 4], &[("oneway", "yes")]),
            way(40, &[4, 3], &[("oneway", "yes")]),
            relation(100, 10, 2, 20, &[("restriction", value)]),
        ],
    }
}
#[test]
fn node_via_no_turns_choose_an_actual_legal_detour() {
    for value in [
        "no_left_turn",
        "no_right_turn",
        "no_straight_on",
        "no_u_turn",
    ] {
        let (map, report) = import(restricted_junction(value), options()).unwrap();
        assert_eq!(report.input_turn_relations, 1);
        assert_eq!(report.imported_turn_restrictions, 1);
        assert_eq!(report.ignored_turn_relations, 0);
        let graph = RoadNetwork::new(map).unwrap();
        let plan = graph
            .shortest_route("osm-node-1", "osm-node-3", &[])
            .unwrap();
        assert_eq!(
            plan.edge_ids,
            [
                "osm-way-10-0-forward",
                "osm-way-30-0-forward",
                "osm-way-40-0-forward"
            ]
        );
        assert!(plan.distance_m > 300.0);
        assert!(
            graph
                .shortest_route("osm-node-1", "osm-node-3", &["osm-way-30-0-forward".into()])
                .is_err()
        );
        // A restriction applies to an arrival along its declared from edge.
        assert_eq!(
            graph
                .shortest_route("osm-node-2", "osm-node-3", &[])
                .unwrap()
                .edge_ids,
            ["osm-way-20-0-forward"]
        );
    }
}
#[test]
fn only_turn_import_forbids_alternatives_even_when_selected_exit_is_closed() {
    for value in ["only_left_turn", "only_right_turn", "only_straight_on"] {
        let (map, _) = import(restricted_junction(value), options()).unwrap();
        let graph = RoadNetwork::new(map).unwrap();
        assert_eq!(
            graph
                .shortest_route("osm-node-1", "osm-node-3", &[])
                .unwrap()
                .edge_ids,
            ["osm-way-10-0-forward", "osm-way-20-0-forward"]
        );
        assert!(
            graph
                .shortest_route("osm-node-1", "osm-node-4", &[])
                .is_err()
        );
        assert!(
            graph
                .shortest_route("osm-node-1", "osm-node-3", &["osm-way-20-0-forward".into()])
                .is_err()
        );
    }
}
#[test]
fn relation_direction_minus_one_selects_only_real_incoming_and_outgoing_edges() {
    let doc = OverpassDocument {
        elements: vec![
            node(1, 0., 0.),
            node(2, 1., 0.),
            node(3, 2., 0.),
            way(10, &[2, 1], &[("oneway", "-1")]),
            way(20, &[3, 2], &[("oneway", "-1")]),
            relation(100, 10, 2, 20, &[("restriction", "no_straight_on")]),
        ],
    };
    let (map, _) = import(doc.clone(), options()).unwrap();
    let rule = &map.turn_restrictions[0];
    assert_eq!(rule.from_edge, "osm-way-10-0-reverse");
    assert_eq!(rule.to_edge, "osm-way-20-0-reverse");
    assert!(
        RoadNetwork::new(map)
            .unwrap()
            .shortest_route("osm-node-1", "osm-node-3", &[])
            .is_err()
    );
    for way_id in [10, 20] {
        let mut wrong = doc.clone();
        for element in &mut wrong.elements {
            match element {
                OverpassElement::Way { id, tags, .. } if *id == way_id => {
                    tags.insert("oneway".into(), "yes".into());
                }
                _ => {}
            }
        }
        assert!(
            import(wrong, options())
                .unwrap_err()
                .contains("directed from/to")
        );
    }
}
#[test]
fn same_way_u_turn_splits_explicit_via_without_blocking_through_traffic() {
    let doc = OverpassDocument {
        elements: vec![
            node(1, 0., 0.),
            node(2, 1., 0.),
            node(3, 2., 0.),
            way(10, &[1, 2, 3], &[]),
            relation(100, 10, 2, 10, &[("restriction", "no_u_turn")]),
        ],
    };
    let (map, report) = import(doc.clone(), options()).unwrap();
    assert_eq!(map.nodes.len(), 3);
    assert_eq!(map.edges.len(), 4);
    assert_eq!(report.imported_turn_restrictions, 2);
    assert_eq!(
        map.turn_restrictions
            .iter()
            .map(|r| (r.from_edge.as_str(), r.to_edge.as_str()))
            .collect::<Vec<_>>(),
        [
            ("osm-way-10-0-forward", "osm-way-10-0-reverse"),
            ("osm-way-10-1-reverse", "osm-way-10-1-forward")
        ]
    );
    let graph = RoadNetwork::new(map).unwrap();
    assert_eq!(
        graph
            .shortest_route("osm-node-1", "osm-node-3", &[])
            .unwrap()
            .edge_ids,
        ["osm-way-10-0-forward", "osm-way-10-1-forward"]
    );
    assert_eq!(
        graph
            .shortest_route("osm-node-3", "osm-node-1", &[])
            .unwrap()
            .edge_ids,
        ["osm-way-10-1-reverse", "osm-way-10-0-reverse"]
    );
    let mut one_way = doc;
    if let OverpassElement::Way { tags, .. } = &mut one_way.elements[3] {
        tags.insert("oneway".into(), "yes".into());
    }
    assert!(
        import(one_way, options())
            .unwrap_err()
            .contains("reverse edge")
    );
}
#[test]
fn ambiguous_interior_way_roles_and_unsupported_same_way_rules_fail() {
    let mut doc = topology();
    doc.elements
        .push(relation(100, 10, 2, 20, &[("restriction", "no_left_turn")]));
    assert!(import(doc, options()).unwrap_err().contains("ambiguous"));
    let mut doc = restricted_junction("no_left_turn");
    if let OverpassElement::Way { nodes, tags, .. } = &mut doc.elements[5] {
        *nodes = vec![3, 2, 4];
        tags.remove("oneway");
    }
    assert!(import(doc, options()).unwrap_err().contains("ambiguous"));
    for value in ["no_straight_on", "only_left_turn"] {
        let doc = OverpassDocument {
            elements: vec![
                node(1, 0., 0.),
                node(2, 1., 0.),
                way(10, &[1, 2], &[]),
                relation(100, 10, 2, 10, &[("restriction", value)]),
            ],
        };
        assert!(import(doc, options()).unwrap_err().contains("same-way"));
    }
}
#[test]
fn applicable_missing_members_and_dangling_connectivity_are_never_ignored() {
    for reference in [0, 99] {
        for member_index in 0..3 {
            let mut doc = restricted_junction("no_left_turn");
            if let OverpassElement::Relation { members, .. } = doc.elements.last_mut().unwrap() {
                members[member_index].reference = reference;
            }
            assert!(import(doc, options()).is_err());
        }
    }
    for mutate in 0..5 {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { members, .. } = doc.elements.last_mut().unwrap() {
            match mutate {
                0 => {
                    members.pop();
                }
                1 => {
                    members.push(members[0].clone());
                }
                2 => {
                    members[1].member_type = "way".into();
                }
                3 => {
                    members[1].role = "to".into();
                }
                _ => {
                    members[1].role = "unknown".into();
                }
            }
        }
        assert!(import(doc, options()).is_err());
    }
    let mut doc = restricted_junction("no_left_turn");
    if let OverpassElement::Relation { members, .. } = doc.elements.last_mut().unwrap() {
        members[1].reference = 4;
    }
    assert!(
        import(doc, options())
            .unwrap_err()
            .contains("does not belong")
    );
    let mut doc = restricted_junction("no_left_turn");
    doc.elements.push(doc.elements.last().unwrap().clone());
    assert!(import(doc, options()).unwrap_err().contains("duplicate"));
}
#[test]
fn unsupported_applicable_conditions_modes_exceptions_and_values_are_rejected() {
    for extra in [
        ("restriction:conditional", "no_left_turn @ (Mo-Fr)"),
        ("restriction:motorcar:conditional", "no_left_turn @ (wet)"),
        (
            "restriction:motor_vehicle:conditional",
            "no_left_turn @ (sunset-sunrise)",
        ),
        (
            "restriction:vehicle:conditional",
            "no_left_turn @ (weight>3.5)",
        ),
        ("except", "bus"),
        ("except", ""),
        ("except", "bicycle;"),
        ("restriction", "no_entry"),
        ("restriction", "only_u_turn"),
    ] {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { tags, .. } = doc.elements.last_mut().unwrap() {
            tags.insert(extra.0.into(), extra.1.into());
        }
        assert!(import(doc, options()).is_err(), "{extra:?}");
    }
    for extra in [&[][..], &["restriction:unknown", "restriction:bicycle"][..]] {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { tags, .. } = doc.elements.last_mut().unwrap() {
            tags.remove("restriction");
            for key in extra {
                tags.insert((*key).into(), "no_left_turn".into());
            }
        }
        assert!(import(doc, options()).is_err());
    }
}
#[test]
fn specific_motorcar_precedence_and_explicit_nonmotor_exemptions_are_bounded() {
    for (specific, value) in [
        ("restriction:motorcar", "only_right_turn"),
        ("restriction:motor_vehicle", "only_right_turn"),
        ("restriction:vehicle", "only_right_turn"),
    ] {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { tags, .. } = doc.elements.last_mut().unwrap() {
            tags.insert(specific.into(), value.into());
            tags.insert(
                "restriction:conditional".into(),
                "no_left_turn @ (Mo-Fr)".into(),
            );
        }
        let (map, _) = import(doc, options()).unwrap();
        assert_eq!(map.turn_restrictions[0].kind, TurnRestrictionKind::Only);
    }
    for mode in ["bicycle", "foot", "bus", "hgv"] {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { tags, members, .. } = doc.elements.last_mut().unwrap() {
            tags.remove("restriction");
            tags.insert(format!("restriction:{mode}"), "no_left_turn".into());
            members[0].reference = 99; // Unretained nonmotor topology need not exist.
        }
        let (map, report) = import(doc, options()).unwrap();
        assert!(map.turn_restrictions.is_empty());
        assert_eq!(report.ignored_turn_relations, 1);
    }
    for mode in ["motorcar", "motor_vehicle", "vehicle", "bicycle; motorcar"] {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { tags, .. } = doc.elements.last_mut().unwrap() {
            tags.insert("except".into(), mode.into());
        }
        assert!(
            import(doc, options())
                .unwrap()
                .0
                .turn_restrictions
                .is_empty()
        );
    }
    let mut doc = restricted_junction("no_left_turn");
    if let OverpassElement::Relation { tags, .. } = doc.elements.last_mut().unwrap() {
        tags.insert("except".into(), "bicycle; foot".into());
    }
    assert_eq!(import(doc, options()).unwrap().0.turn_restrictions.len(), 1);
}
#[test]
fn filtered_roads_cannot_silently_remove_an_only_turn_rule() {
    for (value, filtered_way, succeeds) in [
        ("no_left_turn", 10, true),
        ("no_left_turn", 20, true),
        ("only_left_turn", 10, true),
        ("only_left_turn", 20, false),
    ] {
        let mut doc = restricted_junction(value);
        for element in &mut doc.elements {
            match element {
                OverpassElement::Way { id, tags, .. } if *id == filtered_way => {
                    tags.insert("highway".into(), "footway".into());
                }
                _ => {}
            }
        }
        match import(doc, options()) {
            Ok((map, report)) => {
                assert!(succeeds);
                assert!(map.turn_restrictions.is_empty());
                assert_eq!(report.ignored_turn_relations, 1);
            }
            Err(message) => {
                assert!(!succeeds);
                assert!(message.contains("filtered road"));
            }
        }
    }
}
#[test]
fn restriction_order_does_not_affect_graph_report_or_plan() {
    let mut doc = restricted_junction("no_left_turn");
    doc.elements.push(relation(
        101,
        30,
        4,
        40,
        &[("restriction", "only_straight_on")],
    ));
    let mut reversed = doc.clone();
    reversed.elements.reverse();
    for element in &mut reversed.elements {
        if let OverpassElement::Relation { members, .. } = element {
            members.reverse();
        }
    }
    let (a, ar) = import(doc, options()).unwrap();
    let (b, br) = import(reversed, options()).unwrap();
    assert_eq!(format!("{a:?}"), format!("{b:?}"));
    assert_eq!(format!("{ar:?}"), format!("{br:?}"));
    assert_eq!(
        RoadNetwork::new(a)
            .unwrap()
            .shortest_route("osm-node-1", "osm-node-3", &[])
            .unwrap()
            .edge_ids,
        RoadNetwork::new(b)
            .unwrap()
            .shortest_route("osm-node-1", "osm-node-3", &[])
            .unwrap()
            .edge_ids
    );
    let (_, legacy_report) = import(topology(), options()).unwrap();
    assert_eq!(legacy_report.input_turn_relations, 0);
    assert_eq!(legacy_report.ignored_turn_relations, 0);
    assert_eq!(legacy_report.imported_turn_restrictions, 0);
}

#[test]
fn malformed_relation_types_unknown_qualifiers_and_conflicting_imports_fail() {
    for kind in [None, Some("route"), Some("restriction:motorcar")] {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { tags, .. } = doc.elements.last_mut().unwrap() {
            tags.remove("type");
            if let Some(kind) = kind {
                tags.insert("type".into(), kind.into());
            }
        }
        assert!(
            import(doc, options())
                .unwrap_err()
                .contains("restriction type")
        );
    }
    for key in [
        "restriction:motorcar:forward",
        "restriction:unknown",
        "restriction:conditional:motorcar",
    ] {
        let mut doc = restricted_junction("no_left_turn");
        if let OverpassElement::Relation { tags, .. } = doc.elements.last_mut().unwrap() {
            tags.insert(key.into(), "no_left_turn".into());
        }
        assert!(import(doc, options()).unwrap_err().contains("qualifier"));
    }
    for (base, to, value) in [
        ("no_left_turn", 20, "only_right_turn"),
        ("only_left_turn", 20, "only_right_turn"),
        ("only_left_turn", 30, "only_right_turn"),
    ] {
        let mut doc = restricted_junction(base);
        doc.elements
            .push(relation(101, 10, 2, to, &[("restriction", value)]));
        assert!(
            import(doc, options())
                .unwrap_err()
                .contains("turn restriction")
        );
    }
}
#[test]
fn shared_via_u_turn_does_not_prohibit_a_different_way_exit() {
    let mut doc = topology();
    doc.elements
        .push(relation(100, 10, 2, 10, &[("restriction", "no_u_turn")]));
    let (map, _) = import(doc, options()).unwrap();
    assert_eq!(map.turn_restrictions.len(), 2);
    let graph = RoadNetwork::new(map).unwrap();
    let plan = graph
        .shortest_route("osm-node-1", "osm-node-4", &["osm-way-30-0-forward".into()])
        .unwrap();
    assert_eq!(
        plan.edge_ids,
        ["osm-way-10-0-forward", "osm-way-20-0-forward"]
    );
}
#[test]
fn input_relation_limit_applies_even_to_explicitly_nonmotor_restrictions() {
    let mut doc = restricted_junction("no_left_turn");
    doc.elements.pop();
    for id in 1..=10_001 {
        doc.elements.push(relation(
            id,
            99,
            98,
            97,
            &[("restriction:bicycle", "no_left_turn")],
        ));
    }
    assert!(
        import(doc, options())
            .unwrap_err()
            .contains("10000 restriction relations")
    );
}

#[test]
fn member_reference_accepts_legacy_decimal_strings_but_not_lossy_or_invalid_ids() {
    use serde::de::value::{
        Error, F64Deserializer, I64Deserializer, StrDeserializer, U64Deserializer,
    };
    for value in ["247778550", "1", "18446744073709551615"] {
        assert_eq!(
            deserialize_member_ref(StrDeserializer::<Error>::new(value)).unwrap(),
            value.parse::<u64>().unwrap()
        );
    }
    assert_eq!(
        deserialize_member_ref(U64Deserializer::<Error>::new(247778550)).unwrap(),
        247778550
    );
    for value in [
        "",
        "0",
        "+12",
        "-1",
        "1.0",
        " 12",
        "12 ",
        "abc",
        "18446744073709551616",
    ] {
        assert!(
            deserialize_member_ref(StrDeserializer::<Error>::new(value)).is_err(),
            "{value:?}"
        );
    }
    assert!(deserialize_member_ref(U64Deserializer::<Error>::new(0)).is_err());
    assert!(deserialize_member_ref(I64Deserializer::<Error>::new(-1)).is_err());
    assert!(deserialize_member_ref(F64Deserializer::<Error>::new(12.0)).is_err());
}
