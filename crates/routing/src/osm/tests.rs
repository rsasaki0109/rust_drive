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
