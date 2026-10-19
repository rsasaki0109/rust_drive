use super::*;
use crate::{RoadEdge, RoadNetworkSpec, RoadNode};
use rustdriving_core::Vec2;

fn chain(caps: &[Option<f64>]) -> RoadNetworkSpec {
    let nodes: Vec<_> = (0..=caps.len())
        .map(|i| RoadNode {
            id: i.to_string(),
            position: Vec2::new(i as f64, 0.),
        })
        .collect();
    let edges = caps
        .iter()
        .enumerate()
        .map(|(i, cap)| RoadEdge {
            id: format!("edge-{i}"),
            from: nodes[i].id.clone(),
            to: nodes[i + 1].id.clone(),
            points: vec![nodes[i].position, nodes[i + 1].position],
            half_width: 3.,
            speed_limit_m_s: *cap,
        })
        .collect();
    RoadNetworkSpec {
        nodes,
        edges,
        turn_restrictions: vec![],
    }
}

#[test]
fn whole_edge_caps_preserve_uncapped_gaps_and_merge_only_adjacent_equal_limits() {
    let spec = chain(&[Some(2.), None, Some(2.), Some(2.), Some(20.)]);
    let network = RoadNetwork::new(spec).unwrap();
    let p = network
        .shortest_route_with_speed_zones("0", "5", &[])
        .unwrap();
    assert_eq!(p.speed_zones.len(), 3);
    assert_eq!(p.speed_zones[0].end_s_m, 1.);
    assert_eq!(p.speed_zones[1].start_s_m, 2.);
    assert_eq!(p.speed_zones[1].end_s_m, 4.);
    assert_eq!(p.speed_zones[2].speed_m_s, 8.);
    assert_eq!(
        p.plan.edge_ids,
        network.shortest_route("0", "5", &[]).unwrap().edge_ids
    );
}

#[test]
fn canonical_endpoint_roundoff_and_a_centimeter_edge_keep_exact_route_indices() {
    let mut spec = chain(&[Some(4.), Some(1.5), Some(4.)]);
    spec.nodes[2].position.x = 1.01;
    spec.edges[1].points[1].x = 1.0100001;
    spec.edges[2].points[0].x = 1.0099999;
    let network = RoadNetwork::new(spec).unwrap();
    let p = network
        .shortest_route_with_speed_zones("0", "3", &[])
        .unwrap();
    assert_eq!(p.speed_zones[1].start_s_m, p.plan.route.lengths[1]);
    assert_eq!(p.speed_zones[1].end_s_m, p.plan.route.lengths[2]);
    assert!((p.speed_zones[1].end_s_m - p.speed_zones[1].start_s_m - 0.01).abs() < 1e-12);
}

#[test]
fn malformed_limits_and_an_excessive_selected_profile_fail_explicitly() {
    for cap in [0., 20.01, f64::NAN, f64::INFINITY] {
        assert!(RoadNetwork::new(chain(&[Some(cap)])).is_err());
    }
    let caps: Vec<_> = (0..129).map(|i| Some(1. + (i % 2) as f64)).collect();
    let network = RoadNetwork::new(chain(&caps)).unwrap();
    assert!(network.shortest_route("0", "129", &[]).is_ok());
    assert!(
        network
            .shortest_route_with_speed_zones("0", "129", &[])
            .unwrap_err()
            .contains("128")
    );
}
