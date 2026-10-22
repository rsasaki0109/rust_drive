use super::*;
use crate::lanes::{Lane, LaneMapSpec};
use rustdriving_core::Vec2;

fn chain(limits: &[f64], section_length: f64) -> LaneMapSpec {
    LaneMapSpec {
        schema_version: 1,
        lanes: limits
            .iter()
            .enumerate()
            .map(|(i, speed)| Lane {
                id: format!("lane-{i}"),
                centerline: vec![
                    Vec2::new(i as f64 * section_length, 0.),
                    Vec2::new((i + 1) as f64 * section_length, 0.),
                ],
                width_m: 4.2,
                speed_limit_m_s: *speed,
                successors: if i + 1 < limits.len() {
                    vec![format!("lane-{}", i + 1)]
                } else {
                    vec![]
                },
                left: None,
                right: None,
            })
            .collect(),
    }
}

#[test]
fn selected_route_lengths_define_contiguous_caps_and_shared_endpoint_release() {
    let mut map = chain(&[6., 2., 6.], 10.);
    map.lanes[0].centerline.insert(1, Vec2::new(3., 4.));
    map.lanes[1].centerline.insert(1, Vec2::new(10.004, 0.));
    let result = LaneNetwork::new(map)
        .unwrap()
        .following_route_with_speed_zones("lane-0", "lane-2", &[])
        .unwrap();
    let zones = result.speed_zones;
    assert_eq!(zones.len(), 3);
    assert_eq!(zones[0].start_s_m, 0.);
    assert!((zones[0].end_s_m - (5. + 65_f64.sqrt())).abs() < 1e-12);
    assert_eq!(zones[0].end_s_m, zones[1].start_s_m);
    assert_eq!(zones[1].end_s_m, zones[2].start_s_m);
    assert_eq!(zones[2].end_s_m, result.plan.route.length());
    assert_eq!(
        zones.iter().map(|z| z.speed_m_s).collect::<Vec<_>>(),
        [6., 2., 6.]
    );
    assert!(zones.iter().all(|z| z.valid()));
}

#[test]
fn equal_effective_limits_merge_and_high_map_limits_keep_the_consumer_ceiling() {
    let network = LaneNetwork::new(chain(&[20., 10., 2., 2., 8.], 10.)).unwrap();
    let result = network
        .following_route_with_speed_zones("lane-0", "lane-4", &[])
        .unwrap();
    assert_eq!(result.plan.speed_limit_m_s, 2.);
    assert_eq!(result.speed_zones.len(), 3);
    let spans: Vec<_> = result
        .speed_zones
        .iter()
        .map(|z| (z.start_s_m, z.end_s_m, z.speed_m_s))
        .collect();
    assert_eq!(spans, [(0., 20., 8.), (20., 40., 2.), (40., 50., 8.)]);
    let single = network
        .following_route_with_speed_zones("lane-2", "lane-2", &[])
        .unwrap();
    assert_eq!(single.speed_zones[0].start_s_m, 0.);
    assert_eq!(single.speed_zones[0].end_s_m, 10.);
}

#[test]
fn closure_selected_itinerary_and_rotated_geometry_drive_the_speed_policy() {
    let mut map = chain(&[6., 2., 6.], 10.);
    let mut alternative = map.lanes[1].clone();
    alternative.id = "alternate".into();
    alternative.speed_limit_m_s = 3.;
    map.lanes[0].successors.push("alternate".into());
    map.lanes.push(alternative);
    for lane in &mut map.lanes {
        for p in &mut lane.centerline {
            *p = Vec2::new(0., p.x);
        }
    }
    let network = LaneNetwork::new(map).unwrap();
    let result = network
        .following_route_with_speed_zones("lane-0", "lane-2", &["lane-1".into()])
        .unwrap();
    assert_eq!(result.plan.lane_ids, ["lane-0", "alternate", "lane-2"]);
    assert_eq!(result.speed_zones[1].speed_m_s, 3.);
    assert!(
        network
            .following_route_with_speed_zones(
                "lane-0",
                "lane-2",
                &["lane-1".into(), "alternate".into()]
            )
            .is_err()
    );
}

#[test]
fn fixed_route_length_and_interval_budgets_fail_instead_of_truncating_policy() {
    let limits: Vec<_> = (0..129).map(|i| 2. + (i % 2) as f64).collect();
    let network = LaneNetwork::new(chain(&limits, 1.)).unwrap();
    assert!(
        network
            .following_route_with_speed_zones("lane-0", "lane-127", &[])
            .is_ok()
    );
    assert!(
        network
            .following_route_with_speed_zones("lane-0", "lane-128", &[])
            .is_err()
    );
    let merged = LaneNetwork::new(chain(&[2.; 129], 1.)).unwrap();
    assert_eq!(
        merged
            .following_route_with_speed_zones("lane-0", "lane-128", &[])
            .unwrap()
            .speed_zones
            .len(),
        1
    );
    let long = LaneNetwork::new(chain(&[2.; 2], 800.)).unwrap();
    assert!(long.following_route("lane-0", "lane-1", &[]).is_ok());
    assert!(
        long.following_route_with_speed_zones("lane-0", "lane-1", &[])
            .is_err()
    );
}
