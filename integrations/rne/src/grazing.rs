//! A pinned native query regression for the explicitly disclosed grazing case.
//! This empirical example is not a global numerical error bound for Rapier.
use rne_ecs::{World, spawn_named};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RaycastQuery, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use serde_json::Value;

fn triple(value: &Value) -> [f64; 3] {
    std::array::from_fn(|index| value[index].as_f64().unwrap())
}

#[test]
fn pinned_native_capsule_grazing_is_bounded_and_disclosed() {
    let fixture: Value = serde_json::from_str(include_str!(
        "../../../scripts/fixtures/native-capsule-grazing-proof.json"
    ))
    .unwrap();
    let center = triple(&fixture["capsule"]["center_m"]);
    let origin = triple(&fixture["origin_m"]);
    let direction = triple(&fixture["direction_enu"]);
    let radius = fixture["capsule"]["radius_m"].as_f64().unwrap();
    let bottom = fixture["capsule"]["axis_bottom_m"].as_f64().unwrap();
    let top = fixture["capsule"]["axis_top_m"].as_f64().unwrap();

    // An independent f64 cylinder calculation proves this ray truly misses.
    // Its closest point lies between the capsule's end-cap centers.
    let t_closest = -((origin[0] - center[0]) * direction[0]
        + (origin[1] - center[1]) * direction[1])
        / (direction[0].powi(2) + direction[1].powi(2));
    let closest_up = origin[2] + t_closest * direction[2];
    assert!((bottom..=top).contains(&closest_up));
    let analytic_gap = (origin[0] + t_closest * direction[0] - center[0])
        .hypot(origin[1] + t_closest * direction[1] - center[1])
        - radius;
    assert!(analytic_gap > 0.0 && analytic_gap < 0.0001);

    let mut ecs = World::new();
    let actor = spawn_named(&mut ecs, "isolated-grazing-capsule");
    ecs.entity_mut(actor).insert((
        Transform3::from_translation_rotation(
            Vec3::new(center[0], center[2], -center[1]),
            Quat::IDENTITY,
        ),
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Capsule {
                half_height_m: (top - bottom) / 2.0,
                radius_m: radius,
            },
            ..Collider::default()
        },
    ));
    let mut backend = RapierBackend::new();
    let world = backend.create_world(PhysicsWorldDesc::default()).unwrap();
    backend.sync_from_ecs(&mut ecs, world).unwrap();
    let query = |shift: f64| RaycastQuery {
        // ENU perpendicular shift (-north direction, east direction, 0).
        origin_m: Vec3::new(
            origin[0] - shift * direction[1],
            origin[2],
            -origin[1] - shift * direction[0],
        ),
        direction: Vec3::new(direction[0], direction[2], -direction[1]),
        max_distance_m: 45.0,
    };
    let hits = backend.raycast(world, query(0.0)).unwrap();
    assert_eq!(hits.len(), 1, "pinned GJK grazing query must reproduce");
    let toi = hits[0].distance_m;
    assert!((toi - t_closest).abs() < 0.02);
    let hit_up = origin[2] + toi * direction[2];
    let vertical_gap = (bottom - hit_up).max(hit_up - top).max(0.0);
    let surface_gap = (origin[0] + toi * direction[0] - center[0])
        .hypot(origin[1] + toi * direction[1] - center[1])
        .hypot(vertical_gap)
        - radius;
    assert!(surface_gap > 0.0 && surface_gap < 0.0001);
    for shift in [0.00005, 0.0001, 0.001] {
        assert!(backend.raycast(world, query(shift)).unwrap().is_empty());
    }
    assert_eq!(backend.raycast(world, query(-0.001)).unwrap().len(), 1);
    // This queries native geometry directly: no sensor noise, new tolerance,
    // traffic update, or operational sensing behavior is introduced.
}
