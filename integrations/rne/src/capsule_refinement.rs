//! Optional physical capsule recovery alongside actual native Rapier queries.
//! Geometry is the synchronized physical ECS collider, never perception labels.
//! Native ground and other shapes stay native; capsules use synchronized ECS
//! physical geometry to correct both GJK false positives and false negatives.
use rne_ecs::{Entity, Parent, World};
use rne_math::Vec3;
use rne_physics::{
    Collider, ColliderShape, CompoundCollider, ConvexCollider, PhysicsError, RaycastHit,
    RaycastQuery, RigidBody,
};
use rne_world::Transform3;

const MAX_PRECISE_CAPSULES: usize = 1024;
pub(crate) struct PhysicalCapsule {
    entity: Entity,
    pose: Transform3,
    half_height_m: f64,
    radius_m: f64,
}
/// Snapshot actual collider components after native scene synchronization. No
/// scenario identities, actor labels or evaluator geometry are accepted here.
pub(crate) fn snapshot(ecs: &World, ego: Entity) -> Result<Vec<PhysicalCapsule>, PhysicsError> {
    let mut capsules = Vec::new();
    for entity in ecs.iter_entities() {
        if entity.id() == ego {
            continue;
        }
        if entity.get::<RigidBody>().is_none() {
            continue;
        }
        let Some(collider) = entity.get::<Collider>() else {
            continue;
        };
        let ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } = collider.shape
        else {
            continue;
        };
        if entity.get::<Parent>().is_some()
            || entity.get::<CompoundCollider>().is_some()
            || entity.get::<ConvexCollider>().is_some()
        {
            return Err(PhysicsError::InvalidColliderShape {
                reason: "precise capsule recovery supports flat primitive physical colliders only",
            });
        }
        let transform = entity
            .get::<Transform3>()
            .ok_or(PhysicsError::InvalidColliderShape {
                reason: "physical capsule has no rigid pose",
            })?;
        let pose = transform.mul_transform(&collider.local_offset);
        validate_pose(pose, half_height_m, radius_m)?;
        if capsules.len() == MAX_PRECISE_CAPSULES {
            return Err(PhysicsError::InvalidColliderShape {
                reason: "precise physical capsule snapshot exceeds 1024-collider bound",
            });
        }
        capsules.push(PhysicalCapsule {
            entity: entity.id(),
            pose,
            half_height_m,
            radius_m,
        });
    }
    capsules.sort_by_key(|capsule| capsule.entity.index());
    Ok(capsules)
}
fn validate_pose(pose: Transform3, half_height_m: f64, radius_m: f64) -> Result<(), PhysicsError> {
    if pose.scale != Vec3::ONE
        || !pose.translation.is_finite()
        || !pose.rotation.is_finite()
        || (pose.rotation.length_squared() - 1.0).abs() > 1e-10
        || !half_height_m.is_finite()
        || !(0.0..=1e6).contains(&half_height_m)
        || !radius_m.is_finite()
        || radius_m <= 0.0
        || radius_m > 1e6
    {
        return Err(PhysicsError::InvalidColliderShape {
            reason: "precise capsule queries require finite bounded rigid unit-scale physical geometry",
        });
    }
    Ok(())
}
/// Replace native capsule records with exact physical capsule records, recover
/// missed physical capsules, and rerank with unchanged native non-capsule hits.
pub(crate) fn merge_hits(
    capsules: &[PhysicalCapsule],
    query: RaycastQuery,
    mut native: Vec<RaycastHit>,
) -> Result<Vec<RaycastHit>, PhysicsError> {
    native.retain(|hit| !capsules.iter().any(|capsule| capsule.entity == hit.entity));
    for capsule in capsules {
        if let Some(hit) = cast_capsule(capsule, query)? {
            native.push(hit);
        }
    }
    native.sort_by(|a, b| {
        a.distance_m
            .total_cmp(&b.distance_m)
            .then(a.entity.index().cmp(&b.entity.index()))
    });
    Ok(native)
}

fn capsule_distance(
    origin: Vec3,
    direction: Vec3,
    half_height: f64,
    radius: f64,
    max_distance: f64,
) -> Option<f64> {
    let axis = Vec3::new(0.0, origin.y.clamp(-half_height, half_height), 0.0);
    if origin.distance_squared(axis) <= radius * radius {
        return Some(0.0);
    }
    let mut best = f64::INFINITY;
    let mut accept = |t: f64| {
        if t.is_finite() && t >= 0.0 && t <= max_distance {
            best = best.min(t);
        }
    };
    // Finite cylinder: closest approach avoids cancellation in b² - a*c.
    let a = direction.x * direction.x + direction.z * direction.z;
    if a > 0.0 {
        let closest = -(origin.x * direction.x + origin.z * direction.z) / a;
        let point = origin + direction * closest;
        let residual = radius * radius - (point.x * point.x + point.z * point.z);
        if residual >= 0.0 {
            let offset = (residual / a).sqrt();
            for t in [closest - offset, closest + offset] {
                let y = origin.y + t * direction.y;
                if (-half_height..=half_height).contains(&y) {
                    accept(t);
                }
            }
        }
    }
    // Two hemispheres; reject the hidden halves within the cylinder, including
    // their later intersections, instead of approximating by full spheres.
    for sign in [-1.0, 1.0] {
        let center = Vec3::new(0.0, sign * half_height, 0.0);
        let relative = origin - center;
        let closest = -relative.dot(direction);
        let point = relative + direction * closest;
        let residual = radius * radius - point.length_squared();
        if residual >= 0.0 {
            let offset = residual.sqrt();
            for t in [closest - offset, closest + offset] {
                if sign * (origin.y + t * direction.y) >= half_height {
                    accept(t);
                }
            }
        }
    }
    best.is_finite().then_some(best)
}

#[cfg(test)]
fn refine_hit(
    ecs: &World,
    query: RaycastQuery,
    hit: RaycastHit,
) -> Result<Option<RaycastHit>, PhysicsError> {
    let Some(collider) = ecs.get::<Collider>(hit.entity) else {
        return Ok(Some(hit));
    };
    let ColliderShape::Capsule {
        half_height_m,
        radius_m,
    } = collider.shape
    else {
        return Ok(Some(hit));
    };
    let invalid = || PhysicsError::InvalidColliderShape {
        reason: "precise capsule queries require finite rigid unit-scale physical geometry and ray",
    };
    let transform = ecs.get::<Transform3>(hit.entity).ok_or_else(invalid)?;
    let pose = transform.mul_transform(&collider.local_offset);
    validate_pose(pose, half_height_m, radius_m)?;
    cast_capsule(
        &PhysicalCapsule {
            entity: hit.entity,
            pose,
            half_height_m,
            radius_m,
        },
        query,
    )
}
fn cast_capsule(
    capsule: &PhysicalCapsule,
    query: RaycastQuery,
) -> Result<Option<RaycastHit>, PhysicsError> {
    let invalid = || PhysicsError::InvalidColliderShape {
        reason: "precise capsule queries require a finite nonzero ray",
    };
    if !query.origin_m.is_finite()
        || !query.direction.is_finite()
        || query.direction.length_squared() <= 0.0
        || !query.max_distance_m.is_finite()
        || query.max_distance_m < 0.0
    {
        return Err(invalid());
    }
    let PhysicalCapsule {
        entity,
        pose,
        half_height_m,
        radius_m,
    } = capsule;
    let direction = query.direction.normalize();
    let inverse = pose.rotation.conjugate();
    let local_origin = inverse * (query.origin_m - pose.translation);
    let local_direction = inverse * direction;
    // Conservative spherical broad phase around the physical segment. It
    // encloses the entire capsule; it never substitutes for the narrow shape.
    let nearest = (-local_origin.dot(local_direction)).clamp(0.0, query.max_distance_m);
    let bound = *half_height_m + *radius_m;
    if (local_origin + nearest * local_direction).length_squared()
        > bound * bound * (1.0 + 16.0 * f64::EPSILON)
    {
        return Ok(None);
    }
    let Some(distance) = capsule_distance(
        local_origin,
        local_direction,
        *half_height_m,
        *radius_m,
        query.max_distance_m,
    ) else {
        return Ok(None);
    };
    let local_point = local_origin + local_direction * distance;
    let axis = Vec3::new(
        0.0,
        local_point.y.clamp(-half_height_m, *half_height_m),
        0.0,
    );
    let normal = if distance == 0.0 {
        -direction
    } else {
        pose.rotation * (local_point - axis).normalize()
    };
    Ok(Some(RaycastHit {
        entity: *entity,
        distance_m: distance,
        point_m: query.origin_m + direction * distance,
        normal,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::spawn_named;
    use rne_math::Quat;
    use rne_physics::{PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType};
    use rne_physics_rapier::RapierBackend;
    fn isolated(
        center_enu: Vec3,
        radius: f64,
    ) -> (World, RapierBackend, rne_physics::PhysicsWorldId) {
        let mut world = World::new();
        let entity = spawn_named(&mut world, "physical-capsule");
        world.entity_mut(entity).insert((
            Transform3::from_translation_rotation(
                Vec3::new(center_enu.x, center_enu.z, -center_enu.y),
                Quat::IDENTITY,
            ),
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Capsule {
                    half_height_m: 0.5,
                    radius_m: radius,
                },
                ..Collider::default()
            },
        ));
        let mut backend = RapierBackend::new();
        let id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        backend.sync_from_ecs(&mut world, id).unwrap();
        (world, backend, id)
    }
    fn query(origin: Vec3, direction: Vec3) -> RaycastQuery {
        RaycastQuery {
            origin_m: Vec3::new(origin.x, origin.z, -origin.y),
            direction: Vec3::new(direction.x, direction.z, -direction.y),
            max_distance_m: 45.0,
        }
    }
    #[test]
    fn native_far_lower_cap_false_positive_is_removed_without_expanding_shape() {
        let (ecs, backend, id) = isolated(Vec3::new(18.0, 30.16, 0.6), 0.4);
        let q = query(
            Vec3::new(43.9148531811241, -0.0617626863502608, 0.6),
            Vec3::new(
                -0.6575075361208902,
                0.7532458121050419,
                -0.017452406437283508,
            ),
        );
        let native = backend.raycast(id, q).unwrap();
        assert_eq!(native.len(), 1, "reproduce native GJK cap false positive");
        assert!(refine_hit(&ecs, q, native[0]).unwrap().is_none());
    }
    #[test]
    fn native_upper_cap_millimetre_false_positive_is_removed() {
        let (ecs, backend, id) = isolated(Vec3::new(44.0, -4.23, 0.6), 0.4);
        let q = query(
            Vec3::new(36.20432913755455, 0.02921172486818317, 0.6),
            Vec3::new(0.889357316347061, -0.4488289656050037, 0.08715574274765812),
        );
        let native = backend.raycast(id, q).unwrap();
        assert_eq!(native.len(), 1, "reproduce native upper-cap false positive");
        assert!(refine_hit(&ecs, q, native[0]).unwrap().is_none());
    }
    #[test]
    fn missing_owner_city_native_return_is_recovered_from_physical_ecs_geometry() {
        use rne_sensor::LidarRaycaster;
        let (mut ecs, backend, id) = isolated(Vec3::new(28.92, -14.0, 0.6), 0.4);
        let q = query(
            Vec3::new(22.1602983322294, -0.01644450259774075, 0.6),
            Vec3::new(
                0.4175082243535855,
                -0.9085055289356604,
                -0.017452406437283508,
            ),
        );
        let native = backend.raycast(id, q).unwrap();
        assert!(
            native.is_empty(),
            "reproduce observed pinned native false negative"
        );
        let ego = rne_ecs::spawn_named(&mut ecs, "excluded-ego");
        let capsules = snapshot(&ecs, ego).unwrap();
        let hits = crate::EgoFilteredRaycaster {
            backend: &backend,
            ego,
            precise_geometry: Some(&capsules),
        }
        .lidar_raycast(id, q)
        .unwrap();
        assert_eq!(hits.len(), 1);
        assert!((hits[0].distance_m - 15.269952013263778).abs() < 1e-10);
        let center = Vec3::new(28.92, 0.6, 14.0);
        let point = hits[0].point_m - center;
        assert!((point.x.hypot(point.z) - 0.4).abs() < 1e-12);
        assert!((-0.5..=0.5).contains(&point.y));
    }
    #[test]
    fn exact_cylinder_hemisphere_inside_and_maximum_distance_cases() {
        assert_eq!(
            capsule_distance(Vec3::new(-2.0, 0.0, 0.0), Vec3::X, 0.5, 0.4, 4.0),
            Some(1.6)
        );
        assert_eq!(
            capsule_distance(Vec3::new(0.0, 2.0, 0.0), Vec3::NEG_Y, 0.5, 0.4, 4.0),
            Some(1.1)
        );
        assert_eq!(
            capsule_distance(Vec3::ZERO, Vec3::X, 0.5, 0.4, 4.0),
            Some(0.0)
        );
        assert_eq!(
            capsule_distance(Vec3::new(-2.0, 0.0, 0.0), Vec3::X, 0.5, 0.4, 1.5),
            None
        );
        assert_eq!(
            capsule_distance(Vec3::new(-2.0, 0.0, 0.4000001), Vec3::X, 0.5, 0.4, 4.0),
            None
        );
    }
    #[test]
    fn rigid_rotation_and_collider_offset_return_surface_point_and_normal() {
        let mut world = World::new();
        let entity = spawn_named(&mut world, "rotated-offset-capsule");
        let rotation = Quat::from_rotation_z(std::f64::consts::FRAC_PI_2);
        world.entity_mut(entity).insert((
            Transform3::from_translation_rotation(Vec3::new(3.0, 2.0, 1.0), rotation),
            Collider {
                shape: ColliderShape::Capsule {
                    half_height_m: 0.5,
                    radius_m: 0.4,
                },
                local_offset: Transform3::from_translation_rotation(
                    Vec3::new(0.0, 0.2, 0.0),
                    Quat::IDENTITY,
                ),
                ..Collider::default()
            },
        ));
        let pose = world
            .get::<Transform3>(entity)
            .unwrap()
            .mul_transform(&world.get::<Collider>(entity).unwrap().local_offset);
        let q = RaycastQuery {
            origin_m: pose.translation + pose.rotation * Vec3::new(-2.0, 0.0, 0.0),
            direction: rotation * Vec3::X,
            max_distance_m: 4.0,
        };
        let fake = RaycastHit {
            entity,
            point_m: Vec3::ZERO,
            normal: Vec3::ZERO,
            distance_m: 1.61,
        };
        let hit = refine_hit(&world, q, fake).unwrap().unwrap();
        assert!((hit.distance_m - 1.6).abs() < 1e-12);
        assert!(
            hit.point_m
                .distance(pose.translation + pose.rotation * Vec3::new(-0.4, 0.0, 0.0))
                < 1e-12
        );
        assert!(hit.normal.distance(rotation * Vec3::NEG_X) < 1e-12);
    }
    #[test]
    fn removed_native_capsule_candidate_exposes_farther_native_ground() {
        use rne_sensor::LidarRaycaster;
        let (mut ecs, mut backend, id) = isolated(Vec3::new(30.851637857146244, 0.0, 0.6), 1.0);
        let ground = spawn_named(&mut ecs, "physical-ground");
        ecs.entity_mut(ground).insert((
            Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(100.0, 0.5, 100.0),
                },
                ..Collider::default()
            },
        ));
        let ego = spawn_named(&mut ecs, "excluded-ego");
        backend.sync_from_ecs(&mut ecs, id).unwrap();
        let q = query(
            Vec3::new(21.206407390617414, 0.036761813449073345, 0.6),
            Vec3::new(
                0.9948469160371359,
                0.09987505775191116,
                -0.017452406437283508,
            ),
        );
        let native = backend.raycast(id, q).unwrap();
        assert_eq!(native.len(), 2);
        assert_ne!(native[0].entity, ground);
        assert_eq!(native[1].entity, ground);
        let legacy = crate::EgoFilteredRaycaster {
            backend: &backend,
            ego,
            precise_geometry: None,
        }
        .lidar_raycast(id, q)
        .unwrap();
        assert_eq!(legacy, native);
        let capsules = snapshot(&ecs, ego).unwrap();
        let refined = crate::EgoFilteredRaycaster {
            backend: &backend,
            ego,
            precise_geometry: Some(&capsules),
        }
        .lidar_raycast(id, q)
        .unwrap();
        assert_eq!(
            refined,
            vec![native[1]],
            "farther ground remains an actual unchanged Rapier return"
        );
    }
    #[test]
    fn unsupported_scaled_capsule_is_an_acquisition_error() {
        let (mut ecs, backend, id) = isolated(Vec3::new(0.0, 0.0, 0.6), 0.4);
        let q = query(Vec3::new(-2.0, 0.0, 0.6), Vec3::X);
        let hit = backend.raycast(id, q).unwrap()[0];
        ecs.get_mut::<Transform3>(hit.entity).unwrap().scale = Vec3::splat(2.0);
        assert!(refine_hit(&ecs, q, hit).is_err());
    }
    #[test]
    fn snapshot_excludes_ego_and_nonphysical_components_and_recovery_has_no_duplicates() {
        let (mut ecs, backend, id) = isolated(Vec3::new(0.0, 0.0, 0.6), 0.4);
        let ghost = spawn_named(&mut ecs, "not-a-native-rigid-body");
        ecs.entity_mut(ghost).insert((
            Transform3::IDENTITY,
            Collider {
                shape: ColliderShape::Capsule {
                    half_height_m: 0.5,
                    radius_m: 0.4,
                },
                ..Collider::default()
            },
        ));
        let ego = spawn_named(&mut ecs, "excluded-ego");
        ecs.entity_mut(ego).insert((
            Transform3::IDENTITY,
            RigidBody::default(),
            Collider {
                shape: ColliderShape::Capsule {
                    half_height_m: 0.5,
                    radius_m: 0.4,
                },
                ..Collider::default()
            },
        ));
        let capsules = snapshot(&ecs, ego).unwrap();
        assert_eq!(capsules.len(), 1);
        let q = query(Vec3::new(-2.0, 0.0, 0.6), Vec3::X);
        let native = backend.raycast(id, q).unwrap();
        assert_eq!(native.len(), 1);
        let merged = merge_hits(&capsules, q, native).unwrap();
        assert_eq!(merged.len(), 1);
        assert!((merged[0].distance_m - 1.6).abs() < 1e-12);
    }
    #[test]
    fn physical_capsule_snapshot_hard_bound_is_enforced() {
        let mut ecs = World::new();
        let ego = spawn_named(&mut ecs, "ego");
        for _ in 0..=MAX_PRECISE_CAPSULES {
            ecs.spawn((
                Transform3::IDENTITY,
                RigidBody::default(),
                Collider {
                    shape: ColliderShape::Capsule {
                        half_height_m: 0.5,
                        radius_m: 0.4,
                    },
                    ..Collider::default()
                },
            ));
        }
        assert!(snapshot(&ecs, ego).is_err());
    }
}
