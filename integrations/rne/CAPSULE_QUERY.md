# Optional precise physical capsule queries

`--precise-capsule-rays` requires `--lidar-3d --ground-segmentation
--vehicle-body`. It changes only the opted-in sensor query wrapper. Unflagged
queries retain the original pinned RNE behavior and serialized outputs. The RNE
revision, physics integrator, collision witnesses and independent acceptance
tolerances are unchanged.

Pinned Rapier/Parry uses single-precision support-map/GJK ray intersections for
capsules. Regressions reproduce both near-tangent false positives and a genuine
missing return in the urban recording: at t=18.8 s, beam 1959 intersects the dog
walker's physical capsule at 15.269952 m, while the native query returns no hit.
That ray penetrates the cylinder by roughly 9.7 cm, so widening an oracle's
boundary tolerance cannot repair it.

The optional wrapper still executes native Rapier queries for ground and other
shapes. After native scene synchronization, it snapshots actual physical ECS
capsules and rigid collider offsets, excluding ego. It computes the finite
cylinder and exposed hemispheres in f64, replaces native capsule records with
these physical intersections, and sorts them with the unchanged non-capsule
returns by distance and entity index. This rejects false positives, recovers
missing capsule returns, and produces at most one return per capsule. No shape
expansion or evaluator tolerance is used.

The snapshot is bounded to 1,024 capsules per acquisition. It requires flat,
primitive, rigid unit-scale colliders; unsupported hierarchy, compound geometry,
invalid poses or capacity overflow cause an acquisition error. There is no
recovery for native misses on other shapes and no general engine precision or
safety claim.

Only synchronized physical query components enter this wrapper. Actor labels,
evaluation poses, schedules and independent oracle decisions never enter
perception or the driver. Evidence records `precise_capsule_rays=true`,
`recovers_native_false_negatives=true`, the snapshot bound and the complete
`capsule_query_refinement` contract. Sensor-only replay consumes the actual
measurements; it does not query physical geometry again.

Regressions reproduce the pinned native upper/lower-cap false positives and
the missing dog-walker return, then verify exact intersections, rigid offsets,
range limits, unsupported scale, ego exclusion, bounded snapshots, duplicate
removal and farther native-ground selection. The unflagged native grazing
regression retains the original backend limitation.
