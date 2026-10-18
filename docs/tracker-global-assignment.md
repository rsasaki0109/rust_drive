# Global association of measured LiDAR tracks

Established tracks now share a one-to-one assignment of the complete accepted
detection batch. The strict **distance < 3 m** gate is unchanged. The objective
first maximizes the number of existing tracks receiving a measurement, then
minimizes the sum of their predicted-to-measured Euclidean distances. A Hungarian
solver with explicit unmatched columns solves each connected candidate component.
Positions are predicted at the scan acquisition time using each track's original
observation time and measured velocity. No simulator identities or intentions
enter association.

Previously, each detection immediately claimed its nearest available track.
For tracks at x=0 and 2 m, detections at x=0.9 and -2.1 m could update only one
existing identity and create a third. Reversing those detections updated both.
The new assignment updates both existing identities in either order. A separate
two-match example reduces total distance from 3.8 to 2.0 m.

Tracks are ordered by identity and detections by x, y and radius for deterministic
ties. Permuting distinct detections preserves established-track results. Identical
detections are interchangeable. New unmatched detections still receive identities
in their supplied order; initial identity numbers do not describe physical object
identity. Alpha-beta gains, per-track velocity intervals, speed cap, radius
smoothing and observation expiry retain their existing values.

## Association limits and failure handling

The candidate search checks at most **1,000,000 track/detection pairs** after
expiry. Each connected component may contain at most **64 tracks and 64
detections**. A shared **2,000,000-unit** budget accounts for candidate pairs,
component distance-matrix entries and solver column scans. Disconnected components
can collectively exceed 64 objects; a 128-component check verifies this case.
These are work limits, not measured latency guarantees or global track-count
limits. Candidate adjacency traversal and allocation have separate overhead.

Exceeding a limit returns `TrackerUpdateError::AssociationLimit`. The existing
staged `try_update` leaves tracks, accepted scan time and identity allocation
unchanged. The legacy `update` returns cached tracks on rejection. The driving
pipeline uses the checked API, marks `InvalidLidar`, preserves its published
tracks, occupancy grid and accepted-LiDAR time, and requests emergency braking.
Recovery requires a valid acquisition newer than the latest fault time.

## Reproduce checks

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-perception --test tracker_assignment
cargo test --locked -p rustdriving-perception association::tests
cargo test --locked -p rustdriving-pipeline --test tracker_assignment
bash scripts/check.sh
```

Three original direct-tracker failures and one raw-point-cloud failure remain
retained with their source and original executable. Independent recursive
enumeration checks every partial injective assignment in 300 small fixtures,
including missed observations and nonzero velocities, plus an exact gate-boundary
case. Reversed detection order must preserve the established-track assignment.
The oracle uses neither the solver, dummy penalties nor component decomposition.

Known circular point sets separately verify clustering, association, alpha-beta
values and exact sensor-log replay. Another complete raw-point sequence creates
72 measured clusters: a subsequent connected component exceeds the limit,
brakes without partial publication, rejects cached pre-fault evidence and then
recovers on a new valid acquisition. The inputs are synthetic measurement
controls, without a real-sensor identity or physical accuracy benchmark.

## Viewed reference regressions

All 49 baseline seed-7 episodes and six additional seeded episodes pass their
unchanged physical acceptance and exact replay. One baseline case,
`multiple-blocked`, changes four output files; 241 of 245 outputs match the parent.
In its seed-7 run, maximum track count falls from four to three for three authored
obstacles, while the stopping position moves forward and clearance decreases.
The separately enforced **3 m** regression floor remains unchanged:

| Seed | Parent minimum clearance (m) | Updated minimum clearance (m) |
| --- | --- | --- |
| 1 | 3.921872 | 3.579863 |
| 7 | 3.664173 | 3.532493 |
| 42 | 3.592950 | 3.592950 |

All three episodes stop with zero collisions and road violations. The maximum
0.342009 m clearance reduction is retained as a physical tradeoff, without a
uniform improvement claim. The other two compared crossing cases retain their
minimum clearances across seeds 1, 7 and 42. These are viewed simulation
regressions; track count does not independently qualify object identities.
Affected historical logs remain schema-readable but can fail exact verification
with the updated algorithm. Their original executable and outputs remain retained.

The maximum-cardinality policy preserves the existing preference to reuse a
gated track. Distance alone can still choose the wrong physical identity when
objects cross or detections merge. Appearance, class, calibrated uncertainty,
occlusion reasoning and independently labeled automotive tracking accuracy remain
unimplemented or unverified. A mathematically optimal assignment is not evidence
of correct physical correspondence.
