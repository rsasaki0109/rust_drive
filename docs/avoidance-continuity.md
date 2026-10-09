# Completing an avoidance shift without premature center return

The previously retained 6 m/s RNE detour mission now reaches its destination without an optional curvature cap. The world, closure notification, speed, duration and acceptance criteria are unchanged. `route-handover-fast` preserves that configuration as a positive regression; only its display name differs. The opening README GIF is the actual seed-7 CPU RNE dynamic run, rendered from telemetry at 3× speed.

## Failure and repair

At baseline commit `cc9ba4d05c7b90685969203b7e0c8d34696e5256`, the seed-7 mission switched routes successfully but stopped near the obstacle until 70 s elapsed. It was collision-free, yet failed goal acceptance. Its 1401 sensor ticks replayed successfully with that implementation. The [baseline snapshot](handover-results.json) retains the failed summary and original positive-suite measurements.

Inspection of the sensor recording showed repeated changes between the center and avoidance targets before the stop. Changes in the tracked constant-velocity forecast temporarily made a center-return candidate clear. The original score preferred that candidate even during an unfinished shift, resetting the maneuver anchor. Increasing the blocked-path penalty alone reproduced the same failed trajectory; it did not address those earlier free-candidate changes.

The planner now adds **one score unit** to a center-return candidate when all these conditions hold:

- A nonzero lateral target is established and its anchored transition has not completed in route arc length.
- A predicted object's current position, projected onto the known route, lies ahead or alongside the circular footprints and within the planning horizon.
- That object's envelope spans the centerline, with the existing 0.3 m margin.

This is a modest hysteresis preference. It uses estimated ego state and tracked predictions, with no simulator truth or object labels. It expires when the shift completes, the object passes behind, or its envelope clears the centerline. It does not force a target: corridor checks, reachable speed bounds, synchronized collision sweeps, retimed stops and stationary-hold rechecks still reject infeasible candidates. The other score terms and calibration remain unchanged.

A preliminary policy that retained avoidance until passing every nearby object introduced an occluded-crossing deadlock, insufficient clearance and a road-boundary violation. That policy was rejected. Limiting the preference to unfinished shifts and objects spanning the centerline addresses those regressions; their existing acceptance constraints remain in the suite.

Unit tests distinguish a temporarily clear forecast from an observed obstacle still ahead, release the preference after transition completion, permit center return for an off-center object, and require braking/rejection for a full-width obstruction. The reference integration tests exercise stopped handover, world continuity and complete replay at 6 m/s across three seeds. The RNE test that previously asserted the failed mission now requires actual goal acceptance and unchanged clearance.

## Measured results (2026-10-09 UTC)

Formatting, Clippy with warnings denied, locked release builds and **100 workspace tests** pass. The reference check script passes fifteen scenario/replay pairs. **11 RNE tests** pass. The seeded suite passes **72 runs**, including **24 live-navigation runs**, across seeds 1/7/42 and both backends. All have zero collisions, road violations and closed-edge entry violations, full replay, valid profile kinematics and their unchanged minimum-clearance and normal steering-rate constraints. [Full current snapshot and retained baseline](avoidance-results.json).

Time, tick count and handover time use seed 7; clearance is the minimum across the three seeds for that row.

| Backend | Fixture | Time (s) | Replay ticks | Handover (s) | Worst clearance (m) |
|---|---|---:|---:|---:|---:|
| Reference | 4 m/s handover | 55.60 | 1113 | 11.40 | 0.781 |
| Reference | 6 m/s handover | 41.15 | 824 | 9.10 | 0.809 |
| Reference | No route | 20.00 | 401 | — | 64.007 |
| Reference | Reopened detour | 59.75 | 1196 | 15.10 | 0.668 |
| RNE dynamic | 4 m/s handover | 55.90 | 1119 | 11.30 | 0.719 |
| RNE dynamic | 6 m/s handover | 40.25 | 806 | 9.15 | 1.458 |
| RNE dynamic | No route | 20.00 | 401 | — | 64.015 |
| RNE dynamic | Reopened detour | 59.75 | 1196 | 15.10 | 0.779 |

The repaired RNE seed-7 mission reaches the destination at 40.25 s with 1.485 m minimum clearance and five emergency ticks, compared with the baseline's failed 70 s episode, 0.952 m clearance and 38 emergency ticks. Its route handover occurs at 9.15 s with 0.0139 m/s estimated speed and zero actual speed. The GIF's scenario, summary and route history match the independently validated suite recording; its 806 sensor ticks fully replay. The 1200 × 720 GIF has 135 frames.

The change does not improve every metric: for example, RNE `route-direct` seed 1 clearance decreases from 0.943 to 0.854 m, above its unchanged 0.5 m floor, and RNE `multiple-blocked` seed 7 decreases from 3.779 to 3.602 m, above its 3 m floor. Complete rows preserve those outcomes; the claim is repaired completion with all existing gates maintained.

## Reproduce

```sh
bash scripts/check.sh
bash scripts/setup-rne.sh
bash scripts/check-hazards.sh --output artifacts/pass-continuity
# With the documented Pillow environment activated:
bash scripts/rne-demo.sh dynamic assets/rne-demo.gif scenarios/route-handover-fast.json
```

The RNE integration still uses pinned engine `df6007aa40315e81d12ae00fc1f60369e393a178`. Its native vehicle dynamics, steering lag and Rapier ray-query scene are unchanged. This is a recorded top-down visualization, not an RNE camera capture. The simulator's circular/swept collision and active-corridor evaluators score physical outcomes independently of replay. The [navigation protocol](handover.md) remains unchanged.

## Limits

This repairs a particular authored planar detour across seeds 1/7/42. It does not establish arbitrary high-speed feasibility, combined tire-force constraints, continuous moving-route handover, general deadlock recovery, interaction-aware prediction or real-vehicle safety. Emergency fallback still occurs. The preference is heuristic, and route projection can be ambiguous on self-crossing paths. Exact replay requires regenerated outputs from the revised planner; sensor schema 1 remains readable.
