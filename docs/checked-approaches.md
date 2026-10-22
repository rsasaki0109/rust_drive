# Replanning a checked stopping approach

Experimental `replan_checked_approaches: true` lets a stationary vehicle replace
a retained obstruction stop with a newly checked approach ending in another
complete stop. It requires stop retention, stop search, a declared lateral road
reserve and the recent supported-velocity occupancy option with its existing
prerequisites. The flag and `checked_approach` output are omitted when disabled.
Default adoption remains withheld.

## Problem and policy

The preceding supported original missions stop 35 m or more from their first
obstacle. The initial maximum-speed reachable set requests braking; after fresh
raw velocity support contracts that occupancy, the retained stop still waits
for an entirely unblocked nominal path. The available eight-second motion
envelope prevents that complete path. A farther checked stopping approach can
be feasible while the old stopping commitment remains fixed.

The planner first runs its ordinary selection. Only a stationary checked
`Yield` proposal, with measured and planned speeds at most 0.2 m/s and all
positions within 0.2 m of the measured estimate, can request a new approach.
A separate copy retains maneuver geometry but removes the old obstruction and
yield stop anchors. It evaluates the existing candidate family, speed limits,
collision sweeps and complete eight-second stationary tails. Complete paths take
priority; among checked stops it prefers the greatest endpoint route progress.
An unconfirmed copy cannot change the operational planner's stop ownership.

The enabled policy also uses the supplied width reserve up to a bounded 4 m
side offset, instead of the inherited 3.5 m cap. A 5.5 m half-width with a 1.25 m
vehicle radius and 0.35 m reserve gives 3.9 m. Every actual candidate remains
inside the same supplied corridor. Previously selected side intent and its
original maneuver anchor survive temporary centerline braking; the next path
still joins the current measured pose and passes all ordinary checks.

No motion/pose uncertainty budget, physical clearance or road acceptance reserve
is reduced. This adds a checked proposal and a bounded confirmation policy;
it does not prove that a complete passing path exists.

## Confirmation and ownership

A proposed stopping endpoint must gain at least 2 m of supplied-route progress.
At least three distinct accepted scene acquisitions must span 0.2 s. Consecutive
acquisitions and their control-time age must remain within 0.35 s. The same
sorted blocker identities, lateral target within 0.01 m and endpoint within 1 m
of the first proposal must persist. The estimated position and route progress
must remain within 0.2 m of the first supporting acquisition's anchor.

Only healthy scenes with every cold-motion track confirmed can authorize
adoption. Temporary mapped stopping routes cannot authorize it. Cached control
ticks can withdraw support but cannot add acquisitions or adopt a proposal.
Faults consume their control timestamp; an older cached scene cannot seed the
replacement prefix. Route changes clear support without restoring consumed
acquisitions, and localization epoch reset creates fresh identity scope.

On the confirming fresh acquisition, the current proposal must again pass its
full speed, corridor and forecast checks, including the complete stop and tail.
That checked proposal and its planner state replace the old stopping commitment.
Normal health and controller guards still own final braking. The component uses
acquired forecasts, estimated ego state and the supplied route; actor truth,
world labels and future commands remain evaluation-only.

`checked_approach` records acquisition counts/clocks, the proposed endpoint and
route progress, selected side, blocker identities and whether this fresh scene
adopted it. Counts saturate at three; acquisition clocks still enforce the
minimum elapsed span. Empty support reports empty fields. The independent Python inspector
reconstructs acquisition confirmation and certifies each adopted accelerated
path against the emitted forecasts with a separate interval-bound method.
It verifies the emitted proposal evidence, not optimality of unpublished
counterfactual paths. Complete sensor replay separately binds every record and
the configured policy to the executed implementation.

## Fixed evaluation

All 48 preceding worlds, both plants, seeds 7 and 42, sensing/noise, raw fits,
motion/pose/cold budgets, goals and original deadlines remain frozen. Only the
new flag is added. The 65 s original goal failures remain scored, along with
1 m physical clearance, 0.1 m road reserve, localization accuracy and 99%
forecast coverage at 0.2, 1, 2, 4 and 8 s. No first observation is removed.

Separate reference canaries retain the original 3.5 m cap, the usable-width
proposal, and the final side-intent policy. Their mission progress is respectively
46.135184, 47.148895 and 45.360956 m, compared with the preceding 20.412331 m.
None completes the mission. The final policy preserves the chosen passing
intent through center braking; a separately recorded native canary reaches
45.860028 m versus 18.217081 m. These are correlated viewed trials, not held-out
or independent automotive evidence.

The first evaluation adapter omitted the new width flag from its active-route
decision inspector. A subsequent review also used the initial route for the
entire navigation recording. Both adapter errors are retained and corrected:
navigation decisions and candidate containment now use the independently
reconstructed active route at each tick. The same archived executions are
rescored; raw streams, inputs, commands and physical gates are not changed.

The final 48-condition comparison passes regression with **no new failed gates**.
It executes 60,831 control ticks and independently certifies all 60 adopted
approaches using 3,990 interval bounds. The four supported original missions
gain local progress under their unchanged 65 s deadlines:

| Plant | Seed | Previous progress | Final progress | Goal reached |
| --- | ---: | ---: | ---: | --- |
| Reference 2D | 7 | 20.412331 m | 45.360956 m | No |
| Reference 2D | 42 | 20.418688 m | 45.201026 m | No |
| Native dynamic | 7 | 18.217081 m | 45.860028 m | No |
| Native dynamic | 42 | 18.291472 m | 46.757519 m | No |

Goal arrivals remain **20**, strict qualification remains **36/48**, and default
adoption fails. Eight disabled detour controls and four supported original
missions retain their failed source outcomes. Four other original missions lack
the preceding heading prerequisite and remain unmodified; all eight original
mission failures remain open. The final reference seed-7 frame still reports
forecast conflicts on the center and retained side, approximately 2.96 and
3.06 s into their nominal paths. A confirmed approach does not supply a complete
passing trajectory.

All five coverage horizons meet the unchanged 99% gate in all 48 conditions.
Every eligible acquired identity is covered: 21,880 at 0.2 s, 21,656 at 1 s,
21,376 at 2 s, 20,816 at 4 s and 19,696 at 8 s. Changed commands can change later
sensor returns and end-of-record censoring; the counts are from new executions.

There are no collisions or road violations. Minimum swept physical clearance
is 4.340147 m, minimum actual road margin is 0.228404 m, maximum position error
is 0.298495 m and maximum yaw error is 0.017222 rad. The wider candidates reduce
the preceding minimum road margin from 0.345882 m and increase maximum position
error from 0.246167 m; both remain inside the original gates. These tradeoffs
remain recorded rather than presented as uniform improvement.

All twenty detour pairs retain their comparison gates. The twelve negative pairs
preserve exact operational output bytes, physical frames, acquired inputs and
summary bytes. Both final canaries reproduce all five raw/replay streams of
their matching full-cohort executions byte for byte. All 52 physical executions
and 260 encoded/decoded archives remain retained. The versioned
[result packet](../assets/checked-approaches-results.json) includes every frozen
input, complete score, excluded canary, correction and validation fingerprint.

```sh
python3 scripts/test_checked_approaches.py
python3 scripts/check_checked_approaches.py --output artifacts/checked-approaches
bash scripts/check.sh
```

The complete required check passes **994 Rust tests, 287 Python tests and 49
standard run/replay cases**. All 245 standard output files retain exact baseline
bytes. Native formatting, 67 release tests and strict all-target Clippy pass.
The delivered reference binary reproduces **844 prior sensor logs / 704,142
ticks** with exact output bytes. The final source inventory, both executable
hashes, unchanged dependency locks and native-engine pin are retained in the
result packet. Remote CI has not been observed.

## Limits

A verified approach is conditional on the existing forecasts and control
tracking. The inherited endpoint hold does not establish physical coverage
beyond the eight-second prediction horizon. Larger side candidates and repeated
approaches can change later sensing, commands and availability. The option adds
no measured confidence calibration, interaction prediction, real-time bound,
real-vehicle integration or cross-platform result.

The original incomplete missions remain unresolved. Comparative engineering
maturity remains **about 29%** versus Autoware, Apollo and openpilot; the 30%
waypoint remains unmet.
