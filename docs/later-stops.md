# Searching later checked stopping positions

Experimental `search_later_obstruction_stops: true` evaluates stopping positions
beyond the planner's original collision-retiming heuristic. It requires
`replan_checked_approaches` and all of that policy's existing prerequisites.
The option defaults to false and is omitted from old recording headers.

## Behavior and ownership

The nominal contact time is computed for a different speed profile from the
eventual stopping trajectory. Subtracting four meters from the nominal contact
distance need not identify the farthest feasible stop, especially for crossing
traffic. Feasibility is not monotonic in stopping distance: a retimed path can
cross after an actor has cleared, or encounter its expanding forecast later.

Before the ordinary and earlier-stop fallback, the new option visits existing
geometry knots beyond that heuristic in descending accumulated arc length.
There are at most 80 proposals from a path with at most 81 points. Each proposal
rebuilds the reachable acceleration/deceleration profile and verifies that the
actual path reaches its requested endpoint. A stationary prefix cannot count as
reaching a farther stop. The unchanged collision sweep then checks every moving
segment and the complete eight-second stationary tail. Rejection of every
proposal preserves the ordinary/earlier fallback. No binary search or new path
spacing is introduced.

An active obstruction hold, an unfinished earlier-search brake, and an already
settled yield endpoint retain their ownership. A newly checked later approach
does not arm the earlier-search braking latch. Replacing an existing stationary
stop still requires the preceding three fresh acquired confirmations spanning
0.2 seconds; cached or unhealthy scenes cannot authorize restart. Every accepted
proposal retains the ordinary controller and fault guards.

The implementation uses supplied routes, estimated ego state and acquired
forecasts. Motion, cold-start and pose budgets, road reserves, sensing/noise,
goals and deadlines remain unchanged. Simulator truth is evaluation-only.

## Independent inspection

The existing `searched_stop` marker records use of a stopping search without
identifying its direction. The new inspector certifies **every selected checked
stop**, including records with a false marker. It verifies finite positive time
intervals, nonnegative speeds, exact constant-acceleration distance integrals,
the supplied acceleration/deceleration limits and a complete stationary tail.
A separate midpoint/Lipschitz interval proof checks the full accelerated path
against emitted forecast centers and radii. An interior radius peak cannot hide
behind clear endpoint samples. Fault and emergency ticks must retain braking.

This certifies published paths conditional on the supplied forecasts. It does
not establish optimality of unpublished candidates or real-world forecast
calibration. Full sensor-only replay binds the recorded proposal and command to
the implementation. The stopping-search marker alone cannot prove which search
direction produced a trajectory.

## Evaluation protocol

All 48 preceding inputs remain frozen, adding only the new option. The original
1 m physical-clearance, 0.1 m road-reserve, localization-accuracy, original-goal
and 99% forecast-coverage gates remain scored at 0.2, 1, 2, 4 and 8 seconds.

The first reference canary used 16 equally spaced distance proposals. It changed
a proposed endpoint but preserved the original physical summary and missed the
65-second goal. The second protocol enumerates the existing geometry knots
instead of introducing a coarser grid. Its reference canary increases progress
from 45.360956 to 45.516994 m; its native canary retains 45.860028 m. Both still
miss the same goal. These are viewed development trials, not held-out evidence.

The first canary's aggregate process rejected a change to the unused native
executable while that executable was being built. The selected reference
executable remained unchanged and its physical recording is retained. All final
evaluation runs use immutable copies of both completed executables. The failed
process and its fingerprints remain in the evidence packet.

## Fixed results

All 48 conditions retain their preceding gates over 60,831 control ticks.
Arrivals remain **20** and strict qualification remains **36/48**. The twelve
inherited source failures comprise eight disabled detour controls and four
supported original mission failures. The four other original missions remain
unchanged outside this supported cohort; all eight original incomplete missions
remain open. Default adoption fails.

| Plant | Seed | Previous progress (m) | Later-search progress (m) | Goal reached |
| --- | ---: | ---: | ---: | --- |
| Reference | 7 | 45.360956 | 45.516994 | No |
| Reference | 42 | 45.201026 | 45.389465 | No |
| Native | 7 | 45.860028 | 45.860028 | No |
| Native | 42 | 46.757519 | 45.028213 | No |

The native seed-42 loss is about 1.73 m. The minimum actual road margin falls
from 0.228404 to 0.206591 m, retaining the fixed 0.1 m requirement. More stopping
proposals and more confirmed adoptions do not demonstrate better complete
mission availability.

The inspector certifies **35,171 selected checked-stop control ticks**, including
repeated stationary proposals, using **1,595,070 independent interval bounds**.
Of those ticks, 2,911 carry the search marker, which includes either direction.
All five forecast horizons cover every eligible sample: respectively 21,880,
21,656, 21,376, 20,816 and 19,696 samples. The minimum physical swept clearance
is 4.340147 m; maximum position/yaw errors are 0.298495 m and 0.017222 rad.
No collisions or road violations occur. All 20 detour pairs retain gates, and
all 12 negative pairs retain operational outputs, acquired inputs, physics and
summary bytes. Four unpaired known-closure controls remain in the full matrix.

The exact required `bash scripts/check.sh` passes 1,002 Rust tests, 295 Python
tests and all 49 standard runs/replays, with 245 standard streams matching the
retained archives byte for byte. Native formatting, Clippy and all 67 tests
pass. All 892 preceding recordings replay exactly over 764,973 ticks. The two
final canaries reproduce all five streams of their corresponding matrix runs.
All 51 physical executions, including the excluded first canary, retain 255
compressed streams with encoded and decoded hashes.

The first full check and concurrent replay exhausted temporary storage. Only
rebuildable native debug cache was removed; the whole exact required command
and the unverified replay jobs were rerun without changing operational sources
or thresholds. Initial failures remain recorded. The
[versioned result packet](../assets/later-stops-results.json) binds the frozen
inputs, protocols, executables, source inventory, comparisons and archive hashes.
The comparative engineering estimate remains about 29%; the 30% waypoint is unmet.

## Reproduction

Run `bash scripts/check.sh` and build the pinned native integration before
running the two-plant matrix:

```sh
python3 scripts/check_later_stops.py --output artifacts/later-stops
```

Use `--plant reference-2d` or `--plant rne-dynamic` to evaluate one plant. The
command fails on any newly failed gate and preserves inherited mission failures
in its strict result. CI runs the independent corruption tests and both plant
qualifications, retaining their full reports and compressed streams.

No real-time, real-vehicle, measured KITTI, remote-CI or cross-platform result is
claimed. Bounded enumeration increases planning work; finite candidate count is
not a latency guarantee. The experiment stays off by default.
