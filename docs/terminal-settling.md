# Maintaining a completed terminal stop

Small position errors previously restarted the planner after it had already
stopped near its desired goal, one meter before the supplied route endpoint.
In the fixed reactive-follower world, renewed motion could reset the eight-second
physical residence timer and miss the 65-second completion deadline. The default
seed-42 failure and experimental slow-motion failures at seeds 1 and 42 remain
retained in `artifacts/slow-motion-v1`; they are not relabeled as successes.

## Planner behavior

A candidate settles when the estimated route progress is within 0.5 m of the
desired goal, measured speed is at most 0.2 m/s, estimated lateral displacement
from its target is at most 0.2 m, and heading differs from its initial tangent by
at most 0.1 rad. These are planner tolerances, independent of simulator acceptance.
The existing reachable speed profile receives a zero cruise target, preserving
initial speed and calibrated braking authority. A stationary input remains at its
actual estimated pose; it does not invent motion to the exact nominal setpoint.

Containment, curvature/speed feasibility, synchronized obstacle sweeps, retiming
and the entire eight-second planned hold still decide feasibility. A feasible
settled candidate precedes normal candidate costs so speed rewards cannot restart
a completed maneuver on a wide road. A conflicting settled candidate is rejected;
other candidates remain available. No remaining candidate returns an empty
emergency trajectory. An emergency stop is not reported as a collision-free
planned hold. When a conflicting forecast clears, the same settling policy avoids
a new acceleration merely to correct a few centimeters.

The planner's `terminal_settling` option defaults to true. The pipeline disables
it whenever the selected route is a temporary navigation, traffic-signal,
stop-sign, intersection or lane-change stopping prefix. Those operational stop
locations retain their preceding approach profiles. There is no new latch,
residence timer, actor-policy input or sensor-log configuration flag.
Route handover still creates a fresh planner. Moving, distant, laterally unfinished
or unaligned candidates retain their approach profiles. Prediction, vehicle/actor
models, scenario durations, goal residence and physical clearance floors are
unchanged. Slow-motion prediction remains experimental and off by default.

## Checks and reproduction

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-planning --test terminal_settling
cargo test --locked -p rustdriving-sim --test terminal_settling
bash scripts/check.sh
target/release/rustdriving run \
  --scenario scenarios/traffic-follower-deadline.json --seed 42 \
  --output artifacts/terminal-deadline
```

Analytic planner fixtures cover rotated roads, position changes around the goal,
reachable monotonic braking, a late crossing during the hold, recovery after that
forecast clears, unfinished approaches and a retained wide-road refuge. The
original planner fails the stationary/restart fixtures. An intermediate version
repairs narrow-road residence but still chooses a moving candidate on a wide road;
its failed test, source, executable and six physical recordings are retained.

An additional intermediate version passed the original physical floors but
changed temporary navigation stops: seed-7 detour clearance fell from 1.416 to
0.654 m in `route-handover`, and from 1.387 to 0.633 m in `route-reopen`.
That version was not adopted. Its source, executable and complete 80-run matrix
remain retained. A new physical regression requires over 1 m in these unchanged
worlds; restricting the tolerance to the destination preserves their approach.

The physical regression uses the unchanged deadline scenario at seeds 1/7/42 with
each predictor policy. It independently inspects the final eight seconds of truth
positions/speeds and commands: no residence reset, no positive acceleration,
less than 0.02 m displacement, zero contacts/road violations and at least 1 m
clearance. Every recorded sensor tick must replay through a fresh pipeline.
Truth is used only for independent physical scoring.

## Reference measurements

The final required check passes **575 tests and 49 default scenario/replay pairs**.
The independent comparison covers **80 reference episodes: 78 physical passes and
two retained insufficient-range failures**. All 80 replay exactly. Coordinate,
filter-interval and forecast audits cover 62,489 sensor ticks; 31 episodes
also retain every physical tick for an independent swept-clearance recomputation.
[Compact results and tested source hashes](terminal-settling-results.json).

The deadline and its eight-second residence remain unchanged:

| Prediction policy | Seed 1 | Seed 7 | Seed 42 |
| --- | ---: | ---: | ---: |
| Default | 63.85 s | 63.75 s | 63.70 s |
| Experimental slow motion | 63.50 s | 63.45 s | 63.45 s |

All six are contact-free and on-road, with minimum swept clearance of
**about 2.997 m**. The final eight seconds contain no positive acceleration commands;
maximum physical excursion is **about 0.00313 m**, below the fixed 0.02 m check.
Four failed parent completion results are repaired, including the newly recorded
parent experimental seed 7. The original three documented failed recordings and
all new parent recordings remain unchanged.

Final seed-7 detour clearances are restored to **1.415806 m** (`route-handover`)
and **1.386690 m** (`route-reopen`), exactly matching the parent. The smallest
passing clearance change across all comparisons is -0.010077 m; every authored
clearance floor remains unchanged. The insufficient-range controls still fail at
**0.296276 / 0.303729 m** clearance, exactly matching the parent. They remain
excluded from positive results.

The original binary replays the retained old deadline recording. The changed
binary rejects that recording's changed expectations without a success report;
an unaffected sensor-fault recording still replays. All 245 historical live
outputs were restored byte-for-byte with separate live/backup inodes. The pinned
compiler, lockfile, release settings and physical worlds remain unchanged. Local
artifacts retain compressed recordings, hashes, independent audits, earlier
rejected sources/executables and the rejected adoption attempts.

## Limits

The deadband sacrifices exact centimeter positioning within the terminal region.
It is not a parking controller, a calibrated uncertainty bound or a guarantee
against an approaching vehicle. A rear forecast can still reject a stationary
hold and cause emergency braking. General interaction prediction, road-rule
reasoning and recovery from a trapped stopping place remain absent. This change
has no new RNE, real-vehicle, external automotive accuracy or real-time qualification.
