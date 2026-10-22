# Reachable occupancy before acquired velocity confirmation

Experimental `forecast_cold_max_speed_m_s: 15.0` retains an object's reachable
position set while its initial measured velocity is unsupported. It requires
`forecast_envelope`, `fit_recent_track_motion`, measured-disc enclosure and the
continuous forecast-consistency guard. Omission preserves preceding behavior
and serialization. The declared speed must be finite and between 0.7 and 15 m/s.

## Why the first acquisitions need a separate model

The preceding pose-uncertainty qualification repaired distant world-coordinate
errors, but eight moving-body conditions still failed long-horizon coverage.
Their first two or three acquired observations had insufficient velocity support.
A first position cannot determine velocity. Treating a newly born zero-velocity
track as established stationary motion leaves its future occupancy too small.

The new option makes that missing information explicit. Its maximum speed is a
caller-supplied conditional prior, with 15 m/s matching the existing track-speed
cap in this authored protocol. It is not inferred from actor truth, estimated
probability or measured automotive calibration. A physical object outside that
prior, incorrect association or unsupported physical shape can still escape the
set. First acquisitions remain eligible for the original coverage scorer.

## Geometry and acquisition ownership

For acquisition-anchored track position `p`, acquisition age `a`, existing nominal
forecast center `c(t)` and declared position error `e`, the additional occupied
radius while velocity is unconfirmed is

```text
max(existing_extra_radius(t), |c(t) - p| + e + maximum_speed * (a + t))
```

This encloses the complete speed disc anchored at `p`, including every direction
and the age consumed before delivery/control. The base shape radius and nominal
centers stay intact. Linear interpolation of endpoint radii encloses the
intermediate reachable disc by convexity of distance. The claim ends at the
existing eight-second horizon; the ordinary endpoint hold adds no later reach
guarantee. Established motion-error budgets and the pose multiplier stay fixed.

Confirmation needs at least three real associated raw measurements spanning
0.2 s. A centered least-squares fit must have RMS residual at most 0.12 m, radius
range at most 0.12 m and velocity agreement with the current track within the
existing declared velocity-error budget. Support retains the shortest recent
suffix spanning 0.2 s, bounded by 32 samples, 0.6 s and 128 identities. Fresh
observations and the last observation must remain within 0.35 s. Cached scans
cannot vote, renew support or reset acquisition age.

The first successful support prefix is retained in `cold_motion` diagnostics;
`last_observed_acquisition_s` continues advancing. Confirmation remains latched
until an acquisition gap, scan fault, track expiry or pipeline epoch reset.
Subsequent motion is still monitored by the continuous guard. This is an initial
velocity-support mechanism, without continuous probabilistic confidence or a
guarantee that later acceleration stays within the existing declared budget.

A scan fault clears confirmation and consumes the fault's control timestamp.
An old cached acquisition cannot seed a new prefix after that fault. A fitted
initial speed above the declared maximum is an error. The reusable component
validates and commits evidence and forecasts transactionally. On error, the
pipeline clears forecast positions and brakes with `InconsistentForecast`.

## Composition

The operation runs in the continuous tracking frame after the ordinary motion
envelope and before the forecast-consistency guard. That guard therefore checks
the occupancy actually served to the planner. After world projection, the
separate current-pose envelope is added as before; pose padding cannot hide a
continuous-frame miss. No localization/simulator dependency enters the reusable
prediction component.

Observed-obstruction detours require every current cold track to have confirmed
velocity before admitting new blocker evidence. A large unknown-speed occupancy
set is insufficient evidence of a stationary blocked map edge. Existing stopped
handover, physical separation and ordinary health checks remain in force.

## Frozen qualification and independent checks

All 48 preceding pose-supported conditions were frozen before implementation:
44 authored detour/control conditions and four supported original missions,
covering both the reference bicycle and RNE native dynamics and seeds 7/42.
Only `forecast_cold_max_speed_m_s: 15.0` is added. Original worlds, sensing/noise,
geometry fits, motion/pose budgets, goals/deadlines, clearance, road reserve,
accuracy and 99% coverage gates stay fixed. Changed driving commands can change
later acquired observations in the new physical executions.

The Python oracle reconstructs support from raw associated acquisitions using
independent batch sums, checks all published support clocks/identities and derives
the complete reachable-radius floor. The motion checker independently retains
the original nominal centers and minimum motion envelope before checking that
floor; a large cold radius cannot conceal a changed nominal path. The continuous
guard oracle uses the served cold occupancy, stripping only independently
calculated world-pose padding. Physical coverage still uses simulator truth only
in post-run evaluation, including all first observations.

Rust tests exercise complete discs in many directions at and between knots,
acquisition age, fresh/cached observations, fault/gap resets, independent IDs,
bounded high-rate support, shape/velocity disagreement, transactional malformed
inputs and speed-bound errors. Pipeline tests check sensor-to-forecast behavior,
fail-closed braking and replay tampering. Python adversarial tests reject forged
padding, age, support clocks, confirmation and changes to frozen source inputs.

```sh
python3 scripts/test_cold_motion.py
python3 scripts/check_cold_motion.py --check-contract --output artifacts/cold-motion
bash scripts/check.sh
```

The explicit `--check-contract` verifies support, geometry and replay while
retaining failed physical gates in the report. Without it, the qualification
command fails on a newly failed gate. CI uses contract mode and uploads the
complete failed qualification; it does not establish regression or adoption.

## Recorded result and excluded integrations

The delivered runtime uses the first complete 48-condition cohort. All cases
pass independent support, nominal-motion geometry, pose, continuous-guard and
complete sensor-replay contracts over 60,882 control ticks. The cold envelope is
served for 298 forecasts, with 48 acquired velocity confirmations. All first
observations remain scored. The eight preceding moving-body coverage failures
pass; total forecast failures fall **8 → 1** and strict qualification improves
**28 → 36/48**. Goal arrivals remain 20, including the eight authored observed
detours. All twelve negative pairs retain identical operational output bytes,
acquired inputs, physical frames and summaries.

The remaining coverage failure is new: native original mission seed 7 had
previously passed that gate. Startup braking changes its later approach; after
confirmation, a stationary physical actor acquires apparent fitted velocities
up to about 1.3 m/s. Nine acquired forecasts miss at least one physical horizon.
Coverage is 98.435% at 4 s and 98.505% at 8 s, below the unchanged 99% gate.
The initial reachable set cannot certify later fitted motion. Regression and
default adoption therefore **fail**, despite the higher strict-pass count.

Every case passes 0.2 s and 1 s coverage, enclosing all 21,856 and 21,632 eligible
physical forecasts respectively. Minimum swept clearance is 4.402142 m, minimum
active-road margin 0.345882 m, maximum position error 0.282521 m and maximum yaw
error 0.017222 rad. All original clearance, road and accuracy gates pass. These
finite viewed worlds do not establish general availability. Cold extra radii
reach 154.853640 m at the long horizon, a substantial conservative planning cost.

Two integration trials are preserved and excluded from the delivered runtime.
The second full 48-condition cohort prevents unsupported velocity from arming a
new persistent obstruction stop. It repairs the native forecast regression but
introduces two reference stopped-handover failures: actual confirmation speed
reaches 0.058292 m/s against the original 0.05 m/s limit. A later trial requires
an already active retained stop before proposing a detour; both affected canaries
then fail to propose a route and miss their goals. No gate, input, initial
observation or failed trial is discarded. The original runtime is restored and
its source and executable hashes are checked against the first cohort. The
experiment contains 99 physical executions, including these trials and a native
repair canary, with 495 archived recording streams.

The final unchanged-source `bash scripts/check.sh` passes 970 Rust tests,
266 Python tests and all 49 standard driving/replay scenarios. All 245 standard
output files retain exact preceding bytes. Native formatting, 67 release tests
and strict all-target Clippy pass on the delivered runtime. All 748 preceding
sensor logs, covering 582,378 control ticks, retain exact replay output bytes.
All 495 experimental streams have their encoded and decoded SHA-256 verified.
Intermediate checks interrupted for the two source corrections are retained;
the final check completes on restored delivered sources. The
[result packet](../assets/cold-motion-envelope-results.json) records the complete
failed qualification, excluded trials, fixed inputs and source/binary/lock hashes.
Remote CI, cross-platform success and real-time operation remain unverified.

Eight disabled detour controls and four supported original missions retain
missed-goal failures. Four other original mission conditions lack the preceding
heading prerequisite and remain unmodified in the prior evidence. All eight
original mission failures remain unresolved. The feature stays off by default;
speed/shape confidence calibration, post-confirmation motion, useful complete
missions and broader domains remain open. Comparative maturity remains about
29%; the 30% waypoint is unmet.
