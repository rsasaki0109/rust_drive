# Recent measured velocity point hypotheses

Optional `forecast_recent_velocity_hypotheses: true` preserves a bounded set of
recent supported velocity alternatives in obstacle occupancy. It requires the
existing cold-motion option, which already requires raw motion fitting,
measured-disc enclosure, declared motion bounds and forecast guarding. Omission
preserves preceding behavior and serialization. The option stays off by default.

## Problem and model

The preceding cold-motion feature covers unsupported initial velocity but
confirms only its initial support. Native original mission seed 7 subsequently
fits apparent motion to a stationary physical actor while the ego approach
changes. A single recent fitted velocity can then move the long-horizon nominal
forecast away from the physical body. The current declared velocity-error budget
does not cover that case. Changing stopping retention instead produced physical
confirmation or goal regressions in the preceding excluded trials.

The new component retains recently acquired track velocities as point hypotheses.
For current acquired position `p`, acquisition age `a`, nominal center `c(t)`,
retained velocity `v` and existing position-error allowance `e`, the radius floor
for that alternative is

```text
|c(t) - (p + v * (a + t))| + e
```

Each knot takes the maximum of the existing extra radius and every such floor.
The current motion envelope and its declared position/velocity/acceleration
budgets remain intact. Nominal centers, physical base radii, acquisition clocks
and current-pose padding stay intact. If those radii already enclose every
alternative, all prediction values are identical to the preceding model.

Historical velocities are extrapolated from the current acquisition anchor;
old positions do not become new physical observations. The extra alternatives
are measured point velocities with the existing position allowance. They are
not complete velocity-error or acceleration-error tubes around every past
velocity, calibrated uncertainty, or a guarantee of future actor behavior.
The original current-velocity error envelope remains independently required.

Between knots, both the nominal center segment and each extrapolated point path
are affine in time. Convexity of their distance means linear interpolation of
the endpoint radius floors contains the intermediate point disc. The claim ends
at the existing eight-second horizon; the endpoint hold adds no later guarantee.

## Acquisition and state ownership

Only velocities from fresh accepted acquisitions with confirmed cold-motion
support enter history. Unsupported startup velocities are excluded. Cached
control ticks cannot add a sample or renew its timestamp. An unrelated new scene
cannot renew an unobserved identity. History retains the latest 32 samples per
identity within 0.8 s of the current control time, with at most 128 identities.
The window follows the predictor's existing bounded recent-motion horizon.

Loss of support, scan failure or an acquisition gap over 0.35 s clears the
identity's history. Track expiry releases it; a pipeline epoch reset creates a
fresh component. A fault consumes its control timestamp, preventing an older
cached scene from reseeding history. `recent_velocity_hypotheses` reports sample
counts and first/last acquisition times for every current identity, including
empty histories. Input, support-identity, clock and occupancy errors are
transactional; the pipeline clears malformed forecasts and brakes.

The reusable prediction component depends only on core SI contracts. It runs
after the cold envelope and before the continuous forecast-consistency guard,
so the guard checks occupancy actually served to the planner. World projection
and current-pose expansion follow. No actor truth, route labels, simulator
behavior or future commands enter the component. Existing stopping retention,
observed-detour proposals and handover policy remain unchanged.

## Fixed comparison and independent checks

Before implementing or scoring the model, all 48 preceding inputs were frozen
with only `forecast_recent_velocity_hypotheses: true` added. Both plants, seeds,
worlds, sensor schedules/noise, fitting, cold prior, motion/pose budgets, routes,
goals/deadlines and physical/accuracy/99% coverage gates remain fixed. Every
first observation remains scored. Changed commands can change later sensor
returns, acquisition counts and end-of-record censoring in the new executions.

An initial evaluation applies the declared point-set model to the unchanged
parent recordings. It removes the remaining coverage failure on those fixed
traces. This is explicitly a shadow calculation without new physical execution
or mission-availability evidence. A separate native repair canary and the
complete 48-condition comparison then execute the actual enabled pipeline.

The Python oracle independently rebuilds recent sample ownership from accepted
scene clocks and the separately reconstructed cold support. It checks every
published count/timestamp and contains every historical point disc. The motion
checker independently requires the original minimal nominal-motion envelope
before adding the cold and historical floors. A large historical radius cannot
hide a changed nominal center or arbitrary padding. Pose removal strips only
independently calculated current-pose padding, retaining the historical set for
continuous-guard reconstruction. Physical coverage remains evaluation-only.

Rust tests check every retained velocity disc at and between knots under frame
rotations, exact unchanged already-enclosed forecasts, cached age, unsupported
acquisitions, faults/gaps, identity expiry, bounded high-rate input, malformed
inputs, horizon overflow and transactional failures. Pipeline tests bind support,
serialization and every diagnostic to sensor replay. Python corruption controls
reject changed centers, padding, clocks, support, sample types and frozen policies.

```sh
python3 scripts/test_velocity_hypotheses.py
python3 scripts/check_velocity_hypotheses.py --output artifacts/recent-velocity-hypotheses
bash scripts/check.sh
```

## Recorded result

The frozen comparison executes 48 conditions in the reference 2D and native
dynamic plants, with seeds 7 and 42 and 60,882 control ticks. The remaining
native original-mission forecast-coverage failure falls **1 → 0**, without new
failed gates. All 48 conditions meet the unchanged 99% coverage gate at each
scored horizon; every eligible acquired identity sample is covered:

| Forecast horizon | Covered / eligible |
| --- | ---: |
| 0.2 s | 21,856 / 21,856 |
| 1 s | 21,632 / 21,632 |
| 2 s | 21,352 / 21,352 |
| 4 s | 20,792 / 20,792 |
| 8 s | 19,672 / 19,672 |

Goal arrivals remain **20**, and strict qualification remains **36/48**. The
remaining twelve failed source outcomes are eight disabled detour controls and
four supported original missions. The other four original incomplete missions
lack the preceding heading prerequisite and stay unmodified; all eight original
missions remain unresolved. Passing forecast regression does not pass default
adoption or establish general mission completion.

Independent reconstruction verifies 15,101,202 retained point discs. The largest
observed history has nine samples; the largest difference from the current
tracked velocity is 1.290044 m/s. Minimum swept physical clearance is 4.402142 m,
minimum actual road margin is 0.345882 m, maximum position error is 0.246167 m and
maximum yaw error is 0.017222 rad. No collision or road violation occurs.

In 31 conditions the preceding occupancy already encloses all retained points;
their operational output bytes, acquired inputs, physical frames and summary
bytes exactly match the parent after removing only the new status field. All
twenty detour pairs retain their comparison gates, including twelve negative
pairs with identical operational bytes and physics.

The separate repair canary and complete cohort retain all 49 physical
executions, including their five raw/replay streams each. The initial shadow
calculation remains separate and contributes no new physical execution.
The versioned [result packet](../assets/recent-velocity-hypotheses-results.json)
retains the protocol, frozen inputs, binary fingerprints, full scores and
validation provenance.

The complete required `bash scripts/check.sh` passes 981 Rust tests, 278 Python
tests and 49 standard run/replay cases. All 245 standard output files match the
retained baseline bytes. Native formatting, 67 release tests and strict Clippy
pass. The delivered reference binary also reproduces all 796 prior sensor logs
and 643,260 ticks with exact operational output bytes. The tested operational
source inventory, both executable hashes, dependency locks and native-engine
revision are retained in the packet. Remote CI has not been observed.

These are already viewed authored simulation conditions. Historical velocity
points do not establish confidence calibration, shape/association correctness,
arbitrary future acceleration, or coverage beyond eight seconds. Both this
feature and its prerequisites remain experimental and disabled by default.
Comparative engineering maturity stays **about 29%** versus Autoware, Apollo and
openpilot; the 30% waypoint remains unmet.
