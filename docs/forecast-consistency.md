# Measured forecast consistency and braking

`forecast_consistency_guard: true` adds an optional measured-model consistency
guard to scenarios and the sensor-only `PipelineConfig`. It requires both a
declared `forecast_envelope` and `enclose_track_measurements`; otherwise configuration
validation fails. The guard stays **off by default**. It detects a later acquired
proxy outside an earlier forecast and applies braking. It does not estimate or
repair the forecast's uncertainty budget.

## Observation and command contract

`ForecastConsistencyGuard` belongs to the independent prediction crate and uses
only core contracts. Its bounded history retains at most 128 identities, 16
acquired references per identity and 64 knots per forecast. Each reference binds
the original acquisition stamp, control-time forecast origin, positions and radii.
Measurements and references share continuous wheel/gyro coordinates; absolute
localization corrections cannot move just one side of the comparison.

For a new acquired disc, the guard selects the newest retained forecast from a
strictly older acquisition whose control origin is at or before the acquisition
and whose recorded horizon covers it. Linear interpolation supplies the center
and occupied radius. Center distance plus measured radius must not exceed the
old occupied radius, apart from a 1e-8 m numerical tolerance. A reference is never
extrapolated beyond its horizon. New identities and acquisitions without a usable
reference are counted as **unmonitored**, including cold startup.

A miss adds `InconsistentForecast` health and applies the existing finite emergency
command in the same control call: -6 m/s² acceleration and zero steering. Sensor
or localization faults prevent new references and reset recovery confirmation.
Cached/rejected scans cannot renew geometry or contribute a vote. Navigation,
signal, stop and intersection faults retain their own command duties; monitor
recovery alone cannot clear them.

Recovery requires comparable, contained measurements for every active identity
and every identity responsible for the hold. At least three distinct acquisitions
must span 0.2 s, with no gap above 0.35 s, and their acquisition stamps must be
strictly later than the fault's control-time cutoff. At 20 Hz this can require
five observations. Delayed pre-fault evidence cannot clear a post-fault hold.
Disappearance, track expiry, replacement identity and an empty scene do not clear
a responsible identity. Such a hold can persist until a new externally initialized
localization epoch. Uncovered motion discards references and requires reset;
capacity and clock discontinuity also fail closed.

`PipelineOutput.forecast_consistency` records `Observing`, `Holding`, `Confirming`
or `ResetRequired`, fault/cutoff/evaluation/acquisition clocks, comparable and
unmonitored counts, held identities, confirmation support and boundary misses.
Centers use continuous coordinates. Counts and witnesses describe the last
evaluated acquisition and persist through cached calls. `complete_scene_check`
means every required identity was comparable, not that there were no misses.
Disabled mode omits the flag and diagnostic field. Planning diagnostics remain
the final output field.

## Fixed qualification and retained failures

The preceding 52 standalone enclosure conditions include 20 without declared
forecast error budgets. An initial all-condition preflight accepts 32 supported
configurations and rejects those 20 before any physical execution. No budgets are
added to make unsupported configurations pass. Final qualification changes only
the guard flag on the 32 supported parent inputs across both plants. Original
source goals/deadlines, 1 m clearance, 0.1 m road reserve, localization accuracy
and 99% physical forecast coverage remain scored, including startup.

An independent Python audit reconstructs old references, continuous geometry,
radial boundary witnesses, acquisition/control clocks, held identities and
confirmation without using producer diagnostics as state. Complete Rust sensor
replay separately verifies perception, tracking, transport and outputs. Only the
physical scorer reads actor truth. Sustained holds are scored after the frozen
allowance of 1.5 s plus entry speed divided by 3 m/s²; shorter intervals remain
explicitly censored for physical standstill, with every command still checked.

| Result | Both plants, 32 supported conditions |
| --- | --- |
| Monitor, command and physical hold contract | 32/32 pass |
| Independently checked control ticks | 35,952 |
| Comparable acquired measurements / unmonitored | 12,270 / 32 |
| Measured misses / affected conditions | 60 / 24 |
| Fault-braking control ticks | 10,053 |
| Confirmed recovery events | 31 |
| Sustained / physically censored hold intervals | 11 / 31 |
| Actual speed after sustained-hold allowance | 0 m/s in all 9,050 scored samples |
| Physical collisions / road violations | 0 / 0 |
| Minimum physical clearance | 2.916569 m |
| Goal completion, parent → enabled | **16 → 5** |
| Conditions with newly failed strict gates | **12** |
| Physical forecast coverage failures, parent → enabled | **27 → 28** |
| Overall strict qualification, parent → enabled | **0/32 → 0/32** |

All 11 lost goal completions remain source-acceptance failures; six release-goal
checks and one new coverage failure also remain failed, with overlap across
conditions. Eleven episodes end holding an identity that is no longer observed.
Thirty-one successful recoveries do not establish safe absence handling. The
eight original incomplete missions still miss their goals. **Regression and
adoption qualification fail.** The option has an explicit availability cost and
is not enabled in ordinary scenarios.

The read-only preceding-recording scan is separate evidence: among these 32
supported conditions it checks 12,173 acquired proxies and finds 52 misses in 24
conditions. The broader 52-condition scan includes unbounded forecasts and must
not be presented as supported calibration or intervention results. Changing
control changes future observations, so these counts need not match.

## Reproduction and limits

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_forecast_consistency.py
# Fresh directory; explicit contract check retains failed adoption gates.
python3 scripts/check_forecast_consistency.py --check-contract --output artifacts/forecast-consistency
# Without --check-contract the retained regression failures give a nonzero exit.
bash scripts/check.sh
```

The independent acquisition audit supports planar LiDAR recordings. Changing-route
and mapped-duty guard combinations have not received this physical qualification.
Unit tests cover delayed acquisitions, cached delivery, incomplete scenes, lost
identities, upstream faults, independent navigation faults, motion reset, invalid
geometry, capacity, bounded history and rigid frame changes. Corruption controls
reject changed witnesses, clocks, counts, early release, removed braking and
missing capture.

The unchanged-source `bash scripts/check.sh` passes 892 Rust tests, 183 Python
tests and all 49 standard drive/replay cases; all 245 standard output files retain
exact bytes. Native formatting, 65 release tests and strict all-target Clippy pass.
All 464 supported preceding logs preserve exact replay bytes across 284,066 ticks.
All 160 physical evidence archives have encoded and decoded SHA-256 verified.
CI includes independent unit/corruption checks and both plant contract runs;
remote results for this branch remain unverified.

The [result packet](../assets/forecast-consistency-results.json) retains frozen
inputs, failures, source/binary/lock fingerprints and archive receipts. A contained
measured proxy can still miss the physical body; missed detections, association
errors, calibrated future coverage, safe disappearance handling and useful mission
completion remain open. This does not qualify road operation, real-time performance
or general cross-platform behavior. Comparative maturity remains **about 29%**,
with the 30% waypoint unmet.

The subsequent [complete-sweep recovery](clear-sweep-recovery.md) adds explicit
completed ray outcomes and stopped synchronous confirmation. Its 32 enabled
conditions restore goal completion from 5 to 16 and remove all eleven final
disappearance holds without new failed gates against this guard baseline. The
original forecast and mission failures remain scored, and default adoption fails.
