# Recovery from acquired complete empty planar sweeps

`clear_sweep_recovery` optionally permits a measured-forecast hold to recover
after a stopped, newly observed empty scene. It requires the existing forecast
consistency guard, declared forecast budgets and measured-disc preservation.
The option stays off by default and supports planar acquisition only.

## Acquired sensing contract

An ordinary `LidarScan.points: []` still supplies no negative sensing evidence.
The optional `LidarScan.sweep` records one outcome per commanded ray:

- `Clear`: the physical ray query completed without a surface return in range.
- `Return(index)`: this ray owns that original measured body-frame point.
- `Unavailable`: incomplete query, dropped return or missing acquisition.

Every return must own exactly one original point and match its commanded bearing
and calibrated range. Duplicate owners, unowned points, absent ray slots,
unavailable rays and mismatched calibration reject acquisition before map
correction or tracker acceptance. Explicit failures cannot become clear sensing.
The configured angular pattern must cover the full azimuth within bounded memory.
Unannotated legacy scans retain their preceding validation and serialized schema.

The reference backend records outcomes inside its actual 720 first-return queries,
with no extra random draws. The native adapter records the actual checked physics
queries before radiometry/noise can discard a return. A queried surface without
a delivered point becomes `Unavailable`. It does not infer successful no-return
queries by filling gaps in the returned cloud. Both adapters retain their original
measured points, clocks, noise, plant and sensor settings.

The patterns differ: reference beams advance from -π in steps of 2π/720; native
beams run in the opposite body-frame direction from π with an inclusive endpoint,
in steps of -2π/719. Both declare the actual 0.2–45 m range and 0.05 m return
validation tolerance. Capture alone is selected with the simulation-only
`record_planar_sweep` flag. It never puts actor identities or world truth into the
pipeline configuration. Recovery automatically requests capture in both adapters.

## Conditional resolution and release

The qualification declares a checked center radius of 40 m and a minimum resolved
disc radius of 0.4 m. Calibration checks include the cyclic seam, blind near range,
far range and the maximum transverse beam gap: `R sin(max_gap / 2)` must be below
the minimum disc radius. The reference bound is about 0.174532 m; the native bound
uses its actual 719-interval gap. This is an ideal opaque external circular-body
model intersecting the sensing plane. It does not establish physical body shape,
return probability, missing-surface detection or future-motion uncertainty.

Only an accepted sweep with **every ray clear** can contribute. Any return vetoes
this recovery basis, even if clustering would discard it. The acquisition and
wheel/gyro measurement must both have the current control stamp, measured speed
must be at most 0.2 m/s in magnitude, and upstream sensor/localization health must
be clear. Delayed scans, cached wheel readings, moving observations and scans at
or before the post-fault cutoff cannot contribute.

Three distinct qualified acquisitions must span at least 0.2 s without gaps above
0.35 s. The confirmation count saturates at three; 20 Hz requires five acquisitions
to establish the span. Positive measured-disc confirmation and negative empty-sweep
confirmation have separate support windows and cannot be combined. Any new return
or sensor fault discards incomplete empty-sweep confirmation. Independent mapped
and navigation faults retain their own command obligations after monitor release.

After confirmation the monitor records the retired responsible identities and
clears its forecast hold. It does not clear tracks, predictions, planner stopping
duties or independent faults. Ordinary collision checks still consume retained
predictions until their normal tracking lifetime ends. Without this explicitly
enabled sensing basis, a disappearing identity continues to hold as before.
Uncovered motion still requires a new localization epoch.

The optional `forecast_consistency.clear_sweep` diagnostic retains current
confirmation support, last qualified resolution evidence, a bounded last-release
witness and a saturating release count. The ordinary `complete_scene_check`
continues to mean comparable measured proxies; empty-sweep recovery does not
fabricate a successful positive comparison. Disabled output omits the new fields.
Rust callers constructing legacy `LidarScan` literals supply `sweep: None`.

## Physical qualification

The fixed cohort pairs capture-only and recovery-enabled versions of all 32
preceding supported forecast-guard inputs, in both plants. It preserves original
worlds, sensing/noise, forecast budgets, cold acquisitions, 65-second mission
deadlines and every physical/accuracy/coverage gate. Twenty earlier configurations
without forecast envelopes remain outside this guard's supported qualification.

Capture controls compare exact operational output bytes, complete physical frames,
summaries and original sensor values with the preceding archived runs. An
independent Python audit reconstructs ray/point ownership, cyclic angular gaps,
stopped acquisition admission, positive/negative confirmation, identity retirement
and braking. It never uses producer diagnostics as its state. Complete Rust replay
separately verifies transport, perception, tracking and every control output.
Physical truth is used only by the unchanged evaluation scorers.

All 32 capture comparisons preserve exact operational bytes, complete physical
frames/summaries and original sensor values. All 32 enabled monitor/brake/recovery
contracts pass without newly failed gates relative to the preceding guard.

| Result | Capture only | Empty-sweep recovery |
| --- | --- | --- |
| Physical conditions | 32 | 32 |
| Complete control ticks | 35,952 | 28,054 |
| Goal completions | 5 | **16** |
| Conditions ending with an unobserved held identity | 11 | **0** |
| Measured forecast misses | 60 | 60 |
| Fault-braking control ticks | 10,053 | **504** |
| Positive measured-confirmation recoveries | 31 | 31 |
| Complete-empty-sweep recoveries | 0 | **11** |
| Acquired sweeps / rays | 17,990 / 12,952,800 | 14,039 / 10,108,080 |
| Original returned points bound to rays | 196,428 | 196,428 |
| Minimum physical clearance | 2.916569 m | 2.916569 m |
| Minimum physical road margin | 0.639097 m | 0.639097 m |
| Physical collisions / road violations | 0 / 0 | 0 / 0 |
| Source-acceptance failures | 19 | **8** |
| Clear-path release-goal failures | 6 | **0** |
| Physical forecast-coverage failures | 28 | **28** |
| Strict qualification | 0/32 | **0/32** |

Five repaired goals use the reference plant and six use native dynamics. All 32
enabled episodes end with the monitor observing rather than holding. The eight
original incomplete mission conditions remain incomplete. Relative to the earlier
standalone enclosure inputs, one inherited forecast-coverage regression remains;
those earlier controls had 27 coverage failures. **Default adoption still fails.**
No error budget, cold acquisition, physical clearance or goal deadline was waived.
Repeated ticks and rays are correlated samples from these authored conditions.

A separate post-hoc physical audit checks every one of the 70 qualified empty
acquisitions against recorded actor centers and all 11 release events against
actual ego speed. All actor centers are outside the declared 40 m region, and
actual speed at every empty-sweep release is 0 m/s. These checks use evaluator
truth only and add no input tuning or replacement qualification gates. With
recovery enabled all 42 braking intervals end before the older sustained-hold
allowance; they remain explicitly censored for that separate standstill score.
The actual release-speed audit supplies the narrower observed stopping evidence.

The final-source `bash scripts/check.sh` passes 910 Rust tests, 194 Python tests
and all 49 standard drive/replay cases; all 245 standard output files retain exact
bytes. Native formatting, 67 release tests and strict all-target Clippy pass.
All 496 preceding supported logs preserve exact replay bytes across 320,018 ticks.
After correcting only a new native validation fixture's duration, all 32 native
physical cases were reexecuted; every one of their 160 decoded streams matches
the preceding execution exactly. All 480 archives across 96 retained executions
have encoded and decoded SHA-256 verified; the final matrix contains 64 conditions.
CI requires both the acquisition/recovery contract and no newly failed gates
against the preceding guard in each plant. Remote results remain unverified.

The [result packet](../assets/clear-sweep-recovery-results.json) retains every
capture/control result, physical failure, source/binary fingerprint and archive
receipt. This feature addresses the preceding monitor's availability regression;
calibrated forecasting and the eight original incomplete missions remain open.
It does not qualify vehicle, real-time or general cross-platform operation.
Comparative maturity stays **about 29%**, with the 30% waypoint unmet.

## Reproduction

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_clear_sweep.py
# Fresh output directories; original failed physical/coverage gates stay scored.
python3 scripts/check_forecast_consistency.py --check-contract --output artifacts/forecast-baseline
python3 scripts/check_clear_sweep_recovery.py --baseline-output artifacts/forecast-baseline --output artifacts/clear-sweep-recovery
bash scripts/check.sh
```
