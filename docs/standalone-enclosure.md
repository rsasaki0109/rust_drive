# Qualify acquired-disc preservation independently

The enclosure checker now accepts the runtime's supported standalone policy:
`enclose_track_measurements: true` with initial-motion confirmation disabled.
Its shared admission check previously required confirmation for this valid
configuration. `--initialization legacy` now selects the matching original
unconfirmed controls. Both header policies must match the frozen input;
another enabled option cannot hide a lost required flag. Omitted false defaults
remain valid, while nonboolean flags fail the independent check.

The Rust driver, native adapter, geometry formula, forecast budgets and defaults
are unchanged. This adds physical simulation qualification for a previously
unit-tested composition. The default checker mode remains `confirmed`.

## Fixed standalone evaluation

All 52 preceding base conditions use the original unconfirmed tracker and only
add acquired-disc preservation. Worlds, sensing, seeds, calibration and deadlines
remain unchanged. They include both plants, missions, narrow passes, released
objects, heading faults, slow/priority crossing and stopped leads. Controls come
from `track-initialization-baselines.json`; confirmed-mode results are never
substituted for these older controls.

| Measurement | Original unconfirmed controls | Standalone enclosure |
|---|---:|---:|
| Conditions without newly failed strict gates | — | 52/52 |
| Conditions satisfying every strict gate | 20/52 | 20/52 |
| Forecast-coverage failures | 29 | 27 |
| Original mission goals missed | 8 | 8 |
| Localization-accuracy failures | 6 | 6 |
| Failed acquired-disc containments | Not recorded by original mode | 0 / 17,056 |
| Confirmed initial-motion fits | Disabled | 0 |

Reference mission seed 7 without heading and native mission seed 7 with heading
now meet every forecast-coverage horizon. Those missions still miss their original
deadlines. Across 160 horizon comparisons, coverage improves in 56, worsens in
6 and is equal in 98; 13 acquisition denominators differ after changed physical
motion. All initial forecasts and all physical/control frames remain scored.

The new cohort contains 52 physical executions and 37,158 control ticks.
Independent boundary witnesses check measured-disc containment and minimum
sufficient radius. A full filter oracle reconstructs ordinary alpha-beta
position/velocity corrections and verifies zero initial-motion confirmations.
Sensor-only replay rebuilds every episode. Physical and 99% coverage gates,
sample floors and original deadlines retain [the combined protocol](measurement-enclosure.md).

The preceding combined mode's 52 recordings are reaudited under the stricter
header checks. Every filter and geometry result is unchanged; this creates zero
physical runs. The two cohorts cover 104 configurations across the same 52 base
conditions, rather than 104 independent worlds. Each mode is compared only with
its matching controls. Identical failure totals do not establish interchangeable
forecasts or better generalization.

Final result rows preserve `initialization_mode` separately from the existing
`initialization` filter diagnostics. The first complete report had overwritten
the specification's similarly named mode field with those diagnostics. The
final audit verifies that every result row reconstructs the same frozen input,
preserving all numeric outcomes and recordings with zero additional physical runs.

The mandatory `bash scripts/check.sh` passes 852 Rust tests, 149 Python tests
and 49 standard drive/replay cases. All 245 standard outputs remain byte-identical.
All 260 new physical archives have encoded and decoded SHA-256 verified.
The unchanged native binary retains its preceding 64-test qualification. The
unchanged reference binary retains its 244 exact replay receipts. Those older
executions are reused, with no new execution claimed. CI adds a declared curved-road standalone control
in each plant; remote CI has not run for this local branch.

The shared manifest retains initial-motion support constants as metadata; they
are inactive with `initialization: legacy`. The zero-confirmation filter audit
and exact false/omitted header admission independently check that distinction.

## Reproduce

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_measurement_enclosure.py
# A fresh directory is required; omit --initialization for the combined mode.
python3 scripts/check_measurement_enclosure.py --initialization legacy --output artifacts/standalone-enclosure
bash scripts/check.sh
```

The [result packet](../assets/standalone-enclosure-results.json) retains source
hashes, frozen inputs, process outcomes, failures and archive receipts.
These authored planar simulations establish measured proxy preservation within
the declared scope. Forecast calibration, unseen geometry, mission completion,
measured automotive sensing and real-time/vehicle operation remain open.
Both options stay off by default. Comparative maturity stays **about 29%**;
the 30% waypoint remains unmet.
