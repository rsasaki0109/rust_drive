# Confirmed initial track motion

Optional `confirm_track_initialization: true` separates early shape refinement
from velocity estimation. A partial visible surface can initially produce a
different circle center/radius; differentiating that change previously
manufactured motion for a stationary target. The policy remains off by default
and uses no simulator labels, future observations or truth.

## Acquisition contract

Each identity starts with zero velocity. Streaming centered least squares uses
every admitted raw detection of that identity in the continuous wheel/gyro
frame. At least three observations must span 0.2 seconds. Support restarts after
0.6 seconds or a radius range exceeding 0.12 meters. Confirmation requires
position residual RMS at most 0.12 meters and speed at most 15 m/s. The fit
supplies current position and velocity; ordinary alpha-beta updates then resume.
Pending positions still use the normal position correction and velocity stays
zero. This does not establish that the physical object is stationary.

The 0.12-meter tolerances reuse the existing circle-fit residual scale; these
are declared heuristics without measured uncertainty calibration. The fit does
not detect later shape changes, classify objects or estimate acceleration.
Motion can remain unconfirmed. Storage and update work per pending fit are
constant; no observations inside the current window are downsampled.

Each identity owns its acquisition times. Empty scans and other objects cannot
supply support or renew its age. Existing 0.6-second identity expiration and
association limits remain unchanged. Rejected scans preserve tracks, support,
identity allocation and accepted time atomically. Motion loss clears the scene;
a newly initialized pipeline uses the configured policy with fresh support.

Enabled outputs add `track_measurements`: the last successful scan's raw
associated `id`, `stamp`, `center` and `radius`. Centers use the continuous
tracking frame, unlike published world-frame tracks. Missing/rejected scans
retain the old acquisitions; successful empty scans clear the list. Absolute
pose corrections cannot rewrite these measurements. Disabled logs omit it.

## Fixed evaluation and retained failures

The [baseline packet](../assets/track-initialization-baselines.json) retains all
40 preceding stop-search conditions and 12 preceding envelope conditions:
slow crossing, stopped lead and priority crossing, both plants, seeds 7/42.
The 40 include heading, narrow passing, blocked/released objects and sensing
faults. Each trial changes only the new flag. Worlds, deadlines, acquisition
schedules, calibration and forecast budgets remain fixed.

| Complete comparison | Result |
|---|---:|
| Conditions without newly failed strict gates | 52/52 |
| Conditions satisfying every strict gate | 20/52, unchanged |
| Original mission goals still missed | 8/8 |
| Forecast-coverage failures | 29 → 27 |
| Localization-accuracy failures | 6, unchanged |
| Horizon comparisons improving / worsening / equal | 87 / 15 / 58 |
| Complete physical/control ticks | 37,159 |
| Independently reconstructed confirmations | 58 |

Two reference mission conditions now meet every coverage horizon: seed 7 without
heading and seed 42 with heading. For the former, 0.2-second coverage improves
98.860% → 99.511%, retaining 614 eligible acquisitions; 2/4/8-second horizons
reach 100%. Other short-horizon comparisons worsen, including heading-enabled
seed 7: 98.860% → 98.371%. Its existing failure remains scored.

Coverage still requires at least 100 eligible acquisitions and 99% coverage at
0.2/1/2/4/8 seconds. Initial unconfirmed predictions, unmatched acquisitions and
absent actors remain in the denominator; end-of-record censoring is explicit.
No warm-up exclusion, larger envelope or changed association gate is used.
Denominators differ in 28/160 horizon comparisons because closed-loop motion
changes; every observation in both episodes is scored. Physical gates retain
1-meter clearance, 0.1-meter road reserve, position RMSE/max error 0.3/0.6 meters
and maximum yaw error 0.1 rad. Comparison success cannot waive a failing gate.

An independent Python oracle uses dense batch sums and direct residuals over
complete raw measurements, rather than the Rust streaming recurrence. It checks
positions, velocities, radius, identity/time ownership and cached acquisitions.
Maximum position/velocity disagreement is below 6.7e-14. Raw sensor replay
separately verifies clustering, association and controls; the batch oracle alone
does not establish perception accuracy.

The first collector stopped on null optional coverage in a heading fixture.
Five completed physical recordings remain preserved; each has all five streams
identical to its complete-cohort counterpart. Only two first process-exit
receipts had been saved; the other three exit codes are explicitly unknown.
The complete 52-condition cohort saves every process outcome and replay. There
are 57 physical executions across 52 conditions. The result packet retains
first errors, failed unit receipts, early source variants and recording hashes.

The final auditor also enforces the original 0.1-meter road reserve explicitly
for the twelve forecast fixtures; reauditing the same 52 recordings changes no
outcome and creates no physical runs. Binary-path defaults now handle the Windows
executable suffix; Windows execution remains unverified.

The mandatory `bash scripts/check.sh` passes 843 Rust tests, 138 Python tests and
49 standard drive/replay cases, retaining all 245 previous output files exactly.
The native adapter passes 64 tests, formatting, release build and strict
all-target Clippy. All 192 preceding sensor logs retain exact replay outputs;
every current episode also replays completely. All 285 physical recording
archives have both encoded and decoded SHA-256 verified. CI includes both
plants' comparison jobs; remote CI has not run for this local branch.

## Reproduction and limits

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_track_initialization.py
# A fresh output directory is required; default evaluates both plants.
python3 scripts/check_track_initialization.py --output artifacts/confirmed-tracks
bash scripts/check.sh
```

The [result packet](../assets/track-initialization-results.json) records local
qualification. Automotive sensing, measured uncertainty calibration, general
shapes/association, new physical map/reset compositions, real-time operation
and real vehicles remain unqualified. Comparative maturity stays **about 29%**;
the 30% waypoint remains unmet.
