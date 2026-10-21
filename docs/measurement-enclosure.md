# Preserve acquired object geometry

Optional `enclose_track_measurements: true` preserves the newest measured
detection disc at the corrected track center. It remains off by default.
Ordinary radius smoothing can lose part of a larger or displaced observation,
even when the detector supplied that geometry. This policy repairs that loss
using the minimum sufficient radius, without changing future-motion budgets.

## Geometry and acquisition contract

Let the corrected track center be `p`, the newest measured center/radius `q, r`,
and the preceding track radius `R`. Ordinary smoothing gives `s = 0.8R + 0.2r`.
The new reported radius is:

```text
max(s, distance(p, q) + r)
```

It is the smallest radius at the fixed corrected center containing both the
ordinary smoothed disc and the newest measured disc. Position correction,
velocity, association gates and identity allocation are unchanged for identical
input observations. Shape contraction retains the existing 0.8 decay instead
of keeping an old maximum forever. Other tracks do not supply geometry.

The policy uses acquired detections in the continuous wheel/gyro frame. A rigid
continuous-to-world transform preserves containment through absolute position
corrections. Missing/rejected scans cannot renew a radius or measurement stamp.
Successful empty scans clear raw measurements without refreshing retained tracks;
normal identity expiration and motion-loss invalidation remain unchanged.

The flag can be enabled independently of `confirm_track_initialization`; both
options publish the existing `track_measurements` evidence. With confirmation,
the initial-motion fit still uses raw radii, so an enclosing track radius cannot
restart or otherwise contaminate its stable-shape support. Direct library users
can call `Tracker::default().with_measurement_enclosure()` or apply the builder
to `Tracker::with_confirmed_initialization()`.

This encloses a measured proxy. Partial visibility, a rejected circle fit,
incorrect association or localization error can still miss physical geometry.
The acquisition contract does not establish future object containment or
calibrated automotive uncertainty.

## Fixed comparison and independent checks

The complete preceding 52-condition confirmed-initialization matrix is retained:
both reference and RNE plants, original mission, narrow passing, blocked and
released objects, heading faults, slow/priority crossing and stopped lead.
Seeds, roads, objects, sensing, deadlines and calibration are unchanged; only
the new flag changes. Standalone use without initial-motion confirmation is
covered by Rust and closed-form Python tests; this physical cohort combines
both options.

| Measurement | Parent | New policy |
|---|---:|---:|
| Conditions losing some newest measured disc | 52/52 | 0/52 |
| Failed acquired-disc containments | 11,067 / 17,049 | 0 / 17,054 |
| Conditions without newly failed strict gates | — | 52/52 |
| Every strict gate satisfied | 20/52 | 20/52 |
| Forecast-coverage failures | 27 | 27 |
| Original mission goals missed | 8 | 8 |
| Localization-accuracy failures | 6 | 6 |

The parent maximum containment shortfall is 0.483172 m. The complete new cohort
records 37,164 physical/control ticks. Closed-loop motion changes the number of
observations; the two denominators represent complete episodes, not identical
measurement samples. All initial predictions, unmatched acquisitions and
end-of-record censoring remain scored under the preceding coverage protocol.
Across 160 horizon comparisons, coverage improves in 45, worsens in 11 and is
equal in 104; 14 denominators differ. This does not repair the remaining
forecast calibration or qualify default adoption.

The unchanged physical gates require 1 m clearance, 0.1 m road reserve,
position RMSE/max error at most 0.3/0.6 m and heading error at most 0.1 rad.
Coverage still requires at least 100 eligible acquisitions and 99% coverage at
0.2/1/2/4/8 seconds, with unchanged declared error budgets and original deadlines.
Comparison success does not waive any remaining failed gate.

The independent geometric oracle constructs the measured circle's exact radial
extremum away from the corrected center. Containing that boundary witness and
the smoothed disc proves containment; an active boundary proves the radius is
minimal. The separate full filter oracle reconstructs positions, velocity,
radius, acquisition ownership and initial-motion fits. Rust regressions sample
circle boundaries under rotations/translations and cover shrinking shapes,
invalid batches, cached scans, identity expiration and confirmation composition.
Sensor-only replay independently rebuilds perception, association and controls.

A final audit strengthens explicit unique raw-evidence requirements for new
or renewed tracks. It reuses all 52 original recordings, changing no numeric
result and creating no physical executions. First recordings, source versions
and reports remain preserved in the [result packet](../assets/measurement-enclosure-results.json).

The maximum boundary deficit and nonminimal-radius slack are both below
6.2e-14 m, against the fixed 1e-8 m numeric acceptance tolerance. Full filter
position/velocity reconstruction differs by at most 7.8e-14. All 260 recording
archives have both encoded and decoded SHA-256 verified.

The mandatory `bash scripts/check.sh` passes 852 Rust tests, 145 Python tests
and 49 standard drive/replay cases, retaining all 245 preceding output files
exactly. The native adapter passes 64 tests, formatting, release build and strict
all-target Clippy. All 244 checked preceding logs retain exact replay outputs;
all 52 current episodes also replay completely. CI includes both plants'
comparison jobs; this local branch's remote CI remains unverified.

## Reproduction and limits

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_measurement_enclosure.py
# A fresh output directory is required. Default evaluates both plants.
python3 scripts/check_measurement_enclosure.py --output artifacts/measured-enclosure
bash scripts/check.sh
```

This remains an authored circular-object planar simulation feature. Unseen body
shape, measured sensor calibration, new physical map/reset compositions and
real-time/vehicle qualification remain open. Comparative engineering maturity
stays **about 29%**, with the 30% waypoint unmet.
