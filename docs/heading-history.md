# Absolute heading through GNSS and map history

Enabling both `absolute_heading_fusion` and `gnss_latency_compensation` now
retains acquired synchronous heading observations through acquisition-time GNSS
replay. Both options remain off by default. The combination was previously
rejected during configuration validation. The [fixed result packet](../assets/heading-history-results.json)
records the simulation trials and first audit failures.

The separate [delayed-heading option](heading-latency.md) now also admits heading
delivery up to 0.2 s after acquisition. The original synchronous policy and
fixed evaluation below retain their bounds and results.

## Observation and replay contract

Heading remains acquired body yaw in world ENU, with mounting/north calibration
and a declared angular variance. Heading must be synchronous within 1e-9 s;
this feature does not admit delayed heading. Existing GNSS delay of at most
0.5 s and the explicit map-scan delay of at most 0.2 s retain their limits.
Measured relocalization remains an unsupported combination and is rejected.

The bounded history keeps the admitted GNSS, heading and map observations, with
at most 64 of each and 64 prediction intervals over its existing 0.5 s window.
If count pruning folds a correction into the oldest remaining snapshot, its
time/priority boundary is retained. An insertion requiring an earlier equal-time
prior is unavailable and brakes; a later covered fractional acquisition remains
admissible.
At equal acquisition time, the order is motion prediction, GNSS, heading, then
map. Fractional GNSS/map acquisitions split the measured-motion interval.
Map registration queries see the acquisition-time prior after equal-time GNSS
and heading, before the map correction. No simulator pose or future acquisition
enters this prior.

Every replayed observation must still pass its original innovation gate. A
conflict refuses the whole transaction: pose, full covariance, observation
ownership, event snapshots and replay counters remain unchanged. A GNSS rewind
conflicting with accepted heading produces the existing invalid-GNSS braking
hold, requiring a later valid acquisition. Rejected heading samples never enter
the admitted-event history and cannot be revived by a rewind.

Heading diagnostics retain original-delivery observed/accepted stamps, counts,
decision and NIS. GNSS diagnostics retain their separate ownership. Internal
reapplication cannot renew either accepted age, clear a heading fault, or count
another sensor acquisition. Missing required heading still brakes after 0.35 s;
GNSS still expires after 0.75 s. Only a newly accepted heading can clear its
innovation/input fault. Motion loss keeps the existing invalid-epoch braking.

`gnss_latency.history_heading_corrections` reports retained heading events;
`replayed_heading_corrections` counts internal later/equal-time reapplications.
Zero fields are omitted for compatibility with preceding recordings. Direct
library callers use `GnssHistory::with_retained_corrections()` and must submit
every correction through the history wrapper.

## Frozen simulation evaluation

Twenty-two physical episodes use the same source curved-road scenario, a fixed
off-road circular object, original 45 s deadline and declared chassis/course
calibration. Eight pairs cover both reference and native RNE plants, seeds
203/211, 150/350 ms GNSS delivery and physical ±3° startup offsets. Each pair
changes only the GNSS-history policy and diagnostic name; both include the declared synchronous heading
sensor (variance 1e-4 rad², period four 20 Hz ticks). Six additional enabled
episodes cover heading dropout, rejected heading jumps and GNSS dropout, each
with physical braking and resumed goal completion in both plants.

| Measurement | Result |
|---|---:|
| Enabled strict accuracy/physical/goal cases | 14/14 |
| Naive delayed-GNSS controls passing strict qualification | 0/8 |
| Paired position-RMSE reduction | 92.8–97.3% |
| Maximum enabled position RMSE / position error | 0.062790 / 0.182185 m |
| Maximum enabled body-heading error | 0.015595 rad |
| All six declared fault windows, maximum actual hold speed | 0 m/s |
| Minimum enabled actual road reserve / swept clearance | 2.047982 / 8.192698 m |
| Full-state / covariance-entry oracle maximum difference | 2.85e-14 / 6.67e-16 |

The physical gates remain RMSE ≤0.3 m, maximum position error ≤0.6 m, heading
error ≤0.1 rad, swept clearance ≥1 m, road reserve ≥0.1 m and actual fault-hold
speed ≤0.05 m/s. All frames and all nine covariance entries are independently
reconstructed from complete acquired inputs. The Python oracle rebuilds the full
measured history, uses dense Gaussian heading conditioning, and never reads Rust
history checkpoints. Complete sensor-only replay separately checks the pipeline.

The first auditor rejected all eight naive controls because scenario serialization
omits a false compensation flag. The correction restores only that documented
false default before comparing declared inputs; an omitted true flag and changed
duration/geometry still fail. The original report remains intact. The corrected
audit reuses exactly the same 22 recordings, creating zero new physical runs.
All eight controls retain their actual localization failures, and four retain
source-acceptance failures. Passing the comparison does not qualify those controls.

After the count-boundary guard, the final source reruns all 22 unchanged inputs.
Every one of their five recorded streams is byte-identical to the corresponding
first execution. Total retained evidence is 44 physical executions, 21,124 ticks
and 220 encoded/decoded-hash-verified archives across 22 input conditions. These
repeated conditions are correlated checks, not 44 independent trials. Final
replay also retains exact outputs for 158 preceding logs and 22 current logs.

Rust regressions additionally cover fractional acquisitions, both GNSS/map arrival
orders, equal-time heading/map priors, both chassis references, atomic refusal,
rejected-sample ownership, bounded event storage, stale required heading and
full sensor-log replay. Auditor corruption tests reject changed state, covariance,
stamps, counts and omitted braking. The exact mandatory `bash scripts/check.sh`
passes 834 Rust tests, 131 Python tests and all 49 standard drive/replay cases;
all 245 preceding output files remain byte-identical. The native adapter passes
64 tests, formatting, release build and strict all-target Clippy. Local
qualification receipts are included in the result packet; remote CI is unverified.

## Reproduction and limits

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_heading_history.py
# A fresh output directory is required.
python3 scripts/check_heading_history.py --output artifacts/heading-history
bash scripts/check.sh
```

These are authored planar simulations on a clear curved road with an off-road
object, not new automotive data or obstacle-negotiation qualification. Maps are
composed in the chronological Rust tests; the new physical cohort has no map
observations or relocalization. The separate stop-search matrix remains 20/40
strictly qualified, with its mission, forecast and heading failures retained.
Measured heading/antenna noise, cross-sensor correlation, clock calibration,
heading bias estimation, delayed heading, localization resets, vehicle operation
and real-time/cross-platform qualification remain open. Comparative engineering
maturity stays **about 29%**, with the 30% waypoint unmet.
