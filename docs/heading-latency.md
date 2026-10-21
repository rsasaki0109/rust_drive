# Acquisition-time absolute heading

Optional `heading_latency_compensation: true` fuses an acquired ENU body heading
up to 0.2 seconds after acquisition. It requires both `absolute_heading_fusion`
and `gnss_latency_compensation`. The new option stays off by default; the
synchronous heading API and preceding recording defaults retain their behavior.

## Measured-history contract

`GnssHistory::correct_heading_at` reconstructs the measured acquisition prior,
conditions the full pose covariance, then replays the covered wheel/gyro motion
and every admitted later correction. Equal-time ordering remains prediction,
GNSS, heading, then map. Fractional acquisitions split the measured motion
interval. Both GNSS/heading arrival orders and delayed maps use this ordering.

Every replayed correction must pass its original innovation gate. A conflict
with accepted GNSS, heading or map evidence refuses the complete transaction:
pose, covariance, event snapshots, counters and live observation diagnostics
remain unchanged. The pipeline brakes on that refusal. Rejected headings are
never added to the event history. The existing 0.5-second history and bounds of
64 prediction intervals and 64 of each correction remain unchanged.

Heading freshness is still measured from acquisition, with the original
0.35-second stopping limit. Arrival cannot renew that clock or GNSS's separate
0.75-second clock. A heading acquired at or before a latched fault's delivery
time cannot clear it; a later accepted acquisition is required. Duplicate and
old valid samples neither count again nor clear faults. Invalid/future samples,
unsupported delay, missing history and motion discontinuity keep braking.
Heading does not reinitialize an invalid localization epoch.

Both simulator adapters acquire the original independently noisy body-heading
sample before a shared transport queue delays it. Acquisition values, variance
and timestamp stay immutable through missing acquisitions. `heading_sensor`
contains the simulator-only `delay_ticks`; it never enters operational
calibration or the sensor replay header. Runtime fusion only sees the acquired
measurement, declared variance and sensor clock.

## Frozen evaluation

The 26 fixed conditions use both reference and native RNE plants on the same
curved road, fixed off-road object and original 45-second deadline. Eight pairs
change only the compensation flag and diagnostic name. They use seeds 227/229,
physical ±3-degree startup offsets, 50/200 ms heading delivery, and 350/50 ms
GNSS delivery, covering both arrival orders. Two additional enabled conditions
use equal 150 ms delays. Six enabled conditions exercise heading loss, rejected
heading jumps and GNSS loss with stopped holds and resumed goal completion.
Two 250 ms heading controls exercise the unsupported-delay stop.

The first complete cohort has zero internal later-GNSS reapplications: its
5 Hz streams exercise reversed delivery of equal-acquisition observations and
delayed motion, without a later fix already present at heading insertion.
A separately frozen follow-up retains those 26 configurations and adds two
10 Hz heading conditions, seed 251, 200 ms heading and 50 ms GNSS delivery.
Each must actually replay a later-acquisition fix. Both pass, each with 114
GNSS reapplications. The combined evidence has 28 physical configurations and
17,725 ticks; the original paired conditions remain correlated comparisons.

| Fixed measurement | Result |
|---|---:|
| Enabled accuracy/physical/goal conditions | 18/18 |
| Sensor-fault standstill and resumed-goal conditions | 6/6 |
| Disabled/unsupported complete emergency standstills | 10/10 |
| Negative controls meeting the original goal gate | 0/10 |
| Maximum enabled position RMSE / position error | 0.072898 / 0.182414 m |
| Maximum enabled body-heading error, including startup | 0.052466 rad |
| Minimum enabled swept clearance / road reserve | 8.172170 / 2.030969 m |
| All six fault windows, maximum actual hold speed | 0 m/s |
| Full-state / covariance-entry oracle maximum difference | 2.85e-14 / 4.45e-16 |

All physical/control frames remain scored. Positive conditions require position
RMSE at most 0.3 m, maximum position error at most 0.6 m, maximum body-heading
error at most 0.1 rad, swept clearance at least 1 m, road reserve at least 0.1 m
and actual fault-hold speed at most 0.05 m/s. Disabled and unsupported controls
retain their original source/goal failures separately from the required complete
emergency standstill check. Their expected stopping does not qualify those
missions as successful.

An independent Python evaluator reconstructs the complete acquired history
without Rust checkpoints or recorded estimates. It uses dense Gaussian heading
conditioning, checks every full covariance and independently owns delivery
counts, timestamps, NIS, fault latches and replay counts. Rust tests additionally
compose fractional delayed heading with GNSS/map arrival orders and both chassis
references, and verify complete rollback after GNSS or map conflict. Complete
sensor-only replay separately reconstructs the runtime pipeline. Closed-form
and corruption controls reject changed policy, clocks, counts, state, covariance,
required braking and uncovered motion.

The final auditor rechecks all 28 saved recordings with unchanged numerical
outcomes and zero additional physical runs. Earlier auditor sources and reports
remain preserved. The first required command passed its tests and all standard
outputs while three auditor/CI sources changed; its source-closure assertion is
retained as failed. The complete final-source `bash scripts/check.sh` passes
861 Rust tests, 156 Python tests and 49 standard drive/replay cases. All 245
standard outputs remain byte-identical. The native adapter passes 65 tests,
formatting and strict release all-target Clippy. All 348 checked preceding
sensor logs retain exact replay output bytes, and all 140 new physical archives
have encoded and decoded SHA-256 verified. CI includes both physical cohorts;
remote CI for this local branch remains unverified.

## Reproduce and boundaries

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_heading_latency.py
# A fresh output directory is required.
python3 scripts/check_heading_latency.py --output artifacts/heading-latency
python3 scripts/check_heading_latency.py --replay-composition --output artifacts/heading-latency-composition
bash scripts/check.sh
```

The [result packet](../assets/heading-latency-results.json) retains frozen inputs,
source inventories, first checks, complete outcomes and archive receipts.
These authored planar simulations use declared synthetic heading noise and
sensor clocks. Measured noise/cross-sensor correlation, time-offset estimation,
heading bias calibration, localization-reset composition and new physical map
compositions remain open. This does not repair the preceding eight missed
obstacle-negotiation goals or calibrate forecast coverage. Real vehicle,
real-time and cross-platform operation remain unqualified. Comparative maturity
stays **about 29%**; the 30% waypoint remains unmet.
