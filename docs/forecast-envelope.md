# Acquisition-origin motion error envelopes

`forecast_envelope` optionally preserves both the existing nominal motion and
constant-velocity continuation with declared position, velocity and acceleration
error budgets. It is **off by default**. These inputs are assumptions supplied
by the caller, without an estimated covariance, calibrated confidence level or
actor intention. The separate `confirm_slow_motion` option is unchanged.

```json
"forecast_envelope": {
  "position_error_m": 0.2,
  "velocity_error_m_s": 0.3,
  "acceleration_error_m_s2": 0.1
}
```

The optional field belongs to `PipelineConfig` and simulation scenarios, and is
recorded in the sensor-only replay header. The independent prediction crate
offers `ObservedBraking::with_envelope`. The serialized contract is
`rustdriving_core::ForecastEnvelopeConfig`; no new dependency or runtime is
required. All three budgets must be finite and nonnegative, bounded respectively
by 20 m, 15 m/s and 6 m/s². Unknown fields fail deserialization. Omitting the
policy preserves the preceding forecast and serialization behavior. Zero budgets
still preserve the alternative of continuing at the observed velocity.

## Geometry and clocks

For acquired position `p`, observed velocity `v`, acquisition stamp `stamp` and
control time `now`, let `u = now - stamp + future_seconds`. The alternative has
center `p + v*u` and extra radius:

```text
position_error_m + velocity_error_m_s*u + acceleration_error_m_s2*u²/2
```

The acquisition age is consumed on every forecast; repeated control calls or
cached observations cannot restart its budget. Constant-velocity continuation
uses the measured velocity even below the default 0.7 m/s deadband. Existing
observed-braking and unconfirmed-slow occupancy remain included. At each of the
41 existing knots, the new center/radius form the smallest disc enclosing both
the nominal occupied disc and the error-inflated continuation disc. The measured
physical radius is retained, with additional occupancy in
`Prediction.additional_radius_m`. An overflowing envelope produces an empty
forecast and the existing downstream braking behavior.

This covers motion whose acquisition position and velocity errors satisfy the
declared Euclidean budgets and whose acceleration magnitude remains within the
declared acceleration budget, provided its physical radius is covered by the
tracked radius. Between knots, linear interpolation of enclosing discs retains
both affine nominal occupancy and the convex quadratic error tube. The operation
takes place in continuous tracking coordinates before the current world-frame
transform. Circular budgets are invariant under that rigid transform. Planning,
priority-zone checks and lane-change/return/launch previews already consume
the additional radius; no second collision representation is introduced.

Coverage ends at the existing eight-second forecast horizon. The existing
endpoint-hold convention after that horizon does **not** extend this conditional
coverage claim. Actor birth/missed detections, association errors, incorrect
shape radii, ego localization/map errors and budgets exceeded by real motion
remain unresolved. The policy does not calibrate these budgets from data or
establish complete long-term reachability.

## Fixed paired evaluation

The [results](../assets/forecast-envelope-results.json) retain five preceding
authored worlds: the curved mission, slow crossing, stopped/resuming lead,
priority crossing and blocked road. Seeds 7/42 run in reference and native
dynamic RNE plants, with the policy enabled/disabled: forty episodes. Source
geometry, sensing, startup priors, deadlines and existing rules remain unchanged.
Both modes additionally set `full_rate_recording: true`, an evaluation-only
scenario option that retains all 20 Hz physical frames and never enters the
operational header. Its sensor inputs and driver outputs are independently
checked against the preceding sparse recording mode.

The frozen physical floor is 1 m, with zero contacts/road/rule violations and
unchanged scenario outcomes/deadlines. Whole-episode localization gates are
0.3 m RMSE, 0.6 m maximum position error and 0.1 rad maximum yaw error. The
coverage gate requires at least 100 identity/acquisition pairs per horizon and 99%
physical-body coverage at each of 0.2/1/2/4/8 seconds. Each distinct identity and
acquisition contributes once. Association uses the closest acquisition-time
physical center within 2 m and a 0.5 m separation from the next candidate;
unmatched/ambiguous identities remain in the denominator. End-of-record censoring
and subsequent actor absence are reported explicitly. Association truth appears
only in this post-run scorer and cannot command the vehicle. The cohort reuses
viewed authored worlds and provides no held-out automotive calibration.

The arithmetic auditor reconstructs the original nominal prediction from complete
timestamped track history. It independently checks both input-disc containments
and the minimum possible enclosing radius, rather than generating the Rust
center. Adversarial tests reject lost alternatives, omitted or shifted radii,
inflated arbitrary bubbles, shortened arrays, nonfinite budgets, clock resets
and a removed replay policy. Rust tests independently integrate changing
two-dimensional acceleration at 1 ms, sample inter-knot coverage, compose slow
and braking hypotheses, and verify operational braking and exact replay.

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-prediction --test envelope
cargo test --locked -p rustdriving-pipeline --test forecast_envelope
python3 scripts/test_forecast_envelope.py
# Requires built root/native release executables. A failed driving or coverage
# gate returns 1 while preserving every raw recording and replay.
python3 scripts/check_forecast_envelope.py --output artifacts/forecast-envelope
bash scripts/check.sh
```

Use a fresh output directory. Inputs are frozen before driving. All raw JSON
streams are retained as losslessly verified XZ archives with encoded/decoded
SHA-256 receipts. The first source-snapshot attempt exhausted disk by copying
unneeded image assets; only duplicate/incomplete copies were removed, and its
pre-compilation failure log remains retained. Before any physical trial, a
separate recording revision added the explicit full-rate option to both modes;
the unused earlier input manifest and interrupted build attempt remain recorded.

## First fixed results and adoption failure

All forty recordings pass complete sensor replay and independent nominal/envelope
arithmetic: **32,948 control ticks and 31,290 forecasts**. Neither policy has
contacts, road departures or priority violations. Recorded minimum clearance is
1.166301 m for legacy and 2.950875 m for the envelope. These are each policy's
own trajectories and recording lengths; four incomplete experimental missions
mean they do not demonstrate a same-exposure clearance improvement.

| Gate / outcome | Legacy, 20 episodes | Envelope, 20 episodes |
|---|---:|---:|
| Original scenario acceptance | 20 | 15 |
| Goal completion, among 12 goal requests | 12 | 8 |
| Driving plus whole-episode accuracy | 16 | 11 |
| 99% coverage at every prescribed horizon | 0 | 2 |
| Complete qualification | 0 | 0 |

All four experimental curved missions miss their unchanged 65 s deadline. The
reference blocked-road seed-42 run also fails the original stop acceptance.
Four native seed-7 cases in each mode retain the 0.1 rad launch-heading failure.
No case was reclassified as a positive control to remove those failures. The two
envelope cases meeting all coverage requirements are incomplete missions, so
they do not qualify complete adoption.

Physical body coverage, aggregated over each mode's own distinct acquisitions:

| Future horizon | Legacy covered / eligible | Envelope covered / eligible |
|---|---:|---:|
| 0.2 s | 4,664 / 7,512 (62.09%) | 7,424 / 7,951 (93.37%) |
| 1 s | 4,181 / 7,416 (56.38%) | 7,266 / 7,855 (92.50%) |
| 2 s | 3,768 / 7,296 (51.64%) | 7,047 / 7,735 (91.11%) |
| 4 s | 3,090 / 7,056 (43.79%) | 6,661 / 7,495 (88.87%) |
| 8 s | 1,990 / 6,370 (31.24%) | 5,771 / 6,904 (83.59%) |

Each mode retains 48 unmatched acquisitions in every applicable denominator.
Coverage remains below the frozen 99% gate. The largest eight-second shortfall
is 32.117281 m for legacy and 26.229708 m for the envelope. These illustrative
budgets do not cover initial velocity/shape errors and subsequent actor motion
reliably. The worst experimental witness is the reference seed-7 lead at time
zero: a new track reports zero velocity while its physical center moves at
4 m/s, exceeding the illustrative 0.3 m/s velocity-error assumption. A separate
raw-record diagnostic reproduces the reported 26.229708 m shortfall exactly.
The four incomplete missions end stopped at 41.55–41.89 m progress; the failed
blocked-road run still moves at 0.4 m/s at its deadline. Budgets were not enlarged
after viewing results. There was no runtime, world, numerical-gate or
physical/coverage-checker change after the first physical trial.

The feature remains opt-in research. Next work needs independent measurement of
track/shape and motion-model errors, better stationary/moving hypotheses, and
arrival/stop availability alongside coverage. Increasing every radius cannot
by itself satisfy these gates. Comparative maturity stays **about 29%** versus
Autoware/Apollo/openpilot; the **30% waypoint remains unmet**.

The required `bash scripts/check.sh` passes **781 Rust tests, 99 Python tests and
49 standard drive/replay pairs**. All 245 preceding standard output files remain
byte-identical. Native format, locked release build, 63 release tests and strict
all-target Clippy also pass. The final 1,009 operational source hashes match the
pre-trial snapshot; all 200 encoded/decoded physical archives verify. Separate
circle-to-polyline reconstruction confirms every full-rate circular footprint
stays within the supplied road corridor (minimum margin 0.575006 m). Validation
is Linux x86_64 simulation, without real-vehicle, real-time or cross-platform
qualification.

All 44 preceding yield-recovery recordings also reproduce every output and replay
report byte exactly (**59,792 ticks**), including retained historical failures.
This does not reclassify those earlier physical or accuracy failures as passes.
