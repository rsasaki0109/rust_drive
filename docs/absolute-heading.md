# Synchronous absolute body-heading observations

`PipelineConfig.absolute_heading_fusion` optionally requires externally acquired
body heading in world ENU. `SensorFrame.absolute_heading` contains `stamp` in
seconds, `yaw_rad` counterclockwise from east (+x), and `variance_rad2` in rad².
Adapters must supply mounting, north-reference and clock calibration. The input
is body orientation, not velocity course, lane direction or an inferred route yaw.
Angles must lie in [-π, π]; variance must be finite, positive and at most π².

The mode is **off by default**. Missing fields remain absent in older JSON,
headers and outputs. A supplied heading with fusion disabled brakes as an
unconfigured input. In the default synchronous policy, every accepted observation must be
synchronous with the current control time within 1e-9 s. Valid duplicate/older
acquisitions do not refresh acceptance; a new delayed/future sample is invalid.
The original increment rejected GNSS history compensation and measured
relocalization. A subsequent [heading-event history](heading-history.md) now
supports GNSS/map replay while retaining synchronous heading. A separate
[acquisition-time heading policy](heading-latency.md) now admits up to 0.2 s
delivery delay. In this original synchronous policy, delayed heading
fusion and a new measured-localization epoch remain unsupported.

## Fusion and failure handling

Each step predicts wheel/gyro motion, corrects GNSS, then corrects heading before
processing the scan. The three-state EKF wraps `observation - body_yaw` to the short arc. Its scalar
innovation variance is `P[yaw,yaw] + variance_rad2`. A normalized squared
innovation above 9 is rejected. Accepted updates condition x/y/yaw through the
full covariance column; Joseph covariance, symmetry and positive definiteness
are checked before committing. This also adjusts position when the prior has
position/yaw correlation. Initial covariance and existing GNSS gates are unchanged.

Separate `HeadingDiagnostics` record observed/accepted stamps, last decision,
innovation and counts. `PipelineOutput.absolute_heading` and full
`pose_covariance` are emitted for the enabled mode. Heading never refreshes GNSS
accepted age or counters and cannot reconstruct a missing rigid-motion interval.

Missing accepted heading for more than 0.35 s causes `StaleAbsoluteHeading`.
Malformed/unsynchronized input and rejected innovations hold braking immediately.
Only a strictly newer accepted acquisition can clear an input/innovation fault;
an old good sample cannot withdraw it. The existing emergency command uses
-6 m/s² requested acceleration and zero steering. Physical stopping still depends
on the declared plant and its measured motion; this is not a vehicle safety claim.

Freshness and innovation gates do not establish sensor correctness. In
particular, a consistent installation/north-reference bias can be accepted under
the startup uncertainty. Two 0.2 rad biased controls fail the 0.1 rad accuracy
gate; the native control also has only **0.609730 m** swept clearance against
the unchanged 1 m floor. Bias estimation and hardware calibration remain open.

## Declared simulation experiment

The optional simulator-only `heading_sensor` specifies variance, period and
half-open dropout/bias windows. Both backends use the same independent keyed
uniform noise whose pre-wrap variance is declared; the EKF uses a local angular
noise approximation. The qualification declares
1e-4 rad² variance (0.01 rad standard deviation), one acquisition every four
20 Hz ticks, and no change to GNSS/LiDAR RNG streams. Neither the synthetic
sensor settings, physical yaw offsets, fault schedules nor truth enter the
operational header. Only the acquired noisy heading does.

The frozen 58-case design contains the preceding 32 fitted-road worlds, twelve
±3° physical-start cases at seed 131, eight required-stop cases, four recovery
goals, and two persistent biased-heading negative controls at seed 171. The
32 base inputs add only the declared sensor and fusion flag. Other geometry,
actors, speed limits, initial covariance and deadlines retain their bytes.
Clearance stays at least 1 m, actual circular road margin at least 0.1 m, RMSE
at most 0.3 m, position error at most 0.6 m and yaw error at most 0.1 rad.
Passing progress 33–37 m still requires at least 2.6 m actual lateral distance.

| Check | Result |
|---|---|
| Five preceding native launch-heading failures | All pass with the additional declared sensor; max yaw error 0.017219 rad in those five viewed cases |
| Base strict qualification | 28/32, previously 23/32 without heading observations |
| New ±3° startup goals / recovery goals | 12/12 / 4/4 |
| Sensor-loss, absent-provider, jump and GNSS-loss holds | 8/8 |
| Initially absent provider | Both plants remain at actual 0 m/s; all 241 commands independently verified as emergency braking |
| All positive strict qualification | 52/56; four inherited missions still miss their 65 s goal |
| Complete integration comparison | **57/58; fails** because reference mission seed 42 adds a forecast-coverage failure |
| Persistent biased controls | Both remain failed; native physical passing/clearance also fail |
| Calibrated-positive max yaw error / min clearance / min road margin | 0.019814 rad / 1.087901 m / 0.198781 m |
| Independent full-state/posterior reconstruction | Maximum state difference 2.85e-14; every covariance entry checked |
| Previous no-heading logs | 64/64 exact replays, 45,474 ticks |
| Disabled native physical sentinels | 4/4, all five streams byte-identical to their parent |

The new reference forecast failure is not waived. Coverage at 1/2/4/8 s is
98.680% / 98.993% / 98.958% / 98.881%, below the existing 99% requirement.
Its original source-goal failure remains too. The other preceding forecast
failures and all four incomplete missions remain scored. Adding heading does
not qualify the experimental forecast budgets or default adoption.

Two first inputs mistakenly requested a blocked-road stop, which requires at
least 10 m progress, for a sensor absent at startup. Those failures remain in
the first report. The two follow-up physical cases use the existing sensor-fault
expectation and omit the provider entirely, retaining duration and every numeric
physical gate. The first follow-up auditor also rejected the lack of active
speed profiles; that error remains recorded. Final audit instead requires every
command to be a zero-steering emergency brake, every heading absent, and actual
speed at most 0.05 m/s at every tick. Corruption tests check that this cannot
accept a moving or normally actuated recording. No new physical trials were
created for that audit correction; the other 56 recordings are reused explicitly.

The [complete result asset](../assets/absolute-heading-results.json) retains
both failed initial stages, all frozen inputs, source/binary receipts and raw
archive hashes. There are **65 actual physical executions**: 58 first cases,
two corrected inputs, one focused development repeat and four disabled controls.
The final matrix has 58 rows, including 56 original recordings and the two
corrected sensor-fault recordings. All 325 execution streams and 895 references
across retained stages pass encoded/decoded hashes. These are authored simulation
regressions with correlated reused worlds, not independent automotive trials.

## Reproduction and limits

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_absolute_heading.py
# A fresh output directory is required. Full integration currently exits 1.
python3 scripts/check_absolute_heading.py --check-integration-regression \
  --output artifacts/absolute-heading
# The native 29-case integration subset passes locally; remote CI is unverified.
python3 scripts/check_absolute_heading.py --plant rne-dynamic \
  --check-integration-regression --output artifacts/absolute-heading-native
bash scripts/check.sh
```

The final mandatory check passes 816 Rust tests, 119 Python tests and 49 standard
run/replay pairs, with all 245 prior outputs byte-identical. The native adapter
passes 64 tests, formatting, release build and strict Clippy. Two earlier checks
were intentionally cancelled to correct the evaluator/source closure; partial
logs and cancellation reasons remain retained. The final check uses test
optimization level 1 with the same required command and production profile.
The native CI definition includes all 29 heading cases and uploads evidence on
failure; remote execution and other platforms remain unverified.

No actual compass/dual-antenna receiver, measured noise covariance, mounting or
north calibration, magnetic disturbance model, heading-bias estimator or vehicle
operation is established. The mode requires synchronous input and assumes
independent calibrated measurement noise. Comparative engineering maturity
remains **about 29%**, and the 30% waypoint remains unmet.
