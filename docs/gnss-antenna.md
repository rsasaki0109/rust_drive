# GNSS antenna mounting calibration

The shared EKF can fuse a horizontal GNSS position measured at an antenna
displaced from the estimated vehicle pose origin. Supply explicit mounting
calibration in body forward/left meters:

```json
{"gnss_antenna_offset_m":{"x":1.2,"y":0.35}}
```

The displacement is relative to the same body reference used by `initial_pose`
and the plant's odometry calibration. Both components must be finite and the
planar mounting radius at most 10 m. Invalid calibration is rejected before
driving. `Ekf::with_gnss_antenna_offset` also exposes the calibration independently
of the pipeline, without changing existing constructors.

## Observation and history contract

For body pose `(x, y, yaw)` and fixed mounting vector `a`, the observation is
`h = [x, y] + R(yaw)*a` in world ENU. With rotated displacement `(dx, dy)`, its
Jacobian is:

```text
H = [1  0  -dy]
    [0  1   dx]
```

The innovation covariance is `H*P*H' + R`. The existing joint NIS threshold
36 applies to this antenna observation. In particular, yaw uncertainty affects
the measured antenna position and cannot be discarded by merely subtracting
a rotated offset and treating the result as a direct body-position measurement.
The optional GNSS covariance shape remains in world ENU; per-fix scalar variance
still scales that shape.

The producer whitens the two observation rows and applies scalar Joseph updates
using one frozen prior linearization. An unwrapped tangent correction accumulates
both rows before the final pose/yaw update. This preserves the position/heading
cross covariance, including when yaw crosses ±π. Failed numeric updates are
atomic. Duplicate observations and innovation rejections retain the existing
freshness/count ownership.

Acquisition-time GNSS rewind evaluates this observation at the acquisition pose.
Filter/history clones retain the mounting calibration through synchronous and
delayed-map replay. The existing recovery policy compares complete **body-pose**
proposals after replay, then withholds or commits them normally. Its position/yaw
bounds do not become antenna-position bounds.

An absent or null mounting calibration retains the legacy correction path and
omits the new field. Explicit zero mounting uses the legacy filter math, with
bit-identical state/covariance checks for isotropic and correlated observations.
Like the other explicit localization calibrations, a supplied mounting vector
enables the full `pose_covariance` output. Consequently explicit zero and absent
calibration can have different serialized headers/posterior fields.

## Physical synthesis and independent checks

`Scenario.gnss_antenna_mount_m` separately specifies the simulation's physical
antenna location. Both plants synthesize a fix from the **acquisition** body pose,
that physical mount, and the existing keyed ENU noise/fault schedule. The physical
mount stays out of the operational header. It never supplies or overrides
`gnss_antenna_offset_m`; deliberate missing/wrong calibration remains possible.
The native integration test checks this separation using unequal physical and
operational vectors and a nonzero physical spawn heading.

The independent Python filter forms the full joint matrix gain and Joseph
posterior instead of the producer's whitened scalar updates. Unbounded
chronological GNSS/map reconstruction checks pose, every covariance entry,
original NIS, accepted/observed clocks and counts, map priors and replay counters.
Raw-sample checks independently rotate the physical mount at acquisition time.
Truth enters only observation synthesis and physical scoring.

Closed-form tests check the position/heading posterior and innovation gate;
rotated-world controls check covariance transformations across yaw wrapping.
Corruption controls reject missing cross covariance even with an unchanged
position trace. Default and explicit-zero library paths retain exact values.

## Fixed evaluation protocol

All 88 scenario files and their hashes are fixed before the first cohort run.
The main 72 positives use two mounts, seeds 1/7/42, both reference/native plants,
and six existing straight, curved, persistent-bias and delayed-map worlds.
Eight additional confirmations use 809/2027 on curved and delayed-map worlds.
Eight paired controls retain the same physical mount, noise, prior, route and
timing while omitting only operational mounting calibration.
The additional seed values were used in earlier unmounted bias experiments;
these are new mounting/world combinations, not an independent data holdout.

The previous explicit ±3-degree startup prior and ENU noise shapes are retained.
Positive requirements are RMSE ≤0.2 m, maximum position error ≤0.4 m, yaw error
≤0.1 rad, clearance ≥1 m and the original goal/deadline. Bias compositions retain
the previous 0.8 m fault over `[8, 11)`, actual stop ≤0.05 m/s during `[9.5, 11.4)`,
healthy recovery after 13 s and the same bounded recovery calibration. Each of
the eight pairs additionally requires ≥40% maximum-error reduction. Comparison
outcomes remain separate from qualified positive behavior.

## Recorded outcomes

All **80/80 calibrated positives** pass the fixed physical, accuracy and original
deadline gates on their first cohort execution. All **24/24 persistent-bias
compositions** reach an actual stop of **0 m/s**, recover on healthy observations
and reach the goal. The eight no-calibration comparisons remain in the report;
none meets the positive accuracy requirements. Their maximum position errors
span **1.174798–1.445759 m**. Minimum paired maximum-error reduction is
**71.766%**, against the frozen ≥40% requirement.

Across the 80 positives, worst RMSE is **0.160574 m**, maximum position error
**0.344946 m** and maximum yaw error **0.0354035 rad**. Independent reconstruction
and exact sensor replay pass all **88 recordings / 41,819 ticks**. Native cases
use three independent processes with the unchanged case checker; every case is
executed once, and the complete reference prefix and scheduling receipts remain
recorded. There are no algorithm or acceptance-gate changes after trials begin.

The exact required `bash scripts/check.sh` passes **704 Rust tests**, **50 Python
tests** and all **49 standard drive/replay pairs**. All **245 standard files**
retain their preceding bytes. The separate native release suite passes **63
tests**, strict all-target Clippy and formatting. All **346 previous recordings**
retain exact replay outputs, including earlier failed and expired controls.
Both final runtime binaries match the physical candidates byte for byte; locks,
toolchains and the pinned native engine revision are unchanged.

These are Linux x86_64 observations; other configured CI platforms have not been
locally observed. Historical/current standard streams remain in verified
immutable archives rather than duplicate mutable `artifacts/check` files.
[Complete measurements, failures and provenance](../assets/gnss-antenna-results.json).
The comparative engineering estimate increases modestly to **about 25%**;
the 30% waypoint remains unmet.

## Reproduce and limits

After the pinned engine setup:

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_gnss_antenna.py
python3 scripts/check_gnss_antenna.py --output artifacts/gnss-antenna
```

Use a fresh output directory. `--smoke --plant reference-2d` or
`--smoke --plant rne-dynamic` selects 16 cases per plant. First outcomes and all
five raw files are retained with verified lossless XZ compression, raw hashes
and stored hashes. Source/binary receipts remain under `artifacts/gnss-antenna-v1/`.

This planar model assumes externally supplied, exact, rigid mounting geometry;
it does not estimate extrinsics or their uncertainty. Pitch, roll, vertical
antenna position, delayed odometry and receiver calibration remain outside its
scope. The original default-prior launch-heading failures remain recorded;
automatic initialization is still absent. Simulation results do not qualify
real-vehicle localization or measured uncertainty coverage.
