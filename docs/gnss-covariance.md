# Optional ENU GNSS covariance calibration

The shared driving pipeline accepts a constant horizontal GNSS covariance shape.
It handles unequal east/north precision and correlated errors, including through
acquisition-time GNSS and delayed-map replay. This is an experimental calibration
option. The frozen evaluation passes its 32 ordinary/map driving episodes but
fails all 16 calibrated persistent-bias episodes. It does not qualify stronger
fault stopping or measured GNSS uncertainty.

The separate opt-in [bounded recovery policy](gnss-recovery-guard.md) now repairs
those 16 stopping/accuracy failures without changing covariance calibration.
The covariance-only recordings and no-guard failures below remain retained;
supplying a covariance shape does not implicitly enable recovery protection.

## Configuration and measurement contract

`PipelineConfig.gnss_covariance_shape` is an optional symmetric positive definite
2×2 matrix `M`, expressed in fixed world ENU coordinates. Entries are
dimensionless and its trace must equal two within `1e-12`. Each unchanged raw
`Gnss` record still carries its scalar `variance` in m²; with calibration the
effective horizontal measurement covariance is `R = variance * M`. Thus the
scalar remains the mean horizontal-axis variance. For example:

```json
{
  "gnss_covariance_shape": [[1.0, 0.9], [0.9, 1.0]]
}
```

This example has a 19:1 eigenvalue ratio, with the more uncertain direction along
east+north. It is fixed external configuration, independent of road heading.
There is no per-fix shape, runtime calibration switch, receiver adapter, bias
state or calibration learned from simulator truth.

The original joint innovation gate remains `NIS <= 36`. Accepted measurements
use LDL whitening of `R` and generalized scalar Joseph updates. Each complete
update is atomic: nonfinite/underflowed noise or an invalid computed posterior
returns `Invalid` without advancing observation/acceptance clocks. The existing
pipeline then holds braking until a genuinely new post-fault fix is accepted.
Innovation rejection keeps its original diagnostic ownership and does not
refresh accepted-fix freshness. History snapshots retain immutable calibration;
map/GNSS rewinds keep their original ages, bounds and conflict handling.

Supplying either an initial pose covariance or this GNSS shape publishes the
complete 3×3 `PipelineOutput.pose_covariance` in every tick. Its x/y trace remains
`position_variance`; replay checks all entries. The optional recovery policy
also publishes the full posterior. Omitting all three options keeps the
original scalar arithmetic and omitted JSON fields.

## Simulation boundary and fixed evaluation

`Scenario.gnss_covariance_shape` supplies operational calibration. The separate
optional `Scenario.gnss_noise_shape` controls only observation synthesis and is
never copied or inferred into the operational header. Both validate their own
matrix. The new noise consists of keyed, bounded uniform components transformed
by a Cholesky factor, with covariance `0.02 * gnss_noise_shape` m². The same seed
and acquisition tick produce the same noise in both plants, independent of
LiDAR draws, driving trajectory and filter calibration. Legacy noise generation
is retained when the simulation option is absent.

Every scenario was materialized and hashed before executing the 64-member
cohort. Seeds are 1, 7, 42 and 313, in reference-2d and native RNE dynamic plants.
The two shapes are `diag(1.9, 0.1)` and `[[1, .9], [.9, 1]]`. Both calibrated
and isotropic controls use the same declared initial pose prior
`diag(1, 1, (3*pi/180)^2/3)` and identical acquisition noise.

The 48 calibrated episodes cover straight/curved driving, both map/GNSS arrival
orders, and two persistent-bias worlds. Sixteen paired isotropic controls change
only operational shape availability. Faults have norm 0.8 m, act across
acquisitions in `[8, 11)` seconds, and point along the narrow covariance axis.
The frozen requirements retain RMSE ≤0.2 m, maximum position error ≤0.4 m,
clearance ≥1 m and original deadlines. Bias cases additionally require actual
speed ≤0.05 m/s within `[9.5, 11.4)`, healthy recovery after 13 seconds, and at
least 40% paired maximum-error reduction. The checker also requires coverage
of at least ten rejected fixes in each calibrated bias episode.

## Recorded outcomes and limits

| Evidence | Result |
| --- | --- |
| Ordinary and map driving | 32/32 pass; worst RMSE 0.111479 m, maximum error 0.328520 m |
| Calibrated persistent-bias driving | 0/16 pass the frozen accuracy/stop requirements |
| Paired error-reduction requirement | 0/16 pass; calibration alone does not remove sustained bias |
| Independent joint-matrix history audit | 64/64 recordings, all 28,226 ticks |
| Exact sensor replay | 64/64 new recordings; 190/190 previous recordings retain exact replay bytes |

The independent auditor solves joint matrix gains/inverses and Joseph updates,
and reconstructs the full chronological history rather than using producer
snapshots or its sequential corrections. It checks every covariance entry,
pose, speed, NIS, accepted/observed clock, correction count and map replay event.
Truth is used only to score physical accuracy, clearance, stopping and the
declared observation-noise recipe.

The bias failures are consequential. In reference north-bias seed 7, fixes
arriving at 8.4 and 8.6 seconds are rejected with NIS about 80.3 and 57.9. As
prediction uncertainty increases, the still-biased fix arriving at 8.8 seconds
has NIS about 31.8 and is accepted. The brake hold clears before a physical stop.
Nominal covariance modeling and innovation gating therefore do not establish
sustained-bias protection. Changing this recovery policy needs a separately
defined mechanism and qualification; this change does not tune gates to conceal
the failed comparison.

All first physical trials and failed audit attempts are retained. The auditor
was repaired to allow serialized nested defaults and omitted zero GNSS delay,
and to require delayed-GNSS coverage only when GNSS was actually delayed. Saved
records were re-audited without physical reruns or altered numerical accuracy,
clearance, stopping or reduction gates. CI separately checks ordinary successes,
all reconstruction/replay checks and the retained seed-7 bias failures; that CI
expectation is not a passing overall qualification result.

The authored bounded noise is not a measured receiver model or Gaussian
coverage experiment. The NIS threshold is not assigned a calibrated false-alarm
probability. Physical GNSS covariance calibration, general bias detection and
automotive validation remain open. Comparative maturity stays about **23%**.

## Reproduce

After the documented pinned native engine setup:

```bash
source scripts/env.sh
cargo build --release --locked --bin rustdriving
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_gnss_covariance.py
python3 scripts/check_gnss_covariance.py --output artifacts/gnss-covariance
```

The full checker intentionally exits 1 for the recorded persistent-bias
qualification failures. It freezes all cases before running, preserves failed
trials and publishes `report.json` after each episode. A seed-7 subset uses
`--seeds 7`; `--plant reference-2d` or `--plant rne-dynamic` selects one plant.

[Measurements, failed comparisons and provenance](../assets/gnss-covariance-results.json).
Local raw recordings and all validation attempts are retained under
`artifacts/gnss-covariance-shape-v1/`, with SHA-256 receipts and lossless archive
encoding recorded per file. Some completed gzip recordings were transcoded to
XZ with verification of every original raw byte; earlier storage references
resolve through `lossless-cohort-transcoding.json`. Historical and current
standard outputs remain in verified immutable archives; duplicate mutable
`artifacts/check` streams are vacated to fit this workspace.
