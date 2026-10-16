# Geometry-preserving projective XYZ fusion

The maintained Rust tracker now has an explicit **`geometry: "vertices"`** mode
that retains measured XYZ through reference-view changes, fusion and pyramid
reduction. It prevents a demonstrated plane deformation in the original
depth-only representation. **It is not adopted as the default:** continuous
root accuracy improves slightly in room and substantially regresses in desk2.

## Complete recorded outcomes

Both modes use the original viewed TUM depth indices 100–279, K=525 calibration,
first-camera optical root, 0.1 m / 0.1 rad accuracy gates and acquisition clocks.
Each accepts all 179 updates without expiry or resets. Numeric pose labels are
read only after every operational outcome in each recording is saved.

| Model | Room accurate / 179 | Desk2 accurate / 179 | Maximum position, room / desk2 (m) | Maximum rotation, room / desk2 (rad) |
|---|---:|---:|---|---|
| Original depth | 64 | 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| Preserved XYZ | 67 | 85 | 0.299737 / 0.192257 | 0.141437 / 0.347606 |

The unchanged adoption rule requires room above 64, desk2 at least 161, complete
tracking and no worse maximum position/rotation error in either recording.
The XYZ mode fails this rule. The camera-only default reproduces **all 360
original outcomes exactly**. The [source-bound packet](../assets/projective-vertices-v1/summary.json)
and [validation receipt](../assets/projective-vertices-v1/validation.json) retain
the source, full trajectories, raw audit and failed adoption decision.

## The geometric correction

The original model projects a point into a rounded target cell and retains its
Z value. On the next use, X and Y are reconstructed on that cell's center ray.
The resulting point can leave its measured plane. Repeating this resampling
also changes the geometry used to compute normals and coarse correspondences.

In XYZ mode, target cells index visible points; they do not define their rays.
The model keeps the exact transformed point selected by the existing nearest-Z
buffer, with first row-major index winning exact ties. Compatible observations
average all three coordinates with the existing weight cap of 16. Filling,
nearer replacement and farther rejection retain their original Z-band rules.
Pyramid cells average four supported XYZ points; cross products use their actual
neighbor positions. The same XYZ reduction prepares each measured source frame.
Projection thresholds, Huber loss, iteration/support/condition/increment guards,
keyframe thresholds, root composition and atomic failure/expiry rules remain
unchanged. No dependencies or Cargo pins change.

A separate known-plane control applies 40 prescribed view changes and returns to
the initial view. Intermediate plane RMS reaches **0.0149077 m** for the old
depth representation and **2.22e-15 m** for XYZ. At the final returned view,
the respective RMS values are 5.55e-5 m and 2.11e-15 m. The modes retain 12,288
and 10,630 points respectively, so their visible supports differ. This control
checks geometric preservation under supplied transforms; it is not an estimated
physical trajectory or an explanation of every observed drift error.

All XYZ pyramid levels also retain a tilted plane, and hand controls cover
off-center point retention, nearest-depth/index ties and three-coordinate
capped fusion. A rejected update preserves the XYZ model and accepted timestamp.

## Interface and independent checks

`TrackerConfig.geometry` accepts `ModelGeometry::Depth` (default) or
`ModelGeometry::Vertices`. JSON `Configure` accepts `geometry: "depth"` or
`"vertices"`. Missing fields retain legacy behavior. XYZ currently requires
the original point/forward/target-plane/central-difference policies; combined
experimental policies reject before configuration acknowledgment.

The optional crate passes **32 tests**. Four existing ray-cast motion controls
pass their original 4 mm / 0.004 rad gates. The separate NumPy oracle reconstructs
all **6,912 fitted equations, 358 returned-pose equations and 358 prescribed model
generations** from the raw depth PNGs, without reading reference poses. It checks
all reported systems, counts and fusion partitions. Seven analytic mutations,
including substituting depth geometry for XYZ, are rejected. Operational Rust
and independent reconstruction sources are archived separately from historical
versions. These are viewed indoor development results, with no held-out or
vehicle-localization qualification.

Transport checks also repair an acceptance-test gap: an invalid `Configure`
must reject before any acknowledgment. An exit code of 2 alone could instead be
caused by reaching EOF without frames after erroneously accepting configuration.
A deliberately faulty wrapper strips an unknown geometry value, obtains a valid
acknowledgment and then exits 2 at EOF. The strengthened checker detects this
mutation. Duplicate configuration, malformed JSON and frame-count controls
verify their exact allowed output prefixes. CI runs these controls and raw XYZ
analytic reconstruction without downloading recordings.

Core verification retains 371 Rust tests, 49 scenario replays and all 245 prior
deterministic output hashes. Subjective maturity remains about 20% versus mature
driving OSS; the geometric correction and test repair do not establish a
continuous localization accuracy gain.

## Reproduction

Use the [original inputs and environment](projective-depth-rust.md#reproduction),
build the standalone crate and choose fresh output paths:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --geometry vertices --output artifacts/xyz-recorded
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --geometry vertices --output artifacts/xyz-analytic
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --geometry vertices --analytic-reports artifacts/xyz-analytic \
  --recorded-reports artifacts/xyz-recorded --output artifacts/xyz-raw
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth-comparison.py \
  --baseline artifacts/projective-reprojection/first-point \
  --candidate artifacts/xyz-recorded --raw-audit artifacts/xyz-raw \
  --output artifacts/xyz-adoption.json
```

The evaluator and adoption checker save the complete failures and exit 1.
The baseline path refers to the locally retained original run; another checkout
can reproduce its canonical outcomes using the archived legacy evaluator and
the default current binary. The packet includes both scored and pre-label
sensor ledgers; the comparison checker binds the baseline to the original native
ledger and binds its raw audit to the exact candidate freeze and ledger hashes.
