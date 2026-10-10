# Registered-grid RGB-D camera-model comparison

This fixed, viewed-data experiment changes only the operational camera model
used by the [continuous native/multiscale comparison](multiscale-temporal.md).
The official TUM guidance motivates a pinhole model for the already registered
RGB/depth pixel grid. It does **not** supply a newly measured camera calibration.
The first runs improve some root-error classifications but all four branches
still lose tracking and fail complete acceptance.

## Authority and explicit pixel model

[The official file-format page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)
states:

> The color and depth images are already pre-registered using the OpenNI driver
> from PrimeSense, i.e., the pixels in the color and depth images correspond
> already 1:1.

> We recommend to use the ROS default parameter set (i.e., without undistortion),
> as undistortion of the pre-registered depth images is not trivial.

> We already pre-scaled the depth images of all sequences accordingly, so that
> no action on your side is necessary.

The page calls those default intrinsics **uncalibrated**. The
[explicit model](../assets/registered-grid-temporal-v1/camera-model.json) uses
640 × 480 pixels, **fx = fy = 525**, **cx = 319.5**, **cy = 239.5**, axial Z,
5,000 depth units/metre and invalid depth zero. It applies **no additional
undistortion and no additional 1.035 depth multiplier**. This is a documented
baseline interpretation of already registered/pre-scaled data, not recovery of
native infrared/RGB extrinsics or a measured vehicle-camera calibration.

The secondary `pyslam` Freiburg 1 RGB K/D profile remains pinned provenance
only. Its 517-pixel focal model and Brown distortion coefficients do not become
an additional warp of registered depth. Historical adapters and their camera
assumptions remain unchanged. [The authority audit](../assets/registered-grid-temporal-v1/authority-audit.json)
verifies exact official-page snapshot hashes and the three normalized quotes;
it does not establish that this model eliminates trajectory drift.

## Fixed comparison and first results

[The design](../assets/registered-grid-temporal-v1/design.json) was fixed before
the new continuous fits. Its SHA-256 is
`cfd56dd22e2aadd1f3fdaba8b3162a58d19a783d62438d4d5694bbca3ab41ac3`.
The camera-model SHA-256 is
`511cff92d365983efe40fdb90c9336ac1c122ebef6269594a07081b4eda8ecf3`.
The only changed operational factor is the registered-grid K. Native extraction,
multiscale quotas, depth checks, matching, registration/refinement defaults,
accepted-reference composition, clocks and accuracy gates remain fixed.

Both already viewed recordings retain original depth indices **100–279**, one
initialization, **180 frames and 179 updates** per branch. Root gates remain
**0.1 m / 0.1 rad**; RGB/depth and reference brackets remain at most **0.02 s**.
Expiry beyond **0.20 s** latches loss, repeated acquisitions never renew tracking,
and no reset or recovery is allowed. Numerical reference poses are interpreted
only after all operational fits.

| Recording / branch | Prior accepted / root-accurate | Registered-grid accepted / root-accurate | First loss |
| --- | ---: | ---: | ---: |
| Room / native | 58 / 52 | 58 / 56 | 164 |
| Room / multiscale | 92 / 49 | 92 / 54 | 199 |
| Desk2 / native | 16 / 16 | 16 / 16 | 127 |
| Desk2 / multiscale | 19 / 15 | 19 / 17 | 131 |

Every denominator remains **179**, including all rejections. Both recorded
evaluations exit **1**, and every branch's complete criterion remains false.
Accepted counts and loss indices are unchanged; this result does not repair
continuous availability.

| Maximum accepted root errors | Translation | Rotation |
| --- | ---: | ---: |
| Room / native | 0.102196 m | 0.097711 rad |
| Room / multiscale | 0.189739 m | 0.130158 rad |
| Desk2 / native | 0.087243 m | 0.088855 rad |
| Desk2 / multiscale | 0.115328 m | 0.100275 rad |

Room/native maximum position error increases from 0.098846 m to 0.102196 m,
even though more updates meet both accuracy gates. The camera change does not
improve every error measure.

The [room report](../assets/registered-grid-temporal-v1/room-report.json.gz) and
[desk2 report](../assets/registered-grid-temporal-v1/desk2-report.json.gz), with
their [room freeze](../assets/registered-grid-temporal-v1/room-freeze.json) and
[desk2 freeze](../assets/registered-grid-temporal-v1/desk2-freeze.json), preserve
the first outcomes. The independent [room audit](../assets/registered-grid-temporal-v1/room-audit.json)
and [desk2 audit](../assets/registered-grid-temporal-v1/desk2-audit.json) pass
integrity, retaining all 180 frames and 179 updates in each recording.
The [complete regression replay](../assets/registered-grid-temporal-v1/regression-suite.json)
reproduces both full reports apart from timing, with unchanged source/protocol
bindings and the expected evaluation exits 1. Audits exit 0. Source authority,
input/model integrity and reproducibility do not turn failed pose acceptance
into success.

## Reproduction interfaces

Follow the explicit [room acquisition](temporal-recorded-motion.md#reproduction)
and [desk2 acquisition/provenance](independent-recorded-motion.md#regression-reproduction)
instructions. Use the optional RGB-D environment with pinned
`scripts/requirements-visual.txt` dependencies. Raw recordings remain ignored.
The commands below select existing verified inputs and new output files:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-rgbd-registered-grid-temporal
grid_binary=integrations/rgbd/target/release/rustdriving-rgbd-registered-grid-temporal
grid_manifest=data/tum-fr1-room-temporal/manifest.json
grid_raw=data/tum-fr1-room-temporal/raw
grid_run=artifacts/registered-grid-temporal/local-room
mkdir -p "$grid_run"
"$grid_binary" --prepare-freeze --manifest "$grid_manifest" \
  --output "$grid_run/freeze.json"
"$grid_binary" --manifest "$grid_manifest" --raw "$grid_raw" \
  --freeze "$grid_run/freeze.json" --output "$grid_run/report.json"
python3 scripts/check-registered-grid-temporal.py \
  --report "$grid_run/report.json" --manifest "$grid_manifest" \
  --raw "$grid_raw" --freeze "$grid_run/freeze.json" \
  --output "$grid_run/audit.json"
```

Preserve the expected evaluator exit 1 and continue to audit its retained report;
an unhandled `set -e` would skip that audit. Select desk2's manifest/raw paths
and another output directory for its comparison. Use the actual binary path
when `CARGO_TARGET_DIR` is configured. The tested wrapper runs both complete
recorded regressions and their independent audits:

```sh
python3 scripts/check-registered-grid-temporal-suite.py \
  --binary "$grid_binary" \
  --output artifacts/registered-grid-temporal/local-regression
```

The output directory must be fresh. Wrapper exit 0 means it reproduced both
failed physical results with passing integrity, not that tracking succeeds.

No raw pixel warping, second depth correction, source-report replacement,
automotive generalization, calibrated covariance or RNE driving integration
is claimed. More accurate viewed classifications remain narrow evidence;
maturity stays **about 20% subjectively**, with **30% and 50% unmet**.
