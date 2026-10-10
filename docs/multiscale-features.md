# Bounded multiscale image-feature comparison

An optional Rust frontend extracts features at native, half and quarter image
resolution. The standalone `rustdriving-rgbd-multiscale-pairs` binary compares it
with the unchanged original frontend on **previously viewed adjacent RGB-D
pairs**. This is a pairwise diagnostic, not continuous odometry, localization
permission or vehicle integration. The original continuous room result remains
**52/179 root-accurate updates**, and the original desk2 result remains
**16/179** before tracking loss.

## Implementation and bounds

The frontend uses fixed quotas of **200 native, 120 half and 80 quarter
features**, globally at most **400 features and 256 matches**. Unused quota is
not reassigned. Each level retains the original FAST detector, intensity-centroid
orientation and rotated fixed-pattern BRIEF descriptor. The original extractor,
matching function, robust 3D registration and pixel refinement sources are
unchanged. This additive prototype is used by the standalone diagnostic; it
does not replace the library or driving defaults.

Half resolution uses a 2 × 2 integer box average, `(sum + 2) / 4`, with floor
dimensions and trailing incomplete cells omitted; quarter resolution repeats
that operation. A level feature maps to original pixel coordinates as
`level_pixel * scale + (scale - 1) / 2`. The report preserves level coordinates,
original coordinates and descriptor provenance. Levels too small for the
original descriptor border are skipped without padding.

Matching retains bidirectional mutual nearest/second-nearest tests, strict
ratio below 0.8, Hamming distance at most 64, tie rejection and one-to-one
associations. Cross-level duplicates are not silently removed; original
ambiguity tests may reject them. Depth uses the fractional original-coordinate
ray and nearest original depth sample, retaining the unchanged nine-sample
stencil, 0.3–5 m range and 0.05 m spread gate. Registration and reprojection
receive measured correspondences, without reference-pose input.

The diagnostic bounds frames to 180, each encoded sensor image to 4 MiB,
counted sensor bytes to 128 MiB and serialized reports to 64 MiB. These are
capacity limits, not measured memory use or real-time performance claims.

## Viewed pairwise results

Each consecutive pair is fitted independently. There is no composed root,
accepted-reference replacement, accepted-pose age, loss/recovery state or
permission decision. Numerical reference poses enter scoring only after all
sensor fits. The unchanged pair-error gates are **0.1 m / 0.1 rad**; all
**179 pairs** remain in each denominator, including rejected and repeated
acquisitions. A failed pair does not remove the next pair from this diagnostic.

| Viewed recording | Original accurate fitted pairs | Multiscale accurate fitted pairs | Denominator |
| --- | ---: | ---: | ---: |
| Room | 138 | 165 | 179 |
| Desk2 | 124 | 154 | 179 |

All previously fitted original pairs remain fitted with the multiscale
frontend. Seven repeated room RGB associations and eighteen repeated desk2
associations remain rejected. These counts are **relative-pair** results; they
cannot be substituted for the original continuous root-accuracy results.
Repeated executions produce byte-identical reports on the measured host.

| Maximum fitted pair errors | Original translation / rotation | Multiscale translation / rotation |
| --- | --- | --- |
| Room | 0.025708 m / 0.027948 rad | 0.085031 m / 0.041195 rad |
| Desk2 | 0.042108 m / 0.042571 rad | 0.054113 m / 0.050373 rad |

More accurate fitted pairs accompany **larger maximum errors** in both
recordings. These viewed measurements do not demonstrate uncertainty
calibration, improved accumulated drift or held-out generalization.

## Analytic controls and remaining false matches

A twofold image-scale control yields **zero correct original matches versus
48 correct multiscale matches**. In the 90° rotation control, the original
frontend has **80 correct of 84 matches**, while multiscale has **97 correct of
104**. The seven multiscale false matches remain reported; rotation handling
and mutual descriptor agreement do not guarantee unique correspondence.

An actual generated textured-plane RGB/depth pair at 1.5 m depth shifts pixels
by `[8, 4]`. Both fitters recover the analytic `previous_from_current`
translation **[-0.05, -0.025, 0] m**. Measured consensus support grows from
**74 to 94 inliers**. This generated pair is a sensor-loop control, not a
recorded scene or vehicle-motion measurement. The scale/rotation controls
measure pixel correspondence only and carry no independent 3D motion claim.

The independent checker reconstructs pyramid pixels, original-coordinate
mapping, FAST/BRIEF, mutual association, measured depth, consensus and pixel
refinement. **Full independent mathematical audits pass on all 179 pairs in
each recording**, and each rejects **20 non-noop report mutations**. These
integrity results verify the reported fits and failures, not continuous
localization or held-out accuracy.

The final checker used for both complete mathematical audits has SHA-256
`78e9466da3c30605b1f25a8037a6c51fcfb2198c8caacbacde85a6eb9ccb3737`.
Its earlier revision was changed only to add output-admission guards and
provenance metadata, without retuning estimator or auditor mathematics. Both
full recording audits were then rerun on the final source; mathematical
results match the earlier audits exactly after excluding the added checker
identity. Full numerical audits and guard/control checks remain separate.
The final-source analytic control also passes its independent reconstruction
and rejects **17 non-noop mutations**. Output-admission tests reject an existing
proof file, a symlink, a broken symlink and a symlinked parent before input
reads, preserving earlier evidence.

The [publication packet](../assets/multiscale-pairs-v1/packet.json) binds
[room report](../assets/multiscale-pairs-v1/room-report.json.gz),
[desk2 report](../assets/multiscale-pairs-v1/desk2-report.json.gz), their respective
[room freeze](../assets/multiscale-pairs-v1/room-freeze.json) and
[desk2 freeze](../assets/multiscale-pairs-v1/desk2-freeze.json), and
[room oracle](../assets/multiscale-pairs-v1/room-oracle.json) and
[desk2 oracle](../assets/multiscale-pairs-v1/desk2-oracle.json). The
[control](../assets/multiscale-pairs-v1/control.json),
[control oracle](../assets/multiscale-pairs-v1/control-oracle.json),
[repeat/source check](../assets/multiscale-pairs-v1/repeat-source-check.json),
[output guards](../assets/multiscale-pairs-v1/output-guards.json) and
[checker provenance](../assets/multiscale-pairs-v1/checker-provenance.json)
preserve repeatability and the guard-only source transition. Original images
and numerical reference tables remain ignored. Frozen original sources and
earlier failed continuous outcomes remain unchanged.

## Reproduction

Use the existing optional RGB-D environment and pinned
`scripts/requirements-visual.txt` Python dependencies. Work from the repository
root. Both recordings are already viewed; these commands create regression
evidence. Acquire missing raw files explicitly under their documented terms:

```sh
python3 scripts/fetch-temporal-dataset.py tum-fr1-room-temporal
mkdir -p artifacts/multiscale-pairs/acquisition
python3 scripts/fetch-independent-dataset.py tum-fr1-desk2-independent \
  --archive "$desk2_archive" \
  --preregistration assets/recorded-independent/desk2-v1/design.json \
  --acquisition-log artifacts/multiscale-pairs/acquisition/desk2.json
```

Set `desk2_archive` to an ignored archive path outside CI evidence uploads.
For an already retained desk2 dataset, replace the acquisition-log option with
`--verify-only`; do not overwrite earlier acquisition evidence.
[Desk2 provenance](tum-source-provenance.md) describes attribution and hashes.

Compile the standalone binary, then select either manifest and matching raw
directory. Use a new output directory for every comparison:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-rgbd-multiscale-pairs
pair_binary=integrations/rgbd/target/release/rustdriving-rgbd-multiscale-pairs
pair_manifest=data/tum-fr1-room-temporal/manifest.json
pair_raw=data/tum-fr1-room-temporal/raw
pair_run=artifacts/multiscale-pairs/local-room
mkdir -p "$pair_run"
"$pair_binary" --prepare-freeze --manifest "$pair_manifest" \
  --output "$pair_run/freeze.json"
"$pair_binary" --manifest "$pair_manifest" --raw "$pair_raw" \
  --freeze "$pair_run/freeze.json" --output "$pair_run/report.json"
python3 scripts/check-multiscale-features.py \
  --report "$pair_run/report.json" --manifest "$pair_manifest" \
  --raw "$pair_raw" --freeze "$pair_run/freeze.json" \
  --output "$pair_run/oracle.json"
"$pair_binary" --control --output "$pair_run/control.json"
python3 scripts/check-multiscale-features.py \
  --report "$pair_run/control.json" --output "$pair_run/control-oracle.json"
```

For desk2, select `data/tum-fr1-desk2-independent/manifest.json`, its `raw`
directory and a separate output directory. Use the actual binary location when
`CARGO_TARGET_DIR` is configured. A diagnostic CLI exit 0 means its bounded
execution completed; it does not require every pair to fit or authorize motion.

The independent checker accepts ordinary JSON. To inspect a published compressed
report without modifying it, decompress into a new ignored output file:

```sh
python3 -m gzip -d < assets/multiscale-pairs-v1/room-report.json.gz \
  > "$pair_run/published-room-report.json"
```

Continuous trajectory integration, tracking-loss behavior and driving fusion
remain unimplemented for this frontend. Maturity remains **about 20% by
subjective comparison with Autoware, Apollo and openpilot**; the **30% and 50%**
waypoints remain unmet. Better viewed pair support is useful development
evidence, not a replacement for those broader acceptance gates.
