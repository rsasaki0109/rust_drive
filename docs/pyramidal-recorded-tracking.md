# Pairwise pyramidal recorded-image tracking

The opt-in `rustdriving-rgbd-tracked` binary replaces descriptor association with
bounded measured image-patch tracking. It uses the existing 180-frame room
recording as a **viewed calibration regression**. Every selected image and its
numerical reference source was already evaluated. This is not an unseen
recording, an automotive test or a calibrated confidence claim.

## Measured correspondences

The original extractor detects up to 400 FAST/BRIEF features. For each previous
feature, an original Rust translation-only inverse-compositional Lucas–Kanade
tracker constructs three grayscale levels with a 5×5 binomial filter. It
subtracts patch means and follows 9×9 patches, from coarse to fine, with at most
10 iterations per level. Each solved step is bounded to two level pixels;
convergence uses the unclamped step norm of at most 0.01 pixels.

A patch must have average minimum gradient eigenvalue at least 4, condition
number at most 100 and final mean-normalized photometric RMS at most 15 grayscale
units. A separately initialized backward fit must return within one original
pixel. Border, mask, texture, conditioning, convergence, photometric and
forward/backward failures retain explicit per-level witnesses. Forward and
backward fits begin without motion priors or reference labels. Work is bounded
by 3,110,400 bilinear samples and 24,000 iterations for at most 400 points.
The two pyramids also produce 192,000 downsampled pixels with 25 fixed filter
taps each; those operations are separate from the bilinear-sample count.

Tracked endpoints use their own subpixel buffer, with original previous-feature
indices and no fabricated BRIEF descriptors. Both depths must pass the unchanged
3×3 validity, 0.3–5 m range and 0.05 m spread checks. Depth projection uses the
nearest integer pixel, while refinement observes the measured subpixel endpoint.
The first at most 256 valid depth pairs in previous extraction order enter the
original robust rigid fit; all additional endpoints and depth witnesses remain
reported with their selection status. Original consensus and refinement gates
are unchanged.

## Continuous reference and evaluation

The last accepted measured grayscale image, depth, newly detected FAST features
and root pose become the next reference. A rejected fit does not replace them
or renew the accepted pose clock. The original 0.20-second expiry, observed-RGB
freshness guard and latched loss remain. There is one initialization, no reset,
recovery or coarse-pose fallback. This is pairwise tracking with reference
reseeding; it does not maintain persistent landmark identities.

Reference poses are evaluated after all sensor fits, using the unchanged
0.1 m / 0.1 rad accumulated root gates. All 179 updates remain in the denominator.
The 180 selected depth rows contain only 173 distinct RGB acquisitions: seven
reused images must be rejected, giving at most 172 fresh-image accepted updates.
The original all-updates physical gate remains unchanged and cannot pass on
this selection. Accepted coverage and accurate coverage are reported separately.

[Declared design](../assets/recorded-tracked/room-v1/design.json) records the
synthetically selected defaults and full window before this candidate's first
fit. Original descriptor-based and depth-supported sources and their failed
outcomes remain preserved.

## Measured outcome

The frozen first trial **fails, evaluator exit 1**. It performs one initialization
and retains all 179 updates and valid reference scores. This variant regresses
substantially against both earlier methods and remains opt-in research.

| Same 179-update recording | Original descriptors | Depth-supported descriptors | Pairwise patch tracking |
| --- | ---: | ---: | ---: |
| Accepted updates | 58 | 59 | 6 |
| Root-accurate updates | 52 | 49 | 6 |
| Rejected updates | 121 | 120 | 173 |
| First latched tracking loss, source index | 164 | 170 | 112 |

The last accepted patch-tracked reference is 106. At 107, 22 of 248 tracks pass
image gates and 12 pass depth selection, but the original rigid consensus fails.
At 108–111 no complete tracks pass; coarsest-level nonconvergence accounts for
193, 207, 210 and 207 rejected tracks respectively. Age expiry at 112 is
0.200645 s, just beyond the unchanged 0.20 s limit. Loss remains latched through
279. [All-update failure detail](../assets/tracked-failure-detail.json).

The six accepted updates have maximum root errors 0.016004 m / 0.020773 rad.
Their short coverage does not demonstrate improved continuous accuracy.
Neither the iteration cap, photometric gates nor physical accuracy gates were
changed after observing this failure. Motion-aware image alignment and recovery
remain unresolved.

The [first source archive](../integrations/rgbd/baselines/tracked-room-v1/SOURCE.json),
[first outcome](../assets/recorded-tracked/room-v1/phase.json) and
[first complete report](../assets/recorded-tracked/room-v1/results.json) remain
unchanged. The original independent auditor then failed one negative-test
selection: it tried to drop a correspondence whose invalid-depth witness was
already unselected, so its mutation changed nothing. This was a test-selection
error, not a successful physical protocol.

The [auditor-only repair](../assets/recorded-tracked/room-v1/audit-repair.json)
selects an actually selected witness and requires every tracked mutant to change
its input. A new mixed invalid-first/valid-later synthetic contract reproduces
the error and rejects a genuine corruption. The repaired auditor successfully
[checks the unchanged first report](../assets/recorded-tracked/room-v1/audit-repaired.json)
against its original archived sources. Its running checker hash differs from
that first source freeze; both identities remain explicit.

A [second source archive](../integrations/rgbd/baselines/tracked-room-v2/SOURCE.json)
changes only the frozen checker hash. The
[viewed repair replay](../assets/recorded-tracked/room-v2/phase.json) produces
identical operational and scoring fields after excluding CPU time and the two
source-freeze fields. Its [archived audit](../assets/recorded-tracked/room-v2/audit.json)
and [viewed regression](../assets/tracked-regression.json) verify this outcome;
exit 1 continues to mean a failed physical protocol. The original driving and
descriptor-based paths are unchanged.

## Synthetic controls and known failures

All 66 optional RGB-D release tests pass, including 17 in the new binary:
eight tracker tests and nine adapter/metadata/clock tests. The additional
[12-case diagnostic](../assets/tracked-synthetic-exercises.json) keeps every
point outcome and work total, with a standalone reproducible Rust exercise.
Each case attempts 63 points; no recorded images or reference labels are used.

The translation control accepts all points with maximum endpoint error
0.169603 pixels. A rotational case reaches the actual forward/backward
rejection gate after all six level fits converge, with inconsistency
1.44184 pixels. These controls also demonstrate substantial unresolved errors:
a repeated texture translated 32 pixels accepts all 63 points at wrong
endpoints, each 32 pixels away; a 0.1-radian rotation accepts 27 points with
maximum accepted error 26.2897 pixels. Exposure gain 1.15 accepts all points
but increases maximum error to 0.895636 pixels; gain 1.8 rejects all points.
Bidirectional consistency and bounded work therefore do not establish correct
association under arbitrary appearance or motion.

## Reproduction

Use the existing optional RGB-D dependencies and pinned Rust. No new dependency
or toolchain is introduced. Raw images remain ignored; consult their documented
terms before acquisition.

```sh
python3 -m pip install -r scripts/requirements-visual.txt
python3 scripts/fetch-temporal-dataset.py tum-fr1-room-temporal
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-rgbd-tracked
python3 scripts/check-recorded-tracked-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-tracked \
  --output artifacts/recorded-tracked/local-reproduction
rustc --edition 2024 -O scripts/exercise-pyramidal-tracking.rs \
  -o artifacts/exercise-pyramidal-tracking
artifacts/exercise-pyramidal-tracking > artifacts/pyramidal-tracking-synthetic-exercises.json
```

Use the actual binary location if `CARGO_TARGET_DIR` is set. The binary requires
`--regression`. The suite independently preregisters source-bound metadata and
synthetic controls, checks preserved first-trial identities, reproduces every
non-timing report field and independently reconstructs actual pixel tracks,
depth witnesses, rigid consensus, pixel refinement, clocks and reference scores.
Suite exit 0 means integrity and the declared failure were reproduced; it does
not change evaluator exit 1 to a passed physical protocol. CI runs both the
synthetic stress exercise and this regression alongside the earlier methods.

## Limits

Mean subtraction tolerates additive brightness offset. Exposure gain,
scale/rotation invariance, uniqueness in repeated textures, loop closure,
global relocalization, calibrated covariance, measured vehicle extrinsics and
driving fusion are unimplemented or unverified. Passing synthetic controls
cannot establish these capabilities. The original dataset's
[provenance and license limitations](../data/tum-fr1-room-temporal/SOURCE.md)
remain; raw images are ignored and are not redistributed. Depth is not interpolated
at the subpixel RGB endpoint, and published pinhole profiles do not constitute
physical RGB/infrared extrinsics calibration. Bounded work counts do not
establish a real-time deadline.

## Independent recording availability

A [bounded metadata-only source search](../assets/tracked-validation-source-discovery.json)
inspected 16 pinned public Git trees before acquiring any new images or reading
new index/reference-table contents. Distinct desk2 metadata was available, but
its mirror contained no original RGB/depth images. Another complete room mirror
had the same reference Git blob as the viewed recording and cannot support a
new-recording claim. The prospective independent window stays fixed at original
depth indices 100–279; it has not been qualified or scored.

Both official TUM dataset/terms pages failed at the configured proxy CONNECT
stage (403, curl exit 56); no origin license text was received. This is a
specific access/source limitation, not proof that datasets are unavailable
elsewhere. Existing public code licenses do not grant rights to dataset images.
Supported access to the official source and terms, or a complete independently
licensed RGB-D recording, is still required for this proposed separate trial.
The environment draft adds `cvg.cit.tum.de` while preserving existing custom
hosts; saving that draft does not apply its policy or establish data access.
