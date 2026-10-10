# Metadata-qualified recorded RGB-D motion

The optional `rustdriving-rgbd-qualified` binary requires source-bound metadata
qualification before opening image bytes, headers, features or fits. It runs the
existing measured visual odometry and bounded pixel-reprojection refinement with
unchanged defaults. Earlier RGB-D entry points and their archived evidence remain
separate. This new path addresses source-label integrity before a first trial;
it does not establish accurate motion estimates merely by qualifying a source.

## Immutable selection and acquisition timeline

1. A deterministic candidate order and original depth window **100–135** were
   declared before reading candidate indices, reference timestamps or images.
   Freiburg 1 room was first; alternative recordings would be considered only
   after a metadata failure. No candidate was selected using features, geometry,
   numeric reference poses or accuracy.
2. The complete pinned room indices and reference file passed timestamp and
   row-arity checks. All **4,887 non-comment reference rows** have eight tokens
   and finite, strictly increasing timestamps. Only the timestamp column was
   parsed numerically. Nearest original RGB associations and reference brackets
   passed the exact gates below. The search stopped at this first qualified
   candidate; no other candidate was qualified in this search.
3. An immutable metadata-only pre-transfer journal recorded that decision.
   Authorized opaque byte acquisition then fixed lengths, SHA-256 hashes and
   Git-tree blob identities for **76 files**, totaling **21,516,513 bytes**.
   Acquisition inspected no PNG signature, header, dimensions or pixels and
   parsed no numeric reference pose columns. Bounds are **4 MiB per file** and
   **32 MiB per dataset**. The inventory is 36 depth PNGs, 36 RGB PNGs, the three
   complete metadata tables and one separately pinned camera document.
4. The mandatory qualification helper, Rust adapter and independent checker
   reconstruct metadata evidence from the actual pinned tables. Algorithm,
   configuration, acquisition, qualification and checker sources are frozen
   before the first image decode or fit. The independent preregistration audit
   is metadata-only; an acquisition journal does not substitute for this freeze.
5. Operational estimates finish before reference pose values enter physical
   scoring. Preserve the first freeze, sources, estimates and failures. Later
   runs on this recording are **viewed regression**, identified with
   `--regression`, rather than additional first trials.

The sequence is TUM `rgbd_dataset_freiburg1_room`, pinned at
`edrishakimi1/Indoor-SLAM-Floorplan-with-Gaussian-Splatting`
commit `1d5b2c3e1ee186abc1709042b739e9cd8a3b41d1`. Its 36 selected depth
observations span **1305031914.108031–1305031915.276431 seconds**, or
**1.168400 seconds**. Index 100 initializes the measured origin; all 35 following
updates are held out and remain in the denominator, including rejected updates.
There are 36 unique associated RGB acquisitions and no repeated associations.
Nearest-timestamp ties choose the earlier original RGB index. No image is
resized, retimed, synthesized or removed to improve coverage.

[The source record](../data/tum-fr1-room-qualified/SOURCE.md) retains exact
provenance, camera values and the pre-transfer journal hash;
[the manifest](../data/tum-fr1-room-qualified/manifest.json) fixes every input.

## Operational and evaluation contract

| Check | Unchanged bound |
| --- | --- |
| Absolute RGB/depth timestamp gap | At most **0.02 s** |
| Reference interpolation bracket | At most **0.02 s**, without extrapolation |
| Accepted-pose age | At most **0.20 s**; stale tracking latches lost |
| Accepted root position error | At most **0.1 m** |
| Accepted root rotation error | At most **0.1 rad** |

Timestamp gates use unrounded source seconds. Rounded microseconds in summaries
are presentation only. The selected metadata maximum RGB/depth gap is
0.012958049774169922 s and maximum reference bracket is
0.010399818420410156 s. These are availability measurements, not pose results.

The adapter uses existing FAST/BRIEF features, registered-depth checks, robust
3D consensus and Huber pixel refinement bounded to 256 original consensus
inliers and eight iterations. Neither estimator nor accuracy settings were
tuned against this recording. A rejected update supplies no current pose and
does not refresh the accepted reference or its clock. The observed RGB clock
still advances; repeated images cannot retry a failed fit or renew tracking.
There is no reference-pose initialization, prediction, reset or coarse fallback.

Metadata qualification checks full-source timestamp integrity and row arity;
it deliberately does not inspect numeric pose values or quaternion norms.
After all sensor fits, the strict evaluation-only reader validates finite pose
components, unit quaternions and strictly increasing times across the complete
reference source, bounded to 4 MiB and 30,000 rows. Invalid labels preserve sensor
traces but make all accuracy references unscorable, with an explicit label
failure and exit 2. Rows are not deduplicated, normalized or trimmed to rescue a
selected window. Exit 0 denotes physical protocol acceptance; exit 1 denotes a
failed physical protocol. Independent integrity success is separate from those
physical outcomes.

The preserved first trial passes **35/35 accepted and accurate updates**, with
physical **exit 0**, one initialization and no tracking loss. Maximum root
errors are **0.075700 m** and **0.084661 rad**, below the unchanged 0.1 m / 0.1 rad
gates. All 35 refinements report convergence. This result covers only the fixed
1.1684-second interval. The [first-trial report](../assets/recorded-qualified/first-room-v1/results.json),
[before/after journals](../assets/recorded-qualified/first-room-v1/phase0.json)
and [independent audit](../assets/recorded-qualified/first-room-v1/audit.json)
retain the full evidence. Earlier desk, office, sitting and invalid-label failures
remain unchanged; this sequence does not repair those protocols.

## Reproduction

Run from the repository root. Fetching is explicit opt-in and preserves HTTPS
proxy and TLS verification. Raw inputs stay ignored. Output evidence and freeze
files use new paths; do not overwrite a previous trial.

```sh
python3 -m pip install -r scripts/requirements-visual.txt
python3 scripts/fetch-qualified-dataset.py tum-fr1-room-qualified
python3 scripts/fetch-qualified-dataset.py tum-fr1-room-qualified --verify-only
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-rgbd-qualified

qualified_manifest=data/tum-fr1-room-qualified/manifest.json
qualified_raw=data/tum-fr1-room-qualified/raw
qualified_binary=integrations/rgbd/target/release/rustdriving-rgbd-qualified
qualified_run=artifacts/recorded-qualified/local-regression-v1
mkdir -p "$qualified_run"
python3 scripts/qualify-rgbd-metadata.py \
  --manifest "$qualified_manifest" --raw "$qualified_raw" \
  --output "$qualified_run/qualification.json"
```

Use the actual binary path if `CARGO_TARGET_DIR` is configured. The helper reads
only the three metadata tables. The Rust adapter requires `--qualification`
and independently validates the same evidence before its image-reading stage;
a rehashed invented proof is insufficient.

The original first-trial workflow prepares a freeze without `--regression`,
then runs the independent preregistration audit before decoding:

```sh
"$qualified_binary" --manifest "$qualified_manifest" \
  --qualification "$qualified_run/qualification.json" \
  --prepare-freeze "$qualified_run/first-freeze.json"
python3 scripts/check-recorded-qualified.py \
  --manifest "$qualified_manifest" --raw "$qualified_raw" \
  --qualification "$qualified_run/qualification.json" \
  --freeze "$qualified_run/first-freeze.json" --preregister-only \
  --output "$qualified_run/preregistration.json"
```

`--prepare-freeze` reads the manifest and qualification proof, not raw images or
reference poses. It cannot be combined with `--raw`, `--freeze` or `--output`.
Archive the exact freeze and bound sources before a first operational run.
Repeating these commands cannot make an already viewed sequence fresh again.

For subsequent local reproduction, prepare a separate regression freeze and
use `--regression` for both preparation and evaluation:

```sh
"$qualified_binary" --regression --manifest "$qualified_manifest" \
  --qualification "$qualified_run/qualification.json" \
  --prepare-freeze "$qualified_run/regression-freeze.json"
"$qualified_binary" --regression --manifest "$qualified_manifest" \
  --qualification "$qualified_run/qualification.json" --raw "$qualified_raw" \
  --freeze "$qualified_run/regression-freeze.json" \
  --output "$qualified_run/results.json"
python3 scripts/check-recorded-qualified.py \
  --manifest "$qualified_manifest" --raw "$qualified_raw" \
  --qualification "$qualified_run/qualification.json" \
  --freeze "$qualified_run/regression-freeze.json" \
  --report "$qualified_run/results.json" --output "$qualified_run/audit.json"
```

Retain the evaluator exit status even when the independent checker succeeds.
The checker reconstructs measured features, correspondences, pixel refinement,
tracking state, root composition and reference scores independently; numerical
agreement cannot change failed availability or accuracy into acceptance.

The CI reproduction suite audits the immutable first source archive, recomputes
metadata qualification, runs the same binary explicitly as viewed regression,
and requires exact non-timing numerical evidence and unchanged source/configuration:

```sh
python3 scripts/check-recorded-qualified-suite.py \
  --binary "$qualified_binary" --output artifacts/recorded-qualified/ci-reproduction
```

## Scope, calibration and licensing limits

This is one short indoor recorded sequence. A distinct physical room or camera
relative to other Freiburg 1 recordings is not independently established.
Qualification does not establish long-duration mapping, global relocalization,
loop closure, automotive generalization, calibrated covariance, driving
integration, real-time performance or real-vehicle safety.

The pinned published TUM1 profile uses 640×480 images, fx=517.306408,
fy=516.469215, cx=318.643040, cy=255.313989 and 5000 depth units per metre.
The optical frame is x-right/y-down/z-forward. This is a research source-model
assumption, without independently measured infrared extrinsics, registration,
in-situ metric calibration or vehicle calibration. Published RGB distortion
is recorded as provenance; the estimator uses pinhole projection without extra
Brown–Conrady correction, range correction or RGB/depth time compensation.
Image dimensions and encoding were published assumptions during acquisition
and are checked only by the frozen decoder.

Raw images, reference labels and camera YAML are not redistributed or relicensed.
Public GitHub mirrors provide pinned transfer provenance, not a redistribution
grant. Original TUM license terms remain unverified: the official dataset page
returned HTTP 403 through the configured proxy during earlier acquisition.
Published camera values are documented without copying upstream algorithm code.
See [the pixel-refinement contract](recorded-reprojection.md) for the underlying
estimator and earlier viewed development evidence.
