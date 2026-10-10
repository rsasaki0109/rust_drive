# Independent recorded RGB-D motion protocol

RustDriving has preregistered an offline evaluation on **TUM RGB-D Freiburg 1
desk2**, a distinct recording from the previously evaluated material. The
official description calls it a second recording of the same four desks. It
does not provide an independent room, sensor family or automotive environment.
See [source provenance](tum-source-provenance.md).

## Current status

The [prospective design](../assets/recorded-independent/desk2-v1/design.json)
fixes the source, window, estimator, gates and resource bounds before new
timestamp-table contents or archive acquisition. Source discovery is recorded
in [the provenance journal](../assets/recorded-independent/desk2-v1/source-discovery.json).
Official archive acquisition now works through the redirect to
`webshare.cvg.cit.tum.de`. The discovery journal's CONNECT 403 is historical,
rather than a current acquisition blocker. Acquisition, timestamp qualification
and the archived-source preflight completed before the first pixel decode.
The first continuous sensor trial then **failed**: 16 of 179 updates were
accepted and root-accurate, while 163 were rejected.

The original formal motion auditor also failed with
`KeyError: source_rgb_distortion`: its inherited calibration-schema expectation
did not match this manifest. This checker defect is separate from the failed
estimator criterion. The original checker, source snapshot and result remain
byte-exact. An additive `scripts/check-recorded-independent-v2.py` repairs the
audit schema; its full motion audit now **passes integrity, exit 0**, independently
scoring all 179 continuous outcomes. It confirms the estimator's failed physical
criterion. Build and synthetic checks remain separate.

Desk2 is now **viewed data**. Later execution is a regression, not a new first
trial or fresh generalization claim. Earlier source archives and historical
license statements remain unchanged.

## Fixed selection and exposure order

Original depth indices **100 through 279 inclusive** supply **180 frames and
179 updates**. Index 100 initializes one camera root; every subsequent frame
has the `independent_recording` role. Each selected depth timestamp uses the
nearest RGB entry in the **complete original RGB table**; an exact tie chooses
the earlier original RGB index. No accuracy-based window search, restart,
failure deletion or evaluator-driven recovery is allowed.

The stages must remain separate:

1. Preregister source choice, fixed window, unchanged defaults, accuracy gates
   and archive resource caps.
2. Acquire the official compressed archive through the inherited proxy with
   TLS verification. Record its final URL, exact byte count, SHA-256 and MD5.
   Verify an authoritative published checksum if available; locally computed
   hashes alone identify the acquired bytes.
3. Scan a bounded archive inventory and read all three complete metadata
   tables. Interpret depth/RGB indices and only ground-truth row arities and
   timestamps. Acquisition already obtains compressed image bytes, and the
   archive scan decompresses payload bytes. This is **not PNG header inspection
   or pixel decoding**; numerical reference-pose columns remain uninterpreted.
4. Qualify finite, strictly increasing complete-source timestamps, fixed-window
   nearest RGB gaps of at most **0.02 s**, and reference interpolation brackets
   of at most **0.02 s**, with no extrapolation. Preserve a qualification failure
   without replacing the source or window.
5. Copy only the qualified selection as opaque image bytes and bind exact
   original paths, byte counts and hashes in the manifest. Seal the complete
   source/configuration/qualification freeze **before the first pixel decode**.
6. Run every operational fit continuously. Only after all fits finish, parse
   and validate the complete numerical ground-truth table, score every update
   and run the independent audit.

Qualifier fields such as `pixels_read: false` describe that helper's own
actions. They cannot imply that the earlier compressed acquisition and
decompression never encountered image payload bytes. Exposure journals must
record those earlier stages separately.

## Unchanged estimator and acceptance

This path retains the original descriptor/depth/reprojection estimator:
measured image features, descriptor associations, registered-depth checks,
bounded 3D consensus and robust pixel reprojection refinement. It does not
select the depth-supported matching or Lucas–Kanade alternatives. The original
`ImageFeatureConfig`, `VisualOdometry3dConfig` and `ReprojectionConfig3d`
defaults remain fixed; their source hashes are in the prospective design and
their full values are recorded in the preserved source freeze.

All nine integer-nearest depth-patch samples must be valid, within **0.3–5 m**,
and have spread at most **0.05 m**. Depth uses **5,000 units/m** and zero is
invalid. The fixed 640 × 480 Freiburg 1 camera profile is documented in the
design; it is a pinned published profile, with no independently measured
vehicle calibration or additional pixel undistortion.

Accepted `previous_from_current` poses compose into the single initial camera
root. Refinement failure rejects the update: it supplies no coarse fallback,
current root pose or accepted-reference/clock renewal. Repeated RGB acquisitions
are compared with the last **observed** RGB acquisition, including rejected
observations, so a failed image cannot retry or renew tracking. Accepted-pose
expiry is checked before image freshness; age beyond **0.20 s** latches loss
without restart.

The fixed criterion requires **all 179 updates** to be accepted and within
**0.1 m translation / 0.1 rad rotation** root-error gates. Rejected, repeated
and unscorable updates stay in the denominator. Duplicate source associations
may make that complete criterion unattainable; their existence is not a reason
to change it.

## First recorded outcome and preserved evidence

The fixed window spans **5.974762916564941 s**, with **162 distinct RGB
acquisitions and 18 repeated associations**. All 180 frames and 179 update
references remain selected, with one initialization and no restart.

| First estimator result | Updates |
| --- | ---: |
| Accepted and within both root-error gates | 16 |
| Rejected | 163 |
| Full denominator | 179 |

Maximum accepted errors are **0.08769042283767553 m translation** and
**0.09250872159426188 rad rotation**, below the unchanged gates. This conditional
accuracy does not establish continuous localization: the full criterion fails,
and the evaluator exits 1. The first rejection at index **102** is a repeated
RGB acquisition. The first geometric fit rejection at **121** has ten descriptor
matches but only four valid-depth correspondences, below the unchanged minimum
of twelve; no pixel refinement runs. Accepted-pose expiry latches loss at
**127**. All subsequent frames remain selected without recovery or a reset.

The complete source tables have **639 depth entries, 640 RGB entries and 2,428
ground-truth rows**. Timestamp qualification passes. Rounded summaries record
maximum association gap **17,030 µs** and reference bracket **11,400 µs**;
qualification uses unrounded source timestamps.

The [first freeze](../assets/recorded-independent/desk2-v1/freeze.json) has SHA-256
`d1d813c0d953a4ae4cd27abbc00dee4c817ee39f4e1ecd854c8457477074f9bf`.
[Pre-pixel preservation](../assets/recorded-independent/desk2-v1/pre-pixel-preservation.json)
archives 21 bound sources before official image decoding or numerical reference
parsing. The [metadata/archive preflight](../assets/recorded-independent/desk2-v1/preflight.json)
passes independently reconstructed qualification and source binding before
recorded pixels. Its synthetic image controls are disclosed separately.

[Manifest](../assets/recorded-independent/desk2-v1/manifest.json),
[acquisition](../assets/recorded-independent/desk2-v1/acquisition.json),
[qualification](../assets/recorded-independent/desk2-v1/qualification.json),
[full result](../assets/recorded-independent/desk2-v1/results.json),
[evaluator status](../assets/recorded-independent/desk2-v1/evaluation-status.json),
[original auditor status](../assets/recorded-independent/desk2-v1/audit-status.json)
and [original source snapshot](../assets/recorded-independent/desk2-v1/source-snapshot.tar.gz)
retain the first-trial evidence. Raw recordings are not redistributed. A passed
preflight is separate from the subsequently completed motion audit; integrity
success does not convert the estimator failure into success.

[The additive v2 audit](../assets/recorded-independent/desk2-audit-v2/audit-v2.json)
and [calibration proof](../assets/recorded-independent/desk2-audit-v2/calibration-v2.json)
verify the original report without changing estimator math or source history.
The audit rejects **12 calibration/inventory**, **44 source/provenance** and
**73 sensor/state** corruptions. It reconstructs measured features, descriptors,
associations, bounded consensus, reprojection refinement, continuous root/clock
state and physical errors. Its SHA-256 is
`59773d67017fdb998bc41dd3137049e74d6013531b5852dfd74b5ef72e61c1a0`;
the running v2 checker SHA-256 is
`8c4e96b63cdd4f2c59907d8a71dbb43be972d4290547ae2495e10214fc34ce70`.
Both the running v2 and original frozen checker identities remain explicit.
[The first-outcome record](../assets/recorded-independent/desk2-v1/first-outcome.json)
retains the estimator failure and corrected integrity success together with
[the original audit error](../assets/recorded-independent/desk2-v1/audit-error.log).

## Bounded acquisition and evidence

| Resource | Preregistered maximum |
| --- | ---: |
| Compressed archive | 512 MiB |
| Entire decompressed archive stream, including headers and padding | 2 GiB |
| Archive members | 10,000 |
| Individual member | 4 MiB |
| Selected raw inventory | 128 MiB |
| Three complete metadata tables combined | 4 MiB |
| Entries in each source image index | 20,000 |
| Ground-truth rows | 30,000 |

These are rejection bounds, not measured peak memory, throughput or runtime.
The full source archive and selected raw inventory have separate budgets. The
implementation must enforce decompressed-stream limits while scanning, not
only after summing extracted file sizes. It must reject traversal, links,
duplicate paths, foreign roots and invalid sizes, and accept only ordinary
files and directories. Blanket archive extraction is outside this protocol.

The manifest binds the exact design-byte SHA-256, official archive identity,
calibration descriptor, all selected original indices and timestamp associations,
and the exact distinct selected images plus three complete metadata tables.
The pinned `raw/camera-calibration.yaml` is an explicitly allowed 1,615-byte
documentation sidecar outside that selected-sensor-and-metadata inventory;
its separate descriptor fixes its bytes and SHA-256. Other extra files remain
invalid. A valid qualification binds the manifest, design,
acquisition helper and qualification helper, and can be reconstructed from
actual metadata. A later result must bind the sealed source/configuration freeze;
an integrity audit passing is separate from physical accuracy passing.

## Regression reproduction

Run from the repository root, using Python with the pinned
`scripts/requirements-visual.txt` dependencies. The first history is reproduced
from its preserved freeze, report and source snapshot. Do not replace them.
Later sensor executions need new evidence paths and explicit `--regression`
during both freeze preparation and evaluation; they cannot restore unviewed
status. Acquisition remains explicit opt-in and preserves proxy/TLS checks.

On a fresh checkout, set `desk2_archive` to an ignored local archive path
outside any directory uploaded as CI evidence. CI uses `$RUNNER_TEMP` for the
archive. Create only the acquisition-log parent and explicitly fetch:

```sh
mkdir -p artifacts/recorded-independent/acquisition
python3 scripts/fetch-independent-dataset.py tum-fr1-desk2-independent \
  --archive "$desk2_archive" \
  --preregistration assets/recorded-independent/desk2-v1/design.json \
  --acquisition-log artifacts/recorded-independent/acquisition/acquisition.json
```

For a retained dataset, use verification instead of reacquisition:

```sh
python3 scripts/fetch-independent-dataset.py tum-fr1-desk2-independent \
  --archive "$desk2_archive" \
  --preregistration assets/recorded-independent/desk2-v1/design.json --verify-only
```

The following wrapper is locally **tested**. It qualifies metadata, preserves a
source snapshot, prepares an explicit regression freeze, runs the continuous
sensor trial and executes the repaired full independent audit. Choose a new
output directory; the wrapper creates it itself.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-rgbd-independent
python3 scripts/check-recorded-independent-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-independent \
  --archive "$desk2_archive" \
  --manifest data/tum-fr1-desk2-independent/manifest.json \
  --raw data/tum-fr1-desk2-independent/raw \
  --output artifacts/recorded-independent/local-regression
```

Use the actual binary path when `CARGO_TARGET_DIR` is configured. The
[completed regression suite](../assets/recorded-independent/desk2-regression/suite.json)
records qualification **0**, freeze preparation **0**, evaluator **1** (expected
physical failure), and independent audit **0**. It retains all 179 updates and
matches the first result's non-timing evidence. Wrapper exit **0** means the
preserved failed physical result was correctly reproduced and audited; it does
not mean the physical criterion passed or provide fresh generalization evidence.
Standalone evaluator exit **0** means complete physical acceptance, **1** means
a valid failed physical trial, and **2** means an input or execution failure.

[Output guards](../assets/recorded-independent/desk2-regression/output-guards.json)
separately reject an existing directory and a broken symlink before work,
preserving earlier output. The final wrapper adds that fresh-output admission
check before resolving paths; the full numerical suite and the subsequent
guard-only verification are distinct evidence. The wrapper SHA-256 is
`649c23544145a7be7a6f23007ea946d67c17e60785e40f516e7e0a98f103519b`.
A manifest-tamper rejection also guards the bound source inventory. Existing
files, directories and broken symlinks cannot consume a sequence or be
overwritten. An execution failure can leave an empty or partial report; that
is failed evidence, never a successful result.

The frozen original `scripts/check-recorded-independent.py` passes the preserved
metadata preflight but fails the formal motion audit on the calibration schema.
The additive `scripts/check-recorded-independent-v2.py` audit passes integrity
on that original failed trial. Its proof binds both the running checker and
original frozen checker without changing estimator math, calibration or results.

[The historical CI repair](../assets/recorded-independent/desk2-regression/ci-repair.json)
separately documents a build-target selection error: an archived-source rebuild
attempted the newly added independent binary without its future preregistration
asset. Selecting only the intended historical evaluator binary repairs that
replay, preserving its original outcome and sources. This is a reproducibility
repair, not a recorded-motion accuracy improvement.

## Executed synthetic validation

The private Rust test decodes **180 actual generated RGB/depth PNG frames**,
extracts FAST/BRIEF features, fits measured depth and refines all **179**
nonzero updates with one camera root. A known 1.5 m textured plane shifts one
pixel per frame. Every update meets the analytic root-motion gates; maximum
root errors are approximately **7.77e-16 m / 0 rad**. Generated raw bytes are
**68,196,826**, within 128 MiB. The independent Python implementation reconstructs
the pixels, features, matching, SVD consensus, refinement and clocks, and rejects
ten actual Rust-witness or qualification corruptions.

This packet is explicitly synthetic and **does not exercise official-source
qualification**. Its archive identity is null and the ordinary production audit
rejects it. It is a sensor-loop control, not a desk2 result or a performance
benchmark. Separate tests verify official-schema/source bindings, full-source
metadata, hostile tar headers and payload limits, failure journals, no-op
mutation guards, repeats and latched expiry. The publication validation record
is [independent-validation.json](../assets/independent-validation.json).

Run the bounded acquisition controls with
`python3 scripts/test-independent-dataset.py`. To independently reproduce the
synthetic pixel packet, set an **absolute**
`RUSTDRIVING_INDEPENDENT_SYNTHETIC_OUTPUT` path for `cargo test --release
--manifest-path integrations/rgbd/Cargo.toml --bin rustdriving-rgbd-independent`,
then run `python3 scripts/check-recorded-independent.py --self-test
--synthetic-packet <that-path>/fixture.json --output <new-proof.json>`.
The export exists only under `cfg(test)`; the shipping CLI has no bypass.

This is offline indoor camera-motion research. It supplies no vehicle-pipeline
integration, automotive generalization, calibrated accumulated uncertainty,
loop closure or global relocalization claim. The project remains approximately
**20% mature by subjective comparison with Autoware, Apollo and openpilot**;
this is not a measured capability score. The broader 30% goal is not met, and
preregistration or synthetic success alone does not advance that estimate.
