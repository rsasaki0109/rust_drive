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
The archive request redirects to `webshare.cvg.cit.tum.de`, where the observed
proxy CONNECT request was denied. A supported environment configuration draft
has been saved; saving it does not publish or apply the change.

No desk2 raw archive, timestamp qualification, sealed pre-pixel source freeze,
sensor fit or accuracy result is claimed here. Code or build checks establish
development readiness separately from an actual recorded-data evaluation.
The Rust implementation, bounded acquisition controls and independent synthetic
audit have been executed locally; official-source reproduction remains pending.
Previous trials, source archives and their historical license statements remain
unchanged.

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
their full values belong in the eventual source freeze.

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
to change it. No pose-accuracy or acceptance counts are available yet.

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

## Reproduction interfaces

Run from the repository root. The preregistered interfaces are:

| Stage | New interface |
| --- | --- |
| Official archive acquisition | `scripts/fetch-independent-dataset.py` |
| Timestamp-only qualification | `scripts/qualify-rgbd-independent.py` |
| Standalone evaluator | `rustdriving-rgbd-independent` in `integrations/rgbd` |
| Independent evidence audit | `scripts/check-recorded-independent.py` |

The following official-source sequence is **not yet executed**. Run it only
after the actual redirect host is available, using Python with the pinned
`scripts/requirements-visual.txt` dependencies. Every evidence path must be new.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml
mkdir -p artifacts/recorded-independent/desk2-first
python3 scripts/fetch-independent-dataset.py tum-fr1-desk2-independent \
  --archive artifacts/recorded-independent/desk2-source.tgz \
  --preregistration assets/recorded-independent/desk2-v1/design.json \
  --acquisition-log artifacts/recorded-independent/desk2-first/acquisition.json
python3 scripts/qualify-rgbd-independent.py \
  --manifest data/tum-fr1-desk2-independent/manifest.json \
  --raw data/tum-fr1-desk2-independent/raw \
  --output artifacts/recorded-independent/desk2-first/qualification.json
integrations/rgbd/target/release/rustdriving-rgbd-independent \
  --manifest data/tum-fr1-desk2-independent/manifest.json \
  --qualification artifacts/recorded-independent/desk2-first/qualification.json \
  --prepare-freeze artifacts/recorded-independent/desk2-first/freeze.json
python3 scripts/check-recorded-independent.py \
  --manifest data/tum-fr1-desk2-independent/manifest.json \
  --raw data/tum-fr1-desk2-independent/raw \
  --archive artifacts/recorded-independent/desk2-source.tgz \
  --qualification artifacts/recorded-independent/desk2-first/qualification.json \
  --freeze artifacts/recorded-independent/desk2-first/freeze.json \
  --preregister-only --output artifacts/recorded-independent/desk2-first/preflight.json
integrations/rgbd/target/release/rustdriving-rgbd-independent \
  --manifest data/tum-fr1-desk2-independent/manifest.json \
  --raw data/tum-fr1-desk2-independent/raw \
  --qualification artifacts/recorded-independent/desk2-first/qualification.json \
  --freeze artifacts/recorded-independent/desk2-first/freeze.json \
  --output artifacts/recorded-independent/desk2-first/results.json
```

Keep the evaluator's real exit status: **0** means the complete physical
criterion passed, **1** retains a valid but failed physical trial, and **2** is
an input or execution failure. If a result was produced, run the same audit
command without `--preregister-only`, adding `--report .../results.json` and a
new `--output .../audit.json`. Archive the first code sources and artifacts
without overwriting them. Later runs require `--regression` in both freeze
preparation and evaluation and cannot recreate a first unviewed trial.

The evaluator reserves a new report destination before sensor work. Existing
files, directories and broken symlinks cannot consume a new sequence or be
overwritten. An execution failure can leave an empty or partial new report;
that is failed evidence, never a successful result.

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
