# Continuous multiscale RGB-D comparison

This preregistered **viewed-data regression** compares the original and
multiscale image frontends while tracking a continuous camera root. It addresses
a boundary of the [adjacent-pair diagnostic](multiscale-features.md): accurate
independent pair fits do not establish an accurate or continuously available
trajectory. Both recordings and their reference poses have already been viewed;
this experiment cannot supply a fresh held-out claim.

## Preregistered design and current status

[The fixed design](../assets/multiscale-temporal-v1/design.json) was preserved
before the new continuous fits in commit `41d6614`. Its exact SHA-256 is
`d0b6a8f21ded03ce8125e30e47440d223c77567d0f2e41b6d9031c32e1ea34df`.
It fixes source manifests, the existing frontends, continuous-state behavior,
unchanged pose-fitting defaults, evaluation gates and resource bounds.

The first continuous comparison has executed on both viewed recordings and
**fails physical acceptance in both**, evaluator exit 1. Room multiscale root
accuracy falls from the native branch's **52/179 to 49/179**; desk2 falls from
**16/179 to 15/179**, despite more accepted fits. Every branch eventually loses
tracking. Full independent audits now **pass integrity** on both recordings;
the complete repeat wrapper and optional full validation also pass. Integrity
success confirms the failed physical outcomes rather than changing them.

The original source histories remain unchanged: the preceding room **52/179**
and desk2 **16/179** continuous failures are preserved. Their sources, gates
and failed reports are not replaced by this comparison. The new native branch
has the same acceptance/accuracy counts as those earlier failures. A separate
180-row comparison finds zero mismatches in accepted/initialized decisions,
relative/root poses, observed/accepted clocks, references and loss state on
both recordings. Complete wrapper reproduction also verifies the fixed sources,
protocol and both full reports after excluding CPU timing alone.

## Fixed inputs and independent branches

Both frontends process the same **180 original frames, depth indices 100–279**,
on each viewed recording, with **179 updates** retained. The native branch
uses the original extractor with at most 400 features. The multiscale branch
retains fixed native/half/quarter quotas of **200/120/80**, without reallocating
unused quota, and the previously validated feature module. Each branch has its
own accepted reference, root, clock and loss state.

Measured initialization occurs only at the first selected frame. It validates
depth-supported geometry with unchanged bounded registration requirements and
at most 256 valid points, then initializes the operational root to identity.
Initialization failure latches loss; a later frame cannot restart the branch.
The numerical reference origin is used only during later scoring.

Accepted updates refine the measured `previous_from_current` transform against
the **last accepted** image/depth reference. Compose that transform into the
accepted reference's root and atomically replace the reference, root and accepted
acquisition timestamp. Rejected updates supply no current root and do not renew
the reference, pose or accepted clock. No coarse-only fallback is permitted.

This differs from fitting every adjacent pair independently: after a rejection,
the next attempted fit still uses the last accepted reference. Unconditionally
composing adjacent estimates across failure would violate this protocol.

## Freshness and latched loss

Accepted-pose expiry is checked **before RGB freshness or image decoding**.
Age greater than **0.20 s**, with the fixed **1e-9 s comparison tolerance**,
latches loss. Every subsequent frame remains present and rejected; no recovery,
chunk boundary reset or new origin is allowed.

A strictly newer RGB acquisition becomes observed before geometric fitting;
it remains observed if the fit fails. Repeated or stale acquisitions reject
without image decoding or clock renewal. They cannot retry a failed image,
refresh localization permission or extend accepted-pose age. Each branch has
exactly one possible initialization, regardless of observed failures.

## Unchanged sensing, geometry and scoring

RGB/depth association gaps are at most **0.02 s**. Depth retains the original
all-nine-samples stencil: every sample valid, **0.3–5 m**, spread at most
**0.05 m**. Multiscale features use the fractional original-coordinate ray
with the nearest measured original depth sample. Matching remains bounded to
256 pairs, and original `VisualOdometry3dConfig` and `ReprojectionConfig3d`
defaults remain unchanged.

All operational fits in **both branches** finish before numerical reference
poses are parsed. Reference interpolation requires a bracket at most **0.02 s**,
with no extrapolation. Score each composed root against the single reference
origin fixed by the first source reference. Reference poses never initialize
the operational root or choose sensor correspondences.

The unchanged root gates are **0.1 m translation and 0.1 rad rotation**. Every
rejected, repeated or unscorable update remains in the **179-update denominator**.
Complete physical acceptance requires one initialization and all 179 updates
accepted, reference-valid and accurate. Invalid numerical labels preserve the
completed sensor report, leave references unavailable and return exit 2.

| Resource | Fixed limit |
| --- | ---: |
| Frames | 180 |
| Manifest bytes | 512 KiB |
| Individual encoded image | 4 MiB |
| Counted sensor bytes | 128 MiB |
| Serialized report | 64 MiB |

These are capacity bounds, not measured peak memory, throughput or real-time
claims. Metadata-only source/manifest/design freezing must precede the new
continuous experiment. New output files use exclusive creation; first reports
must be preserved, including failures.

## First continuous outcome

All 180 frames and **179 valid-reference updates** remain in each branch. Each
branch initializes once and never restarts. Room covers **5.97323203086853 s**;
desk2 covers **5.974762916564941 s**. Loss indices below are original depth
indices, not positions in a filtered report.

| Recording / branch | Accepted updates | Root-accurate | Rejected | First latched loss |
| --- | ---: | ---: | ---: | ---: |
| Room / native | 58 | 52 | 121 | 164 |
| Room / multiscale | 92 | 49 | 87 | 199 |
| Desk2 / native | 16 | 16 | 163 | 127 |
| Desk2 / multiscale | 19 | 15 | 160 | 131 |

The last accepted room reference is index 158 for native and 192 for multiscale;
desk2's last accepted reference is 120 for native and 125 for multiscale.
Tracking lasts longer with multiscale, but accepting more observations does not
improve the number within both root-error gates. In room, the **34 additional
accepted updates contribute zero accurate roots**. On the 58 common accepted
updates, accuracy falls from 52 to 49; indices **155, 156 and 158** lose their
previous accuracy classification. Desk2's three added fits contribute one
accurate root, but the sixteen common fits score fourteen versus native sixteen:
indices **119 and 120** regress. Every gained and lost outcome remains selected.

| Maximum accepted root errors | Translation | Rotation |
| --- | ---: | ---: |
| Room / native | 0.09884618627335028 m | 0.11037314331801358 rad |
| Room / multiscale | 0.21486397253894426 m | 0.14928743712003295 rad |
| Desk2 / native | 0.08769042283767553 m | 0.09250872159426188 rad |
| Desk2 / multiscale | 0.12290100245476307 m | 0.10617091066391768 rad |

Both room branches first exceed a root-accuracy gate at index **149**. Desk2
native has no accepted accuracy failure, while multiscale first fails at **119**.
Multiscale's larger maximum errors and lower root-accurate counts are retained,
rather than calling later tracking loss an accuracy improvement. The preceding
adjacent-pair gains therefore **do not transfer to continuous root accuracy**.
This viewed failure supplies no held-out or automotive generalization claim.

[Room first report](../assets/multiscale-temporal-v1/room-report.json.gz) and
[desk2 first report](../assets/multiscale-temporal-v1/desk2-report.json.gz) retain
all sensor/state rows and scoring. Their respective
[room freeze](../assets/multiscale-temporal-v1/room-freeze.json) and
[desk2 freeze](../assets/multiscale-temporal-v1/desk2-freeze.json) bind the sources,
manifest and fixed design. Raw recordings remain ignored. Independent complete
mathematical audits now verify **all 180 frames / 179 updates** in both branches
on each recording, rejecting **15 non-noop corruptions for room** and **17 for
desk2**. The desk2 chronology additionally exercises a duplicate before loss.
These audits reconstruct geometry, root composition, clocks and errors without
changing the first outcomes. Exact wrapper reproduction also passes, preserving
both first failed outcomes.

[Room audit](../assets/multiscale-temporal-v1/room-audit.json) and
[desk2 audit](../assets/multiscale-temporal-v1/desk2-audit.json) retain the first
independent mathematical checks. Additional public proofs include
[analytic sensor/state audit](../assets/multiscale-temporal-v1/control-audit.json),
[auditor boundary/algebra controls](../assets/multiscale-temporal-v1/auditor-self-test.json),
[output guards](../assets/multiscale-temporal-v1/output-guards.json),
[inventory preflight](../assets/multiscale-temporal-v1/inventory-preflight.json),
[native-state comparison](../assets/multiscale-temporal-v1/native-comparison.json)
and [first-stage journal](../assets/multiscale-temporal-v1/first-stage-journal.json).
The [completed regression suite](../assets/multiscale-temporal-v1/regression-suite.json),
[validation summary](../assets/multiscale-temporal-v1/validation.json) and
[bound evidence packet](../assets/multiscale-temporal-v1/evidence.json) retain
the complete local reproduction and publication identities.

## Execution and audit interfaces

On a fresh checkout, follow the explicit acquisition and source-attribution
instructions for the [room recording](temporal-recorded-motion.md#reproduction)
and [official desk2 recording](independent-recorded-motion.md#regression-reproduction).
Raw images, complete tables and the official source archive are not supplied
by the public report packet. Keep the desk2 archive outside evidence uploads.
Acquiring the same viewed inputs does not recreate a fresh accuracy trial.

Use the existing optional RGB-D environment, or prepare an isolated Python
environment with the pinned audit dependencies:

```sh
python3 -m venv /tmp/rustdriving-rgbd-venv
. /tmp/rustdriving-rgbd-venv/bin/activate
python3 -m pip install -r scripts/requirements-visual.txt
```

The commands below run from the repository root and assume the selected
manifest/raw directory has been acquired and verified. Choose fresh output
paths. Both datasets are viewed: these commands generate regression evidence,
not a new first trial.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-rgbd-multiscale-temporal
temporal_binary=integrations/rgbd/target/release/rustdriving-rgbd-multiscale-temporal
temporal_manifest=data/tum-fr1-room-temporal/manifest.json
temporal_raw=data/tum-fr1-room-temporal/raw
temporal_run=artifacts/multiscale-temporal/local-room
mkdir -p "$temporal_run"
"$temporal_binary" --prepare-freeze --manifest "$temporal_manifest" \
  --output "$temporal_run/freeze.json"
"$temporal_binary" --manifest "$temporal_manifest" --raw "$temporal_raw" \
  --freeze "$temporal_run/freeze.json" --output "$temporal_run/report.json"
```

For desk2, select `data/tum-fr1-desk2-independent/manifest.json`, its `raw`
directory and a separate new output directory. Use the actual binary path if
`CARGO_TARGET_DIR` is configured. Preserve the evaluator's real exit status:
**0** means both complete physical protocols pass; **1** means valid inputs with
an accuracy/availability failure; **2** means an input, label or I/O failure.
The first recorded outcomes exit 1 and retain their reports. Preserve that
status, then run the independent audit on the retained report. A shell using
`set -e` must handle this expected exit explicitly so it does not skip the audit.

The independent auditor's interfaces are below. The first full recorded-data
audits and the complete repeat suite pass integrity.

```sh
python3 scripts/check-multiscale-temporal.py \
  --report "$temporal_run/report.json" --manifest "$temporal_manifest" \
  --raw "$temporal_raw" --freeze "$temporal_run/freeze.json" \
  --output "$temporal_run/oracle.json"
"$temporal_binary" --control --output "$temporal_run/control.json"
python3 scripts/check-multiscale-temporal.py \
  --report "$temporal_run/control.json" --output "$temporal_run/control-oracle.json"
python3 scripts/check-multiscale-temporal.py --self-test \
  --output "$temporal_run/self-test.json"
"$temporal_binary" --executable-receipt --output "$temporal_run/executable-receipt.json"
```

Control mode audits an actual analytic sensor/state export; self-test mode
separately checks boundary and composition contracts. The complete auditor
reconstructs both branches' measured geometry, root composition and
observed/accepted clocks, retains all 180 rows, and independently recomputes
root errors and summaries. Its evidence binds the design and original
mathematical-oracle source hashes. The executable receipt is separate from the
deterministic report. Integrity success is separate from physical acceptance.

The `scripts/check-multiscale-temporal-suite.py` wrapper preserves first
sources/outcomes and reproduces both viewed comparisons in a fresh directory,
including their expected physical failure and independent audits:

```sh
python3 scripts/check-multiscale-temporal-suite.py \
  --binary "$temporal_binary" \
  --output artifacts/multiscale-temporal/local-suite
```

Do not create that output directory in advance. The completed suite records
seven stages: executable receipt **0**; room freeze **0**, evaluation **1** and
audit **0**; desk2 freeze **0**, evaluation **1** and audit **0**. It verifies exact
source/protocol bindings and complete non-timing reports for both recordings.
Suite exit 0 means the preserved failed outcomes were reproduced and
independently audited, not that the physical protocols passed. No retuning after the first outcome
is allowed for this fixed comparison. Current remote CI success is not claimed
by these local checks.

The final source passes [required core checks](../assets/multiscale-temporal-v1/core-validation.json)
with **371 Rust tests**, existing
49 executions/replays and 245 byte-identical generated files. Optional RGB-D
formatting, release Clippy and release all-targets checks pass **94 tests**
([optional proof](../assets/multiscale-temporal-v1/optional-validation.json)).
The opening native 3D GIF is unchanged. These build/regression checks demonstrate
reproducibility separately from the preserved failed camera-motion criterion;
they do not increase the maturity estimate.

This remains offline indoor camera-motion research, separate from the primary
reference/RNE driving pipeline. It supplies no vehicle integration, measured
extrinsics, calibrated covariance or accumulated confidence. More accepted fits
cannot establish permission to drive. The project remains **about 20% mature
by subjective comparison with Autoware, Apollo and openpilot**, not a measured
benchmark; the **30% and 50%** waypoints remain unmet until their broader
capability and validation gates are satisfied.
