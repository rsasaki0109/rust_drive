# Adaptive ground extraction on measured terrain

The optional Rust density-aware terrain extractor improves agreement on the
previously viewed ISPRS scans, but **fails the newly frozen Autzen acceptance**.
It is an offline research algorithm; it does not authorize road-ground removal
in the driving pipeline. The original progressive morphological filter (PMF)
and its failed generalization results remain unchanged.

`rustdriving_perception::terrain_adaptive::classify_ground_adaptive` receives only
XYZ in an explicit metre, Z-up frame. It builds a sparse minimum-height grid,
screens elevated returns with a slope-limited lower envelope, gathers support
by physical XY distance, and fits deterministic robust local planes. Unsupported
points remain non-ground. Bounds cover input points, cells and candidate work;
defaults allow 500,000 points, 250,000 cells and 100,000,000 work units. Confidence
measures available geometric support, not a calibrated probability of correctness.
A result with insufficient confidence must not authorize terrain removal.

The frozen defaults use a 1 m cell, 16 m support radius, slope limit 0.45,
0.18 m residual limit and 6–32 support neighbours. At least 12 supported cells
and a 0.5 supported-cell fraction are required for confidence. Broad isolated
roofs, shallow ground-like obstacles, discontinuities and poor density remain
ambiguous. This is not automotive semantics or a model of LiDAR beam acquisition.

## Measured results and split history

Six ISPRS samples from sites 1–2 informed calibration. Sites 3–7 were already
viewed during the original PMF evaluation, so the nine samples now serve as
**regression data**, even though their acquisition manifest retains the historic
`held_out` field. They cannot establish fresh generalization for this algorithm.

| Evaluation partition | Samples | Precision | Recall | Micro F1 |
|---|---:|---:|---:|---:|
| Original PMF calibration | 6 | 0.9457 | 0.5608 | 0.7041 |
| Original PMF held out at its freeze | 9 | 0.9310 | 0.1505 | 0.2592 |
| Adaptive calibration | 6 | 0.9859 | 0.7041 | 0.8215 |
| Adaptive previously viewed regression | 9 | 0.9636 | 0.7036 | 0.8133 |
| Adaptive fresh Autzen reference comparison | 1 | 1.0000* | 0.2174 | 0.3571 |

The original PMF failed six sparse samples with F1 zero. The adaptive aggregate
also hides material variation: calibration `samp11` has F1 0.5187 and reports
insufficient confidence; regression `samp42`, `samp52` and `samp53` have F1 below
0.8. Confidence alone does not establish accuracy.

| Sample | Partition | Adaptive F1 | Support confidence |
|---|---|---:|---|
| samp11 | Calibration | 0.5187 | Insufficient |
| samp12 | Calibration | 0.9011 | Sufficient |
| samp21 | Calibration | 0.9365 | Sufficient |
| samp22 | Calibration | 0.8841 | Sufficient |
| samp23 | Calibration | 0.8157 | Sufficient |
| samp24 | Calibration | 0.8688 | Sufficient |
| samp31 | Regression | 0.9657 | Sufficient |
| samp41 | Regression | 0.8812 | Sufficient |
| samp42 | Regression | 0.6784 | Sufficient |
| samp51 | Regression | 0.8696 | Sufficient |
| samp52 | Regression | 0.6419 | Sufficient |
| samp53 | Regression | 0.7295 | Sufficient |
| samp54 | Regression | 0.8261 | Sufficient |
| samp61 | Regression | 0.8897 | Sufficient |
| samp71 | Regression | 0.8622 | Sufficient |
| Autzen | Fresh reference comparison | 0.3571 | Insufficient |

[The source/configuration freeze](../assets/adaptive-ground-freeze.json) was
recorded before inspecting the fresh Autzen result. Its unchanged gate requires
precision ≥ 0.9, recall ≥ 0.7, F1 ≥ 0.8 and sufficient confidence. Autzen fails
recall, F1 and confidence. No parameter was retuned to turn this result into a pass.

*Autzen's scored references contain **2,719 ground points and zero labelled
non-ground points**: TP 591, FN 2,128, FP/TN zero. Another 7,934 of its 10,653
coordinates have excluded reference classes. Every coordinate still enters
classification; exclusions apply only when scoring. Reported precision 1.0 is
therefore an arithmetic result on positive-only references and **does not measure
false-ground rejection**. The embedded TerraScan classifications are pre-existing
upstream references; independently reviewed manual truth is not established.
Autzen is one new physical environment, not multiple independent test sites.

The evaluator decodes measured coordinates before processing, converts Autzen's
international feet to metres with factor 0.3048, and recentres at the first
measured point. It reads/decodes reference labels only after classification.
Original point indices, excluded scoring indices, confusion matrices, confidence,
input hashes and observed XYZ object bounds remain in the full JSON reports.
Measured object bounds do not imply semantic identity or hidden physical extents.

## Reproduce

From the repository root, acquire the ignored SHA-pinned raw files:

```sh
python3 scripts/fetch-datasets.py --dataset isprs-terrain
python3 scripts/fetch-additional-datasets.py --dataset pdal-autzen
cargo run --release --locked --bin rustdriving-dataset-eval -- adaptive-ground \
  --split calibration_original --output artifacts/adaptive/calibration.json
cargo run --release --locked --bin rustdriving-dataset-eval -- adaptive-ground \
  --split regression --output artifacts/adaptive/regression.json
cargo run --release --locked --bin rustdriving-dataset-eval -- adaptive-ground \
  --dataset pdal-autzen --split fresh_heldout \
  --freeze assets/adaptive-ground-freeze.json \
  --output artifacts/adaptive/autzen.json
```

Fresh evaluation requires matching algorithm-source, configuration and manifest
hashes in the freeze. Successful evaluation writes a completed report; that does
not mean the accuracy gate passed. A failed invocation invalidates an earlier
completed report rather than leaving a stale success artifact.

The measured release-profile timings were **273.822–1,799.758 ms** across the 15
ISPRS scans and **105.158 ms** for the single Autzen scan. Each is one synchronous
wall-clock measurement of classification, excluding file parsing and scoring.
These are neither a repeatable latency distribution nor a real-time guarantee.
Point counts range from 7,492 to 52,119 for ISPRS. Acquisition and parsing bounds
limit resources; they do not establish a runtime deadline.

The final local reports are `/tmp/adaptive-calibration-original-final.json`,
`/tmp/adaptive-regression-original-final.json` and `/tmp/adaptive-fresh-autzen.json`.
Raw data is ignored; see [ISPRS provenance](../data/isprs-terrain/SOURCE.md),
[Autzen provenance and licence scope](../data/pdal-autzen/SOURCE.md) and the
[unchanged PMF failure stage](datasets.md). Generalization and operation on
automotive recorded sensors remain unvalidated.

## Native pipeline check

The optional `--adaptive-terrain` profile runs the same bounded classifier on
validated native XYZ returns, caps points/cells at 20,000 and work at 8 million,
and retains braking on unsupported acquisitions. It preserves the previous PMF
profile and default outputs. Low measured objects and overhead geometry are
exercised by acquisition/replay tests; this is geometric separation, not semantics.

```sh
bash scripts/setup-rne.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
cargo build --release --locked --bin rustdriving
python3 scripts/check-adaptive-native.py --compact --output artifacts/adaptive-native
```

The independently checked 22-second lead-stop run advances 23.54 m, ends at zero
speed and retains a 4.872 m bounded physical actor clearance. All 441 sensor-only
replay outputs agree. It has no ground-only collision-relevant AABB, but 14 mixed
AABBs retain 111 ground points; complete ground removal is not established.
All 221 acquisitions are confident in this positive run. Fault/recovery behavior
is demonstrated by pipeline tests, not a native fault-injection claim.
[Native evidence](../assets/adaptive-native-results.json) remains one authored
success beside the separate failed Autzen generalization result.

The frozen version-1 report schema and its algorithm identity marker retain their
legacy `rustdrive` spelling for compatibility with published evidence. The
current Cargo package and Rust module use `rustdriving-perception` and
`rustdriving_perception`; this project rename does not revise the frozen
classification parameters or historical results. Source hashes change with the
import spelling, so new validation must record its current source fingerprint.
