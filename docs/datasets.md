# Measured-data evaluation sources

RustDriving has revision- and SHA-pinned acquisition manifests for two small measured point-cloud sources. **Acquisition verification is separate from algorithm acceptance**: downloaded bytes and documented labels establish inputs, not perception or localization performance. Raw data remain ignored because repository-level BSD notices do not establish separate redistribution rights for the identified original third-party datasets.

```sh
# Requires Python 3 and curl; checks TLS, sizes and SHA-256 before installation.
python scripts/fetch-datasets.py
python scripts/fetch-datasets.py --verify-only
# Optional output root: files go in artifacts/datasets/DATASET/raw.
python scripts/fetch-datasets.py --output artifacts/datasets
```

The default acquisition contains **32 files / 8,190,410 bytes**, below a 15 MB raw-data budget. It keeps original file bytes without downsampling or persistent ASCII expansion. A failed download or checksum mismatch does not replace existing data with unverified bytes. See the versioned manifests for every hash and source path.

The acquisition script rejects malformed pins, path traversal, duplicate destination names (including case-only Windows collisions), oversized manifests and nonpositive/excessive file sizes before downloading. `python scripts/test-fetch-datasets.py` checks these failures, same-size raw-data tampering and preservation of existing data when a download has the wrong bytes, without network access.

| Source | Documented measured origin | Evaluator reference | Calibration / held-out | Scope |
| --- | --- | --- | --- | --- |
| [ISPRS terrain via PCL](../data/isprs-terrain/SOURCE.md) | Airborne laser scanning; 2003/2004 ISPRS filter comparison | Paired ground-only point clouds | Sites 1–2 / sites 3–7 | Ground extraction, not automotive beam or vehicle-motion reproduction |
| [ASL apartment via libpointmatcher](../data/libpointmatcher/SOURCE.md) | Tutorial explicitly identifies real apartment scans | No independent physical relative-pose ground truth | cloud_0 / cloud_1 | Semi-synthetic transform recovery on measured geometry; natural-pair residuals only |

Freeze algorithm parameters using calibration data before running held-out acceptance. Labels, known perturbations and physical reference poses belong only in the evaluator; algorithms receive the input cloud and explicitly documented initialization/calibration. Record configuration, raw hashes, coordinate transforms and all failures alongside scores. Report each sample as well as aggregate metrics; an aggregate must not conceal a failed held-out site. Any use of held-out results to change settings invalidates that held-out status and must be disclosed.

PCD terrain files are LZF `binary_compressed` XYZ float32, with large UTM coordinates and existing float32 quantization. The apartment files are ASCII VTK POLYDATA. Preserve the raw files and use bounded parsing; derived coordinates need explicit frame/unit assumptions. No benchmark here supplies an end-to-end real driving episode, automotive annotations, dynamic traffic or a certified safety error bound.

The car example files from libpointmatcher are omitted: their acquisition is insufficiently documented and supplied transforms are not independently measured pose truth. No synthetic clouds, authored perturbations or simulator ray casts should be presented as original measured sensor motion.

## Executable baseline and observed failures

```sh
cargo run --release --locked --bin rustdriving-dataset-eval -- \
  --python python --output artifacts/datasets/report.json
python scripts/check-datasets.py --report artifacts/datasets/report.json \
  --output artifacts/datasets/oracle.json
python scripts/test-fetch-datasets.py
```

The evaluator uses native bounded PCD/LZF and ASCII VTK readers, verifies all raw hashes before and after processing, and invalidates old completed reports when a new invocation fails. `--data-root` selects the same alternate acquisition root as the fetcher's `--output`. Python and curl serve acquisition and hashing; ground classification, XYZ components and registration run in Rust. The historical PCL files contain zero padding after the declared compressed block; readers validate that reservation separately from the block and reject nonzero trailing bytes.

Parameters were frozen before evaluating sites 3–7. The PMF baseline uses a 1 m minimum-height grid, 0.15 m initial height, slope 0.3, maximum height 2.5 m and increasing radii 1/2/4/8/16 cells. Neighbor support must be sufficient; it does not extrapolate unknown terrain. Ground labels never enter the algorithm. An exact float32 XYZ subset lookup supplies evaluator labels; repeated identical XYZ samples receive the same reference label, with duplicate counts retained in the full report.

| Split | Sites | Ground precision | Ground recall | Ground micro F1 |
|---|---:|---:|---:|---:|
| Calibration | 6 | 0.9457 | 0.5608 | 0.7041 |
| Held out at parameter freeze | 9 | 0.9310 | 0.1505 | 0.2592 |

These are failed generalization results, not an accuracy acceptance. Six sparse sites (`samp51`, `52`, `53`, `54`, `61`, `71`) have F1 **0** and insufficient support. The baseline stays unchanged; an aggregate must not hide those sites. All 15 measured clouds were processed, totaling 384,955 input samples; the independent Python oracle reconstructs every confusion matrix and 5,542 observed XYZ AABBs from raw input and original point indices. AABBs describe measured surfaces, without object semantics, hidden geometry or automotive annotation accuracy.

The apartment evaluation selects measured points at Z 0.5–2 m and the first point in each 0.2 m XY voxel, retaining 179/161 points from the two views. Four **semi-synthetic**, warm-initialized SE(2) cases impose known transforms and 1 mm deterministic noise on those measured points; all four recover translation/yaw within 0.0001 m/rad. Initial error is only `(0.04 m, -0.03 m, 0.015 rad)`. This does not establish convergence from a poor/global initialization. Conditional least-squares covariance and nominal ellipsoid statistics are reported; four correlated cases cannot establish calibration or 95% coverage. The untransformed natural scan pair is **rejected for insufficient overlap**, with no physical pose-accuracy claim.

[Compact scores, parameters and failures](../assets/measured-data-results.json) retain provenance; the ignored full report carries every predicted index for independent reconstruction. Nine oracle mutations reject false scores, missing sites, changed splits/parameters, altered provenance, duplicated indices, fabricated extents/errors and invented natural-pair ground truth. Future changes informed by these results must call these sites regression data and obtain a new independent held-out source before claiming generalization.
