# Provided-file depth evaluation

`scripts/evaluate-kitti-depth.py` executes the projection and sparse-depth
binaries on a frozen selection, then scores the resulting cells against
supplied annotated depth PNGs. It does not download data or submit results.

The current real KITTI run remains **blocked by missing authorized inputs**.
The official [raw-data page](https://www.cvlibs.net/datasets/kitti/raw_data.php)
requires login; the [depth-completion page](https://www.cvlibs.net/datasets/kitti/eval_depth.php?benchmark=depth_completion)
routes all depth downloads through login too. Both pages were successfully
retrieved from this environment for the current access check. No account or
local KITTI files are available, and no historical download path is used to
circumvent that gate. [Current access/status record](../assets/kitti-depth-evaluation-v1/status.json).

An additional [Argoverse measured-cloud experiment](argoverse-sparse-depth.md)
actually runs the binaries on an accessible official sample. Its mathematical
consistency results remain separate from KITTI and physical depth accuracy.

## Fixed metric contract

The reference must be a grayscale 16-bit PNG with `metres = value / 256`;
zero denotes missing reference. It must have the same full canvas as the
prediction. Cropped validation-selection images are rejected, without implicit
resizing or offset estimation. Use full-resolution labelled `train`/`val` maps
associated with the same recording, original frame ID and camera 2 or 3.

All finite positive predictions at valid reference pixels enter:

- RMSE and MAE in metres;
- inverse RMSE and MAE in 1/km, using `1000 × (1/prediction − 1/reference)`;
- reference coverage, `paired pixels / all valid reference pixels`;
- prediction density and missing-reference/prediction counts.

Errors are conditional on overlap. Missing predictions stay in the coverage
denominator; they are never filled with zero depth or interpolated. There is
no Eigen crop, depth-range clipping, learned completion or official benchmark
background interpolation. Pixels are pooled across scored frames, rather than
averaging per-frame RMSE. No accuracy acceptance threshold is invented.
An error-free sparse overlap can still have poor coverage.

Every selected frame stays in the output, including process failures, invalid
references and zero overlap. Aggregate metrics are explicitly labelled
`metrics_over_scored_frames`; counts expose unavailable frames. Zero overlap
has null errors, not zero errors. `complete_evaluation` means every frame has
some comparable pixels, without asserting full coverage or quality acceptance.
Annotated references may share LiDAR measurements with inputs, so comparison
does not establish independent sensor calibration accuracy.

## Supply a manifest and freeze it

All file names are explicit relative POSIX paths under the selected data root;
input symlinks, traversal, repeated frame IDs and mismatched source file names
are rejected. The selection is bounded to 1–20 frames per run.

```json
{
  "schema_version": 1,
  "recording_id": "2011_09_26_drive_0001_sync",
  "camera_index": 2,
  "split": "val",
  "reference_kind": "kitti_annotated_depth",
  "reference_encoding": "uint16_png_div_256",
  "provenance": "Describe the authorized official archive and acquisition here",
  "calibration": {
    "camera": "2011_09_26/calib_cam_to_cam.txt",
    "velo": "2011_09_26/calib_velo_to_cam.txt",
    "imu": "2011_09_26/calib_imu_to_velo.txt"
  },
  "frames": [
    {
      "frame_id": "0000000005",
      "points": "2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000005.bin",
      "reference_depth": "val/2011_09_26_drive_0001_sync/proj_depth/groundtruth/image_02/0000000005.png"
    }
  ]
}
```

The example describes paths and schema; it is not an acquired or evaluated
selection. Do not infer an available labelled drive/split from the example.
The provenance and recording association are supplied declarations, not
cryptographic authentication of dataset membership.

With both binaries built, run in the pinned NumPy/Pillow environment:

```sh
python scripts/evaluate-kitti-depth.py \
  --manifest /path/to/selection.json --root /path/to/authorized-kitti \
  --projection-binary integrations/rgbd/target/release/rustdriving-kitti-project \
  --depth-binary integrations/kitti-depth/target/release/rustdriving-kitti-depth \
  --prepare-freeze --output artifacts/kitti-depth-evaluation/prepared
python scripts/evaluate-kitti-depth.py \
  --manifest /path/to/selection.json --root /path/to/authorized-kitti \
  --projection-binary integrations/rgbd/target/release/rustdriving-kitti-project \
  --depth-binary integrations/kitti-depth/target/release/rustdriving-kitti-depth \
  --freeze artifacts/kitti-depth-evaluation/prepared/freeze.json \
  --output artifacts/kitti-depth-evaluation/first
```

Preparation hashes encoded input bytes, the selection, binaries, evaluator and
metric contract without decoding references or running the estimator. Evaluation
requires that freeze to match. All operational projections/rasterizations finish
before any reference PNG is decoded. Output directories must be new.
Exit 0 means every selected frame was comparable; exit 1 preserves a complete
report with unavailable/unscorable frames; exit 2 is an admission/input error.
None of these is an accuracy approval or an official KITTI score.

Thirteen tests include hand-computed units/errors/coverage, no-overlap/null
metrics, invalid PNG/cell controls, pixel-weighted aggregation, metadata-only
freeze preparation, changed-input refusal, actual authored-fixture binary
execution and exact repeat results. These verify the evaluator, without being
real KITTI accuracy evidence.

```sh
python scripts/test-evaluate-kitti-depth.py
```
