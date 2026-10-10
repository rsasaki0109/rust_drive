# Second preregistered recorded-motion temporal subset

This twelve-frame TUM RGB-D Kinect subset selects original depth-index entries
**260–271**, spanning 1305031110.835248–1305031111.199395 seconds. Selection was
declared before acquisition, geometry decoding or fitting. Independent mocap
**timestamp availability only** verified that each interpolation bracket is at
most 0.02 seconds. Neither reference transforms nor fit/accuracy outcomes selected
frames. It follows the frozen 200–211 trial's missing-reference failure; that
trial remains archived and is now viewed regression, not fresh evaluation.

All 11consecutive pair estimates and all 11first-cloud map fits remain in the
physical denominator. No settings were tuned after the first trial: registration,
preprocessing, covariance allowances and all numeric accuracy/validity gates are
unchanged. The new source/checker protocol is separately frozen before its first
fit. See [protocol and retained failures](../../docs/recorded-motion.md).
The inherited first-three-frame `calibration` labels are format compatibility;
no new frame tuned this protocol. The nine later pairs are labelled `held_out`;
all eleven second-trial pairs were freshly unfitted.

Repository `MarcelBruckner-TUMProjects/3D-Scanning-Motion-Capture`, revision
`f367047ee71f5304c6d7deaec55c4874bb8b035e`, source directory
`Exercise_1/data/rgbd_dataset_freiburg1_xyz/`. Exact sizes and SHA-256 are committed
in the manifest. Registered-depth/default teaching calibration is fx=fy=525,
cx=319.5, cy=239.5,5000 depth units/metre; optical x-right/y-down/z-forward.
Independent world-from-camera mocap uses metres and xyzw quaternions; its poses
enter scoring only after sensor fitting. No vehicle calibration is supplied.

This is the **same indoor room, camera and sequence** as all previous windows,
not an independent environment or automotive localization benchmark. Raw inputs
remain ignored and are not redistributed. Original TUM licence terms were not
independently retrievable: the official page
<https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned HTTP 403 through the
configured proxy on 2026-10-10. A public mirror supplies transfer, not a dataset
redistribution grant. No direct-network/policy bypass or TLS exception was used.

```sh
python3 scripts/fetch-recorded-motion.py --dataset tum-fr1-xyz-motion-v2
python3 scripts/fetch-recorded-motion.py --dataset tum-fr1-xyz-motion-v2 --verify-only
```

Transfer verifies every pinned source hash and bounds files/total size. Its
one-time `--prepare-manifest` step refuses to overwrite the existing manifest;
normal reproduction uses the committed selection and hashes.
