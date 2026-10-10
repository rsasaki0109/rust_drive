# Preregistered measured keyframe temporal subset

This optional TUM RGB-D Kinect subset contains **36 consecutive original depth
index entries 340–375**, timestamps 1305031113.507987–1305031114.675655 seconds
(1.167668 seconds). The fixed window was declared before image acquisition,
depth decoding or sensor fitting. Selection checked **timestamp availability
only**: every motion-capture interpolation bracket is at most 0.02 seconds,
with observed maximum 0.010100127 seconds. Reference pose values, sensor errors
and acceptance outcomes did not choose the window.

Frame 340 initializes a measured reference cloud; all **35 subsequent updates**
are temporal held-out observations. Earlier source windows through index 271
are viewed calibration/regression data. Registration, preprocessing, reference
replacement/expiry, uncertainty allowances and numerical accuracy gates must
be source/hash frozen before the first decode of this subset. Rejections and
missing references remain in the physical denominator; no post-score tuning,
truth reset or invented pose interpolation is permitted.

The first protocol-5 trial is now complete and its exact sources, preregistration
and failed outcomes remain immutable in
[the archived baseline](../../integrations/rgbd/baselines/keyframes-first-v1/README.md)
and `assets/recorded-keyframes/first-temporal-v1/`: 33 of 35 updates were
accepted, only 13 met root accuracy gates, and two ambiguous fits rejected.
An independent integrity audit preserves every failure. A subsequent mandatory
chronology/expiry ordering fix changes source hashes and uses **protocol 6**;
all subsequent fits on these same frames are **viewed regression**, never a new
held-out trial. Registration thresholds and uncertainty allowances did not
change. The original manifest's selection statement describes the first trial.

Source repository: `MarcelBruckner-TUMProjects/3D-Scanning-Motion-Capture`, revision
`f367047ee71f5304c6d7deaec55c4874bb8b035e`, directory
`Exercise_1/data/rgbd_dataset_freiburg1_xyz/`. All 38 input files (depth index,
evaluation-only motion capture and 36 depth PNGs) have exact transfer lengths
and SHA-256 in [manifest.json](manifest.json). Acquisition bounds each file to
512 KiB and the full subset to 8 MiB, preserves configured HTTPS/TLS and does
not decode PNGs. Registered-depth teaching calibration is fx=fy=525,
cx=319.5, cy=239.5, 5000 units/metre; optical x-right/y-down/z-forward. Mocap
world-from-camera poses use metres and xyzw quaternions and enter scoring only
after the operational sensor-fitting phase. No measured vehicle calibration
or native-depth-camera extrinsics are supplied.

This is the **same indoor room, camera and sequence** as previous trials,
not an independent environment, driving benchmark or automotive validation.
Raw data remain ignored and are not redistributed. Original TUM licence terms
were not independently retrievable: the official dataset page
<https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned HTTP 403 through the
configured proxy during previous acquisition. The pinned public teaching
mirror supplies transfer provenance, not a redistribution grant. Fetch is
explicit opt-in; no raw image or full sequence is bundled with the repository.

```sh
python3 scripts/fetch-keyframe-dataset.py
python3 scripts/fetch-keyframe-dataset.py --verify-only
```

The one-time `--prepare-manifest` operation refuses to overwrite an existing
manifest; normal reproduction uses committed selection, lengths and hashes.
