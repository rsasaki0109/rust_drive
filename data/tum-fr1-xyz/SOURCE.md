# TUM RGB-D measured motion subset

These are recorded Kinect depth frames and the original TUM RGB-D
`rgbd_dataset_freiburg1_xyz` motion-capture camera trajectory. They are real
geometry and real camera motion, rather than transformed copies of one cloud.
This is **one short indoor sequence**, not vehicle localization, independent
scene generalization, a camera perception model or a calibrated vehicle sensor.

`manifest.json` pins the public teaching mirror
`MarcelBruckner-TUMProjects/3D-Scanning-Motion-Capture` at revision
`f367047ee71f5304c6d7deaec55c4874bb8b035e`, paths, exact sizes and SHA-256 hashes.
The source directory is `Exercise_1/data/rgbd_dataset_freiburg1_xyz/`.
Both index and ground-truth headers identify the original TUM `.bag` sequence.
The complete 201,100-byte `groundtruth.txt` was also checked byte-for-byte
against the independent `geohot/twitchslam` mirror
`videos/groundtruth/freiburgxyz.txt`; this corroborates transfer, not measurement
accuracy. The original dataset is described at
<https://cvg.cit.tum.de/data/datasets/rgbd-dataset> (formerly `vision.in.tum.de`).

Selection was fixed before any registration scoring: source depth-index entries
0, 10, 20, …, 110 inclusive. Twelve frames cover 1305031102.160407 to
1305031105.830008 seconds, about 3.67 seconds. The first three frames are a
temporal calibration partition and the remaining nine a temporal held-out
partition. All eleven consecutive selected-frame pairs may be reported; none
should be discarded for a poor fit. These partitions share the same environment.

The PNGs are 640 × 480, 16-bit grayscale depth. Zero is invalid; depth in metres
is the unsigned value divided by 5000. The pinned upstream
[`Exercise_1/VirtualSensor.h`](https://github.com/MarcelBruckner-TUMProjects/3D-Scanning-Motion-Capture/blob/f367047ee71f5304c6d7deaec55c4874bb8b035e/Exercise_1/VirtualSensor.h)
lines 48–53 use registered-depth/default intrinsics `fx = fy = 525`,
`cx = 319.5`, `cy = 239.5`; lines 82–91 document the depth scale and link the
original TUM file-format specification. Back-project into the optical frame
with x right, y down and z forward. These are the teaching reader's registered
depth calibration. Separately measured native depth-camera extrinsics, vehicle
extrinsics and a calibration uncertainty budget are **not supplied**.

Ground-truth records are `timestamp tx ty tz qx qy qz qw`: a world-from-camera
translation in metres and an xyzw quaternion. Ground truth is **evaluation only**:
it must not initialize the matcher, level clouds, establish correspondences or
choose pairs. The sequence has natural 6-DoF motion; a 2D matcher needs a stated,
verified projection limitation and cannot silently discard roll/pitch/height.
Pose interpolation must be time-bounded, wrap rotations correctly and preserve
the chosen transform direction.

The official source's dataset license was not independently retrieved in this
environment. A public teaching mirror alone does not establish redistribution
rights, and its code license cannot replace original dataset terms. Consequently
raw PNG/index/trajectory bytes stay ignored and **are not redistributed**.
These manifests provide optional research acquisition, with source attribution;
they make no new license grant. Verify original terms before redistributing data.

```sh
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz --verify-only
```

Acquisition is bounded to small SHA-pinned files through the inherited HTTPS
proxy. No direct networking, TLS exception, official-host policy bypass, full
archive download or learned model is needed.
