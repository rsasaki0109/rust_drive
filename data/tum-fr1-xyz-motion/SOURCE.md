# Preregistered recorded-motion temporal subset

This optional research subset selects **source depth-index entries200–211** of
TUM RGB-D `rgbd_dataset_freiburg1_xyz`, from the same pinned public teaching
mirror as the earlier datasets. It comprises twelve recorded Kinect depth PNGs,
the source index and original independent motion-capture trajectory. Raw inputs
remain ignored and are not redistributed. The manifest pins every byte count
and SHA-256. Hashing a transferred PNG does not decode its geometry.

Selection was declared before acquisition or any new depth decoding, alignment
or motion scoring. The evaluator, fixed 0.06 m additional voxel, stricter 0.15 m
correspondence radius, unchanged conservative matcher gates, provisional
0.02 m / 0.03 rad engineering uncertainty allowances and independent checker were
frozen after calibration on previously viewed original0..110/10, fast120..131
and tight140..151 only. All eleven new pairs and all eleven first-cloud map fits
remain in the denominator. The first three frames retain the historical
manifest's `calibration` label for compatibility; **none of these twelve new
frames was used to tune this protocol**. Nine pairs fall in the manifest's
later `held_out` partition. All eleven are fresh temporal evaluation here.

The selected depth times span1305031108.835163–1305031109.203388 seconds.
It is **the same indoor room, sequence and camera**, not an independent scene,
a vehicle sensor or automotive localization benchmark. The physical trajectory
is read only after every operational pair/map fit finishes, with the existing
≤0.02 s mocap interpolation bracket and no extrapolation. Relative transforms
preserve natural 6DoF motion in the optical x-right/y-down/z-forward frame.

Acquisition repository:
`MarcelBruckner-TUMProjects/3D-Scanning-Motion-Capture`, revision
`f367047ee71f5304c6d7deaec55c4874bb8b035e`, directory
`Exercise_1/data/rgbd_dataset_freiburg1_xyz/`.
The inherited teaching reader uses registered-depth/default intrinsics
fx=fy=525, cx=319.5, cy=239.5 and5000 depth units/metre. This is not a separately
measured camera/extrinsic/calibration uncertainty budget.

The official TUM page
<https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned HTTP 403 when requested
through the configured proxy on 2026-10-10. No policy bypass, TLS exception or
redistribution licence is inferred from the mirror. Dataset terms remain
unverified here; raw data stay private/ignored. The Rust implementation is
Apache-2.0 independently of dataset terms. No separately licensed second-room benchmark was acquired in this task.

```sh
python3 scripts/fetch-recorded-motion.py
python3 scripts/fetch-recorded-motion.py --verify-only
```

`--prepare-manifest` was the one-time preregistration transfer step and refuses
to overwrite an existing manifest. Normal reproduction uses the committed
hash-pinned manifest. Failed or mismatched transfers are rejected.
