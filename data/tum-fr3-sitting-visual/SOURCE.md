# Preregistered new natural recorded sequence: paired RGB-D

This optional subset contains **36 consecutive original depth-index entries
100–135**, timestamps 1341845824.165759–1341845825.311705
seconds (1.145946 seconds), from `TUM RGB-D rgbd_dataset_freiburg3_sitting_xyz`.
Each depth timestamp is paired with the nearest original RGB timestamp; an exact
tie chooses the earlier RGB index. Maximum absolute RGB/depth clock gap is
**0.006021976 seconds**, below the fixed 0.02-second bound.
All 36 observations have motion-capture interpolation brackets below 0.02 seconds
(observed maximum 0.012899876 seconds). Selection
inspected timestamps and file availability only, without RGB/depth pixel
contents, sensor errors or reference pose values.

This new interval was declared before RGB/depth/mocap metadata acquisition or pixel decoding. The initially declared `freiburg3_walking_rpy` indices 100–135 failed timestamp availability: maximum nearest-RGB gap 0.031836987 seconds exceeded 0.02 seconds. That candidate was not selected. Its pixel contents and reference poses were never inspected. This sitting interval was selected using timestamps only. Freeze final algorithms, preprocessing, acceptance, evaluator and independent checker before first pixel decoding. Frame 100 initializes and all 35 following observations are held out; every rejection remains in that denominator. After its first evaluation this same interval is viewed regression.

There are **35 unique recorded RGB images** and **1
repeated RGB associations** among the 36 retained depth observations. Raw RGB
files are deduplicated without changing source timestamps or pixel bytes.
Repeated RGB timestamps must produce explicit operational rejection without
clock renewal; they cannot silently become another visual observation. No
frame is removed from the physical denominator, image resized, synthesized,
interpolated or retimed. RGB/depth synchronization and registration remain
source acquisition assumptions rather than independently measured extrinsics.

The exact source mirror is `yakki12345/DygeoSLAM`, commit
`eae444878fc663fbd41307b1e655e762f988c714`, directory `data/rgbd_dataset_freiburg3_sitting_xyz/`.
[manifest.json](manifest.json) pins source paths, lengths and SHA-256 for
`depth.txt`, `rgb.txt`, evaluation-only `groundtruth.txt`, 36 depth PNGs and
35 unique RGB PNGs. Every primary input was additionally verified against
its exact commit's Git-tree blob identity. PNG signature/IHDR metadata confirmed
640×480 dimensions, uint16 grayscale depth and uint8 RGB/RGBA colour; this header
inspection did **not** decode pixel data. Acquisition is byte-only and leaves
reference poses unparsed; original TUM motion-capture world-from-camera poses
(metres, xyzw quaternions) may enter physical scoring only after all operational
fits finish. The official archive and independent mirrors were not byte-verified.

The separately pinned calibration document comes from `luigifreda/pyslam`,
commit `96019cfafcfc099ac9866884d7143a9ed1451a0d`, `settings/TUM3.yaml`,
1,520 bytes, SHA-256 `251e345996befa8057f5c51642bd4fe93a92907dac2e0564f400cd87eaf5785d`. It is cached only in ignored raw
`camera-calibration.yaml`. Projection uses published fx=535.4,
fy=539.2, cx=320.1, cy=247.6, 5000 depth units/metre and the
optical x-right/y-down/z-forward frame. Published FR3 pinhole camera calibration and zero RGB distortion coefficients; registered depth used without additional pixel undistortion or range correction. Native infrared extrinsics, in-situ calibration verification and vehicle calibration are unavailable.
No extra range correction or operational reference-pose prior is supplied.

The exact subset contains 75 cached source files and
**24,434,187 bytes**, including its calibration document.
Before acquisition the bound was increased to 32 MiB per dataset and 4 MiB per
source file because original RGB PNGs could not fit the initial 15 MiB budget.
Original source bytes are preserved. This helper and metadata are new; earlier
submap manifests, raw inputs, fetchers, calibration and proof hashes remain
unchanged.

This is an indoor recorded sequence. It differs from desk/office records, while independence of the physical room or camera from other Freiburg 3 sequences is not established.
A short interval does not establish long-duration mapping, automotive sensing,
real-vehicle safety or generalization. Motion-capture labels are scoring-only.

Raw data and calibration YAML remain ignored and are **not redistributed**.
The public code/data mirrors provide transfer provenance and no new original
data redistribution grant. Original TUM licence terms remain unverified: the
official dataset page <https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned
HTTP 403 through the configured proxy during earlier acquisition. Fetch is
explicit opt-in, retaining configured HTTPS proxy routing and TLS verification.

```sh
python3 scripts/fetch-visual-datasets.py tum-fr3-sitting-visual
python3 scripts/fetch-visual-datasets.py tum-fr3-sitting-visual --verify-only
```

Reproduction reads the committed exact inventories and hashes. It never prepares,
overwrites or silently reselects a manifest.
