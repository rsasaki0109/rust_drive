# Viewed development interval: paired RGB-D

This optional subset contains **36 consecutive original depth-index entries
100–135**, timestamps 1305031456.709099–1305031457.875920
seconds (1.166821 seconds), from `TUM RGB-D rgbd_dataset_freiburg1_desk`.
Each depth timestamp is paired with the nearest original RGB timestamp; an exact
tie chooses the earlier RGB index. Maximum absolute RGB/depth clock gap is
**0.017854929 seconds**, below the fixed 0.02-second bound.
All 36 observations have motion-capture interpolation brackets below 0.02 seconds
(observed maximum 0.010299921 seconds). Selection
inspected timestamps and file availability only, without RGB/depth pixel
contents, sensor errors or reference pose values.

The exact previous depth manifest is `data/tum-fr1-desk-submaps/manifest.json` (SHA-256 `f701eb864823536f5c364fab4ddb9053065edba76ab63d0f46c4253d5f078f0d`). Earlier depth inputs and reference transforms have been viewed; newly fetched RGB does not make this interval held out.

There are **26 unique recorded RGB images** and **10
repeated RGB associations** among the 36 retained depth observations. Raw RGB
files are deduplicated without changing source timestamps or pixel bytes.
Repeated RGB timestamps must produce explicit operational rejection without
clock renewal; they cannot silently become another visual observation. No
frame is removed from the physical denominator, image resized, synthesized,
interpolated or retimed. RGB/depth synchronization and registration remain
source acquisition assumptions rather than independently measured extrinsics.

The exact source mirror is `FaridRash/slam-track-fusion`, commit
`477a059d640540b7e23fd56ec95f6458167c7af2`, directory `Data/rgbd_dataset_freiburg1_desk/`.
[manifest.json](manifest.json) pins source paths, lengths and SHA-256 for
`depth.txt`, `rgb.txt`, evaluation-only `groundtruth.txt`, 36 depth PNGs and
26 unique RGB PNGs. Every primary input was additionally verified against
its exact commit's Git-tree blob identity. PNG signature/IHDR metadata confirmed
640×480 dimensions, uint16 grayscale depth and uint8 RGB/RGBA colour; this header
inspection did **not** decode pixel data. Acquisition is byte-only and leaves
reference poses unparsed; original TUM motion-capture world-from-camera poses
(metres, xyzw quaternions) may enter physical scoring only after all operational
fits finish. The official archive and independent mirrors were not byte-verified.

The separately pinned calibration document comes from `luigifreda/pyslam`,
commit `96019cfafcfc099ac9866884d7143a9ed1451a0d`, `settings/TUM1.yaml`,
1,615 bytes, SHA-256 `5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf`. It is cached only in ignored raw
`camera-calibration.yaml`. Projection uses published fx=517.306408,
fy=516.469215, cx=318.64304, cy=255.313989, 5000 depth units/metre and the
optical x-right/y-down/z-forward frame. Published FR1 RGB intrinsics used as an explicit pinhole model for pre-registered depth; no Brown-Conrady pixel undistortion is performed, and native infrared extrinsics/vehicle calibration are unavailable. Source distortion coefficients are retained as provenance, not applied to images or asserted correct after registration.
No extra range correction or operational reference-pose prior is supplied.

The exact subset contains 66 cached source files and
**18,425,659 bytes**, including its calibration document.
Before acquisition the bound was increased to 32 MiB per dataset and 4 MiB per
source file because original RGB PNGs could not fit the initial 15 MiB budget.
Original source bytes are preserved. This helper and metadata are new; earlier
submap manifests, raw inputs, fetchers, calibration and proof hashes remain
unchanged.

This is an indoor recorded sequence. The same depth window was already evaluated; this paired check is development, not independent generalization.
A short interval does not establish long-duration mapping, automotive sensing,
real-vehicle safety or generalization. Motion-capture labels are scoring-only.

Raw data and calibration YAML remain ignored and are **not redistributed**.
The public code/data mirrors provide transfer provenance and no new original
data redistribution grant. Original TUM licence terms remain unverified: the
official dataset page <https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned
HTTP 403 through the configured proxy during earlier acquisition. Fetch is
explicit opt-in, retaining configured HTTPS proxy routing and TLS verification.

```sh
python3 scripts/fetch-visual-datasets.py tum-fr1-desk-visual
python3 scripts/fetch-visual-datasets.py tum-fr1-desk-visual --verify-only
```

Reproduction reads the committed exact inventories and hashes. It never prepares,
overwrites or silently reselects a manifest.
