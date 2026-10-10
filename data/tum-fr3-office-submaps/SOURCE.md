# Preregistered measured Freiburg 3 office subset

This optional subset contains **36 consecutive original depth-index entries
100–135** from TUM RGB-D `rgbd_dataset_freiburg3_long_office_household`, timestamps
1341847984.206723–1341847985.414665 seconds (**1.207942 seconds**). The fixed
window was declared before source metadata/image acquisition, depth decoding or
fitting. Selection inspected only timestamp availability: all 36 original
motion-capture interpolation brackets are at most 0.02 seconds, with observed
maximum 0.014400005 seconds. Reference pose values, geometry, sensor errors and
acceptance outcomes did not choose this interval. Frame 100 initializes the map;
all 35 following updates are temporal held out. Algorithm, evaluator, independent
checker, preprocessing and accuracy settings must be frozen before first
sensor decoding. All rejected or inaccurate frames remain in the denominator;
this window cannot be claimed fresh after its first evaluation.

The source mirror is `shihaozhaosiue/SLAM-project_shihao`, exact revision
`1f3bb58bbcbad2ec405c36d6d5a511d2f1cd050b`, directory
`dataFolder/rgbd_dataset_freiburg3_long_office_household/`. Its depth index
lists 2,509 recorded PNGs. The committed [manifest.json](manifest.json) pins
lengths and SHA-256 for the complete depth index, evaluation-only
`groundtruth.txt` and 36 selected PNGs. Each primary input was additionally
verified against the source commit's exact Git-tree blob identity. Acquisition
was byte-only: no depth decoding, geometry fitting or reference-pose values
were inspected. Each source file is below 1 MiB and the complete subset,
including the separately pinned calibration document, is below 12 MiB.

Evaluation-only motion-capture poses are original TUM world-from-camera
transforms, with positions in metres and xyzw quaternions. They may enter
physical scoring only after all operational sensor fits finish. The original
official archive and an independent second source were not byte-verified.
This is a distinct **Freiburg 3 office-household recording and camera calibration
profile**, acquired approximately one year after the earlier Freiburg 1
recordings. One short indoor interval does not establish room diversity,
long-duration performance, automotive relevance or generalization.

The calibration document is independently pinned to `luigifreda/pyslam`, revision
`96019cfafcfc099ac9866884d7143a9ed1451a0d`, `settings/TUM3.yaml` (1,520 bytes,
SHA-256 `251e345996befa8057f5c51642bd4fe93a92907dac2e0564f400cd87eaf5785d`).
The ignored raw `camera-calibration.yaml` supplies fx=535.4, fy=539.2,
cx=320.1, cy=247.6, width=640, height=480 and 5000 depth units/metre. The optical
frame is x-right/y-down/z-forward. Its published RGB distortion coefficients
k1, k2, p1 and p2 are zero. Registered depth uses this explicit pinhole model,
without added pixel undistortion or another range correction. Native infrared
extrinsics, in-situ calibration validation and vehicle calibration are unavailable.
New-mode camera-profile support must preserve earlier recorded-data calibration
and all registration, preprocessing and accuracy gates.

Raw data and the calibration YAML remain ignored and are **not redistributed**.
A public code/data mirror supplies acquisition provenance, without establishing
the original TUM dataset's redistribution terms. The official dataset page
<https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned HTTP 403 through the
configured proxy during earlier acquisition, so original licence terms remain
unverified here. Fetch is explicit opt-in and preserves the configured HTTPS
proxy and TLS verification. No raw image or full sequence is bundled.

```sh
python3 scripts/fetch-submap-datasets.py tum-fr3-office-submaps
python3 scripts/fetch-submap-datasets.py tum-fr3-office-submaps --verify-only
```

Reproduction reads committed exact commits, inventories, bounds and hashes;
it does not prepare or overwrite the preregistered manifest.
