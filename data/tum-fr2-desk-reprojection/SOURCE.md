# Preregistered Freiburg 2 RGB-D reprojection trial

This optional subset contains **36 consecutive original depth-index entries
100–135** from TUM RGB-D `rgbd_dataset_freiburg2_desk`, timestamps
1311868167.682062–1311868168.846276 seconds (**1.164214 seconds**). This first
candidate and fixed window were declared before acquiring its indices or
motion-capture timestamps, opaque image bytes, decoding pixels, fitting sensors
or inspecting numeric reference poses. Selection checked **timestamp and file
availability only**. No geometry, reference pose values or fit errors chose the
sequence, interval, calibration or accuracy settings.

Each depth observation uses the nearest recorded RGB timestamp; an exact tie
chooses the earlier RGB index. All 36 clock gaps satisfy the fixed 0.02-second
bound, with maximum **0.017630100 seconds**. Every motion-capture interpolation
bracket satisfies the fixed 0.02-second bound, with observed maximum
**0.003499985 seconds**. There are **25 unique RGB images and 11 repeated RGB
associations**. All depth rows remain in the protocol denominator; repeated RGB
timestamps must explicitly reject without clock renewal. No image is resized,
synthesized, interpolated, retimed or silently removed. Initial depth index 100
initializes the measured origin; all 35 following observations are held out.

The mirror is `GKoutilya/real-time-predictive-maintenance-dashboard`, exact commit
`2c1304c46bc3d47e9056a01127382c95240a902a`, directory
`rgbd_dataset_freiburg2_desk/`. Its original depth/RGB indices list 2,964 and
2,965 entries respectively. [manifest.json](manifest.json) fixes the complete
depth index, RGB index, evaluation-only motion-capture file, 36 depth PNGs,
25 unique RGB PNGs and separately pinned camera document: **65 cached source
files, 19,363,239 bytes**, below 32 MiB total and 4 MiB per file. Each primary
input has exact length, SHA-256 and source Git-tree blob identity. Pixel-file
SHA-256 was obtained by **opaque byte transfer**, after saving the metadata-only
pre-transfer journal; no PNG signature/header or pixel contents were inspected,
no PNG was decoded and no numeric ground-truth pose columns were parsed during
preparation. Source dimensions and encoding remain published expectations until
validated by the frozen first decoder run.

The pre-transfer metadata journal has SHA-256
`013573864b20d2af7496d793fa108021d2bdd2f29b31cc61ffecdaba7ec51501`
and is retained under ignored
`artifacts/reprojection-dataset-discovery/pre-transfer-preregistration.json`.
It records the fixed window, timestamps, exact published camera model, byte
bounds and absence of pixel inspection before transfer. It is an acquisition
journal; the final algorithm/evaluator/oracle preregistration is a separate
mandatory step **before first pixel decoding and fitting**. Preserve the first
trial, including every rejection and inaccurate estimate. After that trial,
these same inputs are viewed regression and cannot be claimed fresh again.

Camera provenance is `luigifreda/pyslam`, commit
`96019cfafcfc099ac9866884d7143a9ed1451a0d`, `settings/TUM2.yaml`, 1,584 bytes,
SHA-256 `8ec3a8c3b64495e8a308a40e0b75eb77300119c5502184ad77628324e6bae0f0`.
The published model is fx=520.908620, fy=521.007327, cx=325.141442,
cy=249.701764, width=640, height=480, and **5208 depth units/metre**.
The 5208 scale is intentionally preserved; it is not the 5000 scale of earlier
FR1/FR3 trials. Independently published `raulmur/ORB_SLAM2`, commit
`f2e6f51cdc8d067655d90a78c06261378e07e8f3`,
`Examples/RGB-D/TUM2.yaml` (2,076 bytes, SHA-256
`c4cc666d1e7441f9edabdd09b96a10342e04a17024312a2833d9b2c4657ab180`)
agrees on these intrinsic values and 5208 scale. Only published parameter
provenance is recorded; no upstream algorithm code is copied or redistributed.

This is an explicit **research source-model assumption**, without independently
measured calibration. The optical frame is x-right/y-down/z-forward. Published
RGB Brown–Conrady coefficients k1=0.231222, k2=-0.784899, p1=-0.003257,
p2=-0.000105, k3=0.917205 remain metadata provenance. Pinhole projection without
distortion correction is an approximation; its validity for source depth
registration is not asserted. Native infrared-camera extrinsics, in-situ metric
verification and real-vehicle calibration are unavailable. No extra range
correction is applied. Settings are selected from published documentation before
pixel inspection and must never be retuned using held-out accuracy.

Original TUM motion-capture labels are world-from-camera poses, positions in
metres and xyzw quaternions. These remain **evaluation only**, parsed after all
operational sensor fits finish; they cannot supply an initial pose or aid
tracking. The source was not byte-verified against an official archive or a
second raw-data mirror. This is a distinct Freiburg 2 camera profile and natural
recorded sequence, without proof of cross-room generalization, long-duration
mapping, automotive relevance or real-vehicle safety.

Raw data and camera YAML remain ignored and are **not redistributed**. The
public code/data mirror supplies transfer provenance and no new original data
redistribution grant. Original TUM terms remain unverified: the official dataset
page <https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned HTTP 403 through
the configured proxy during earlier acquisition. Fetch is explicit opt-in and
preserves configured HTTPS proxy routing and TLS verification.

```sh
python3 scripts/fetch-reprojection-dataset.py tum-fr2-desk-reprojection
python3 scripts/fetch-reprojection-dataset.py tum-fr2-desk-reprojection --verify-only
```

The fetcher validates committed pins, paths, lengths, SHA-256 and bounded source
inventory. It never decodes images, inspects PNG headers or parses numeric
reference poses. Reproduction never prepares, overwrites or reselects a manifest.
Do not run the first decoder/fitter until the complete algorithm/evaluator/oracle
source freeze and preregistration have been recorded.
