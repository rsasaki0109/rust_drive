# Partly viewed FR1 room temporal extension

This optional subset fixes **180 consecutive original depth entries 100–279**
from TUM RGB-D `rgbd_dataset_freiburg1_room`. The selected depth timestamps are
**1305031914.108031–1305031920.081263 seconds**, approximately **5.973232 seconds**.
This is a **partly viewed temporal extension of the same recording**: indices
100–135 were evaluated in PR11. It is not a fresh environment, independent
recording or generalization trial. The complete numeric ground-truth source
was also parsed during the earlier scoring; only the extension image
acquisitions had not been viewed before this preparation.

The frame roles are explicit: index 100 is `initialization`, indices 101–135
are `viewed_prefix` (35 updates), and indices 136–279 are `unviewed_extension`
(144 updates). All **179 updates** remain selected. These tags document viewing
history rather than claiming every update is held out.

## Fixed metadata-first acquisition

An immutable source/window journal was saved before inspecting extension
metadata or transferring missing image bytes:
`artifacts/room-temporal-acquisition/pre-transfer-preregistration.json`, SHA-256
`bf1687b354a30a7fe66d3100961300e7d390247bb153f2b36688289a1ceb98b0`.
It fixes the window, published camera model, exact 0.02-second association and
reference-bracket gates, 4 MiB per-file bound and 128 MiB dataset bound. This
single window must be rejected on metadata failure; no alternate window was
searched using image features, reference poses, fit results or accuracy.

The source is
`edrishakimi1/Indoor-SLAM-Floorplan-with-Gaussian-Splatting` at commit
`1d5b2c3e1ee186abc1709042b739e9cd8a3b41d1`, directory
`data/rgbd_dataset_freiburg1_room/`. The complete pinned indices have **1,360
depth rows** and **1,362 RGB rows**. All **4,887 non-comment ground-truth rows**
have eight tokens and finite, strictly increasing timestamps, without duplicate
timestamps. This preparation parses only the timestamp column numerically and
checks row arity; it does not reparse numeric pose values or quaternion norms.
Full-source numerical pose validation remains evaluation-only.

Every selected depth timestamp is associated with the nearest original RGB
timestamp; exact ties choose the earlier original RGB index. There are **173
unique RGB images and seven repeated associations**. The repeated associations
occur at depth indices **240, 243, 252, 267, 273, 275 and 278** and remain selected.
No input is resized, synthesized, retimed or deleted. Operational rejection of
repeated RGB acquisitions must not refresh the accepted reference or clock.
The exact maximum RGB/depth gap is **0.017310142517089844 seconds**. The maximum
motion-capture interpolation bracket is **0.011500120162963867 seconds**;
no selected observation requires extrapolation. Gates use unrounded seconds.

[manifest.json](manifest.json) fixes **356 primary files**: 180 depth PNGs,
173 RGB PNGs and the complete depth/RGB/reference tables. The separate camera
document gives **357 cached files totaling 103,275,775 bytes**, under 128 MiB.
The largest primary file is 526,173 bytes, under the 4 MiB per-file bound.
The 76 previously cached files were reused only after exact byte-length,
SHA-256 and original Git-blob checks (the calibration document uses its pinned
SHA-256). The 281 missing image files were downloaded as opaque original bytes
and checked against the pinned Git tree before their SHA-256 inventory was
written. The official archive and an independent second mirror were not
byte-verified.

No PNG signature, header, dimensions or pixels were inspected during this
preparation; no images were decoded, features extracted or fits run. Published
encoding and 640×480 dimensions remain assumptions until the frozen decoder
checks them. The acquisition journal is distinct from the root-owned final
algorithm, qualification and independent-checker freeze before the **first
continuous extended run**. All estimator defaults and operational/accuracy
gates remain unchanged. Reference poses must enter scoring only after all
operational sensor fits finish, never as initialization, prediction or resets.

## Unchanged published camera model

Camera provenance remains `luigifreda/pyslam` at commit
`96019cfafcfc099ac9866884d7143a9ed1451a0d`, `settings/TUM1.yaml`, **1,615 bytes**,
SHA-256 `5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf`.
The published FR1 profile uses fx=517.306408, fy=516.469215, cx=318.643040,
cy=255.313989, width=640, height=480 and **5000 depth units per metre**.
The optical frame is x-right/y-down/z-forward; zero denotes invalid depth.

Published RGB distortion coefficients k1=0.262383, k2=-0.953104,
p1=-0.005358, p2=0.002628 and k3=1.163314 are provenance metadata. The same
pinhole projection without additional Brown–Conrady correction is a research
approximation. Native infrared extrinsics, source depth registration and
in-situ metric calibration are not independently measured. No extra range
correction, temporal compensation, vehicle calibration or recalibration against
this interval is supplied.

## Reproduction and limits

```sh
python3 scripts/fetch-temporal-dataset.py tum-fr1-room-temporal
python3 scripts/fetch-temporal-dataset.py tum-fr1-room-temporal --verify-only
```

This standalone helper pins the source identity, fixed window, frame viewing
roles, published model and full metadata hashes. It verifies exact cached
SHA-256 and Git-blob identities and never inspects image headers or pixels or
parses numeric reference poses. It cannot regenerate or reselect a manifest.
Earlier helpers, manifests, calibration and first-trial evidence remain
unchanged. Separate qualification and final freeze are required before fitting.

Raw images, reference tables and camera YAML remain ignored and are **not
redistributed or relicensed**. Public mirror availability supplies provenance,
not a new redistribution grant. Original TUM terms remain unverified: the
official dataset page <https://cvg.cit.tum.de/data/datasets/rgbd-dataset>
returned HTTP 403 through the configured proxy during earlier acquisition.
Downloads are explicit opt-in and preserve HTTPS proxy and TLS verification.
Published numeric camera parameters are recorded without copying upstream
algorithm code.

This approximately six-second indoor interval tests temporal continuity with
viewed overlap. It establishes no independent physical-room/camera coverage,
long-duration mapping, automotive generalization, calibrated covariance,
real-time performance, driving integration or real-vehicle safety. Metadata
qualification and hash integrity do not establish successful physical motion
estimation. No first continuous-run result is claimed by this source record.
