# TUM Freiburg 1 desk2: source and first-trial identity

Dataset ID: `tum-fr1-desk2-independent`. This is measured indoor RGB-D and
motion-capture data from the **TUM RGB-D Benchmark**, credited to J. Sturm,
N. Engelhard, F. Endres, W. Burgard and D. Cremers. The official description
calls desk2 a second recording of the same four desks. It does not establish
an independent room, automotive environment or independently measured vehicle
calibration.

The [official benchmark License section](https://cvg.cit.tum.de/data/datasets/rgbd-dataset#license)
states that all benchmark data, unless stated otherwise, is licensed under
[Creative Commons Attribution 4.0](https://creativecommons.org/licenses/by/4.0/).
The accompanying code's BSD-2-Clause license is separate. No exception was
found in the [desk2 description](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download#freiburg1_desk2).
Raw recordings remain ignored and are not redistributed by RustDriving.

Required benchmark citation: J. Sturm, N. Engelhard, F. Endres, W. Burgard and
D. Cremers, “A Benchmark for the Evaluation of RGB-D SLAM Systems,” IROS,
October 2012. [Detailed attribution and authoritative page evidence](../../docs/tum-source-provenance.md)
preserve the exact license quote and official BibTeX.

## Acquired archive

- Official URL: `https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz`.
- Observed final URL: `https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz`.
- Root: `rgbd_dataset_freiburg1_desk2/`.
- Compressed size: **349,445,005 bytes**.
- SHA-256: `a569e4cb453a3cd9285bc985fcb109e65f055c75b33a4b155acd9a68d96b77d2`.
- MD5: `9250a26b897f770a6f9b5f4380020784`.
- Entire decompressed stream, including headers and padding: **376,688,640 bytes**, **1,286 members**.
- Selected raw inventory: **96,556,033 bytes**, **345 files** excluding the separately pinned calibration sidecar.

The unversioned URL is bound to these actual acquired bytes. These local hashes
are acquisition identity, not a publisher signature or independently published
checksum; `published_checksum` remains null. Proxy routing and TLS verification
were preserved. Earlier CONNECT 403 observations remain historical evidence;
official archive acquisition now works.

## Fixed selection and exposure

[The preregistered design](../../assets/recorded-independent/desk2-v1/design.json)
has exact SHA-256
`6fa44837d27f6f1f4f69285780ccd5e4883d7b59ace15500699914aca323797a`.
It fixes original depth indices **100–279**, one initialization, **180 frames
and 179 continuous updates**. Nearest RGB uses the complete original source
table, with earlier original index winning an exact tie. Selected images retain
their original pixels and timestamps. No accuracy-based selection, reset or
failure deletion is allowed.

[The public first manifest](../../assets/recorded-independent/desk2-v1/manifest.json)
has SHA-256
`f9de2bcb1037c46c70242f2c1bdbc8b09bbd06c95716d15b256b3c523ba24327`.
It binds 180 depth images, **162 distinct RGB images**, and complete
`depth.txt`, `rgb.txt`, `groundtruth.txt` tables. The window has **18 repeated
RGB associations**. Whole-source table counts are **639 depth, 640 RGB and
2,428 ground-truth rows**.

Compressed acquisition and bounded archive decompression precede timestamp-only
qualification. They encounter opaque image payload bytes without PNG header
inspection or pixel decoding. Metadata qualification interprets reference row
arity and timestamps, not numerical pose columns. After qualification and
source freeze, all sensor fits finish before numerical reference scoring.

The published camera-profile document is pinned separately:
`luigifreda/pyslam@96019cfafcfc099ac9866884d7143a9ed1451a0d/settings/TUM1.yaml`,
1,615 bytes, SHA-256
`5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf`.
It is an explicitly allowed `raw/camera-calibration.yaml` documentation sidecar
outside the selected sensor/metadata inventory. It is not an independent
extrinsics measurement; no additional undistortion is applied.

## Outcome and reproduction

The first continuous trial **fails**, retaining **16 accepted/root-accurate
and 163 rejected updates**, with tracking loss at index 127. The original
formal motion checker separately fails on `source_rgb_distortion` calibration
schema lookup. Its sources and first result remain immutable. The
[additive repaired audit](../../assets/recorded-independent/desk2-audit-v2/audit-v2.json)
now passes integrity and independently scores all 179 continuous outcomes,
without changing estimator math. It confirms failed physical acceptance.
Audit SHA-256:
`59773d67017fdb998bc41dd3137049e74d6013531b5852dfd74b5ef72e61c1a0`.
Running checker SHA-256:
`8c4e96b63cdd4f2c59907d8a71dbb43be972d4290547ae2495e10214fc34ce70`.
The [first-outcome record](../../assets/recorded-independent/desk2-v1/first-outcome.json)
retains both statuses. The [completed viewed regression suite](../../assets/recorded-independent/desk2-regression/suite.json)
reproduces all 179 outcomes, with qualification/freeze exit 0, expected evaluator
exit 1, and independent audit exit 0. Wrapper success means correct reproduction
of the failed physical trial, never fresh generalization or physical success.

This recording and its numerical references are now **viewed**. Later execution
is regression evidence, never a new unviewed trial. The
[protocol and reproduction guide](../../docs/independent-recorded-motion.md)
links the acquisition, qualification, first freeze, pre-pixel source preservation,
result, status and source archive. Preserve those files; use new output paths
and explicit regression mode for local reruns. Earlier historical license
statements remain byte-exact.
