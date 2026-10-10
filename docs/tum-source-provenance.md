# TUM RGB-D source provenance

This record documents prospective use of **TUM RGB-D Freiburg 1 desk2** in
RustDriving's [independent recorded-motion protocol](independent-recorded-motion.md).
It adds authoritative source-page evidence without changing earlier source
archives, trial reports or historical statements that their license had not
been verified.

## Official description and attribution

The [official desk2 download description](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download#freiburg1_desk2)
states:

> This sequence contains several sweeps over four desks in a typical office
> environment (similar to desk, but second recording).

Desk2 is a distinct recording, with the same described four-desk setting. It
is not evidence of a new physical room or an automotive environment. The page
lists 24.86 s total duration and 24.28 s with ground truth. Its approximate
download table size (0.37 GB) and prose size (approximately 0.33 GB) are not
exact archive lengths or checksums.

The [official benchmark page, License section](https://cvg.cit.tum.de/data/datasets/rgbd-dataset#license)
states, with whitespace normalized:

> Unless stated otherwise, all data in the TUM RGB-D benchmark is licensed
> under a Creative Commons 4.0 Attribution License (CC BY 4.0) and the
> accompanying source code is licensed under a BSD-2-Clause License.

The data license is [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/);
the accompanying source-code license is separate. No candidate-specific data
exception was found in the accessible desk2 description. The full Creative
Commons legal code was not fetched by this source-discovery task. RustDriving
does not plan to redistribute raw recordings; local acquisition, source hashes
and derived evaluation evidence are recorded separately. RustDriving's own
code license does not replace the benchmark data's attribution terms.

Attribution: **TUM RGB-D Benchmark**, provided by J. Sturm, N. Engelhard,
F. Endres, W. Burgard and D. Cremers. The official page requests citation of
their publication:

> J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers,
> “A Benchmark for the Evaluation of RGB-D SLAM Systems,” IROS, October 2012.

The following BibTeX reproduces the fields supplied on that official page:

```bibtex
@InProceedings{sturm12iros,
  author = {J. Sturm and N. Engelhard and F. Endres and W. Burgard and D. Cremers},
  title = {A Benchmark for the Evaluation of RGB-D SLAM Systems},
  booktitle = {Proc. of the International Conference on Intelligent Robot Systems (IROS)},
  year = {2012},
  month = {Oct.}
}
```

RustDriving's intended modifications are a fixed original-index selection,
timestamp association and derived estimator/audit reports. The protocol does
not resize, retime or edit selected source images. Those planned modifications
are not evidence that acquisition or evaluation has already run.

## Page evidence and archive identity

[Source discovery](../assets/recorded-independent/desk2-v1/source-discovery.json)
records successful official HTML responses on 2026-10-10, their exact byte
counts, hashes, description, license quote and access failures. The cached
HTML is local ignored acquisition evidence, not redistributed benchmark raw
data.

| Official page | Bytes | SHA-256 |
| --- | ---: | --- |
| Dataset overview | 225,552 | `bcc8ac86593f2c0820269870c87307c57dd28bf9ac3219acc1e31fa49a8aa8a9` |
| Download page | 668,959 | `9734b24cec8185e7eb5c5bced21cfd16686f2c68cd30cb31c5b0071dc0f0e640` |

The official archive URL is
`https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz`.
The observed redirect points to
`https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz`.
The expected archive root is `rgbd_dataset_freiburg1_desk2/`.

At discovery, the first host returned HTTP 302; proxy CONNECT to the redirected
host returned 403, with curl exit 56 and no final origin response. Archive
bytes, exact length, ETag, Last-Modified and authoritative checksum were
unavailable. No checksum link was found on the two accessible official pages;
the redirected directory listing was also inaccessible. A configuration draft
for the actual redirect host has been saved and still requires publication.
The protocol preserves proxy routing and TLS verification.

The source link is unversioned. Its eventual local SHA-256 and MD5 identify
the acquired archive, without proving a publisher signature or independent
published checksum. `published_checksum` remains null in the prospective
design. If authoritative checksum evidence becomes available, preserve it and
resolve any required protocol revision before using new data; do not invent
an expected value from the same downloaded bytes.

The discovery journal also records a metadata-only GitHub mirror lead. Its
table contents were not read, and byte agreement with the official source is
unverified. It is not qualified input or a substitute archive in this protocol.

## Camera provenance and limits

The unchanged Freiburg 1 pinhole profile is documented by
`luigifreda/pyslam@96019cfafcfc099ac9866884d7143a9ed1451a0d/settings/TUM1.yaml`:
1,615 bytes, SHA-256
`5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf`.
It is separately identified as source-calibration documentation: the pinned
`raw/camera-calibration.yaml` is an explicitly allowed sidecar outside the exact
selected-sensor-and-metadata inventory. It specifies 640 × 480 images, focal lengths
517.306408 / 516.469215 pixels, principal point 318.64304 / 255.313989 pixels,
and 5,000 depth units per metre. This is an existing published-profile
assumption, not independently measured camera or vehicle calibration; no
additional undistortion is claimed.

Actual archive identity, metadata qualification and post-fit accuracy remain
pending. New authoritative license evidence changes the prospective provenance
record only. Older “unverified” license statements describe the evidence
available when those immutable artifacts were created and remain byte-exact.
