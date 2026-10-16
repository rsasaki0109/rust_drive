# Original TUM FR1 RPY held-out recording

The first physical accuracy trial fixes original depth indices 100–279, 180
acquisitions and 179 continuous updates, before archive payload acquisition.
The native estimator binary is SHA-256 `0f47ac96222dc940573e2336a66909d25dd6124dd5de76c6e68a1bd6b3cc6e3e`.
The source, two declared configurations, optical-camera calibration, timestamp
qualification and unchanged 0.1 m / 0.1 rad gates are bound in the
[pre-acquisition design](../../assets/projective-information-v1/heldout/rpy/design.json).

Source: [official TUM archive](https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_rpy.tgz), `410268381` bytes, observed
SHA-256 `78103722a25873dbbb4de027eaa8c810a6382100691f02b6cf95c3adc91c4ac1`. The archive identity is recorded from the downloaded
bytes; an independently published checksum was not obtained. HTTPS range
responses were verified against original total length, exact Content-Range and
ETag before assembly, then the complete bounded archive was parsed and hashed.
Raw recordings remain ignored and are not redistributed.

Attribution: TUM RGB-D Benchmark, J. Sturm, N. Engelhard, F. Endres, W. Burgard
and D. Cremers, IROS 2012. The general official benchmark terms state CC BY 4.0
for data unless otherwise specified; accompanying source code has separate
terms. [Authoritative license and attribution record](../../docs/tum-source-provenance.md).
This is another indoor FR1 camera recording, without a demonstrated new physical
room, automotive domain, measured vehicle calibration or external verification.

All original pixels and timestamps remain. Registered optical x-right/y-down/
z-forward depth uses axial metres = stored units/5000. The declared shared
pinhole model is K=525, cx 319.5, cy 239.5, 640×480, with no additional image
undistortion or range correction. Numeric mocap poses are evaluation labels only.
They are decoded after both 180-frame sensor ledgers and independent raw audits.

The first inventory writer stopped on a duplicate source_path keyword after
metadata qualification and opaque file selection. Its error and exact source
are retained. An acquisition-only repair independently verified all selected
bytes against the same archive and explicitly copied each pin's bytes/SHA,
without changing selection, source coordinates, estimator, scoring, or decoding
images/pose numbers. The original pre-acquisition design remains byte-exact.

The preceding plant trial is separately retained as a metadata qualification
failure: indices 232–237 lack reference brackets within 20 ms. Its pixels and
numeric poses were not decoded, no accuracy result is claimed, and its window
was never reselected. See the [guard and validation record](../../docs/projective-information.md).
