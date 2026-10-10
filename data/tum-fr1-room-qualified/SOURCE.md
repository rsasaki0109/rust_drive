# First metadata-qualified recorded room trial

This optional subset contains **36 consecutive original depth-index entries
100–135** from TUM RGB-D `rgbd_dataset_freiburg1_room`, timestamps
1305031914.108031–1305031915.276431 seconds (**1.168400 seconds**).
The candidate order and fixed window were declared before fetching its indices,
checking motion-capture timestamps, acquiring opaque pixel bytes, decoding
images or inspecting numeric reference poses. The first candidate qualified;
the search stopped without evaluating another sequence. Selection used **source
availability and timestamps only**, without features, geometry or accuracy.

Metadata qualification inspected the **entire original ground-truth file**:
all **4,887 non-comment rows** have exactly eight tokens and finite, strictly
increasing timestamps, without duplicates. Only the timestamp column was parsed
numerically; pose values, quaternion norms and physical accuracy remain unseen
until scoring after the first frozen sensor run. Selected observations all have
motion-capture interpolation brackets below 0.02 seconds, with observed maximum
**0.010399818 seconds**. Each depth timestamp uses the nearest original RGB
timestamp; an exact tie chooses the earlier RGB index. All 36 RGB/depth clock
gaps are below 0.02 seconds (maximum **0.012958050 seconds**). There are
**36 unique RGB acquisitions and no repeated RGB associations**. No image is
resized, interpolated, synthesized, retimed or omitted. Initial index 100
initializes the measured origin; all **35 following updates are held out**.

The source is `edrishakimi1/Indoor-SLAM-Floorplan-with-Gaussian-Splatting`, exact
commit `1d5b2c3e1ee186abc1709042b739e9cd8a3b41d1`, directory
`data/rgbd_dataset_freiburg1_room/`. Its depth/RGB indices contain 1,360 and
1,362 original entries. [manifest.json](manifest.json) fixes 75 primary files:
complete depth/RGB indices, evaluation-only motion-capture labels, 36 selected
depth PNGs and 36 selected RGB PNGs. The separate camera document makes
**76 cached files, 21,516,513 bytes**, below 32 MiB total and 4 MiB per file.
Every primary file has an exact length, SHA-256 and source Git-tree blob identity.
The original official archive and an independent second mirror were not
byte-verified.

An immutable metadata-only pre-transfer journal was saved before opaque pixel
transfer, SHA-256
`38151476aa74c0b30c8c39ba1b21c9f7d2f06979e950b22fc49652c31a898700`,
under ignored
`artifacts/qualification-dataset-discovery/pre-transfer-preregistration.json`.
It records the candidate order, fixed window, complete timestamp/arity check,
associations, byte bounds, published camera model and absence of pixel or pose
inspection. The final input SHA-256 inventory was then prepared from **opaque
bytes only**. No PNG signature, header, dimensions or pixel contents were
inspected, no image decoded and no numeric reference pose columns parsed during
preparation. Source encoding and 640×480 dimensions remain published assumptions
until checked by the frozen first decoder.

The acquisition journal is separate from the mandatory **final algorithm,
evaluator and independent-oracle preregistration before first pixel decoding
and fitting**. The default estimator, preprocessing and accuracy settings must
remain unchanged. Preserve all first-trial estimates and failures; subsequent
reuse of this same sequence is viewed regression. Motion-capture transforms may
enter physical scoring only after all operational sensor fits finish, never as
an initial pose, prediction or fit aid. Original labels are world-from-camera
positions in metres and xyzw quaternions.

Camera provenance is `luigifreda/pyslam`, commit
`96019cfafcfc099ac9866884d7143a9ed1451a0d`, `settings/TUM1.yaml`, 1,615 bytes,
SHA-256 `5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf`.
Its published FR1 model is fx=517.306408, fy=516.469215, cx=318.643040,
cy=255.313989, width=640, height=480 and 5000 depth units/metre. The optical
frame is x-right/y-down/z-forward. Published RGB distortion coefficients
k1=0.262383, k2=-0.953104, p1=-0.005358, p2=0.002628 and k3=1.163314 remain
provenance metadata. Pinhole projection without Brown–Conrady correction is an
explicit research approximation; source depth registration, native infrared
extrinsics and in-situ metric calibration are not independently verified.
No extra range correction or vehicle calibration is supplied. The profile is
selected from published documentation before viewing this sequence and is not
tuned against its reference poses or errors.

This is a new recorded FR1 room sequence, without an independent physical-room
or camera claim relative to other FR1 records. Metadata qualification proves
index/timestamp integrity and bounded acquisition, not good physical estimates,
long-duration mapping, automotive relevance or real-vehicle safety.

Raw inputs and camera YAML remain ignored and are **not redistributed**.
The public mirror supplies transfer provenance and no new original-data
redistribution grant. Original TUM licence terms remain unverified: the official
dataset page <https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned HTTP 403
through the configured proxy during earlier acquisition. Fetch is explicit
opt-in, preserving the configured HTTPS proxy and TLS verification. Published
camera parameter provenance is recorded; upstream algorithm code is not copied.

```sh
python3 scripts/fetch-qualified-dataset.py tum-fr1-room-qualified
python3 scripts/fetch-qualified-dataset.py tum-fr1-room-qualified --verify-only
```

This standalone fetcher pins the repository, commit, sequence, published FR1
profile and all qualified source metadata SHA-256/Git blobs. It bounds transfers,
validates all cached SHA-256/Git blobs and never inspects PNG headers, decodes
pixels or parses numeric reference poses. Reproduction never prepares,
overwrites or reselects a manifest. Earlier dataset manifests, fetchers,
calibration and archived evidence remain unchanged.
