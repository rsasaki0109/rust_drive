# Preregistered measured desk submap subset

This optional subset contains **36 consecutive original depth-index entries
100–135** from TUM RGB-D `rgbd_dataset_freiburg1_desk`, timestamps
1305031456.709099–1305031457.875920 seconds (**1.166821 seconds**). The fixed
window was declared before source metadata/image acquisition, depth decoding or
fitting. Metadata selection inspected only timestamp availability: all 36
motion-capture interpolation brackets are at most 0.02 seconds, with observed
maximum 0.010299921 seconds. Reference pose values, geometry, sensor errors and
acceptance outcomes did not choose the window. Frame 100 initializes the measured
map and all 35 following updates are temporal held out. Final algorithm,
evaluator, independent checker, preprocessing and accuracy settings must be
frozen before first decoding. A failed result remains in the denominator; these
frames cannot become a new held-out trial after their first evaluation.

The source mirror is `FaridRash/slam-track-fusion`, exact revision
`477a059d640540b7e23fd56ec95f6458167c7af2`, directory
`Data/rgbd_dataset_freiburg1_desk/`. The source README identifies handheld
movement around a desk; `depth.txt` indexes 595 recorded PNGs. The committed
[manifest.json](manifest.json) pins lengths and SHA-256 for the complete depth
index, evaluation-only `groundtruth.txt` and 36 selected PNGs. Each primary
input was also verified against its exact Git-tree blob identity. Acquisition
is byte-only, with no PNG decoding, geometry fitting or reference-pose inspection.
The subset is 4,505,543 bytes including its separate calibration document, below
the 12 MiB acquisition bound; each source file is below 1 MiB. Depth-index and
motion-capture timestamps remain in seconds. Evaluation-only poses are original
TUM world-from-camera positions in metres and xyzw quaternions; they must enter
scoring only after all operational sensor fits finish. Original official data
and an independent second mirror were not byte-verified in this environment.

This is a **different desk recording and clutter** from earlier `freiburg1_xyz`
trials. Independence of the physical room or camera is **not established**. It
is an indoor temporal check, not cross-room generalization or an automotive
benchmark. No real-vehicle calibration or external odometry is supplied.

Camera provenance is separately pinned to `luigifreda/pyslam`, revision
`96019cfafcfc099ac9866884d7143a9ed1451a0d`, `settings/TUM1.yaml` (1,615 bytes,
SHA-256 `5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf`).
The ignored raw `camera-calibration.yaml` supplies FR1 RGB intrinsics
fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989, width=640,
height=480 and 5000 depth units/metre. The optical frame is x-right/y-down/z-forward.
These published values define an **explicit pinhole approximation** for the
pre-registered depth images. The source's Brown–Conrady RGB distortion
coefficients are retained in metadata for provenance; no pixel undistortion is
performed and its validity after depth registration is not asserted. Native
infrared-camera extrinsics are unavailable. This new optional mode does not
change earlier 525/525/319.5/239.5 teaching-reader calibration.

Raw data and the calibration YAML remain ignored and are **not redistributed**.
The source mirror has an MIT software licence; it does not establish the original
TUM dataset's redistribution terms. The official dataset page
<https://cvg.cit.tum.de/data/datasets/rgbd-dataset> returned HTTP 403 through the
configured proxy during earlier acquisition, so original dataset terms remain
unverified here. The mirrors provide transfer/calibration provenance, not a new
raw-data redistribution grant. Fetch is explicit opt-in and preserves configured
proxy routing and TLS validation.

```sh
python3 scripts/fetch-submap-datasets.py tum-fr1-desk-submaps
python3 scripts/fetch-submap-datasets.py tum-fr1-desk-submaps --verify-only
```

Normal reproduction uses the committed inventory, source commits, bounds and
hashes; it never prepares or overwrites the preregistered manifest.
