# Dense depth diagnostics and an OpenCV reference

Six sensor-only development prototypes and a pinned OpenCV RGB-D reference
were evaluated on the already viewed Freiburg 1 room and desk2 windows.
Projective depth fusion substantially improves desk2, but every method fails
complete accuracy in at least one recording. These results keep the subjective
maturity estimate at **about 20%**; the **30% waypoint remains unmet**.

The new maintained tools are an optional reference evaluator and an independent
root-score auditor. The exploratory algorithms are archived Python research
sources; they have not been ported into the Rust driving pipeline. The original
Rust algorithms, source-bound reports and failed trials remain unchanged.

A subsequent [independent Rust port](projective-depth-rust.md) now implements
the fusion variant and reproduces its viewed trajectories. It remains separate
from vehicle driving, with the same failed physical accuracy outcomes.

## Data, coordinates and fixed acceptance

Both recordings use existing pinned original depth indices **100–279**:
180 acquisitions, one initialization and **179 updates** over approximately
six seconds. [Room provenance](temporal-recorded-motion.md) and
[desk2 provenance](independent-recorded-motion.md) retain official acquisition
and data terms. Desk2 records the same four desks, so it supplies a second
recording rather than an independent room. These windows are viewed development
data; improvements on them supply no held-out generalization result.

The operational pinhole model is the documented
[registered grid](registered-grid-camera.md): 640 × 480, fx = fy = 525,
cx = 319.5, cy = 239.5, axial depth / 5,000 metres, zero invalid, no additional
distortion or depth-scale correction. This remains an uncalibrated default.
Optical axes are x right, y down, z forward. An estimate maps the current depth
camera into the first depth camera, with one retained origin.

Root translation and rotation gates stay **0.1 m and 0.1 rad**. A strictly newer
depth acquisition is the measurement clock; associated RGB may repeat, but
must be nondecreasing and within 20 ms. Fresh depth can support a geometric
update when its associated RGB repeats; this differs explicitly from the old
feature-only adapter's image-freshness contract. An accepted-pose age beyond
0.20 s latches loss. Rejections stay in the denominator, and there are no pose
resets. Numeric reference poses enter scoring after operational fits.

## All development results

Each cell below shows **accepted / root-accurate**, always out of 179 updates.
The [review](../assets/projective-depth-diagnostics-v1/review.json) contains
full error summaries. Original outcomes, independent scores and every attempted
source are retained in [the evidence directory](../assets/projective-depth-diagnostics-v1).

| Method | Room | Desk2 |
|---|---:|---:|
| Adjacent dense point-to-plane ICP | 179 / 58 | 179 / 73 |
| Geometric keyframes | 179 / 71 | 179 / 75 |
| Geometry and image intensity | 179 / 63 | 176 / 17 |
| Geometry and intensity, longer anchors | 179 / 61 | 122 / 15 |
| Bilateral depth filtering and keyframes | 179 / 70 | 179 / 75 |
| Projective depth fusion and keyframes | 179 / 64 | 179 / 161 |
| OpenCV 4.13.0 RGB-D ICP reference | 179 / 83 | 179 / 15 |

The longer-anchor desk2 prototype loses tracking; all other branches retain
updates until the end. High acceptance alone does not establish accuracy.
The depth-fusion desk2 maximum root errors are **0.114800 m / 0.090386 rad**;
room still reaches **0.296290 m / 0.141286 rad**. Thus even the promising fusion
variant fails the unchanged complete criterion. Prototype process exit 0 only
means its exploratory script completed; all quality failures remain recorded.

The prototypes use coarse-to-fine projective associations, surface normals,
Huber point-to-plane residuals and scaled normal-equation solves. The fusion
variant reprojects measured depth into an accepted keyframe, z-buffers nearest
positive axial depth, averages compatible depths with bounded pixel weight 16,
and shifts the view while preserving the root origin. This is a single-view
projective model, without volumetric mapping, loop closure or calibrated
uncertainty. These exploratory sources are not a stable runtime interface.

Independent root scoring found that prototype reference interpolation omitted
normalizing each input quaternion before SLERP. The largest resulting scoring
differences are **1.26e-7 m / 3.67e-7 rad**. The first preservation check rejects
that discrepancy; its source and diagnosis remain retained. Separately stored
normalized-reference scores change **zero accuracy classifications** across
all 12 prototype runs. Neither original numbers nor original accuracy gates
were replaced. [Diagnosis](../assets/projective-depth-diagnostics-v1/preservation-diagnosis.json).

## Pinned OSS reference and independent checks

[`compare-opencv-rgbd.py`](../scripts/compare-opencv-rgbd.py) uses
`opencv-contrib-python-headless==4.13.0.92` (OpenCV 4.13.0), one thread and RNG
seed zero. `RgbdICPOdometry` receives current images/depth as source and the last
accepted acquisition as destination. The solver's resolved pyramid defaults,
library/build/license hashes, evaluator source, manifest and raw-file pins
are recorded before operational fitting. A separate sensor-only ledger is
written before numeric label decoding. This adds no Rust/runtime dependency.

An independently rendered scene with analytic ray/sphere intersections checks
three known nonzero motions. All fit errors meet **0.002 m / 0.002 rad**;
stationary and one-ray metre/axis controls also pass. The official
[OpenCV odometry API](https://docs.opencv.org/4.13.0/df/ddc/classcv_1_1rgbd_1_1Odometry.html)
defines the source-to-destination transform direction. Synthetic success
checks this invocation, without establishing physical trajectory accuracy.

[`check-opencv-rgbd.py`](../scripts/check-opencv-rgbd.py) independently computes
normalized-quaternion reference interpolation, root errors and expiry state.
It verifies complete source inventories, all 358 updates, proper rigid poses
and equality with the pre-label ledger, rejecting nine report corruptions per
recording. A hand-computable half-metre/45-degree interpolation, quaternion
sign invariance and three invalid-time controls pass. It does not reconstruct
OpenCV's internal correspondences or optimization.

Both complete reference evaluations exit **1** for failed quality. Their eight
JSON outputs reproduce byte-for-byte on this Linux host; the earlier reference
version's two sensor ledgers also agree exactly. The
[source-bound audit](../assets/projective-depth-diagnostics-v1/opencv-final-source-audit.json)
passes integrity while keeping complete accuracy false.
The [validation receipt](../assets/projective-depth-diagnostics-v1/validation.json)
also records the core regression and unchanged prior simulation outputs.

## Reproduction and limits

Acquire the pinned room/desk2 files using their existing provenance instructions.
Use a separate evaluation environment and fresh output directories:

```sh
python3 -m venv artifacts/opencv-reference-venv
artifacts/opencv-reference-venv/bin/pip install -r scripts/requirements-opencv-reference.txt
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  artifacts/opencv-reference-venv/bin/python scripts/compare-opencv-rgbd.py \
  --output artifacts/opencv-reference/local
# Expected exit 1 for these retained physical accuracy failures; continue auditing.
artifacts/opencv-reference-venv/bin/python scripts/check-opencv-rgbd.py \
  --comparison artifacts/opencv-reference/local \
  --output artifacts/opencv-reference/local-audit.json
```

`--controls-only` needs no recordings. The new Ubuntu CI job runs analytic
controls; the complete recorded trial and its exact repeat were run locally.
Only the recorded reference comparison was validated on this Linux host.
The archived prototypes require the same NumPy/Pillow environment; bilateral
filtering also uses the optional pinned OpenCV wheel. Their development source
archive is preserved for inspection, alongside full first results and source
hashes. Raw images, depth, poses and OpenCV binaries are not redistributed.
OpenCV packaging is MIT; OpenCV and bundled libraries retain the notices in the
wheel's `LICENSE-3RD-PARTY.txt`, whose hash is recorded in each freeze.

This comparison rules out a simple transform-direction/unit error in the
reference invocation. It does not isolate a single physical drift cause.
Registered-grid approximation, image/depth timing, correspondence noise and
model drift remain possible contributors. Sustained localization, independent
automotive accuracy, measured extrinsics, calibrated uncertainty and driving
integration remain unresolved. This experiment supplies no real-time or
real-vehicle safety evidence.
