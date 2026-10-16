# Recorded Argoverse sparse-depth experiment

The existing projection and depth binaries actually processed **278,790
recorded points** in three scans from the official Argoverse 1.1 tracking sample,
using the same log's front-center camera calibration. This is a projection/
raster consistency experiment. There is no independent physical depth reference,
so physical depth RMSE/MAE remain unavailable. These are not KITTI captures.

## Source and coordinate bridge

The [official Argoverse page](https://www.argoverse.org/av1.html) links the
`tracking_sample_v1.1.tar.gz` archive. The allowed virtual-host S3 URL identifies
the same published bucket/key. The initial metadata check records its
1,875,764,613-byte length and ETag. A bounded streaming reader retained the first
three PLY members in original archive order and their log calibration, after
437,647,360 compressed bytes. The remainder was not acquired; the compressed
prefix hash is explicitly not a complete archive hash.

The selected log is `c6911883-1843-3727-8eaa-41dc8cda8993`. Archive order is
not chronological; the experiment evaluates independent scans and does not
claim a continuous trajectory. Selection preceded point decoding or results.
Every input XYZ/intensity row is retained, with no thinning or label access.

Argoverse points are already in the ego frame. Its source calibration stores
`vehicle_SE3_camera`, so the camera projection uses its inverse and the original
intrinsics. Public API source confirms scalar-first `[w,x,y,z]` quaternion
ordering and the undistorted-image interpretation. The bridge presents these
coordinates to the unchanged KITTI-format binary as one virtual cam0 with
identity shared rectification. Extra native/camera/IMU schema fields are
nonoperational placeholders. They are not KITTI factory calibration or measured
IMU extrinsics. This tests the existing computation on recorded geometry;
literal KITTI-file compatibility remains untested.

## First failure and additive diagnosis

The initial frozen all-positive-depth pixel check passes **1/3 scans**. The other
two retain three and four off-canvas points exceeding the fixed tolerance
`1e-8 px + 1e-12 × |reference pixel|`. Their camera Z is about **0.073–0.662 mm**;
division by near-zero Z magnifies tiny differences between independent vector
quaternion rotation and matrix rotation. Their projections are far outside
the canvas. The original result and checker are preserved; no tolerance was
relaxed and no estimator change was made.

An additive diagnosis uses the same point selection and tolerances to check
all in-frame pixels and every raster cell, depth winner, support count and
source-point assignment. **All three match exactly for discrete selections**;
each original projection/depth report reproduces byte-for-byte. Maximum
in-frame pixel discrepancy is **9.67e-13 px**, and maximum camera-coordinate
discrepancy is **8.53e-14 m**. These are numerical agreement measures, not
physical sensor accuracy. The initial global pixel failure remains failed.

| Scan timestamp | Source points | In-frame points | Occupied pixels | Losing points | Canvas density |
| --- | ---: | ---: | ---: | ---: | ---: |
| 315978412920103000 | 90,155 | 18,141 | 18,014 | 127 | 0.7819% |
| 315978417119737000 | 93,928 | 16,801 | 16,752 | 49 | 0.7271% |
| 315978415619440000 | 94,707 | 17,470 | 17,394 | 76 | 0.7549% |

The canvas is 1920 × 1200. Most pixels have no depth; sparse raster consistency
does not establish depth completion, physical visibility or calibrated uncertainty.
No camera image, pose, independent depth label or motion compensation enters
this experiment. No real-time or driving-performance claim follows.

The [first result](../assets/argoverse-sparse-depth-v1/first-results.json),
[diagnosis](../assets/argoverse-sparse-depth-v1/diagnosis-results.json) and
[source/evidence record](../assets/argoverse-sparse-depth-v1/validation.json)
retain the failures, source identities, controls and limitations. Raw point
clouds, calibration and large derived reports remain ignored locally.

## Reproduction and attribution

Use the pinned NumPy environment and built projection/depth binaries. Every
output directory must be new. The stream acquisition is capped at 2 GB:

```sh
python scripts/fetch-argoverse-depth-sample.py \
  --output artifacts/argoverse-depth-evaluation/raw
python scripts/check-argoverse-sparse-depth.py \
  --raw artifacts/argoverse-depth-evaluation/raw \
  --projection-binary integrations/rgbd/target/release/rustdriving-kitti-project \
  --depth-binary integrations/kitti-depth/target/release/rustdriving-kitti-depth \
  --output artifacts/argoverse-depth-evaluation/first
```

The recorded first command exits **1** for the preserved global numerical gate.
Run the additive diagnosis separately, preserving that result:

```sh
python scripts/diagnose-argoverse-sparse-depth.py \
  --raw artifacts/argoverse-depth-evaluation/raw \
  --first artifacts/argoverse-depth-evaluation/first \
  --projection-binary integrations/rgbd/target/release/rustdriving-kitti-project \
  --depth-binary integrations/kitti-depth/target/release/rustdriving-kitti-depth \
  --output artifacts/argoverse-depth-evaluation/diagnosis
```

Argoverse data/documentation: **© 2018–2019 Argo AI, LLC**, licensed
[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/), under the
[official Terms of Use](https://www.argoverse.org/about.html#terms-of-use).
API code has its separate MIT/retained Apache licensing. Data terms remain
separate from RustDriving's Apache-2.0 code. This is offline research evaluation;
raw data are not redistributed in the repository. Cite Chang et al.,
*Argoverse: 3D Tracking and Forecasting with Rich Maps*, CVPR 2019.
