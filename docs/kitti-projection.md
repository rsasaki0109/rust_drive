# KITTI processed-image calibration and point projection

The optional Rust `rustdriving-kitti-project` tool projects supplied Velodyne
points into a selected KITTI processed, rectified camera using the complete
calibration chain. It does not download data or estimate calibration. Its
authored-fixture CLI validation and complete independent replay pass; no real
KITTI recording has been acquired or evaluated.

## Authority, access and license

The [primary-source receipts](../assets/kitti-projection-v1/source-authority.json)
record the official KITTI pages and Geiger et al., *Vision meets Robotics: The
KITTI Dataset*, IJRR 2013, sections IV-B/IV-C, equations 5, 7 and 8. Receipt
hashes identify the reviewed documents; they are not publisher signatures or
authentication of supplied calibration files. The [fixed design](../assets/kitti-projection-v1/design.json)
was committed before the first authored-fixture CLI projection.

The official raw-data page states **“You must log in to download the raw
datasets!”** No dataset account is configured and no alternate historical
download path is used to bypass that gate. No raw calibration, Velodyne
recording, image or OXTS pose has been acquired. Obtain authorized inputs
separately before running the provided-file command below.

KITTI datasets and benchmarks use **CC BY-NC-SA 3.0**, according to the official
[homepage](https://www.cvlibs.net/datasets/kitti/). The dataset's attribution,
noncommercial and share-alike conditions are separate from RustDriving's
Apache-2.0 code license. Original authored analytic controls are not KITTI data.

## Coordinate model

For Velodyne homogeneous point `X`, the selected camera's processed pixel is:

```text
q = P_rect_selected · embed(R_rect_00) · T_cam0_from_velo · X
(u, v) = (q.x / q.z, q.y / q.z)
```

All four cameras use the **shared `R_rect_00`**, followed by their own full
3 × 4 `P_rect`. Native `K/D/R/T` and the other `R_rect_i` values remain
provenance; they are not additional transforms of already rectified pixels.

Writing `P_rect = [A, b]`, the effective selected-camera transform is
`[I, A⁻¹b] · embed(R_rect_00) · T_cam0_from_velo`. This retains intrinsic
skew and **all three entries of the fourth column**, rather than assuming a
horizontal baseline only. The selected camera center in shared rectified cam0
coordinates is **`−A⁻¹b`**. Multiplication by `T_velo_from_imu` provides the
effective IMU-to-camera transform; no OXTS parsing or pose scoring is performed.

Camera coordinates are **right, down, forward**; Velodyne/IMU coordinates are
**forward, left, up**. Translations and point coordinates use metres, pixels
use the processed image canvas. A point is `inframe` only for positive camera
depth and `0 ≤ u < width`, `0 ≤ v < height`, without an inclusion epsilon.
Positive-depth points outside that canvas remain `outside`; nonpositive-depth
points remain `behind`, with a null pixel. Every input row is retained.

## Provided-file CLI

Use existing authorized files; the calibration names below describe their
roles, not files bundled with a real dataset. Camera **0/1/2/3** selects left
grayscale/right grayscale/left color/right color respectively.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-kitti-project
integrations/rgbd/target/release/rustdriving-kitti-project \
  --calib-cam /path/to/calib_cam_to_cam.txt \
  --calib-velo /path/to/calib_velo_to_cam.txt \
  --calib-imu /path/to/calib_imu_to_velo.txt \
  --points /path/to/velodyne_points.bin --camera 2 \
  --output artifacts/kitti/local-projection.json
```

Use the actual executable path if `CARGO_TARGET_DIR` is configured. Each
calibration file is bounded to 64 KiB. The point file contains **1–200,000**
little-endian float32 `[x, y, z, reflectance]` records, **16 bytes per row**;
all values must be finite. Reflectance is retained and unused in geometry.
Unknown, duplicated, missing, nonfinite or wrong-cardinality calibration
fields are rejected; declared processing metadata is nonoperational.

The JSON output includes calibration/source hashes, effective transforms,
camera center, per-point coordinates/pixels/status and complete counts. These
hashes identify supplied bytes, not factory calibration, a common acquisition
day or membership in an official archive. Output is exclusively created and
admitted before input reads, rejecting existing paths and symlink parents.
A later input error may leave an empty reserved output: use a new path rather
than overwrite it. Exit **0** means valid projection; exit **2** means an
input/output contract error, not an accuracy score.

## Validation boundary

The fixed authored control has 18 point rows and nonidentity shared
rectification, distinct per-camera parameters, skew, a nonzero x/y/z fourth
column, boundary pixels and positive/zero/negative depth. Independent NumPy
checks reconstruct every homogeneous transform and all **72 projected rows**
(18 per camera), with fixed absolute tolerances **1e-10** for matrices and
**1e-8** for coordinates/pixels. These are analytic geometry/input-contract
checks, not measured sensor accuracy.

| Camera | In frame | Outside | Behind | Retained rows |
| --- | ---: | ---: | ---: | ---: |
| 0 | 10 | 6 | 2 | 18 |
| 1 | 8 | 7 | 3 | 18 |
| 2 | 5 | 10 | 3 | 18 |
| 3 | 12 | 5 | 1 | 18 |

The first actual sweep rejects **99 non-noop report corruptions**, **56 actual
malformed-source invocations** and **28 actual output-guard invocations**.
All invalid CLI controls exit 2; all four positive projections exit 0. A
complete independent repeat reproduces these counts and all four projection
JSON files **byte-for-byte**. Audit semantics match after excluding only
commands’ fresh output-directory paths. The [evidence packet](../assets/kitti-projection-v1/evidence.json)
binds all published files, including the [camera-0 projection](../assets/kitti-projection-v1/first-camera-0.json),
[first audit](../assets/kitti-projection-v1/first-audit.json.gz),
[repeat audit](../assets/kitti-projection-v1/replay-audit.json.gz) and
[semantic replay comparison](../assets/kitti-projection-v1/semantic-replay-comparison.json).
The [local validation summary](../assets/kitti-projection-v1/validation.json) records
**371 core Rust tests**, **114 optional RGB-D Rust tests**, **49 scenario/replay
pairs** and **245 byte-identical legacy baseline files**. These checks do not
prove literal real-file compatibility, physical calibration or road accuracy.

Reproduce the authored fixture and full independent sweep using the pinned
NumPy environment from `scripts/requirements-visual.txt`; each output directory
must be new:

```sh
python3 scripts/check-kitti-calibration.py --prepare-fixture \
  --output artifacts/kitti/local-authored-fixture
python3 scripts/check-kitti-calibration.py \
  --fixture artifacts/kitti/local-authored-fixture \
  --binary integrations/rgbd/target/release/rustdriving-kitti-project \
  --output artifacts/kitti/local-authored-audit
```

A successful sweep exits 0 for authored geometry and input integrity. It does
not score a recorded driving scene.

There is no image decoding, timestamp association, spinning-LiDAR deskew,
motion compensation, measured extrinsic estimation, tracking, calibrated
uncertainty or driving integration. Automotive accuracy and real-time
performance remain unverified. This adapter does not change the subjective
**about-20% maturity** assessment; the **30% and 50%** goals remain unmet.
