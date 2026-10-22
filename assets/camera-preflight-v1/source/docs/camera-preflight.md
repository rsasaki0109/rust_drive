# Reject invalid evaluation inputs before loading a camera model

The previous preserved trial completed1056 new forwards before its scorer
rejected an original zero-height signal annotation. The additive preflight
validates every fixed original annotation and JPEG before any native/model
command. It never edits a label, drops an image or substitutes a sample.
The former trial, checker, protocol and original failure remain unchanged.

`camera_preflight.py` uses bounded regular non-symlink inputs, exact fixed
SHA256, strict JSON without duplicate keys/nonfinite constants, unique integer
reference IDs, original image/frame identity and timestamp, complete finite
nondegenerate source-pixel boxes inside1280x720 and the unchanged category
scope. Unscored traffic-sign boxes receive the same geometry validation.
Objects without box2d retain the original scorer's skip behavior; polygon-only
lane labels do not become scored boxes. Each JPEG must fully decode as the
original1280x720 RGB image. Inputs are bounded to4MiB each and4096 objects per
frame. Every image in the fixed group is checked; one error rejects the group.

An accepted in-memory ticket binds each image/annotation path and byte hash.
`guarded_call` rechecks those exact bytes and non-symlink file types before a
model callback. The evaluation runner uses that gate for every actual native
model command and verifies all inputs again before scoring. A saved JSON
receipt alone cannot supply a ticket. Integrity validation reads numeric labels
before inference; they never select images, tune the algorithm, change a
threshold or enter model inputs. All selection and algorithm rules are fixed
before acquisition. The frozen native producer identities are retained.

The new16 group is the next lexicographic original BDD validation pairs from
the same complete pinned tree, excluding all104 previous images and cached
JPEG stems. It contains an original zero-width car box: image23112889-a8ce8bc7,
reference8, `[569.204524,359.6745,569.204524,366.063979]`. The first controls
command rejects the complete56-input group before creating native fixtures or
starting any native command. **New model forwards:0.** No image replacement,
annotation correction, retry or quality-threshold relaxation occurs. Fresh16
quality remains unknown; `adopt_candidate:false`.

The previous invalid16 is a separate negative control: the same zero-height
signal is rejected with0 model callbacks. All previous valid40 inputs pass,
and their tickets allow40 in-memory probe callbacks, with0 native/model
commands. These probes are not neural inference. The18 automated tests also
check positive input, zero width/height, out-of-frame boxes, duplicate IDs/keys,
NaN, unknown boxed categories, boolean timestamps, truncated JPEG, source hash
mismatch, post-validation mutation, symlink replacement and the actual
evaluation control path starting no native subprocess after rejection.

The initial validator incorrectly applied boxed-category checks to polygon-only
objects. Its exact source and diagnostic receipt are retained. That check was
moved after the original no-box skip before any model execution. All actual
boxes remain validated. The first new-group rejection, final validator and
original label bytes are preserved separately from this implementation fix.

```sh
source scripts/env.sh
python -B -m unittest discover -s integrations/onnx -p test_camera_preflight.py -v
python -B assets/camera-preflight-v1/verify.py
python -B integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
python -B integrations/onnx/camera_capacity_fetch.py
python -B integrations/onnx/camera_detail_fetch.py
python -B integrations/onnx/camera_preserved_fetch.py
python -B integrations/onnx/camera_preflight_fetch.py
python -B integrations/onnx/camera_preflight_check.py negative --output artifacts/camera-preflight-check/negative
# Expected exit1: no native/model command starts.
python -B integrations/onnx/camera_preflight_check.py controls --output artifacts/camera-preflight-check/controls
python -B integrations/onnx/camera_preflight_rejected.py --controls artifacts/camera-preflight-check/controls --output artifacts/camera-preflight-check/rejection
```

The receipt auditor reproduces annotation checks without images, weights,
network or inference; it checks saved image-decode receipts against fixed input
pins rather than claiming to decode absent JPEGs. A CI workflow is added, with
remote execution unobserved. Original new annotations retain the full separate
BDD data terms; images, pixel exports, weights and executables are excluded.
This improves evaluation integrity, not measured perception accuracy,
availability, physical safety or driving integration. Maturity remains about20%,
below30%.
