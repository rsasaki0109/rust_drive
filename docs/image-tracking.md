# Offline image-space object tracking

The independent `rustdriving-image-tracking` library and
`rustdriving-camera-track` executable associate image detections across
timestamped frames. They maintain correspondence IDs, estimate image-plane
velocity, bridge short gaps, and retire stale hypotheses. This adds temporal
processing to the existing camera JSON interface; it does not project pixels
into vehicle coordinates or connect camera detections to driving controls.

## Run the example

The checked-in example is an explicitly **synthetic** pair of crossing objects
with two missing frames. It requires no model, image files, Python packages or
network access:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
mkdir -p artifacts/image-tracking-demo
integrations/camera-tracking/target/release/rustdriving-camera-track \
  --sequence integrations/camera-tracking/examples/crossing.json \
  --output artifacts/image-tracking-demo/crossing-new.json
```

Use a new output filename on each run. Exit 0 means the complete sequence was
processed and published, not that correspondence accuracy is known. Errors
exit 2 and identify the offending event. Input errors after successful earlier
events do not publish a partial sequence. Publication reserves sibling staging
before input reads and uses a no-replacement hard link after writing/syncing
complete bytes. Existing files and dangling links remain untouched. The local
Linux filesystem is verified; abrupt termination can leave hidden staging, and
parent-directory fsync/power-loss durability and other platforms are unverified.

## Sequence and library contracts

A version-1 sequence declares one `camera_id`, fixed `width`/`height`, optional
tracker `config`, and 1–128 timestamped `events`. The reusable library has no
sequence-length limit; its state holds at most 300 tracks and 256 image hashes.
Dimensions must be positive, at most 16,384 on each axis and 16 million pixels
in total. Times are finite nonnegative seconds in one caller-supplied clock;
each event must advance by at least one microsecond. Unix seconds or a relative
session clock are accepted. Camera identity and capture times are caller
declarations, not authenticated sensor provenance.

Each event contains `timestamp_s` and an `input` object:

| `kind` | Required fields | Meaning |
|---|---|---|
| `analysis` | `report`, `sha256` | Read a pinned [one-command camera report](camera-analysis.md). Paths are relative to the sequence file unless absolute. |
| `detections` | `image_sha256`, `detections` | Supply measured or explicitly synthetic boxes directly, using COCO contiguous classes 0–79, confidence [0,1] and image-bounded XYXY pixels. |
| `missing` | none | Advance time without a new image. Confirmed tracks may coast; observations never become newer. |
| `failed` | none | Explicit acquisition failure. Retire every active track immediately. |

For `analysis`, the exact report-byte SHA, producer/profile source map, image
identity, dimensions, model identity, execution fields, retained indices and
retained measurement/origin lineage are checked. Model f32 measurements are
canonicalized before f64 tracking. The adapter never opens images, model
weights or reference annotations. It does not independently rerun the detector
or authenticate the report's metadata. Its pinned
[producer source map](../integrations/camera-tracking/producer-sources.json)
deliberately rejects a different analyzer build.

Reusing an encoded-image SHA among the most recent 256 successful image
acquisitions is rejected, including after a failure. This is a conservative
replay check: identical newly captured bytes can also be rejected, while an
older replay outside the cache is undetectable. It does not establish sensor
freshness. Invalid clocks, hashes or detections leave library state unchanged;
callers receiving `Err` must not present an earlier snapshot as current.

## Association and lifetime

The default settings are research assumptions: minimum IoU **0.3**, coasting
limit **0.5 s**, **two consecutive hits** to confirm, velocity gain **0.5**, and
at most **300 tracks**. They are not fitted or independently calibrated.

Each track translates its last observed box using its center velocity in
pixels/second; box size stays at the last observation. The first displacement
initializes velocity; later displacements blend with the prior velocity.
Hungarian assignment accepts only the same class and predicted-box IoU at or
above the gate. It first maximizes valid match count, then total IoU. This
avoids a demonstrated greedy assignment that loses an available match. Ties
are deterministic; they are not evidence of physical identity. Class changes
start separate hypotheses rather than silently merging vehicle labels.

An unmatched tentative track retires immediately. Confirmed tracks coast with
the original confidence, count and observation timestamp unchanged, and expire
when age is strictly greater than the limit. Predictions can extend outside
the image and are explicitly marked `coasting` with no `detection_index`.
Acquisition failure clears active tracks; recovery allocates new IDs. IDs are
monotonic within one tracker session and are never recycled. At capacity,
existing coasting tracks remain and every untracked new detection index is
reported; admissions otherwise use confidence order with input-index ties.

The CLI limits sequence JSON to 8 MiB, each analysis report to 4 MiB and final
JSON to 16 MiB. Each event accepts at most 300 detections. Assignment uses at
most a 300 × 600 cost table and rejects beyond 60 million counted cost/column
operations per event. These bounds do not establish a real-time deadline.

## Verification and remaining work

```sh
cargo test --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/camera-tracking/check.py \
  --binary integrations/camera-tracking/target/release/rustdriving-camera-track \
  --output artifacts/image-tracking-controls-new
```

Eleven Rust tests cover the greedy counterexample, crossing/occlusion, exact
expiry boundary, fault recovery, atomic rejected inputs, class changes,
capacity, 300-way dense assignment, bounded replay history and Unix timestamps.
The separate standard-library Python checker validates **52 synthetic sequences,
307 time steps**, each executed twice with identical JSON. It uses exhaustive
assignment enumeration, independently reconstructs motion/lifetimes and checks
the authored crossing IDs. It also admits all **16** archived first/repeat
native detector reports as isolated one-frame sessions and rejects changed
clocks, geometry, producer metadata and measurement lineage. These eight BDD
images come from separate scenes and are never treated as a physical sequence.
No new model inference or real-sequence tracking accuracy is claimed.

The [evidence index](../assets/image-tracking-v1/files.json) includes every
control input/output, failures, source pins and the existing BDD data terms.
Original images, annotations and model weights are not redistributed. Detector
accuracy remains 78 TP / 31 FP / 60 FN against 138 viewed references, with failed
recall gates. Real camera sequences with independent identities, appearance or
ego-motion compensation, metric projection, calibration and vehicle integration
remain unverified. The OSS-relative subjective maturity estimate stays **about
20%**; this addition does not meet the 30% waypoint.
