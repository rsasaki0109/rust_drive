# Latest-ready camera replay

The optional `rustdriving-camera-latest` command changes only image selection
around the existing Nano detector and freshness guard. At an actual host-clock
checkpoint it selects the latest acquisition already released. Every superseded
image retains its original timestamp, index and hash in the report and counts
as unavailable. A due missing/failed event takes priority over preceding queued
images; the consumer never drops or crosses that event.

Acquisition times, the 200 ms bound, two fresh recovery samples, fault epochs,
four views, model, thresholds and postprocessing are unchanged. A source check
requires the complete measured per-frame controller to equal the original
controller byte for byte. The old Nano command and all prior source/evidence
remain reproducible. This is a synchronous offline experiment: an acquisition
failure cannot preempt active inference, and delivery waits for the consumer.
It supplies no live-camera supervision or vehicle-control permission.

## First measured outcome: no availability improvement

The same eight previously viewed indoor images use their original acquisition
intervals. The frozen execution order is FIFO/latest/latest/FIFO, with no warmup.
All trials retain all eight source images in availability denominators.

| Trial | Inferred | Superseded | Inferred but late | Available / source images |
|---|---:|---:|---:|---:|
| FIFO first | 4 | 0 | 2 | 0 / 8 |
| Latest first | 3 | 5 | 0 | 0 / 8 |
| Latest repeat | 3 | 5 | 0 | 0 / 8 |
| FIFO repeat | 4 | 0 | 0 | 1 / 8 |

Latest first selects indices 0, 3, 7; latest repeat selects 0, 4, 7. Their actual
inferred processing takes 84–135 ms. Although each dispatched image finishes
within its own deadline, expiry of the preceding accepted acquisition establishes
a fault barrier. A newly delivered image acquired before that barrier cannot
recover tracking, and one later fresh sample cannot satisfy two-sample recovery.
Removing old work alone therefore fails to establish useful availability.

The earlier Nano GitHub trial independently makes 244 model forwards and
reproduces the original detections, but both Nano deadline flags fail and both
trials supply 0/8 available results. Its first images take 147.51 and 149.81 ms.
The local 86.68–89.43 ms observations remain valid local samples; their timing
does not transfer to that runner. The decoded job log and result are retained.

This comparison makes 88 actual forwards: 56 timed forwards and 32 current serial
reference forwards. Independent NumPy reconstruction checks every inferred image;
four separate guard replays exactly reproduce all guard outputs. There is no
new BDD accuracy evaluation, unseen recording, identity score or vehicle input.
The unchanged Nano image detector still has the previously measured failed BDD
precision/recall gates. The latest mode is opt-in and the default stays unchanged.

## Reproduce

Cache the existing Nano/S models and TUM images with the
[Nano instructions](camera-nano.md). The S file is used only for a wrong-model
input control; every inference in this comparison uses Nano.

```sh
source scripts/env.sh
python -m pip install -r scripts/requirements-visual.txt
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-nano --bin rustdriving-camera-latest
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python integrations/onnx/camera_latest_check.py controls --output artifacts/latest-example/controls
python integrations/onnx/camera_latest_check.py run \
  --controls artifacts/latest-example/controls --output artifacts/latest-example/validation
```

Every output must be new. `--inspect-sequence` validates all original inputs
without loading a model; `--executable-receipt` binds the complete 43-file native
source closure. A direct replay uses:

```sh
integrations/onnx/target/release/rustdriving-camera-latest \
  --model artifacts/camera-model/yolox_nano.onnx \
  --sequence assets/camera-sequence-v1/sequence.json --output NEW.json
python -B assets/camera-latest-v1/verify.py
python -m unittest discover -s integrations/onnx -p test_camera_latest.py -v
```

The last two commands execute no model and require no original images. The
packet contains source, reports, authored tiny input controls and the remote
job log, without model weights or original images. Subjective maturity stays
about 20% versus mature driving OSS; the 30% waypoint remains unmet.
