# Independently supervised camera replay

The optional `rustdriving-camera-supervised` command runs the unchanged Nano
four-view analysis in one scoped inference worker. The caller owns the original
freshness guard and continues to observe source failures and poll acquisition
expiry while inference runs. This supplies an executable offline supervision
path; camera results still have no vehicle-control permission.

There is one active image and one waiting image. Each newly released image can
replace only that waiting image. Every original source retains its acquisition
text, offset and image hash in the report. A missing/failed source is delivered
in order and drops the waiting image; active work is drained. Every completed
result must pass the unchanged guard using its original acquisition, so a
pre-fault result cannot undo a fault. A worker error delivers a failure instead
of detections. Fatal clock/channel/budget errors publish no successful partial
report. Forward counts are unknown after a worker error because a failing
analysis may have already executed part of its views.

Due sources precede result delivery and new work. The target poll interval is
10 ms; actual `std::time::Instant` checkpoints are retained without synthetic
increments. All accepted results receive a post-tracking tick, and monitoring
continues after the final input/result until its accepted acquisition expires.
The original 200 ms age bound, two-sample recovery, tracker, model, four views,
confidence/NMS thresholds and postprocessing remain unchanged. Standard library
scoped threads/channels provide the handoff; there is no added runtime or
service.

## First measured results

The protocol freezes both execution orders before any valid inference:
latest/supervised/supervised/latest on the original eight-image TUM schedule,
then the same order on an authored fault fixture. The latter uses the original
first image/acquisition and injects failure at +10 ms and missing at +15 ms.
These injected faults are test controls, not recorded physical sensor failures.
All eight original images remain in each ordinary availability denominator.

| Ordinary trial | Inferred | Superseded | Inferred late | Available / source images |
|---|---:|---:|---:|---:|
| Latest first | 4 | 4 | 0 | 2 / 8 |
| Supervised first | 3 | 5 | 0 | 0 / 8 |
| Supervised repeat | 3 | 5 | 1 | 0 / 8 |
| Latest repeat | 3 | 5 | 0 | 0 / 8 |

The supervised trials detect expiry while the next forward is still executing,
5.93 ms and 4.62 ms after the original 200 ms deadline. The preserved fault epoch
rejects a pre-fault acquisition even if that particular result finishes young.
The added monitoring has **not improved ordinary result availability**.

| Fault trial | Failure delivery delay | Delivered before inference finished |
|---|---:|---|
| Latest first | 102.16 ms | No |
| Supervised first | 3.11 ms | Yes |
| Supervised repeat | 2.80 ms | Yes |
| Latest repeat | 104.57 ms | No |

Both supervised fault trials meet the predeclared observed 10 ms failure-delay
gate. Their pre-fault results are rejected and readiness stays false. This is
local evidence from two paired observations, not a portable latency guarantee.
The maximum periodic poll gaps are 24.29/13.12 ms in ordinary trials and
12.99/12.70 ms in fault trials: **every 10 ms periodic target fails**. OS scheduling,
CPU contention and guard work can delay observation. A hung native forward cannot
be cancelled; the caller keeps polling until its 10-second trace budget, and
scoped shutdown still waits for the worker. An external owner would need its
own process supervision. No live camera or hard real-time claim is made.

The comparison performs 100 actual forwards: 68 timed and 32 current independent
serial reference forwards. All inferred outputs exactly match current serial
measurement bits and decisions; independent NumPy reconstruction checks the
unchanged image postprocessing. Eight standalone guard replays exactly reproduce
every guard output, including intermediate polls, faults and final expiry.
There are 8 new native state-transition tests (39 existing tests are reused),
12 Python corruption regressions, 25 fatal-input controls and 2 output protections.
The non-image control also exactly replays its guard without loading a model.

No new BDD accuracy score, unseen recording, identity evaluation or driving
integration is added. The Nano detector's inherited BDD quality gates still
fail. The mode remains optional; subjective maturity versus mature driving OSS
stays about 20%, with the 30% waypoint unmet. The parent Nano PR's full CI has
now completed all ten jobs successfully; this new local variant has no remote
CI observation.

## Reproduce

Use the pinned models and recording from [the Nano instructions](camera-nano.md).

```sh
source scripts/env.sh
python -m pip install -r scripts/requirements-visual.txt
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-supervised --bin rustdriving-camera-latest \
  --bin rustdriving-camera-nano
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python -B integrations/onnx/camera_supervised_check.py controls \
  --output artifacts/supervised-example/controls
python -B integrations/onnx/camera_supervised_check.py run \
  --controls artifacts/supervised-example/controls \
  --output artifacts/supervised-example/validation
```

Every destination must be new. `--inspect-sequence FILE --output NEW` validates
all images and the original schedule before model loading. `--executable-receipt
--output NEW` binds the 43-file native source closure. Direct execution:

```sh
integrations/onnx/target/release/rustdriving-camera-supervised \
  --model artifacts/camera-model/yolox_nano.onnx \
  --sequence assets/camera-sequence-v1/sequence.json --output NEW.json
python -B assets/camera-supervised-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_supervised.py -v
```

The final two commands execute no model and need no original camera images.
The packet retains the complete first measurements, source, authored controls
and independent guard replays; it contains neither model weights nor original
camera images. All prior frozen sources and evidence remain unchanged.
