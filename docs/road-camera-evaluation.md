# Real urban-camera diagnostic

RustDriving's optional standalone `integrations/onnx` workspace executes the
pinned official YOLOX nano ONNX model with Rust-native `tract-onnx` CPU inference.
This evaluator extends the original single NASA portrait check to six real
urban images with independent COCO annotations. It remains offline research;
camera boxes are image coordinates, with no metric depth or vehicle control.

## Frozen protocol and observed failures

Confidence 0.3, classwise NMS IoU 0.45 and class-aware matching IoU 0.5 were fixed
before running these images. The two calibration images and four evaluator-held-out
images are disjoint. Neither partition was used to tune thresholds. Seven classes
are evaluated: person, bicycle, car, motorcycle, bus, truck, traffic light. Other
class predictions are counted separately and excluded from these metrics.
Matching processes descending confidence and allows at most one prediction per
reference, making duplicate detections false positives. Empty denominators are
reported as null rather than perfect accuracy.

| Partition | Images | References | TP | FP | FN | Precision | Recall | Mean matched IoU |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Calibration | 2 | 12 | 3 | 1 | 9 | 0.750 | 0.250 | 0.883 |
| Evaluator held-out | 4 | 25 | 14 | 4 | 11 | 0.778 | 0.560 | 0.802 |
| Combined diagnostic | 6 | 37 | 17 | 5 | 20 | 0.773 | 0.459 | 0.817 |

| Evaluator-held-out class | TP | FP | FN | Precision | Recall |
| --- | ---: | ---: | ---: | ---: | ---: |
| Person | 10 | 1 | 6 | 0.909 | 0.625 |
| Car | 3 | 3 | 4 | 0.500 | 0.429 |
| Bus | 1 | 0 | 0 | 1.000 | 1.000 |
| Traffic light | 0 | 0 | 1 | null | 0.000 |

There are no held-out bicycle, motorcycle or truck references; no accuracy can
be inferred for those classes from this partition. The calibration street image
does contain a bicycle, five cars, three trucks and three traffic lights, but
nine of its twelve references were missed. Each urban-sign hard negative had no
reference road users and produced no road-user/signal prediction. The complete
report includes every missed annotation ID and box, every unmatched prediction,
individual matched IoUs, hashes, timings and partition summaries. Missed small
pedestrians, occluded vehicles and traffic lights remain explicit limitations.

## What is verified

The executable was built with the existing locked dependencies and ran each of
the six photographs twice: twelve actual CPU inference runs, equal repeated
boxes. Four existing Rust decoder/input tests, strict Clippy and formatting
passed. Seven independent scorer mutations were rejected: model identity, image
identity, dimensions, confidence threshold, NMS threshold, nonfinite geometry,
and annotation bytes. A duplicate correct prediction increased only FP. Three
hand-computable overlap cases checked independent IoU arithmetic. The legacy
portrait execution/geometry and malformed-input checks remain separate.

`road_check.py --baseline` compares recorded per-image counts and matching
identities, with absolute matched-IoU tolerance 1e-5. It does not gate on host
timing or disguise low recall as an accuracy pass. See the exact commands,
licenses and pinned inputs in [the integration guide](../integrations/onnx/README.md),
the [frozen protocol](../integrations/onnx/road-protocol.json), and the
[measured report](../integrations/onnx/road-results.json).

## Limits on interpretation

All selected original image records refer to **COCO val2017**, even though the
mirror calls its directory `train2017`. The upstream pretrained model used COCO;
validation and model-selection overlap cannot be ruled out. Evaluator-held-out
means absent from the evaluator's calibration partition, not unseen by the
model. Six selected urban photographs cannot establish driving-domain accuracy,
COCO AP, weather coverage, temporal consistency or deployment readiness. There
is no camera calibration, synchronization, camera/LiDAR fusion, camera-derived
metric obstacle position, traffic-light color recognition or camera control input.

Raw licensed research inputs remain ignored, and no JPEGs or modified previews
are published. COCO reference metadata is credited to the COCO Consortium under
CC BY 4.0; the two CC BY-ND 2.0 images are fetched only as unmodified originals.
This bounded, reproducible evaluator closes the previous portrait-only evidence
gap without claiming a road-ready learned perception stack.
