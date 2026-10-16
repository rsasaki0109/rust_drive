# Vehicle-label consistency and preserved ambiguity

The optional `rustdriving-camera-reconcile` Rust postprocessor reconciles
conflicting car, bus and truck labels from the existing
[four-view YOLOX-S detector](yolox-tiles-camera.md). On the eight viewed BDD
frames it preserves all **78** reference matches and reduces false positives
from **33 to 31**. Each output keeps an original model box, score, class and
source-view identity. It reads detector JSON, without loading images or model
weights. This is offline image research, separate from driving controls.

## Decision rule

Start with the unchanged full image plus up to three square crops, confidence
0.3 and classwise NMS IoU 0.45. Build conflict edges only between different
car/bus/truck classes (COCO 2/5/7), from different source views, with f32 IoU
strictly above 0.45. Reconcile a connected component only when every pair is
connected. Non-clique overlap chains and repeated-class components remain
unchanged and explicitly ambiguous.

Restore each view's original proposals with the existing edge rejection and
coordinate conversion. An eligible proposal must overlap every component
member above the same threshold. Inspect **all** eligible full-image proposals.
Reconcile only if their classes agree on exactly one class. With no full-image
support or multiple full-image classes, keep all hypotheses and disclose the
supporting classes/scores. Select the highest-ranked existing global box of the
uniquely supported class. No score or box is averaged, and crop counts do not
raise confidence. Output order and source rank remain traceable through
retained original indices. The producer's source/profile identity and its entire
global merge are checked before postprocessing.

This assumes useful global classification context; the full-image class can
also be wrong. Overlapping boxes can represent distinct vehicles. The rule
does not prove unique physical object identity or calibrated class probability.

The underlying limits remain four views, 300 detections per view, 1,200 input
proposals and 300 global outputs. The new path charges at most 1,200,000
comparison operations in the earlier full-context path; the final consistency
path allows 1,400,000. It bounds fixture/report input and output JSON to 4 MiB. Image
and model bounds remain those of the pinned S detector. These limits do not
establish real-time performance.

## Preserved failed majority experiment

The first [design](../assets/yolox-vehicle-vote-v1/design.json) used at most one
vote per view, preferred the full image on a tie, and otherwise chose the most
common class. On all eight viewed BDD frames, this removed two false positives
but also lost three correct matches: **75 TP, 31 FP, 63 FN**, compared with
the original **78 TP, 33 FP, 60 FN**. The complete first result and 16 native
image runs remain preserved. Its fixed relative adoption gate fails.

The [full-context design](../assets/yolox-vehicle-context-v1/design.json) was
fixed after that failure. It is explicitly informed by viewed development
results. Overlapping crops share pixels and model weights; their vote counts
provide no independent-observation or posterior-confidence guarantee. The
failed `rustdriving-camera-detect-vehicle-vote` binary and its source remain
unchanged, with source-bound reconstruction in the full-context result.

Prioritizing the highest-scoring full-image class also fails: **76 TP, 30 FP,
62 FN**. Two correctly detected references are lost. The full image itself
supports competing classes in those cases. Its first design, source, binary
interface and complete measured failure remain preserved. The final
[consistency design](../assets/yolox-vehicle-consistency-v1/design.json) was
fixed after inspecting that failure. This is a third viewed development
comparison, without an unseen-data or blind-selection claim.

## Complete final viewed result

| Policy | TP | FP | FN | Precision | Recall |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original four-view detector | 78 | 33 | 60 | 70.27% | 56.52% |
| Majority, failed | 75 | 31 | 63 | 70.75% | 54.35% |
| Highest full-context class, failed | 76 | 30 | 62 | 71.70% | 55.07% |
| Single-class full support | 78 | 31 | 60 | 71.56% | 56.52% |

The final [results](../assets/yolox-vehicle-consistency-v1/results.json) retain
every previous matched reference and per-class count. Three ambiguous
components preserve all their proposals; two components have single-class
full support and each drops one false positive. The fixed relative adoption
check passes. The original quality gate still **fails** because overall
recall remains below 60%, with bicycle, motorcycle and traffic-light recall
below 50%. Existing detector binaries keep their behavior; reconciliation is
an explicit additional step. This small viewed precision improvement leaves
the overall subjective maturity estimate at about 20%, with 30% unmet.
Conditional Brier score rises from 0.15656 to 0.15716, and ECE from 0.07461
to 0.08243. Confidence reliability does not improve in this viewed selection;
the reported gain is a reduction in false positives with reference coverage
preserved.

## Validation and reproduction

Both checkers independently reconstruct the original merge, graph components,
per-view eligible proposals, selections and lineage with NumPy. Controlled
cases cover context ties, crop majorities, same-view class disagreement,
person/vehicle overlap, non-clique chains, repeated-class ambiguity and the
1,200-proposal bound. Malformed geometry/classes/confidence, missing views,
unknown fields, size bounds and existing-output protection are checked.
The full-context controls additionally preserve conflicting hypotheses without
a full-image proposal and override three correlated crop votes without
inflating the retained score.

Both native trials retain the same eight viewed images and all 138 original
BDD references. All native inference finishes twice before numeric reference
scoring. The original class mapping, licenses, IoU 0.5 matching and quality
floors (precision 70%, recall 60%, each present-class recall 50%) remain fixed.
Descriptive confidence reliability is conditional on emitted detections; it
does not supply held-out calibration. Neither trial is fresh held-out evidence.

The two earlier modes execute 32 actual native image processes in total,
including repeats: 128 view forward passes. The final postprocessor executes
16 Rust report transformations against the first/repeat native parents, with
matching results and independent support reconstruction. A second 16-run
check confirms the same result with raw producer JSON in place of wrapped
native parents. These 32 transformations perform **zero additional model
forward passes**. This keeps detector inference distinct from
report transformation and avoids treating repeated crops as independent data.
Its 13 controlled fixtures and nine deliberately corrupted producer reports
check ambiguity, source/profile/tensor identity, boxes, lineage and merge work.

With the existing [model/data environment](yolox-s-camera.md#reproduction),
choose new output directories:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-reconcile
python integrations/onnx/bdd_vehicle_consistency_check.py controls \
  --binary integrations/onnx/target/release/rustdriving-camera-reconcile \
  --output artifacts/vehicle-context-example/controls
python integrations/onnx/bdd_vehicle_consistency_check.py run \
  --binary integrations/onnx/target/release/rustdriving-camera-reconcile \
  --controls artifacts/vehicle-context-example/controls \
  --output artifacts/vehicle-context-example/recorded
# Expected exit 1: relative improvement passes; original quality still fails.
```

For a new image, run the unchanged tiled detector, then the report adapter:

```sh
integrations/onnx/target/release/rustdriving-camera-detect-tiled-s \
  --model artifacts/camera-model-s/yolox_s.onnx --image IMAGE --output RAW.json
integrations/onnx/target/release/rustdriving-camera-reconcile \
  --input RAW.json --output RECONCILED.json
```

The adapter also accepts `--reconcile-fixture FILE --output NEW` for model-free
controls. Use `--executable-receipt --output NEW` to inspect compiled source
identity. The preserved vote and first-context binaries retain their native
`--model FILE --image FILE --output NEW` interfaces. Raw BDD images/labels and
weights remain in ignored storage; derived evidence retains the
[Regents/BAIR data terms](bdd-road-evaluation.md).
