# Mutual scale agreement on measured camera additions

The optional model-free `rustdriving-camera-agreement` keeps the original
four-view detection prefix and accepts supplemental640px measurements only
when the independently processed416px stream supplies a direct same-class
mutual best overlap. This reduces the false positives of the previous union,
but also discards correct detections. It fails the unchanged requirement to
preserve four-view precision. **No default adoption occurs.**

The complete fixed40-image development group has735 mapped references:

| Policy | TP | FP | FN | Precision | Recall |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original4 |451|161|284|73.69%|61.36%|
|640px supported |465|188|270|71.21%|63.27%|
|416px supported |468|178|267|72.45%|63.67%|
| Previous preserved union |478|200|257|70.50%|65.03%|
| Mutual agreement |455|166|280|73.27%|61.90%|

Against the union, agreement removes34 false positives but loses23 matches.
Against original4, it gains4 matches and5 false positives; all451 original
matched reference identities remain. The development precision comparison
fails; overall precision/recall gates pass, but present-class recall still
fails. A prediction prefix alone does not guarantee greedy reference-match
retention, so reference identities and class recall are scored separately.

All40 images were already viewed before this change. This is an explicitly
development-only comparison with no independent sample and no new model
inference. The two complete frozen native parents contribute240 archived
reports/2640 historical forwards, which are not counted again. The current
component executes120 new measured-box fixtures across first/repeat/original
graph parents, with0 model forwards. Labels, classes, box scope, occlusion and
truncation inclusion, confidence0.3, NMS0.45, match IoU0.5 and quality gates
are unchanged. The previous invalid16 groups remain rejected and unmodified.
No image replacement, label repair, threshold sweep or fresh-quality claim
occurs. No hypothesis is tuned after the first complete result.

`--fixture FILE --output NEW` requires three complete source-pixel streams:
`coarse`, `wide`, `detail`, each with corresponding `_origins`, plus width and
height. Each parent must retain the coarse prefix with exact float32 box,
confidence and origin bits, including signed zero. Each stream is limited to
300 valid boxes and unique origins (view<16, rank<300), within16M image pixels.
The fixture API does not attest to image/model execution or native provenance.

Only the noncoarse suffixes supply agreement. For each same-class pair, compute
float32 IoU; overlaps strictly above0.45 qualify. Each side chooses the largest
IoU, breaking ties by lower original stream index. Both must choose each other.
No support chains or reused witnesses occur. Process qualifying wide candidates
in their original order, retain their measured class/score/coordinates, veto
overlap with any coarse box regardless of class, then apply classwise NMS among
additions and append up to300 total boxes. There is no averaging or rescoring.
The pair scan, anchor veto and addition NMS have a combined270,000-comparison
limit. Inputs and complete outputs are bounded to4MiB; input symlinks and
overwriting an existing output are rejected. Complete publication uses the
existing hard-link publication pattern. Correlated crops are not independent
sensors or calibrated probabilities.

Before fixture processing, the existing preflight validates every original
JPEG and annotation and rechecks unchanged bytes before native callbacks.
The independent auditor then reconstructs the complete frozen parents,
decoded-crop receipts, original4 identity and agreement using a dense NumPy
matrix. All120 measurements, witness IoU bits, lineage, decisions, counters
and repeats agree. Rust8/Python16 tests,8 valid/14 invalid native controls and
existing-output preservation pass. The first controls attempt exposed shared
Python dictionaries in synthetic fixture construction: changing one parent
also changed the other prefixes. Independent copies repair the test helper;
its exact initial source, accepted fixture report and traceback are preserved.
The native algorithm and protocol were unchanged, with0 model forwards.

```sh
source scripts/env.sh
cargo test --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-agreement
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-agreement
python -B -m unittest discover -s integrations/onnx -p test_camera_agreement.py -v
python -B integrations/onnx/camera_agreement_check.py controls --output artifacts/camera-agreement-check/controls
python -B assets/camera-agreement-v1/verify.py --binary integrations/onnx/target/release/rustdriving-camera-agreement
# Optional local development reproduction requires the original cached40 JPEGs/annotations.
python -B integrations/onnx/camera_agreement_check.py run --controls artifacts/camera-agreement-check/controls --output artifacts/camera-agreement-check/validation
```

The frozen packet auditor needs no images, weights, network or inference. It
replays original annotation checks and binds saved image-decode receipts;
it does not decode absent JPEGs. Optional `--binary` reruns all120 model-free
fixtures. Raw images, pixel exports, weights, executables and wheels are
excluded. Original annotation bytes and full BDD data terms remain in earlier
immutable packets. Remote CI is unobserved. No driving-control, availability,
physical-safety or automotive-generalization claim follows. Maturity remains
about20%, below30%.
