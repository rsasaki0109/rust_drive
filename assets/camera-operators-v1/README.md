# Guarded camera Resize optimization evidence

This packet preserves a real implementation and local CPU comparison. Read
[the implementation, limits and reproduction commands](../../docs/camera-operators.md).

The original pinned YOLOX-S model and all view pixels, thresholds and output
rules are unchanged. Two validated nearest/asymmetric/floor 2× Resize nodes
become standard tract axis/broadcast/reshape operations. All 714,000 f32 output
values per view match the independent original plan bit for bit on 16 images
twice. The actual lowered parallel executable preserves the complete original
detections and consistency result on the same 16 images twice.

| Local measurement | Original | Lowered |
|---|---:|---:|
| Serial four-view inference mean, 32 alternating-order pairs | 2.200343 s | 2.035672 s |
| Parallel processing first pair | 0.706520 s | 0.585206 s |
| Parallel processing second pair | 0.640254 s | 0.554865 s |

The measured parallel mean is 15.35% shorter (1.181×). There are only two timed
samples per executable on one host. Every 200 ms gate still fails, and each
trace has zero available results out of eight. The inherited BDD accuracy
failure remains 78 TP / 31 FP / 60 FN; this is viewed development data, not a
new holdout or new quality measurement. No driving integration is provided.

## Contents

- `first-profile.json.gz` and `initial-source/`: initial 26-source profiler
  before the lowering, including actual operator and selected-kernel clocks.
- `validation/`, `first-audit-source/`, `first-audit-failure.log`: failed
  first auditor, including a valid eight-forward baseline. It compared JSON
  decimal representations directly instead of the represented f32 bits.
- `f32-canonical-audit.json`: corrected bit comparison accepts that baseline
  and rejects a one-ULP adjacent-f32 change without a tolerance.
- `controls-final/`, `validation-final/`: final 27-source profiler, 264 actual
  forwards, 32 full paired reports, independent timing/operation aggregation,
  all raw-output hashes, seven corrupted-report rejections and legacy-profile
  rejection. Initial controls are also retained in `controls/`.
- `parallel-controls/`, `parallel-validation/`: 25 fatal input controls, two
  output protections, 144 actual forwards, frozen ABBA measured replay, four
  exact standalone guard replays, and 32 complete image reports. Producer
  source counts are 31 for images and 40 for the complete runtime.
- `ci-investigation/`: the prior remote failure log excerpt, unchanged original
  checker, two complete local reproductions (216 forwards each), and six
  exact-equality corruption checks for the new first-difference diagnostic.
  The existing remote research job remains failed and its numeric difference
  remains unconfirmed; local success does not establish a CI repair. See
  `ci-status.json` for the observed other jobs.
- `source/`, `source-sha256.json`: complete current source/documentation snapshot.
  `before.json` pins the original 73 sources, all 69 previous evidence indexes,
  initial commit and initial exploratory protocol. Changes are documented in
  `publication-checks/immutability.json`; all old compiled-model/controller
  sources and all 3,961 previously indexed evidence entries remain unchanged.
- `checks/`, `publication-checks/`: Rust tests/build/lint logs, the 371-test /
  49-replay root check, 245 unchanged core outputs, and publication audits.

Optimization work made **424 model forwards**: eight exploratory, eight in
the failed first audit, 264 final operator comparisons, and 144 integrated
parallel comparisons. The CI investigation made **432 additional forwards**.
The failed first observations were preserved, not replaced by passing results.

## Verify without a model

```sh
python3 -B assets/camera-operators-v1/verify.py
```

This checks file inventories, compressed and original-byte hashes, current and
archived source pins, all final operator reports, all parallel image reports,
the independently reconstructed clock/guard traces, and timing summaries. It
performs no model execution and requires no original images or weights.

`files.json` indexes every packet file except itself. Reports over 64 KiB are
gzip-compressed deterministically (`mtime=0`, no stored filename).
`raw-files.json` pins their exact original bytes and original artifact paths;
decompression does not rewrite JSON values. The raw reports are observations,
not a substitute for rerunning inference when checking a new CPU/compiler.

No original RGB/depth frames, poses, weights or executable binaries are
redistributed here. Tiny control images are authored fixtures already present
in earlier packets. Code and reports are Apache-2.0; TUM and BDD source-data
terms and attribution are copied exactly from the prior CPU packet. Original
data/model acquisition is described in the linked reproduction instructions.

Subjective maturity versus mature driving OSS remains about 20%; 30% is unmet.
