# Frozen S640 capacity comparison

Run `python -B assets/camera-capacity-v1/verify.py` from the repository root.
It requires the pinned NumPy/Pillow tools but no images, weights, network or
model execution. It verifies all indexed bytes, the198-source cumulative
closure, prior7,497 evidence entries and245 unchanged core outputs, then
independently reconstructs all saved model measurements and quality scores.

The new implementation is documented in `docs/camera-capacity.md`. First
measurements include720 S forwards and192 current Nano forwards on the same
viewed8 and fresh16 inputs. S uses10 views and Nano uses the unchanged4;
comparing S coarse4 against S supported10 separates augmentation from the
same-image S/Nano model-capacity comparison. No warmup or timing retry occurs.

`validation/before-inference.json.gz` binds the checker, executable, complete
compiled source maps, controls and input bytes before inference. All120
complete image reports are pinned before parsing numeric annotation content.
`validation/summary.json.gz` contains every match, false positive and false
negative for all four policies, the fixed quality gates and adoption decision.
An execution/integrity check passing does not mean a quality gate passed.

Fresh16 S coarse improves precision/recall to72.60%/61.63% from same-image
Nano65.14%/41.28%. S supported reaches71.15%/64.53% and regresses coarse
precision; motorcycle, truck and traffic-light recall still miss the50% gate.
`adopt_candidate:false`. No default pipeline change follows this trial.

The initial source-list assertion and native fixture failures occurred before
any inference. The original failing native test log and correction note are
retained. The selection-only protocol remains separately frozen; its completion
adds the current Nano comparison before decoding or inference without changing
sample IDs, thresholds, scoring or support policy.

`files.json` pins exact compressed bytes; `raw-files.json` pins decompressed
original bytes. Gzip has deterministic mtime0. Source archives contain only the
11 new files; earlier frozen packets retain the prior187 sources. Original
annotation JSON bytes retain the separate `LICENSE-BDD-data.txt` terms. Code
remains Apache2. JPEGs, pixel PNG exports, weights, executables, wheels and
site-packages are excluded. Remote CI execution is unobserved. This remains an
offline research variant, with no driving-control permission or maturity gain.
