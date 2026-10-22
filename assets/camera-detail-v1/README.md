# Frozen S640/416px detail comparison

Run `python -B assets/camera-detail-v1/verify.py`. The verifier requires the
pinned NumPy/Pillow tools, but no images, weights, network or model execution.
It checks exact compressed/decompressed bytes,207 cumulative sources, all
7,705 prior evidence entries,245 core outputs and independently reconstructs
the viewed small-object diagnosis, complete detections and quality scores.

See `docs/camera-detail.md` for the fixed method and decision. New1920 S
forwards measure detail416px crops on40 inputs and baseline640px crops on
fresh16. Viewed24 baseline reports are reused; their720 forwards are not
counted again. Same model, original4 prefix, score, support rule and full
original class/annotation scope remain. Every repeat and original S graph
reference must reproduce decoded float32 measurements and complete decisions.

The sample protocol is frozen before acquisition. All168 new complete image
reports are pinned before parsing new numeric labels. `files.json` pins exact
bytes and `raw-files.json` pins decompressed original bytes. Gzip has mtime0.
Only9 new top-level source files are copied; prior packets retain earlier
sources and viewed annotations. The initial compiler failure and correction
before any inference remain archived. Passing execution/integrity checks does
not imply that quality gates passed.

Fresh16 detail improves supported baseline precision72.77% to74.11% and
recall64.43% to65.61%; signal TP9 to11 of24 remains below50% recall. Bicycle
and truck recall also fail. Viewed24 has equal overall recall with person/car
regressions. `adopt_candidate:false`; default behavior is unchanged.

JPEGs, decoded PNG/pixel exports, weights, binaries and wheels are excluded.
Original new annotations retain the separately copied complete BDD data terms;
code remains Apache2. Remote CI is unobserved. No real-time, availability,
driving-control permission, physical safety or maturity gain is claimed.
