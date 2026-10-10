# Immutable first sitting visual-motion trial

This archive retains the exact **16 declared operational/evaluation source
files**, dependency lock, pinned toolchain and selected manifest frozen before
the first Freiburg 3 sitting pixel decode. It is an audit source snapshot,
not a standalone build checkout. `SOURCE.json` maps every archived file to its
original path, SHA-256, length and external-freeze field.

The original external freeze is
[`assets/recorded-visual/first-sitting-v1/freeze.json`](../../../../assets/recorded-visual/first-sitting-v1/freeze.json),
SHA-256 `cb98437e063e248dc71a09ce4c6450344cc43d9c20473396f4f1d204babfbf56`.
The first trial accepted and scored **31/35** updates as accurate, with three
bounded-consensus rejections and one duplicate RGB acquisition. Its complete
protocol remains failed, exit 1. All later evaluations of these same frames
are viewed regressions.

The independent checker can verify this exact snapshot with
`--source-snapshot integrations/rgbd/baselines/visual-sitting-first-v1/sources`.
Original PNGs and motion-capture files remain ignored and must be explicitly
fetched from their pinned sources. No raw images are archived here.
[Methods, full evidence and reproduction](../../../../docs/recorded-visual-odometry.md).
