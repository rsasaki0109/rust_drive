# Image-space tracking

This optional Rust library and `rustdriving-camera-track` CLI associate detections
across timestamped frames. They use predicted image boxes and global one-to-one
assignment, with tentative/confirmed/coasting states and bounded expiry.

No inference engine, GPU, ROS, custom runtime or driving crate is required.
The locked dependencies supply serialization and SHA-256. The
[contract, working example and verification](../../docs/image-tracking.md)
explain pixel units, acquisition failures, input bounds and the limits of
synthetic identity checks. This tool supplies neither metric object motion
nor driving controls.

`rustdriving-camera-guard` adds an offline two-clock freshness/recovery layer
around the unchanged tracker. It rejects stale/queued results, holds output
after a fault and requires consecutive fresh acquisitions to recover. See the
[clock contract, saved real-report example and independent checks](../../docs/camera-freshness.md).
Its `ready` flag describes camera-result availability and grants no driving
control permission.
