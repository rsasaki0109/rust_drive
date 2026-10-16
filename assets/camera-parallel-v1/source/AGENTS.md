# RustDriving development

Use the existing checkout; Codex cloud tasks already provide isolation. Do not create Git worktrees unless the user requests one.

Read README.md and docs/architecture.md. This is a simulation-only prototype. Keep implemented, planned, and unverified capabilities distinct; do not use simulator truth as operational sensing/localization. Use SI units and explicit coordinate frames/timestamps.

Activate local tools with `source scripts/env.sh` if Cargo is not on PATH. Run `bash scripts/check.sh` after code changes. Keep Cargo.lock and pinned Rust. `bash scripts/demo.sh` regenerates ignored demo outputs; update assets/demo.* intentionally, with provenance.

Preserve algorithm crate independence and avoid a custom runtime or required ROS dependency. Add real implementations and meaningful independent acceptance checks, not empty module scaffolding. Do not claim CARLA, real-vehicle safety, real-time performance, or cross-platform success without actual validation.
