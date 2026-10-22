//! The original complete four-view contract, kept separate from the grid.
use super::yolox_s;
#[allow(dead_code)]
#[path = "yolox_tiles.rs"]
pub mod yolox_tiles;
#[allow(dead_code)]
#[path = "yolox_vehicle_consistency.rs"]
pub mod yolox_vehicle_consistency;
#[allow(dead_code)]
#[path = "camera_supported_coarse_context.rs"]
mod yolox_vehicle_context;
#[allow(dead_code)]
#[path = "yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
