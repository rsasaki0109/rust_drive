//! Frozen-protocol native/multiscale continuous RGB-D viewed regression.
mod image_features {
    pub use rustdriving_perception::image_features::*;
}
#[path = "../multiscale_temporal_support.rs"]
mod support;
fn main() {
    support::main_entry();
}
