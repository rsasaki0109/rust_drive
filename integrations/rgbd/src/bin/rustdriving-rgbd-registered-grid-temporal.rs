//! Explicit registered-grid native/multiscale continuous viewed RGB-D diagnostic.
mod image_features {
    pub use rustdriving_perception::image_features::*;
}
#[path = "../registered_grid.rs"]
mod registered_grid;
#[path = "../registered_grid_temporal_support.rs"]
mod support;
fn main() {
    support::main_entry();
}
