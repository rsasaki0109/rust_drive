//! Optional calibrated second moments, independent of trajectory and other sensors.
use crate::Scenario;
use rustdriving_core::Vec2;

fn uniform(seed: u64, tick: usize, channel: u64) -> f64 {
    // Counter-based SplitMix64 mixing: no shared mutable sensor RNG stream.
    let mut bits = seed
        .wrapping_add((tick as u64).wrapping_mul(0x9e3779b97f4a7c15))
        .wrapping_add(channel.wrapping_mul(0xd1b54a32d192ed03));
    bits = (bits ^ (bits >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    bits = (bits ^ (bits >> 27)).wrapping_mul(0x94d049bb133111eb);
    bits ^= bits >> 31;
    (bits >> 11) as f64 / ((1u64 << 53) as f64) * 2. - 1.
}

impl Scenario {
    /// Simulation-only ENU noise with covariance `0.02 * gnss_noise_shape` m².
    /// Components before Cholesky transformation are independent bounded uniforms.
    /// Call after validation. Absence retains each backend's legacy noise draws.
    pub fn gnss_noise(&self, seed: u64, acquisition_tick: usize) -> Option<Vec2> {
        self.gnss_noise_shape.map(|shape| {
            let a = shape[0][0].sqrt();
            let b = shape[0][1] / a;
            let c = (shape[1][1] - b * b).sqrt();
            let x = 0.06_f64.sqrt() * uniform(seed, acquisition_tick, 1);
            let y = 0.06_f64.sqrt() * uniform(seed, acquisition_tick, 2);
            Vec2::new(a * x, b * x + c * y)
        })
    }
}
