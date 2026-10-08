//! Constant-velocity baseline with an expanding uncertainty envelope.
use rustdrive_core::{Prediction, Predictor, Track};
pub struct ConstantVelocity {
    pub horizon: f64,
    pub dt: f64,
}
impl Default for ConstantVelocity {
    fn default() -> Self {
        Self {
            horizon: 8.0,
            dt: 0.2,
        }
    }
}
impl Predictor for ConstantVelocity {
    fn predict(&self, tracks: &[Track]) -> Vec<Prediction> {
        assert!(
            self.dt.is_finite() && self.dt > 0.0 && self.horizon.is_finite() && self.horizon >= 0.0
        );
        tracks
            .iter()
            .map(|track| Prediction {
                id: track.id,
                positions: (0..=(self.horizon / self.dt).ceil() as usize)
                    .map(|i| {
                        track.position.plus(track.velocity.scaled(
                            if track.velocity.x.hypot(track.velocity.y) < 0.7 {
                                0.0
                            } else {
                                i as f64 * self.dt
                            },
                        ))
                    })
                    .collect(),
                radius: track.radius,
                dt: self.dt,
            })
            .collect()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use rustdrive_core::Vec2;
    #[test]
    fn extrapolates_velocity() {
        let tracks = vec![Track {
            id: 7,
            position: Vec2::new(1.0, 2.0),
            velocity: Vec2::new(2.0, -1.0),
            radius: 1.0,
            last_seen: 0.0,
        }];
        let p = ConstantVelocity {
            horizon: 2.0,
            dt: 0.5,
        }
        .predict(&tracks);
        assert_eq!(p[0].positions[4], Vec2::new(5.0, 0.0));
    }
}
