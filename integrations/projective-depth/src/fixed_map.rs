//! Local pose against one immutable first measured depth image. No map fusion.
use super::*;

struct Map {
    timestamp: f64,
    pyramid: Vec<Level>,
}

#[derive(Serialize)]
pub struct FixedMapUpdate {
    pub timestamp: f64,
    pub accepted: bool,
    pub initialized: bool,
    pub lost: bool,
    pub rejection: Option<String>,
    pub first_from_current: Option<Pose>,
    pub map_timestamp: Option<f64>,
    pub last_accepted_timestamp: Option<f64>,
    pub iterations: Vec<Iteration>,
    pub final_equation: Option<Equation>,
    pub information_failure: Option<InformationFailure>,
}

/// A local fixed-map baseline. The first qualified acquisition establishes the
/// optical map frame; every later fit directly estimates first-from-current.
/// The original half-metre displacement bound limits this fixed map's domain.
pub struct FixedMapTracker {
    camera: Camera,
    map: Option<Map>,
    prior: Pose,
    last_observed: Option<f64>,
    last_accepted: Option<f64>,
    lost: bool,
}

impl FixedMapTracker {
    pub fn new(camera: Camera) -> Result<Self, String> {
        camera.validate()?;
        Ok(Self {
            camera,
            map: None,
            prior: Pose::identity(),
            last_observed: None,
            last_accepted: None,
            lost: false,
        })
    }

    pub fn step(&mut self, timestamp: f64, depth: &[f64]) -> FixedMapUpdate {
        let mut out = FixedMapUpdate {
            timestamp,
            accepted: false,
            initialized: self.map.is_some(),
            lost: self.lost,
            rejection: None,
            first_from_current: None,
            map_timestamp: self.map.as_ref().map(|m| m.timestamp),
            last_accepted_timestamp: self.last_accepted,
            iterations: Vec::new(),
            final_equation: None,
            information_failure: None,
        };
        let result = (|| -> Result<(), String> {
            if !timestamp.is_finite()
                || !(0.0..=1e12).contains(&timestamp)
                || self.last_observed.is_some_and(|last| timestamp <= last)
            {
                return Err("invalid, duplicate or regressing acquisition clock".into());
            }
            self.last_observed = Some(timestamp);
            if self
                .last_accepted
                .is_some_and(|last| timestamp - last > AGE)
            {
                self.lost = true;
            }
            if self.lost {
                return Err("accepted-pose expiry latched; no reset".into());
            }
            if depth.len() != self.camera.width * self.camera.height
                || depth
                    .iter()
                    .any(|z| !z.is_finite() || !(0.0..=100.).contains(z))
            {
                return Err("invalid bounded depth image".into());
            }
            let current = pyramid(self.camera, depth);
            let target = self
                .map
                .as_ref()
                .map_or(current.as_slice(), |m| m.pyramid.as_slice());
            let (absolute, equation, _) = fit(
                target,
                &current,
                self.prior,
                &mut out.iterations,
                &mut out.information_failure,
                FitPolicy {
                    correspondence: Correspondence::Forward,
                    residual: Residual::TargetPlane,
                    safeguarded: false,
                },
                None,
            )?;
            if !absolute.finite_rigid() {
                return Err("nonrigid fixed-map pose".into());
            }
            // This is the only map mutation. No accepted or rejected later
            // acquisition can change its pixels, frame, pyramid or timestamp.
            if self.map.is_none() {
                self.map = Some(Map {
                    timestamp,
                    pyramid: current,
                });
            }
            self.prior = absolute;
            self.last_accepted = Some(timestamp);
            out.accepted = true;
            out.first_from_current = Some(absolute);
            out.final_equation = Some(equation);
            Ok(())
        })();
        out.rejection = result.err();
        out.initialized = self.map.is_some();
        out.map_timestamp = self.map.as_ref().map(|m| m.timestamp);
        out.last_accepted_timestamp = self.last_accepted;
        out.lost = self.lost;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> (Camera, Vec<f64>) {
        let camera = Camera {
            width: 128,
            height: 96,
            fx: 110.,
            fy: 110.,
            cx: 63.5,
            cy: 47.5,
        };
        let depth = (0..128 * 96)
            .map(|i| {
                1.5 + 0.12 * ((i % 128) as f64 / 9.).sin() + 0.08 * ((i / 128) as f64 / 11.).cos()
            })
            .collect();
        (camera, depth)
    }

    #[test]
    fn accepted_identity_and_rejected_data_leave_the_original_map_unchanged() {
        let (camera, depth) = fixture();
        let mut tracker = FixedMapTracker::new(camera).unwrap();
        assert!(tracker.step(1., &depth).accepted);
        let map = tracker.map.as_ref().unwrap().pyramid[2].vertices.clone();
        let second = tracker.step(1.05, &depth);
        assert!(second.accepted);
        assert_eq!(second.map_timestamp, Some(1.));
        assert!(norm(second.first_from_current.unwrap().translation) < 1e-12);
        assert!(!tracker.step(1.05, &depth).accepted);
        assert!(!tracker.step(1.1, &vec![0.; depth.len()]).accepted);
        assert_eq!(tracker.last_accepted, Some(1.05));
        assert_eq!(tracker.map.as_ref().unwrap().pyramid[2].vertices, map);
        assert_eq!(tracker.map.as_ref().unwrap().timestamp, 1.);
    }

    #[test]
    fn rejected_initial_geometry_does_not_establish_map_or_pose() {
        let (camera, depth) = fixture();
        let mut tracker = FixedMapTracker::new(camera).unwrap();
        assert!(!tracker.step(1., &vec![2.; depth.len()]).accepted);
        assert!(tracker.map.is_none() && tracker.last_accepted.is_none());
        assert!(tracker.step(1.1, &depth).accepted);
        assert_eq!(tracker.map.as_ref().unwrap().timestamp, 1.1);
    }

    #[test]
    fn bad_acquisitions_cannot_hide_expiry_or_reinitialize_a_lost_map() {
        let (camera, depth) = fixture();
        let mut tracker = FixedMapTracker::new(camera).unwrap();
        assert!(tracker.step(1., &depth).accepted);
        assert!(!tracker.step(1.1, &[]).accepted);
        let expired = tracker.step(1.201, &[]);
        assert!(expired.lost && !expired.accepted);
        assert_eq!(expired.last_accepted_timestamp, Some(1.));
        let later = tracker.step(1.25, &depth);
        assert!(later.lost && !later.accepted);
        assert_eq!(later.map_timestamp, Some(1.));
    }
}
