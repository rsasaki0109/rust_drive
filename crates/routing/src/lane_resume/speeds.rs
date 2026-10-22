//! Rebase the unchanged original tail after a new bounded connector.
use super::LaneResumePlan;
use rustdriving_core::{Route, RouteSpeedZone};

impl LaneResumePlan {
    /// Cap the entire new connector, then clip and translate original tail
    /// intervals. Distances remain actual route arc lengths in meters.
    pub fn speed_zones(
        &self,
        original: &Route,
        original_zones: &[RouteSpeedZone],
        connector_limit_m_s: f64,
    ) -> Result<Vec<RouteSpeedZone>, String> {
        let canonical = Route::new(original.points.clone(), original.half_width)?;
        let resumed = Route::new(self.route.points.clone(), self.route.half_width)?;
        if !connector_limit_m_s.is_finite()
            || !(0.1..=8.0).contains(&connector_limit_m_s)
            || canonical.lengths != original.lengths
            || resumed.lengths != self.route.lengths
            || canonical.length() > 1500.
            || resumed.length() > 1500.
            || !self.join_s_m.is_finite()
            || !(0.0..canonical.length()).contains(&self.join_s_m)
            || !self.connector_length_m.is_finite()
            || !(0.0..resumed.length()).contains(&self.connector_length_m)
            || self.connector_length_m == 0.
            || (resumed.length() - self.connector_length_m - canonical.length() + self.join_s_m)
                .abs()
                > 1e-6
            || resumed
                .sample(self.connector_length_m, 0.)
                .0
                .distance(canonical.sample(self.join_s_m, 0.).0)
                > 1e-6
            || original_zones.len() > 128
            || original_zones
                .iter()
                .any(|zone| !zone.valid() || zone.end_s_m > canonical.length())
        {
            return Err("invalid resume speed policy or route join metadata".into());
        }
        let mut zones = vec![RouteSpeedZone {
            start_s_m: 0.,
            end_s_m: self.connector_length_m,
            speed_m_s: connector_limit_m_s,
        }];
        for original in original_zones {
            if original.end_s_m <= self.join_s_m {
                continue;
            }
            let zone = RouteSpeedZone {
                start_s_m: (self.connector_length_m + (original.start_s_m - self.join_s_m).max(0.))
                    .min(resumed.length()),
                end_s_m: (self.connector_length_m + original.end_s_m - self.join_s_m)
                    .min(resumed.length()),
                speed_m_s: original.speed_m_s,
            };
            if zone.end_s_m <= zone.start_s_m {
                continue;
            }
            if let Some(previous) = zones.last_mut().filter(|previous| {
                previous.end_s_m == zone.start_s_m && previous.speed_m_s == zone.speed_m_s
            }) {
                previous.end_s_m = zone.end_s_m;
            } else {
                zones.push(zone);
            }
            if zones.len() > 128 {
                return Err("resume speed policy exceeds 128 intervals".into());
            }
        }
        Ok(zones)
    }
}
