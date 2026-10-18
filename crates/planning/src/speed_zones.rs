//! Fixed-route speed caps, including intervals narrower than geometric samples.
use rustdriving_core::Vec2;

pub(super) fn apply_caps(
    positions: &[Vec2],
    distances: &[f64],
    limits: &super::speed::Limits<'_>,
    initial_speed: f64,
    caps: &mut [f64],
) {
    let Some((route, zones, padding)) = limits.zones else {
        return;
    };
    let deceleration = limits.deceleration;
    let progress: Vec<_> = positions.iter().map(|p| route.project(*p).0).collect();
    for zone in zones {
        let overlaps = |a: f64, b: f64| {
            a.min(b) <= zone.end_s_m + padding && a.max(b) >= zone.start_s_m - padding
        };
        // Preserve measured initial speed when a cap is already active at the
        // first interval. Recover with bounded braking, never a speed clamp.
        let active = overlaps(progress[0], *progress.get(1).unwrap_or(&progress[0]));
        let recovering = active && initial_speed > zone.speed_m_s + 1e-6;
        let bound = |i: usize| {
            if recovering {
                zone.speed_m_s.max(
                    (initial_speed.powi(2) - 2. * deceleration * distances[i])
                        .max(0.)
                        .sqrt(),
                )
            } else {
                zone.speed_m_s
            }
        };
        if positions.len() == 1 && active {
            caps[0] = caps[0].min(bound(0));
        }
        for (i, pair) in progress.windows(2).enumerate() {
            if overlaps(pair[0], pair[1]) {
                caps[i] = caps[i].min(bound(i));
                caps[i + 1] = caps[i + 1].min(bound(i + 1));
            }
        }
    }
}
