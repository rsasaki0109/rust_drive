//! Require time-separated position and velocity evidence below the jitter deadband.
use rustdriving_core::Track;
use std::collections::VecDeque;

pub(super) fn supported(track: &Track, history: &VecDeque<Track>) -> bool {
    let speed = track.velocity.x.hypot(track.velocity.y);
    if !(0.1..0.7).contains(&speed) {
        return false;
    }
    let direction = track.velocity.scaled(1. / speed);
    let mut previous = track;
    for offset in [0.2, 0.4, 0.6] {
        let target = track.last_seen - offset;
        let Some(old) = history.iter().min_by(|a, b| {
            (a.last_seen - target)
                .abs()
                .total_cmp(&(b.last_seen - target).abs())
        }) else {
            return false;
        };
        let elapsed = previous.last_seen - old.last_seen;
        let old_speed = old.velocity.x.hypot(old.velocity.y);
        let alignment =
            (old.velocity.x * direction.x + old.velocity.y * direction.y) / old_speed.max(1e-9);
        let displacement = previous.position.minus(old.position);
        let progress = displacement.x * direction.x + displacement.y * direction.y;
        if (old.last_seen - target).abs() > 0.025 + 1e-9
            || elapsed <= 0.
            || old_speed < 0.1
            || alignment < 0.95
            || progress + 1e-9 < 0.1 * elapsed
            || progress + 1e-9 < 0.9 * displacement.x.hypot(displacement.y)
        {
            return false;
        }
        previous = old;
    }
    // Spaced anchors cannot erase contrary observations between them. Check
    // every acquired velocity and displacement in the same support window.
    let earliest = previous.last_seen;
    let mut newer = track;
    for old in history
        .iter()
        .rev()
        .filter(|old| old.last_seen >= earliest && old.last_seen < track.last_seen)
    {
        let old_speed = old.velocity.x.hypot(old.velocity.y);
        let alignment =
            (old.velocity.x * direction.x + old.velocity.y * direction.y) / old_speed.max(1e-9);
        let displacement = newer.position.minus(old.position);
        let progress = displacement.x * direction.x + displacement.y * direction.y;
        if old_speed < 0.1
            || alignment < 0.95
            || progress + 1e-9 < 0.9 * displacement.x.hypot(displacement.y)
        {
            return false;
        }
        newer = old;
    }
    true
}
