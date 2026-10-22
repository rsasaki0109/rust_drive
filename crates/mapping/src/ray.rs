//! Positive-length intersections with half-open unit grid cells.
use rustdriving_core::Vec2;

pub(super) struct Cells {
    start: [f64; 2],
    delta: [f64; 2],
    cell: [usize; 2],
    limits: [usize; 2],
    time: f64,
    end_time: f64,
    done: bool,
}

impl Cells {
    pub fn new(start: Vec2, end: Vec2, width: usize, height: usize) -> Option<Self> {
        let delta = end.minus(start);
        if !start.finite() || !end.finite() || !delta.finite() {
            return None;
        }
        let start = [start.x, start.y];
        let delta = [delta.x, delta.y];
        let limits = [width, height];
        let (mut enter, mut exit) = (0.0_f64, 1.0_f64);
        for axis in 0..2 {
            if delta[axis] == 0.0 {
                if start[axis] < 0.0 || start[axis] >= limits[axis] as f64 {
                    return None;
                }
            } else {
                let a = -start[axis] / delta[axis];
                let b = (limits[axis] as f64 - start[axis]) / delta[axis];
                enter = enter.max(a.min(b));
                exit = exit.min(a.max(b));
            }
        }
        if enter >= exit {
            return None;
        }
        let mut cell = [0; 2];
        for axis in 0..2 {
            // Clipping roundoff must not put an entry outside the grid.
            let position = (start[axis] + delta[axis] * enter).clamp(0.0, limits[axis] as f64);
            cell[axis] = position.floor() as usize;
            if delta[axis] < 0.0 && position == position.floor() {
                cell[axis] = cell[axis].checked_sub(1)?;
            }
            if cell[axis] >= limits[axis] {
                return None;
            }
        }
        Some(Self {
            start,
            delta,
            cell,
            limits,
            time: enter,
            end_time: exit,
            done: false,
        })
    }

    fn crossing(&self, axis: usize) -> f64 {
        if self.delta[axis] == 0.0 {
            f64::INFINITY
        } else {
            let boundary = self.cell[axis] as f64 + if self.delta[axis] > 0.0 { 1.0 } else { 0.0 };
            // Recompute from the original segment, avoiding accumulated t drift.
            (boundary - self.start[axis]) / self.delta[axis]
        }
    }

    fn advance(&mut self, axis: usize) {
        let step = if self.delta[axis] > 0.0 { 1 } else { -1 };
        match self.cell[axis].checked_add_signed(step) {
            Some(cell) if cell < self.limits[axis] => self.cell[axis] = cell,
            _ => self.done = true,
        }
    }
}

impl Iterator for Cells {
    type Item = (usize, usize);

    fn next(&mut self) -> Option<Self::Item> {
        while !self.done {
            let cell = (self.cell[0], self.cell[1]);
            let x = self.crossing(0);
            let y = self.crossing(1);
            let next = x.min(y).min(self.end_time);
            let positive_length = next > self.time;
            if next >= self.end_time {
                self.done = true;
            } else {
                // A corner crossing enters the diagonal cell directly. Every
                // iteration advances an axis or finishes: at most W + H steps,
                // regardless of the out-of-grid distance of either endpoint.
                if x <= y {
                    self.advance(0);
                }
                if y <= x {
                    self.advance(1);
                }
                self.time = next;
            }
            if positive_length {
                return Some(cell);
            }
        }
        None
    }
}
