//! Spatial candidates preserve the original directed breadth-first traversal.
use rustdriving_core::Vec2;
use std::collections::BTreeMap;

type Cell = (i64, i64);

struct Grid {
    cells: BTreeMap<Cell, Vec<usize>>,
    locations: Vec<(Cell, usize)>,
}

impl Grid {
    fn new(points: &[Vec2]) -> Option<Self> {
        // Avoid allocating an index for small scans. Outside the exact integer
        // cell domain, the original exhaustive traversal remains available.
        if points.len() <= 64 {
            return None;
        }
        let mut cells: BTreeMap<Cell, Vec<usize>> = BTreeMap::new();
        let mut locations = Vec::with_capacity(points.len());
        for (index, point) in points.iter().enumerate() {
            let x = point.x.floor();
            let y = point.y.floor();
            if !x.is_finite()
                || !y.is_finite()
                || x.abs() > (1_u64 << 52) as f64
                || y.abs() > (1_u64 << 52) as f64
            {
                return None;
            }
            let cell = (x as i64, y as i64);
            let bucket = cells.entry(cell).or_default();
            locations.push((cell, bucket.len()));
            bucket.push(index);
        }
        Some(Self { cells, locations })
    }

    fn remove(&mut self, index: usize) {
        let (cell, slot) = self.locations[index];
        let bucket = self.cells.get_mut(&cell).expect("undiscovered point");
        debug_assert_eq!(bucket[slot], index);
        bucket.swap_remove(slot);
        if let Some(&moved) = bucket.get(slot) {
            self.locations[moved].1 = slot;
        }
        if bucket.is_empty() {
            self.cells.remove(&cell);
        }
    }

    fn candidates(&self, index: usize) -> Vec<usize> {
        let (x, y) = self.locations[index].0;
        let mut result = Vec::new();
        // The unchanged adaptive tolerance is at most 0.85 m. Unit cells
        // therefore need only the surrounding 3 by 3 neighborhood.
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(bucket) = self.cells.get(&(x + dx, y + dy)) {
                    result.extend_from_slice(bucket);
                }
            }
        }
        // Swap removal and cell iteration must never change discovery order:
        // queue order also controls every subsequent floating-point reduction.
        result.sort_unstable();
        result
    }
}

pub(super) struct Components<'a> {
    points: &'a [Vec2],
    visited: Vec<bool>,
    seed: usize,
    grid: Option<Grid>,
    #[cfg(test)]
    pub candidate_visits: usize,
    #[cfg(test)]
    pub distance_checks: usize,
}

impl<'a> Components<'a> {
    pub fn new(points: &'a [Vec2]) -> Self {
        Self {
            points,
            visited: vec![false; points.len()],
            seed: 0,
            grid: Grid::new(points),
            #[cfg(test)]
            candidate_visits: 0,
            #[cfg(test)]
            distance_checks: 0,
        }
    }

    fn discover(&mut self, index: usize, queue: &mut Vec<usize>) {
        self.visited[index] = true;
        if let Some(grid) = &mut self.grid {
            grid.remove(index);
        }
        queue.push(index);
    }

    fn expand(
        &mut self,
        point: Vec2,
        tolerance: f64,
        candidates: impl Iterator<Item = usize>,
        queue: &mut Vec<usize>,
    ) {
        for index in candidates {
            #[cfg(test)]
            {
                self.candidate_visits += 1;
            }
            if !self.visited[index] {
                #[cfg(test)]
                {
                    self.distance_checks += 1;
                }
                if point.distance(self.points[index]) < tolerance {
                    self.discover(index, queue);
                }
            }
        }
    }
}

impl Iterator for Components<'_> {
    type Item = Vec<usize>;

    fn next(&mut self) -> Option<Self::Item> {
        while self.seed < self.points.len() && self.visited[self.seed] {
            self.seed += 1;
        }
        if self.seed == self.points.len() {
            return None;
        }
        let mut queue = Vec::with_capacity(1);
        self.discover(self.seed, &mut queue);
        self.seed += 1;
        let mut cursor = 0;
        while cursor < queue.len() {
            let index = queue[cursor];
            let point = self.points[index];
            cursor += 1;
            let tolerance = (0.28 + point.x.hypot(point.y) * 0.015).min(0.85);
            if let Some(grid) = &self.grid {
                let candidates = grid.candidates(index);
                self.expand(point, tolerance, candidates.into_iter(), &mut queue);
            } else {
                self.expand(point, tolerance, 0..self.points.len(), &mut queue);
            }
        }
        Some(queue)
    }
}

#[cfg(test)]
mod tests;
