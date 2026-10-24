// Fleet planning for the authored arcade city. Seconds/metres, directed roads.
// No precipitation label or future person schedule enters this module.
import { nodeKey } from './urban-roads.js';
import { routeToStop, onStopEdge, pickupDistance } from './fleet-routing.js';

const edgeKey = (from, to) => nodeKey(from) + '>' + nodeKey(to);
const length = (from, to) => Math.hypot(to.x - from.x, to.y - from.y);
export const DISPATCH_POLICIES = Object.freeze(['distance', 'eta']);
export const FORECAST_HORIZON = 60;
const PRIOR_SECONDS_PER_METRE = 1 / 6.5;

// Rectangular Hungarian assignment, with private dummy columns. First maximize
// feasible cardinality, then minimize total cost. Negative finite costs are valid.
export function minimumAssignment(costs) {
  const n = costs.length;
  if (!n) return [];
  const m = costs[0].length;
  if (n > 16 || m > 128 || costs.some(row => row.length !== m ||
      row.some(c => c !== Infinity && (!Number.isFinite(c) || Math.abs(c) > 1e6))))
    throw new Error('Invalid or unbounded fleet cost matrix');
  if (!m) return [];
  const finite = costs.flat().filter(Number.isFinite);
  if (!finite.length) return [];
  const offset = Math.min(...finite), span = Math.max(...finite) - offset;
  const dummy = (span + 1) * (n + 1), forbidden = dummy * (n + 1);
  const a = costs.map(row => [...row.map(c => Number.isFinite(c) ? c - offset : forbidden),
    ...Array(n).fill(dummy)]);
  const columns = m + n, u = Array(n + 1).fill(0), v = Array(columns + 1).fill(0);
  const p = Array(columns + 1).fill(0), way = Array(columns + 1).fill(0);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    const used = Array(columns + 1).fill(false), best = Array(columns + 1).fill(Infinity);
    let j0 = 0;
    do {
      used[j0] = true;
      const i0 = p[j0];
      let delta = Infinity, j1 = 0;
      for (let j = 1; j <= columns; j++) if (!used[j]) {
        const reduced = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (reduced < best[j]) { best[j] = reduced; way[j] = j0; }
        if (best[j] < delta) { delta = best[j]; j1 = j; }
      }
      for (let j = 0; j <= columns; j++) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else best[j] -= delta;
      }
      j0 = j1;
    } while (p[j0]);
    do { const j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  return p.flatMap((row, j) => row && j > 0 && j <= m && Number.isFinite(costs[row - 1][j - 1])
    ? [{ row: row - 1, column: j - 1, cost: costs[row - 1][j - 1] }] : [])
    .sort((a, b) => a.row - b.row);
}

export class RoadTravelTimes {
  constructor() { this.tracks = new Map(); this.edges = new Map(); this.samples = 0; }
  observe(taxis, time) {
    if (!Number.isFinite(time) || time < 0) throw new Error('Invalid telemetry timestamp');
    for (const taxi of taxis) {
      const car = taxi.car;
      if (!taxi.onDuty || !car.node || !car.next || !Number.isFinite(car.distance)) {
        this.tracks.delete(taxi.id); continue;
      }
      const key = edgeKey(car.node, car.next), old = this.tracks.get(taxi.id);
      // A service/standby dwell is intentional, rather than road congestion.
      const stop = car.serviceStop;
      const service = ['boarding', 'alighting'].includes(taxi.state) ||
        (stop && Math.hypot(car.x - stop.x, car.y - stop.y) < 1.1 && car.speed < .25);
      if (old && time <= old.last) { this.tracks.delete(taxi.id); continue; }
      if (!old || old.key !== key) {
        if (old?.eligible) {
          const metres = car.distance - old.distance, seconds = time - old.time;
          const pace = seconds / metres;
          if (metres >= 5 && seconds > .1 && seconds <= 240 && pace >= .05 && pace <= 6) {
            const prior = this.edges.get(old.key), weight = prior ? .25 : 1;
            this.edges.set(old.key, { pace: prior ? prior.pace * (1 - weight) + pace * weight : pace,
              count: (prior?.count ?? 0) + 1, updated: time });
            this.samples++;
          }
        }
        this.tracks.set(taxi.id, { key, time, last: time, distance: car.distance,
          eligible: Boolean(old) && !service });
      } else { old.last = time; if (service || car.distance < old.distance) old.eligible = false; }
    }
  }
  seconds(from, to, metres, time) {
    const entry = this.edges.get(edgeKey(from, to));
    const trust = entry && entry.updated <= time ? Math.exp(-Math.max(0, time - entry.updated) / 120) : 0;
    return Math.max(0, metres) * (PRIOR_SECONDS_PER_METRE * (1 - trust) + (entry?.pace ?? 0) * trust);
  }
  estimate(graph, car, stop, time) {
    if (!car.node || !car.next) return Infinity;
    const along = (stop.x - car.x) * stop.dx + (stop.y - car.y) * stop.dy;
    if (onStopEdge(car, stop) && along > -1) {
      if (along < 1.1 && car.speed < .25) return 0;
      return this.seconds(car.node, car.next, Math.max(0, along), time) + 5;
    }
    // Cost the same distance-routed path the arcade steering will actually use.
    const route = routeToStop(graph, car.next, car.node, stop);
    if (!route) return Infinity;
    let seconds = this.seconds(car.node, car.next, length(car, car.next), time), from = car.next;
    for (const [i, to] of route.nodes.entries()) {
      seconds += this.seconds(from, to, i === route.nodes.length - 1 ? stop.along : length(from, to), time);
      from = to;
    }
    return seconds + 5;
  }
  snapshot(time) {
    return { samples: this.samples, edges: this.edges.size, priorSpeed: 6.5,
      entries: [...this.edges].sort(([a], [b]) => a.localeCompare(b)).map(([edge, e]) =>
        ({ edge, secondsPerMetre: e.pace, samples: e.count, age: Math.max(0, time - e.updated) })) };
  }
}

// Discounted request counts with public time-of-day / rail priors. Authored
// priors and weights are frozen experimental choices, not a calibrated forecast.
export function forecastDemand(stops, clock, time, events, train = null) {
  const exposure = 1 - Math.exp(-Math.min(120, Math.max(0, time)) / 60);
  return stops.map(stop => {
    let prior = .18;
    if (clock.night && stop.id === 'nightlife') prior += 1.2;
    else if (!clock.night && stop.id === 'office') prior += .8;
    if (['homes', 'west', 'hotel'].includes(stop.id)) prior += .15;
    if (stop.id === 'station' && clock.railOpen) prior += .3;
    if (stop.id === 'station' && train && ['approach', 'dwell'].includes(train.phase)) prior += .8;
    const observed = events.filter(e => e.type === 'request' && e.stop === stop.id &&
      e.time <= time && time - e.time <= 120).reduce((n, e) => n + Math.exp(-(time - e.time) / 60), 0);
    return { stop, expected: (2 * prior + observed) / (2 + exposure),
      observed, horizonSeconds: FORECAST_HORIZON };
  });
}

export function etaAssignments(graph, travel, taxis, people, time) {
  const cabs = taxis.filter(t => t.state === 'idle' && t.onDuty && !t.retiring)
    .sort((a, b) => a.id.localeCompare(b.id));
  const requests = people.filter(p => ['walking', 'waiting'].includes(p.state) && !p.taxi)
    .sort((a, b) => a.requestAt - b.requestAt || a.id.localeCompare(b.id)).slice(0, 64);
  const estimates = cabs.map(t => requests.map(p => travel.estimate(graph, t.car, p.origin, time)));
  // One second of accrued wait has one second of urgency credit; cardinality
  // remains the primary objective. Already assigned riders never enter a batch.
  const costs = estimates.map(row => row.map((eta, j) => eta - Math.max(0, time - requests[j].requestAt)));
  return minimumAssignment(costs).map(({ row, column }) => ({ taxi: cabs[row], person: requests[column],
    eta: estimates[row][column], distance: pickupDistance(graph, cabs[row].car, requests[column].origin) }));
}

export function forecastStandby(graph, travel, taxis, forecast, bays, time) {
  const held = taxis.filter(t => !t.person && t.onDuty && !t.retiring &&
    t.standbyStop && time - t.standbySince < 20);
  const reserved = new Set(held.map(t => t.standbyStop.id));
  const free = taxis.filter(t => !t.person && t.onDuty && !t.retiring && !held.includes(t))
    .sort((a, b) => a.id.localeCompare(b.id));
  const targets = forecast.filter(f => !reserved.has(bays.get(f.stop.id).id));
  const costs = free.map((t, i) => [
    ...targets.map(f => {
      const bay = bays.get(f.stop.id), eta = travel.estimate(graph, t.car, bay, time);
      const metres = pickupDistance(graph, t.car, bay);
      return eta + metres * .02 - 45 * f.expected + (t.standbyStop && t.standbyStop !== bay ? 8 : 0);
    }),
    ...free.map((_, j) => i === j ? 0 : Infinity),
  ]);
  return minimumAssignment(costs).map(({ row, column, cost }) => ({ taxi: free[row],
    stop: column < targets.length ? bays.get(targets[column].stop.id) : null,
    expected: column < targets.length ? targets[column].expected : 0, cost }));
}

export function percentile(values, q) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.max(0, Math.ceil(q * sorted.length) - 1)];
}
