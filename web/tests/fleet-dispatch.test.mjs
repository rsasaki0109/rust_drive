import test from 'node:test';
import assert from 'node:assert/strict';
import { minimumAssignment, RoadTravelTimes, forecastDemand, etaAssignments, percentile } from '../src/fleet-dispatch.js';
import { makeStop, pickupDistance } from '../src/fleet-routing.js';
import { roadGraph } from '../src/urban-roads.js';
import { cityConditions } from '../src/city-life.js';
const streets = [-100, -50, 0, 50, 100], graph = roadGraph(streets);

function oracle(a) {
  let result = { cardinality: -1, cost: Infinity };
  function visit(row, used, cardinality, cost) {
    if (row === a.length) {
      if (cardinality > result.cardinality || cardinality === result.cardinality && cost < result.cost)
        result = { cardinality, cost };
      return;
    }
    visit(row + 1, used, cardinality, cost);
    a[row].forEach((c, j) => {
      if (!used.has(j) && Number.isFinite(c)) {
        used.add(j); visit(row + 1, used, cardinality + 1, cost + c); used.delete(j);
      }
    });
  }
  visit(0, new Set(), 0, 0); return result;
}
test('global assignment matches exhaustive independent enumeration, including disconnected and negative costs', () => {
  let seed = 71;
  const random = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; };
  for (let i = 0; i < 500; i++) {
    const n = 1 + Math.floor(random() * 4), m = 1 + Math.floor(random() * 5);
    const costs = Array.from({ length: n }, () => Array.from({ length: m }, () =>
      random() < .25 ? Infinity : Math.floor(random() * 40) - 20));
    const actual = minimumAssignment(costs), expected = oracle(costs);
    assert.equal(actual.length, expected.cardinality);
    assert.equal(actual.reduce((n, a) => n + a.cost, 0), expected.cost);
    assert.equal(new Set(actual.map(a => a.column)).size, actual.length);
  }
});
test('batch choice avoids a greedy trap and rejects malformed or unbounded matrices', () => {
  assert.deepEqual(minimumAssignment([[1, 2], [1.1, 100]]).map(a => a.column), [1, 0]);
  assert.equal(minimumAssignment([[Infinity], [1]]).length, 1);
  assert.deepEqual(minimumAssignment([[], []]), []);
  for (const a of [[[NaN]], [[-Infinity]], [[1], []], Array.from({ length: 17 }, () => [1]), [[1e7]]])
    assert.throws(() => minimumAssignment(a), /matrix/);
});
const point = (x, y) => ({ x, y });
const probe = () => ({ id: 'probe', onDuty: true, state: 'idle', car: {
  node: point(0, -100), next: point(0, -50), distance: 0, speed: 6, x: 3.8, y: -60 } });
function delayedRoad() {
  const model = new RoadTravelTimes(), taxi = probe();
  model.observe([taxi], 0);
  Object.assign(taxi.car, { node: point(0, -50), next: point(0, 0), distance: 10 });
  model.observe([taxi], 2);
  Object.assign(taxi.car, { node: point(0, 0), next: point(50, 0), distance: 60 });
  model.observe([taxi], 42); return model;
}
test('completed segment timings learn real odometry/time, expire and never leak future samples', () => {
  const m = delayedRoad(), a = point(0, -50), b = point(0, 0);
  assert.equal(m.samples, 1, 'initial partial edge is discarded');
  assert.equal(m.seconds(a, b, 50, 42), 40);
  assert.equal(m.seconds(a, b, 50, 1), 50 / 6.5, 'future telemetry unavailable');
  assert.ok(m.seconds(a, b, 50, 282) > 50 / 6.5 && m.seconds(a, b, 50, 282) < 40);
  assert.equal(m.snapshot(42).entries[0].samples, 1);
});
test('intentional boarding/parking dwell and odometer resets do not become traffic delays', () => {
  const m = new RoadTravelTimes(), t = probe();
  m.observe([t], 0);
  Object.assign(t.car, { node: point(0, -50), next: point(0, 0), distance: 10 }); m.observe([t], 2);
  t.state = 'boarding'; m.observe([t], 10);
  t.state = 'idle';
  Object.assign(t.car, { node: point(0, 0), next: point(50, 0), distance: 60 }); m.observe([t], 100);
  assert.equal(m.samples, 0);
  t.car.distance = 0; m.observe([t], 101);
  Object.assign(t.car, { node: point(50, 0), next: point(50, 50), distance: 50 }); m.observe([t], 110);
  assert.equal(m.samples, 0);
});
test('a farther cab can win after observed road delay, without weather labels', () => {
  const travel = delayedRoad(), origin = makeStop('test', 'TEST', point(0, 0), point(50, 0));
  const taxi = (id, node, next, x, y) => ({ id, state: 'idle', onDuty: true, car: { node, next, x, y, speed: 6 } });
  const cabs = [taxi('A', point(0, -50), point(0, 0), 3.8, -20),
    taxi('B', point(-50, 0), point(0, 0), -40, -3.8)];
  const person = { id: 'P', state: 'waiting', requestAt: 40, taxi: null, origin };
  assert.ok(pickupDistance(graph, cabs[0].car, origin) < pickupDistance(graph, cabs[1].car, origin));
  const choice = etaAssignments(graph, travel, cabs, [person], 42);
  assert.equal(choice[0].taxi.id, 'B');
  assert.ok(choice[0].eta < travel.estimate(graph, cabs[0].car, origin, 42));
});
test('forecast uses past requests and public service state, ignoring future entries', () => {
  const stops = [{ id: 'office' }, { id: 'nightlife' }, { id: 'station' }];
  const base = [{ type: 'request', stop: 'office', time: 5 }];
  const now = forecastDemand(stops, cityConditions('morning', 10), 10, base);
  assert.deepEqual(forecastDemand(stops, cityConditions('morning', 10), 10,
    [...base, { type: 'request', stop: 'nightlife', time: 11 }]), now);
  assert.ok(forecastDemand(stops, cityConditions('night', 10), 10, [])[1].expected > now[1].expected);
  assert.equal(percentile([], .95), null);
  assert.equal(percentile([1, 2, 3, 4, 100], .95), 100);
});
