import test from 'node:test';
import assert from 'node:assert/strict';
import { TaxiFleet, FLEET_STOPS } from '../src/fleet.js';
import { roadGraph } from '../src/urban-roads.js';
const graph = roadGraph([-150, -100, -50, 0, 50, 100, 150]);
const stop = id => FLEET_STOPS.find(s => s.id === id);
const car = s => ({ x: s.x, y: s.y, node: s.from, next: s.to, speed: 0, distance: 0, serviceStop: s });
const plan = (id, origin, release) => ({ id, origin, destination: origin === 'homes' ? 'hotel' : 'homes',
  purpose: 'shopping', release, walkSpeed: 1, patience: 90, dwell: 3, offset: 8 });
function step(f, seconds) { for (let i = 0; i < seconds * 20; i++) f.update(.05); }
test('changing dispatch policy preserves passenger ownership and only changes future decisions', () => {
  const f = new TaxiFleet(graph, [car(stop('homes'))], { dispatch: 'eta', plans: [plan('P', 'homes', 0)] });
  step(f, .2);
  const taxi = f.taxis[0], rider = taxi.person, target = taxi.car.serviceStop;
  assert.ok(rider);
  f.setDispatchPolicy('distance');
  assert.equal(taxi.person, rider); assert.equal(rider.taxi, taxi.id);
  assert.equal(taxi.car.serviceStop, target);
  assert.throws(() => f.setDispatchPolicy('oracle'), /policy/);
  assert.equal(f.snapshot().dispatch.policy, 'distance');
});
test('forecast standby has distinct bays and never edits poses, odometers or completed counts', () => {
  const cars = [car(stop('clinic')), car(stop('hotel')), car(stop('market'))];
  const f = new TaxiFleet(graph, cars, { period: 'night', dispatch: 'eta', plans: [] });
  const before = cars.map(c => [c.x, c.y, c.distance]);
  step(f, 5);
  const targets = f.taxis.filter(t => t.standbyStop).map(t => t.standbyStop.id);
  assert.equal(new Set(targets).size, targets.length);
  assert.deepEqual(cars.map(c => [c.x, c.y, c.distance]), before);
  assert.equal(f.snapshot().metrics.served, 0);
  assert.equal(f.snapshot().dispatch.forecast.length, FLEET_STOPS.length);
});
test('unreleased future plans cannot change the operational decision prefix', () => {
  function run(future) {
    const cars = [car(stop('clinic')), car(stop('homes')), car(stop('hotel'))];
    const f = new TaxiFleet(graph, cars, { dispatch: 'eta',
      plans: [plan('now', 'homes', 0), plan('future', future, 1000)] });
    step(f, 10);
    const snap = f.snapshot();
    return { dispatch: snap.dispatch, metrics: snap.metrics, taxis: snap.taxis, events: snap.events, standby: snap.standby };
  }
  assert.deepEqual(run('nightlife'), run('office'));
});
test('pickup quantiles report observations separately from unserved demand', () => {
  const f = new TaxiFleet(graph, [car(stop('homes'))], { dispatch: 'eta', plans: [plan('P', 'homes', 0)] });
  assert.equal(f.snapshot().metrics.pickupP95, null);
  step(f, 20);
  const m = f.snapshot().metrics;
  assert.equal(m.requested, 1);
  assert.equal(m.pickupWaitSamples, 1);
  assert.equal(m.pickupP50, m.pickupP95);
  assert.equal(m.etaErrorSamples, 1);
  assert.equal(m.served, 0, 'static fixture cannot complete a trip');
});
