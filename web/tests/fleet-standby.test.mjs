import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { STREETS } from '../src/city.js';
import { roadGraph, nodeKey } from '../src/urban-roads.js';
import { TaxiFleet, FLEET_STOPS } from '../src/fleet.js';
import { cityConditions } from '../src/city-life.js';
import { makeStandbyStop, standbyPriorities } from '../src/fleet-standby.js';
import { atServiceStop } from '../src/fleet-routing.js';

const graph = roadGraph(STREETS, true), farPlayer = { x: -1000, y: -1000 };
const stop = id => FLEET_STOPS.find(s => s.id === id);
const fixture = s => ({ node: graph.get(nodeKey(s.from)), next: graph.get(nodeKey(s.to)),
  x: s.x, y: s.y, speed: 0, distance: 0, serviceStop: s });
const plan = (overrides = {}) => ({ id: 'P1', purpose: 'leisure', origin: 'nightlife', destination: 'west',
  release: 90, walkSpeed: 1.2, patience: 180, dwell: 4, offset: 7, ...overrides });
const step = (fleet, seconds) => { for (let i = 0; i < Math.round(seconds * 20); i++) fleet.update(.05); };

test('standby bays are twelve metres before the pickup in its legal directed curb lane', () => {
  for (const s of FLEET_STOPS) {
    const bay = makeStandbyStop(s);
    assert.equal(bay.from, s.from); assert.equal(bay.to, s.to);
    assert.ok(Math.abs(Math.hypot(bay.x - s.x, bay.y - s.y) - 12) < 1e-9);
    assert.ok((s.x - bay.x) * s.dx + (s.y - bay.y) * s.dy > 11.99);
    assert.equal(atServiceStop(fixture(bay), s), false);
    assert.equal(atServiceStop(fixture(bay), bay), true);
    assert.ok(bay.along > 7 && bay.along + 3.6 < s.along - 4);
  }
});

test('standby follows public periods, inbound service and observed queue pressure', () => {
  const rank = (period, extra = {}) => standbyPriorities(FLEET_STOPS, cityConditions(period, 0), 100,
    extra.requests ?? [], extra.waiting ?? [], extra.train);
  assert.equal(rank('morning')[0].stop.id, 'office');
  assert.equal(rank('night')[0].stop.id, 'nightlife');
  assert.equal(rank('dusk', { train: { phase: 'approach', lastService: true } })[0].stop.id, 'station');
  assert.equal(rank('morning', { waiting: Array.from({ length: 4 }, () => ({
    origin: stop('clinic'), state: 'waiting', taxi: null })) })[0].stop.id, 'clinic');
  assert.equal(rank('morning', { requests: Array.from({ length: 4 }, () => ({
    type: 'request', stop: 'market', time: 99 })) })[0].stop.id, 'market');
  assert.deepEqual(rank('night'), rank('night', { requests: [
    { type: 'request', stop: 'office', time: 101 }, { type: 'request', stop: 'office', time: 39 }] }));
});

test('unreleased travel plans cannot change standby routes or advance a request', () => {
  const fleets = [[], Array.from({ length: 20 }, (_, i) => plan({
    id: 'F' + i, origin: 'clinic', destination: 'homes', release: 1000 }))].map(plans =>
      new TaxiFleet(graph, [fixture(stop('market')), fixture(stop('station'))], { period: 'night', plans }));
  for (const f of fleets) step(f, 20);
  assert.deepEqual(fleets[0].snapshot().standby, fleets[1].snapshot().standby);
  assert.deepEqual(fleets[0].taxis.map(t => t.car.serviceStop.id), fleets[1].taxis.map(t => t.car.serviceStop.id));
  assert.equal(fleets[1].snapshot().metrics.requested, 0);
});

test('a real request immediately preempts standby without moving a car or sharing a bay', () => {
  const cars = [fixture(stop('market')), fixture(stop('station')), fixture(stop('hotel'))];
  const f = new TaxiFleet(graph, cars, { period: 'night', plans: [plan({ release: 2, origin: 'homes' })] });
  step(f, 1);
  assert.equal(new Set(f.taxis.map(t => t.standbyStop?.id)).size, 3);
  const before = cars.map(c => [c.x, c.y, c.distance]);
  step(f, 2);
  const p = f.people[0], cab = f.taxis.find(t => t.id === p.taxi);
  assert.ok(cab);
  assert.equal(cab.standbyStop, null);
  assert.equal(cab.car.serviceStop, p.origin);
  assert.deepEqual(cars.map(c => [c.x, c.y, c.distance]), before, 'planning never writes poses or odometers');
  assert.equal(f.snapshot().metrics.served, 0);
});

test('unchanged signals do not repeatedly reroute cabs; disabling preserves passenger service', () => {
  const f = new TaxiFleet(graph, [fixture(stop('clinic')), fixture(stop('hotel'))], {
    period: 'night', plans: [plan({ origin: 'homes', release: 5 })] });
  step(f, 1);
  const first = f.standbyEvents.filter(e => e.type === 'reposition').length;
  step(f, 3);
  assert.equal(f.standbyEvents.filter(e => e.type === 'reposition').length, first);
  step(f, 2);
  const assigned = f.taxis.find(t => t.person), service = assigned.car.serviceStop;
  f.setStandbyEnabled(false);
  assert.equal(assigned.car.serviceStop, service);
  assert.ok(f.taxis.every(t => !t.standbyStop));
  assert.equal(f.snapshot().standby.enabled, false);
  assert.throws(() => f.setStandbyEnabled('true'), /boolean/);
  f.setStandbyEnabled(true); step(f, .1);
  assert.equal(assigned.car.serviceStop, service);
});

test('off-duty and occupied cabs never receive standby assignments', () => {
  const f = new TaxiFleet(graph, Array.from({ length: 10 }, () => fixture(stop('homes'))), {
    period: 'morning', plans: [plan({ origin: 'homes', release: 0 })] });
  step(f, 1);
  assert.ok(f.taxis.slice(8).every(t => t.state === 'off-duty' && !t.standbyStop && t.car.serviceStop.id.startsWith('depot-')));
  assert.ok(f.taxis.filter(t => t.person).every(t => !t.standbyStop && t.car.serviceStop.id === 'homes'));
  const bays = f.taxis.filter(t => t.standbyStop).map(t => t.standbyStop.id);
  assert.equal(new Set(bays).size, bays.length);
});

test('surplus idle taxis leave service bays instead of stacking in a claimed standby bay', () => {
  const f = new TaxiFleet(graph, Array.from({ length: 12 }, () => fixture(stop('office'))), { period: 'night', plans: [] });
  step(f, 1);
  assert.equal(f.taxis.filter(t => t.standbyStop).length, 9);
  assert.equal(f.taxis.filter(t => t.car.serviceStop === null).length, 3);
  assert.equal(new Set(f.taxis.filter(t => t.standbyStop).map(t => t.standbyStop.id)).size, 9);
});

test('empty and standby distance comes from actual odometers, never elapsed time', () => {
  const car = fixture(stop('market')), f = new TaxiFleet(graph, [car], { plans: [] });
  step(f, 20);
  assert.equal(f.snapshot().metrics.emptyDistance, 0);
  assert.equal(f.snapshot().metrics.repositionDistance, 0);
  car.distance = 17; step(f, .05);
  assert.equal(f.snapshot().metrics.emptyDistance, 17);
  assert.equal(f.snapshot().metrics.repositionDistance, 17);
  f.setStandbyEnabled(false); car.distance = 20; step(f, .05);
  assert.equal(f.snapshot().metrics.emptyDistance, 20);
  assert.equal(f.snapshot().metrics.repositionDistance, 17);
});

test('real traffic reaches standby before a late-night departure and reduces its pickup wait', async () => {
  async function run(standby) {
    // Fresh module instances reset the same authored traffic RNG for each policy.
    const { Traffic } = await import('../src/city.js?standby-comparison=' + standby);
    const traffic = new Traffic(new THREE.Scene(), { urban: true, cityCars: 14 });
    const s = stop('clinic');
    const car = traffic.spawn(false, 0xffc34b, { ...s.from, to: s.to, advance: 10, variant: 'taxi', serviceStop: s });
    car.desired = 9;
    const f = new TaxiFleet(traffic.graph, [car], { period: 'night', plans: [plan()], standby });
    const samples = [];
    for (let i = 0; i < 200 * 30; i++) {
      f.update(1 / 30); traffic.update(1 / 30, f.time, farPlayer);
      if (i % 30 === 0) samples.push({ time: f.time, x: car.x, y: car.y, speed: car.speed, distance: car.distance });
    }
    const snap = f.snapshot(), request = snap.events.find(e => e.type === 'request'),
      board = snap.events.find(e => e.type === 'board');
    assert.ok(request && board, JSON.stringify({ standby, metrics: snap.metrics, events: snap.events }));
    assert.ok(snap.metrics.served >= 1, JSON.stringify(snap.metrics));
    assert.ok(samples.every(s => Object.values(s).every(Number.isFinite)));
    assert.ok(snap.events.filter(e => ['board', 'alight'].includes(e.type)).every(e => e.speed < .25));
    return { wait: board.time - request.time, metrics: snap.metrics, standbyEvents: snap.standby.events,
      requestedAt: request.time, firstBoard: board.time, samples };
  }
  const baseline = await run(false), proactive = await run(true);
  console.log('ACTUAL STANDBY COMPARISON', JSON.stringify({ baseline, proactive }));
  assert.ok(proactive.standbyEvents.some(e => e.type === 'standby-arrive' &&
    e.stop === 'standby-nightlife' && e.time < proactive.requestedAt), 'must physically arrive before the request');
  assert.ok(proactive.wait + 10 < baseline.wait, JSON.stringify({ baseline: baseline.wait, proactive: proactive.wait }));
  assert.ok(proactive.metrics.repositionDistance > 50);
  assert.equal(baseline.metrics.repositionDistance, 0);
});
