import test from 'node:test';
import assert from 'node:assert/strict';
import { Traffic, STREETS } from '../src/city.js';
import { roadGraph, nodeKey } from '../src/urban-roads.js';
import { TaxiFleet, FLEET_STOPS, createDemand } from '../src/fleet.js';
import { cityConditions, NorthLine, FACILITIES } from '../src/city-life.js';

const graph = roadGraph(STREETS, true);
const stop = id => FLEET_STOPS.find(s => s.id === id);
const carAt = id => { const s = stop(id); return { node: graph.get(nodeKey(s.from)), next: graph.get(nodeKey(s.to)), x: s.x, y: s.y, speed: 0, distance: 0, serviceStop: s }; };
const plan = (overrides = {}) => ({ id: 'O1', purpose: 'connection', origin: 'office', destination: 'station',
  release: 0, walkSpeed: 1.3, patience: 120, dwell: 4, offset: 2, ...overrides });
const run = (fleet, seconds, inspect = () => {}) => { for (let i = 0; i < seconds * 20; i++) { fleet.update(.05); inspect(fleet); } };

test('the 15x city clock wraps a day, changes light, supply and last-service availability', () => {
  const morning = cityConditions('morning', 0), dusk = cityConditions('dusk', 0), night = cityConditions('night', 0);
  assert.ok(morning.daylight > dusk.daylight && dusk.daylight > night.daylight);
  assert.deepEqual([morning.taxiSupply, dusk.taxiSupply, night.taxiSupply], [8, 10, 12]);
  assert.ok(night.railOpen);
  assert.equal(cityConditions('night', 81).railOpen, false);
  assert.equal(cityConditions('night', 81).night, true);
  assert.equal(cityConditions('night', 5760).minutes, night.minutes);
});

test('late-night departures concentrate on nightlife and exceed daytime demand', () => {
  const day = createDemand(), night = createDemand(1701, 72, 'night');
  assert.ok(day.filter(p => p.origin === 'office').length >= 8);
  assert.ok(night.filter(p => p.origin === 'nightlife').length > night.length * .5);
  assert.ok(night.filter(p => p.release < 120).length > day.filter(p => p.release < 120).length * 1.5);
  assert.equal(day[0].origin, 'office');
  assert.ok(day.every(p => p.origin !== 'station'), 'scheduled curb agents cannot impersonate train arrivals');
});

test('an office worker walks through an open doorway before requesting or boarding', () => {
  const fleet = new TaxiFleet(graph, [carAt('office')], { plans: [plan()] }), initial = { ...fleet.people[0] };
  assert.ok(initial.y < FACILITIES.office.door.y);
  let previous = initial, checked = 0;
  run(fleet, 10, f => {
    const p = f.people[0];
    if (!['riding', 'boarding'].includes(p.state)) {
      assert.ok(Math.hypot(p.x - previous.x, p.y - previous.y) <= p.walkSpeed * .05 + .002, 'continuous walking, no spawn outside');
      checked++;
    }
    previous = { ...p };
    if (Math.abs(p.y - FACILITIES.office.door.y) < .4) assert.ok(f.doors.office.open >= .92);
  });
  const events = fleet.events, departure = events.find(e => e.type === 'building-depart'), exit = events.find(e => e.type === 'building-exit');
  const request = events.find(e => e.type === 'request'), board = events.find(e => e.type === 'board');
  assert.ok(checked > 20 && departure.time < exit.time && exit.time < request.time && request.time < board.time);
  assert.ok(exit.y > FACILITIES.office.door.y + .39 && exit.doorOpen >= .92);
  assert.equal(request.source, 'office');
});

test('an abandoned office request walks back through the door and goes indoors', () => {
  const f = new TaxiFleet(graph, [], { plans: [plan({ patience: .3 })] });
  run(f, 25);
  assert.equal(f.snapshot().metrics.cancelled, 1); assert.equal(f.snapshot().metrics.served, 0);
  assert.equal(f.people[0].insideBuilding, 'office');
  assert.equal(f.people[0].state, 'cancelled');
  const cancel = f.events.find(e => e.type === 'cancel'), entered = f.events.find(e => e.type === 'building-enter');
  assert.ok(entered.time > cancel.time && entered.doorOpen >= .92);
});

test('night trains enter, stop, open, close and leave; no new service spawns after midnight', () => {
  const line = new NorthLine('night'); let last = null;
  for (let i = 0; i < 400 * 20; i++) {
    line.update(.05);
    const t = line.snapshot();
    if (t && last?.id === t.id) assert.ok(t.x >= last.x && t.x - last.x <= 12 * .05 + .001);
    if (t?.doorOpen > 0) assert.equal(t.speed, 0);
    last = t;
  }
  assert.equal(line.events.filter(e => e.type === 'train-spawn').length, 2);
  const arrivals = line.events.filter(e => e.type === 'train-arrive');
  assert.equal(arrivals.length, 2); assert.ok(arrivals[1].lastService);
  assert.ok(arrivals.every(e => Math.abs(e.x - 25) < .1 && e.speed < .15));
  assert.ok(line.events.filter(e => e.type === 'train-depart').every(e =>
    e.time > arrivals.find(a => a.train === e.train).time + 8));
});

test('train riders walk over the 5 m footbridge before generating a curb request', () => {
  const f = new TaxiFleet(graph, [], { period: 'night' }); let bridge = false;
  run(f, 150, world => {
    for (const p of world.people.filter(p => p.source === 'train' && p.state === 'walking-from-train')) {
      if (p.y >= 139 && p.y <= 161 && Math.abs(p.x - 39.2) < .1) { bridge = true; assert.ok(Math.abs(p.height - 5) < .01); }
    }
  });
  assert.ok(bridge);
  const requests = f.events.filter(e => e.type === 'request' && e.source === 'train');
  assert.ok(requests.length >= 3);
  for (const r of requests) {
    const alighted = f.events.find(e => e.type === 'train-alight' && e.person === r.person);
    assert.ok(alighted && r.time - alighted.time > 40);
    assert.equal(r.height, 0);
    assert.ok(Math.hypot(r.x - stop('station').curb.x, r.y - stop('station').curb.y) < .01);
  }
});

test('a taxi passenger can walk onto the platform and board a stopped train', () => {
  const c = carAt('homes'), f = new TaxiFleet(graph, [c], { plans: [plan({ origin: 'homes', offset: 1 })] });
  f.line = new NorthLine('morning');
  run(f, 10); assert.equal(f.people[0].state, 'riding');
  Object.assign(c, carAt('station'));
  run(f, 145);
  const boarded = f.events.find(e => e.type === 'train-board' && e.person === 'O1');
  assert.ok(boarded && boarded.speed < .15);
  assert.ok(f.events.find(e => e.type === 'rail-platform' && e.person === 'O1').time < boarded.time);
  assert.ok(f.events.some(e => e.type === 'rail-trip-complete' && e.person === 'O1'));
});

test('missing the last train produces a second taxi leg after walking back to the curb', () => {
  const c = carAt('homes'), f = new TaxiFleet(graph, [c], { period: 'night', plans: [plan({ origin: 'homes', offset: 1, patience: 200 })] });
  f.line = new NorthLine('night');
  run(f, 140); assert.equal(f.people[0].state, 'riding');
  Object.assign(c, carAt('station')); run(f, 200);
  const missed = f.events.find(e => e.type === 'missed-last-train' && e.person === 'O1');
  const request = f.events.find(e => e.type === 'request' && e.person === 'O1' && e.leg === 2);
  assert.ok(missed && request && request.time > missed.time + 40);
  assert.equal(request.source, 'missed-train');
});

test('shift changes do not abandon an occupied taxi', () => {
  const cars = Array.from({ length: 12 }, () => carAt('office'));
  const f = new TaxiFleet(graph, cars, { period: 'night', plans: [] });
  f.update(.05); assert.equal(f.snapshot().metrics.onDuty, 12);
  f.period = 'morning'; f.update(.05);
  assert.equal(f.snapshot().metrics.onDuty, 8);
  assert.ok(f.taxis.slice(8).every(t => t.state === 'off-duty'));
});
