import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { Traffic, STREETS } from '../src/city.js';
import { roadGraph, nodeKey } from '../src/urban-roads.js';
import { makeStop, routeToStop, pickupDistance, stopSpeedLimit } from '../src/fleet-routing.js';
import { TaxiFleet, createDemand, FLEET_STOPS, PURPOSES } from '../src/fleet.js';

const graph = roadGraph(STREETS, true);
const farPlayer = { x: -1000, y: -1000 };
const step = (fleet, seconds) => { for (let n = 0; n < Math.round(seconds * 20); n++) fleet.update(.05); };
const fixtureCar = stop => ({ node: graph.get(nodeKey(stop.from)), next: graph.get(nodeKey(stop.to)),
  x: stop.x, y: stop.y, speed: 0, distance: 0, serviceStop: stop });
const plan = (id = 'P1', overrides = {}) => ({ id, purpose: 'commute', origin: 'homes', destination: 'office',
  release: 0, walkSpeed: 1, patience: 50, dwell: 4, offset: 2, ...overrides });

test('individual demand is reproducible and varies purpose, OD, departure, walking and patience', () => {
  const demand = createDemand(1701);
  assert.deepEqual(demand, createDemand(1701));
  assert.notDeepEqual(demand, createDemand(1702));
  assert.equal(new Set(demand.map(p => p.id)).size, demand.length);
  assert.equal(new Set(demand.map(p => p.purpose)).size, 5);
  assert.ok(new Set(demand.map(p => p.origin + ':' + p.destination)).size >= 10);
  assert.ok(new Set(demand.map(p => p.walkSpeed.toFixed(2))).size > 20);
  assert.ok(Math.max(...demand.map(p => p.patience)) - Math.min(...demand.map(p => p.patience)) > 65);
  assert.ok(demand.every((p, i) => p.origin !== p.destination && (!i || p.release >= demand[i - 1].release)));
  assert.ok(demand.every(p => PURPOSES.some(q => q.id === p.purpose)));
});

test('routing reaches the correct directed curb without an illegal U-turn', () => {
  const stop = FLEET_STOPS[0], current = graph.get(nodeKey(stop.from)), previous = graph.get(nodeKey(stop.to));
  const route = routeToStop(graph, current, previous, stop);
  assert.ok(route.distance > 100, 'must drive around instead of reversing from the wrong incoming street');
  let node = current, last = previous;
  for (const next of route.nodes) {
    assert.ok(node.links.some(n => nodeKey(n) === nodeKey(next)));
    assert.notEqual(nodeKey(next), nodeKey(last));
    last = node; node = next;
  }
  assert.equal(nodeKey(last), nodeKey(stop.from));
  assert.equal(nodeKey(node), nodeKey(stop.to));
  const sameEdge = fixtureCar(stop); sameEdge.x -= 8;
  assert.ok(Math.abs(pickupDistance(graph, sameEdge, stop) - 8) < .001);
  assert.equal(routeToStop(graph, current, previous, makeStop('bad', 'UNCONNECTED', { x: 999, y: 0 }, { x: 1000, y: 0 })), null);
});

test('a pickup behind a taxi requires another legal approach instead of freezing or reversing', () => {
  const stop = FLEET_STOPS[0], car = fixtureCar(stop);
  car.x += 8;
  assert.equal(stopSpeedLimit(car), Infinity);
  assert.ok(pickupDistance(graph, car, stop) > 100, 'must finish the current edge and drive around');
  car.x = stop.x - 8;
  assert.ok(stopSpeedLimit(car) > 0 && stopSpeedLimit(car) < 8);
});

test('boarding and dropoff require actual stopped arrival and walking/dwell time', () => {
  const origin = FLEET_STOPS.find(s => s.id === 'homes'), destination = FLEET_STOPS[0];
  const car = fixtureCar(origin), fleet = new TaxiFleet(graph, [car], { plans: [plan()] });
  car.speed = 3; step(fleet, 3);
  assert.equal(fleet.people[0].state, 'waiting');
  assert.equal(fleet.events.filter(e => e.type === 'board').length, 0);
  car.speed = 0; car.x += 10; step(fleet, 1);
  assert.equal(fleet.events.filter(e => e.type === 'board').length, 0);
  car.x = origin.x; step(fleet, .1);
  assert.equal(fleet.people[0].state, 'boarding');
  step(fleet, 2);
  assert.equal(fleet.people[0].state, 'boarding', 'cannot leave before both walking and dwell finish');
  step(fleet, 3);
  assert.equal(fleet.people[0].state, 'riding');
  assert.equal(car.serviceStop, destination);
  step(fleet, 60);
  assert.equal(fleet.snapshot().metrics.served, 0, 'time passing is not a dropoff');
  Object.assign(car, fixtureCar(destination));
  step(fleet, 6);
  assert.equal(fleet.snapshot().metrics.served, 1);
  assert.equal(fleet.taxis[0].person, null);
  assert.equal(fleet.taxis[0].state, 'idle');
  assert.equal(fleet.events.filter(e => e.type === 'complete').length, 1);
  assert.ok(fleet.events.filter(e => ['board', 'alight'].includes(e.type)).every(e => e.speed < .25));
});

test('assignments are exclusive; abandonment releases the taxi without creating a served trip', () => {
  const cars = [fixtureCar(FLEET_STOPS[0]), fixtureCar(FLEET_STOPS[1])];
  const fleet = new TaxiFleet(graph, cars, { plans: Array.from({ length: 7 }, (_, i) => plan('P' + i, { patience: 2 })) });
  step(fleet, .5);
  const assigned = fleet.people.filter(p => p.taxi);
  assert.equal(assigned.length, 2);
  assert.equal(new Set(assigned.map(p => p.taxi)).size, 2);
  assert.equal(new Set(fleet.taxis.map(t => t.person.id)).size, 2);
  step(fleet, 4);
  assert.equal(fleet.snapshot().metrics.cancelled, 7);
  assert.equal(fleet.snapshot().metrics.served, 0);
  assert.ok(fleet.taxis.every(t => t.state === 'idle' && !t.person));
  assert.ok(fleet.people.every(p => !p.taxi));
});

test('real traffic steers, queues and stops to serve individual trips across the city', () => {
  const traffic = new Traffic(new THREE.Scene(), { urban: true, cityCars: 14 });
  const cars = FLEET_STOPS.map((stop, i) => {
    const car = traffic.spawn(false, 0xffc34b, { ...stop.from, to: stop.to, advance: 10 + i, variant: 'taxi', serviceStop: stop });
    car.desired = 8 + i % 3; return car;
  });
  const fleet = new TaxiFleet(traffic.graph, cars);
  const initial = cars.map(c => ({ x: c.x, y: c.y }));
  let maxWalking = 0, maxRiding = 0;
  for (let i = 0; i < 180 * 30; i++) {
    fleet.update(1 / 30); traffic.update(1 / 30, fleet.time, farPlayer);
    maxWalking = Math.max(maxWalking, fleet.people.filter(p => p.state === 'walking').length);
    maxRiding = Math.max(maxRiding, fleet.people.filter(p => p.state === 'riding').length);
    const assigned = fleet.people.filter(p => p.taxi);
    assert.equal(new Set(assigned.map(p => p.taxi)).size, assigned.length);
    for (const taxi of fleet.taxis) assert.equal(Boolean(taxi.person), taxi.state !== 'idle');
  }
  const snapshot = fleet.snapshot();
  console.log('Actual fleet integration:', JSON.stringify({ metrics: snapshot.metrics, maxWalking, maxRiding,
    taxis: snapshot.taxis.map(t => ({ id: t.id, state: t.state, distance: t.distance, stop: t.destination, x: t.x, y: t.y })),
    events: snapshot.events.filter(e => ['board', 'complete'].includes(e.type)) }));
  assert.ok(snapshot.metrics.served >= 8, JSON.stringify(snapshot.metrics));
  assert.ok(maxWalking >= 3 && maxRiding >= 3, 'independent people walk and ride concurrently');
  assert.ok(snapshot.metrics.cancelled > 0, 'unserved requests must expire');
  assert.ok(cars.filter(c => c.distance > 100).length >= 6, 'actual driven routes');
  assert.ok(cars.some((c, i) => Math.hypot(c.x - initial[i].x, c.y - initial[i].y) > 20));
  for (const event of snapshot.events.filter(e => ['board', 'alight'].includes(e.type))) {
    const stop = FLEET_STOPS.find(s => s.id === event.stop);
    assert.ok(Math.hypot(event.x - stop.x, event.y - stop.y) < 1.1);
    assert.ok(event.speed < .25);
  }
  assert.ok(snapshot.people.filter(p => p.state === 'arrived').every(p => p.walked > 5));
  assert.ok(traffic.cars.every(c => [c.x, c.y, c.speed, c.yaw].every(Number.isFinite)));
});
