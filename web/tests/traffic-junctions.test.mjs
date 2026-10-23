import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { Traffic } from '../src/city.js';

const DT = 1 / 120, FAR_PLAYER = { x: -170, y: -170, height: 0 };

function straightRoutes(traffic) {
  // Fix routes for spacing measurements; use the production movement/admission code.
  traffic.choose = car => {
    const dx = car.node.x - car.previous.x, dy = car.node.y - car.previous.y;
    car.next = car.node.links.find(p => (p.x - car.node.x) * dx + (p.y - car.node.y) * dy > 0)
      ?? car.node.links.find(p => p !== car.previous) ?? car.previous;
  };
}
function drive(traffic, cars, seconds, time = 0, player = FAR_PLAYER) {
  let minimum = Infinity;
  for (let i = 0; i < Math.round(seconds / DT); i++) {
    traffic.update(DT, time + i * DT, player);
    for (const c of cars) assert.ok([c.x, c.y, c.speed].every(Number.isFinite));
    for (let a = 0; a < cars.length; a++) for (let b = a + 1; b < cars.length; b++)
      minimum = Math.min(minimum, Math.hypot(cars[a].x - cars[b].x, cars[a].y - cars[b].y));
  }
  return { minimum_separation_m: cars.length > 1 ? minimum : null,
    distances_m: cars.map(c => c.distance) };
}
function crossing({ police = true, ax = -20, by = -36.6, speed = 10, drive: surface } = {}) {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0, drive: surface });
  const east = traffic.spawn(false, 0xffffff,
    { x: -50, y: 0, to: { x: 0, y: 0 }, advance: ax + 50 });
  const north = traffic.spawn(false, 0xffffff,
    { x: 0, y: -50, to: { x: 0, y: 0 }, advance: by + 50 });
  east.speed = east.desired = speed; north.speed = north.desired = 10;
  north.police = police; straightRoutes(traffic);
  return { traffic, cars: [east, north] };
}

test('crossing pursuit and ordinary traffic retain clearance and both make progress', t => {
  const { traffic, cars } = crossing();
  const result = drive(traffic, cars, 10);
  assert.ok(result.minimum_separation_m > 3.3, JSON.stringify(result));
  assert.ok(result.distances_m.every(d => d > 40), 'yielded traffic must resume');
  t.diagnostic(JSON.stringify(result));
});

test('a signal change lets an admitted car clear before cross traffic enters', t => {
  const { traffic, cars } = crossing({ police: false, ax: -8, by: -24, speed: 6 });
  // X has green initially; Y gets green 0.2 seconds into this episode.
  const result = drive(traffic, cars, 10, 7.8);
  assert.ok(result.minimum_separation_m > 3.3, JSON.stringify(result));
  assert.ok(result.distances_m.every(d => d > 25));
  t.diagnostic(JSON.stringify(result));
});

test('four unsignalled approaches clear fairly, independent of update-array order', t => {
  function episode(reverse) {
    const traffic = new Traffic(new THREE.Scene(), { cityCars: 0 });
    traffic.graph.get('0:0').signal = false;
    const starts = [
      { x: -50, y: 0 }, { x: 0, y: -50 }, { x: 50, y: 0 }, { x: 0, y: 50 },
    ];
    const cars = starts.map(p => traffic.spawn(false, 0xffffff, { ...p, to: { x: 0, y: 0 }, advance: 22 }));
    cars.forEach(c => { c.speed = c.desired = 10; });
    straightRoutes(traffic);
    if (reverse) traffic.cars.reverse();
    const result = drive(traffic, cars, 22);
    assert.ok(result.minimum_separation_m > 3.3, JSON.stringify(result));
    assert.ok(result.distances_m.every(d => d > 40), 'every approach must get through');
    return { result, positions: cars.map(c => [c.id, c.x, c.y, c.speed]) };
  }
  const forward = episode(false), reverse = episode(true);
  assert.deepEqual(forward.positions, reverse.positions);
  t.diagnostic(JSON.stringify(forward.result));
});

test('ending a pursuit releases its removed junction owner', () => {
  const { traffic, cars: [east, north] } = crossing({ police: false, ax: -8, by: -35, speed: 8 });
  east.police = true;
  traffic.update(DT, 8, FAR_PLAYER);
  assert.ok([...traffic.junctions.owners.values()].some(owner => owner.id === east.id));
  traffic.setChase(false, FAR_PLAYER);
  drive(traffic, [north], 10, 8 + DT);
  assert.ok(north.distance > 40);
  assert.ok([...traffic.junctions.owners.values()].every(owner => traffic.cars.some(c => c.id === owner.id)));
});

test('a player in the junction blocks entry but a player on the upper deck does not', () => {
  function episode(height) {
    const traffic = new Traffic(new THREE.Scene(), { cityCars: 0 });
    const car = traffic.spawn(false, 0xffffff, { x: -50, y: 0, to: { x: 0, y: 0 }, advance: 25 });
    car.speed = car.desired = 8; straightRoutes(traffic);
    drive(traffic, [car], 5, 0, { x: 0, y: 0, height });
    return car;
  }
  const blocked = episode(0), upper = episode(12);
  assert.ok(blocked.x < -10 && blocked.speed < .5);
  assert.ok(upper.x > 10, 'separate road layers must remain independent');
});

test('crossing cars on a slope coordinate using their own road-relative height', () => {
  const surface = { drive_surface: (x, _y, index) => index === 0 ? x * .5 : index === 1 ? .5 : 0 };
  const { traffic, cars } = crossing({ drive: surface });
  assert.ok(Math.abs(cars[0].height - cars[1].height) > 10);
  const result = drive(traffic, cars, 10);
  assert.ok(result.minimum_separation_m > 3.3, JSON.stringify(result));
  assert.ok(result.distances_m.every(d => d > 40));
});

test('short curve waypoints preserve braking distance to the next red signal', () => {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0 });
  const car = traffic.spawn(false, 0xffffff, { x: 0, y: -50, to: { x: 0, y: 0 }, advance: 22 });
  const origin = traffic.graph.get('0:-50'), junction = traffic.graph.get('0:0');
  const points = [-30, -20, -10].map(y => ({ x: 0, y, signal: false, links: [] }));
  const route = [origin, ...points, junction];
  for (let i = 1; i < route.length - 1; i++) route[i].links = [route[i - 1], route[i + 1]];
  origin.links = origin.links.map(n => n === junction ? points[0] : n);
  junction.links = junction.links.map(n => n === origin ? points[2] : n);
  car.previous = origin; car.node = points[0]; car.next = points[1];
  car.speed = car.desired = 10; straightRoutes(traffic);
  drive(traffic, [car], 4);
  assert.ok(car.y < -10 && car.speed < .5, JSON.stringify({ y: car.y, speed: car.speed }));
});

test('right and left turns join their outgoing lane without trapping opposing traffic', () => {
  for (const exit of [-50, 50]) {
    const traffic = new Traffic(new THREE.Scene(), { cityCars: 0 });
    traffic.graph.get('0:0').signal = false;
    const turning = traffic.spawn(false, 0xffffff, { x: 0, y: -50, to: { x: 0, y: 0 }, advance: 20 });
    const crossing = traffic.spawn(false, 0xffffff, { x: -50, y: 0, to: { x: 0, y: 0 }, advance: 15 });
    for (const c of [turning, crossing]) c.speed = c.desired = 8;
    straightRoutes(traffic);
    const straight = traffic.choose;
    traffic.choose = c => {
      if (c.id === turning.id && c.node.x === 0 && c.node.y === 0 && c.previous.y === -50)
        c.next = traffic.graph.get(exit + ':0');
      else straight(c);
    };
    let joined = false, minimum = Infinity;
    for (let i = 0; i < 1440; i++) {
      traffic.update(DT, i * DT, FAR_PLAYER);
      minimum = Math.min(minimum, Math.hypot(turning.x - crossing.x, turning.y - crossing.y));
      if (Math.sign(turning.x) === Math.sign(exit) && Math.abs(turning.x) > 10) {
        joined = true;
        assert.ok(Math.abs(turning.y - (exit < 0 ? 3.8 : -3.8)) < 1,
          JSON.stringify({ exit, x: turning.x, y: turning.y }));
      }
    }
    assert.ok(joined && turning.distance > 40 && crossing.distance > 40);
    assert.ok(minimum > 3.3, 'turns must clear vehicles waiting on the opposing lane');
  }
});

test('closely spaced junctions cannot hold mutually blocking reservations', () => {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0, urban: true });
  // Keep the authored graph, but isolate this pair from randomly spawned traffic.
  for (const c of traffic.cars) traffic.scene.remove(c.mesh);
  traffic.cars = [];
  const south = traffic.spawn(false, 0xffffff, { x: 150, y: 100, to: { x: 150, y: 70 }, advance: 22 });
  const north = traffic.spawn(false, 0xffffff, { x: 150, y: 0, to: { x: 150, y: 50 }, advance: 30 });
  for (const c of [south, north]) { c.speed = c.desired = 8; c.police = true; }
  traffic.choose = c => {
    const dx = c.node.x - c.previous.x, dy = c.node.y - c.previous.y;
    c.next = c.node.links.filter(p => (p.x - c.node.x) * dx + (p.y - c.node.y) * dy > 0)
      .sort((a, b) => Math.hypot(a.x - c.node.x, a.y - c.node.y) - Math.hypot(b.x - c.node.x, b.y - c.node.y))[0]
      ?? c.previous;
  };
  const result = drive(traffic, [south, north], 12);
  assert.ok(result.minimum_separation_m > 3.3, JSON.stringify(result));
  assert.ok(result.distances_m.every(d => d > 60), 'both cars must clear both junctions');
});

test('a long graph edge stops before the first occupied junction in a group', () => {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0, urban: true });
  for (const c of traffic.cars) traffic.scene.remove(c.mesh);
  traffic.cars = [];
  const south = traffic.spawn(false, 0xffffff, { x: 150, y: 100, to: { x: 150, y: 50 }, advance: 8 });
  south.speed = south.desired = 10; south.police = true; straightRoutes(traffic);
  drive(traffic, [south], 5, 0, { x: 150, y: 70, height: 0 });
  assert.ok(south.y > 80 && south.speed < .5, JSON.stringify({ y: south.y, speed: south.speed }));
});

test('pursuers take unused reservations without interrupting committed crossings', () => {
  for (const [ax, yields] of [[-30, true], [-13, false], [-8, false]]) {
    const { traffic, cars: [east, north] } = crossing({ ax, by: -35 });
    traffic.cars = [east];
    traffic.update(DT, 0, FAR_PLAYER);
    assert.equal(traffic.junctions.owners.get('0:0').id, east.id);
    traffic.cars.push(north);
    traffic.update(DT, DT, FAR_PLAYER);
    assert.equal(traffic.junctions.owners.get('0:0').id, yields ? north.id : east.id);
    const result = drive(traffic, [east, north], 10, 2 * DT);
    assert.ok(result.minimum_separation_m > 3.3, JSON.stringify(result));
    assert.ok(result.distances_m.every(d => d > 40));
  }
});

test('pursuers refresh a moving target before following a stale side-street turn', () => {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0 });
  const target = { x: 80, y: 150, height: 0 };
  const car = traffic.spawn(true, 0xffffff, { x: 50, y: 150 }, target);
  traffic.update(DT, 0, target);
  drive(traffic, [car], 8, DT, { x: 140, y: 150, height: 0 });
  assert.ok(car.x > 120 && Math.abs(car.y - 146.2) < 1,
    'the pursuit must stay on the avenue after the target advances: ' + JSON.stringify({ x: car.x, y: car.y }));
});

test('a late pursuit target change cannot demand a turn beyond its braking distance', () => {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0 });
  const target = { x: 140, y: 150, height: 0 };
  const car = traffic.spawn(true, 0xffffff, { x: 50, y: 150 }, target);
  car.speed = 20;
  drive(traffic, [car], 1, 0, target);
  drive(traffic, [car], 1, 1, { x: 100, y: 100, height: 0 });
  assert.ok(car.x > 100 && Math.abs(car.y - 146.2) < 1,
    'finish the safe approach before rerouting at the next junction');
});
