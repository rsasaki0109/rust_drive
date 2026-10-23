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
