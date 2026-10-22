import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { Traffic } from '../src/city.js';

test('traffic moves independently through the grid with finite bounded states', () => {
  const traffic = new Traffic(new THREE.Scene());
  assert.equal(traffic.cars.length, 20);
  assert.ok(new Set(traffic.cars.map(c => Math.round(c.desired * 10))).size >= 10);
  for (let i = 0; i < 1800; i++) traffic.update(1 / 60, i / 60, { x: 0, y: -22 });
  assert.ok(traffic.cars.filter(c => c.distance > 20).length >= 15);
  for (const c of traffic.cars) {
    assert.ok([c.x, c.y, c.yaw, c.speed].every(Number.isFinite));
    assert.ok(Math.abs(c.x) < 190 && Math.abs(c.y) < 190 && c.speed >= 0 && c.speed <= 14);
  }
});

test('pursuit cars navigate toward the player and are removed when pursuit ends', () => {
  const traffic = new Traffic(new THREE.Scene()), player = { x: 0, y: -22 };
  traffic.setChase(true, player);
  assert.equal(traffic.cars.filter(c => c.police).length, 2);
  let nearest = Infinity;
  for (let i = 0; i < 3600; i++) {
    traffic.update(1 / 60, i / 60, player);
    nearest = Math.min(nearest, ...traffic.cars.filter(c => c.police).map(c => Math.hypot(c.x - player.x, c.y - player.y)));
  }
  assert.ok(nearest < 15, `Police never approached the player: ${nearest} m`);
  traffic.setChase(false, player);
  assert.equal(traffic.cars.length, 20);
  assert.equal(traffic.active, false);
});
