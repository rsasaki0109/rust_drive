import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createCity, STREETS } from '../src/city.js';
import { DISTRICTS, BLOCKS, districtAt, forestTrail } from '../src/regions.js';

// Canvas text is decorative. Use real Three geometry/colliders without a WebGL context.
const savedDocument = globalThis.document;
globalThis.document = { createElement: () => ({ getContext: () => ({ fillRect() {}, fillText() {} }) }) };
let city;
try { city = createCity(new THREE.Scene()); }
finally { if (savedDocument === undefined) delete globalThis.document; else globalThis.document = savedDocument; }

function clearance(x, y) {
  return Math.min(...city.colliders.map(b => Math.hypot(
    Math.max(Math.abs(x - b.x) - b.w, 0), Math.max(Math.abs(y - b.y) - b.h, 0)
  )));
}
function clear(x, y, reason) {
  assert.ok(clearance(x, y) >= 1.8, reason + ': blocked at ' + x + ', ' + y);
}

test('five districts have distinct scenery and safe, correctly labelled spawns', () => {
  assert.equal(DISTRICTS.length, 5);
  assert.equal(new Set(DISTRICTS.map(d => d.color)).size, 5);
  assert.equal(new Set(BLOCKS.map(b => b.district.id)).size, 5);
  assert.equal(city.buildingCount, 64);
  assert.equal(city.signals, 49);
  for (const district of DISTRICTS) {
    assert.equal(districtAt(district.spawn.x, district.spawn.y).id, district.id);
    clear(district.spawn.x, district.spawn.y, district.id + ' spawn');
  }
});

test('all street lanes remain connected across district boundaries', () => {
  // Sweep the two lane centres through every block and junction, independently
  // of the region classifier and scenery placement implementation.
  for (const street of STREETS) for (const side of [-1, 1]) {
    for (let p = -175; p <= 175; p += .5) {
      clear(street + side * 3.8, p, 'north/south road');
      clear(p, street + side * 3.8, 'east/west road');
    }
  }
});

test('forest shortcuts have vehicle-width clearance and join both adjacent roads', () => {
  const forest = BLOCKS.filter(b => b.district.id === 'forest');
  assert.equal(forest.length, 6);
  for (const block of forest) {
    const path = forestTrail(block.x, block.y);
    assert.deepEqual(path[0], { x: block.x - 16, y: block.y });
    assert.ok(Math.abs(path.at(-1).x - block.x - 16) < 1e-9);
    assert.ok(Math.abs(path.at(-1).y - block.y) < 1e-9);
    assert.ok(Math.max(...path.map(p => p.y)) - Math.min(...path.map(p => p.y)) >= 9.9);
    for (let i = 0; i < path.length; i++) {
      const a = path[Math.max(0, i - 1)], b = path[Math.min(path.length - 1, i + 1)];
      const length = Math.hypot(b.x - a.x, b.y - a.y), nx = -(b.y - a.y) / length, ny = (b.x - a.x) / length;
      for (const offset of [-1.5, 0, 1.5]) clear(path[i].x + nx * offset, path[i].y + ny * offset, 'forest corridor');
    }
    for (let offset = 16; offset <= 25; offset += .5) for (const side of [-1, 1])
      clear(block.x + side * offset, block.y, 'forest road connection');
  }
});

test('each central plaza leaves a clear 19 m diameter circle for drifting', () => {
  const plazas = BLOCKS.filter(b => b.district.id === 'plaza');
  assert.equal(plazas.length, 4);
  for (const block of plazas) {
    clear(block.x, block.y, 'plaza centre');
    for (let i = 0; i < 180; i++) {
      const angle = i * Math.PI / 90;
      clear(block.x + Math.cos(angle) * 9.5, block.y + Math.sin(angle) * 9.5, 'plaza drift circle');
    }
  }
});
