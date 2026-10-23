import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import * as THREE from 'three';
import { createStructures, ROAD_STARTS } from '../src/structures.js';
import { createRidge } from '../src/ridge.js';
import { createCity } from '../src/city.js';

async function physics() {
  return (await WebAssembly.instantiate(await readFile(new URL('../public/drive.wasm', import.meta.url)), {})).instance.exports;
}
function state(p) {
  return { x: p.drive_read(0), y: p.drive_read(1), height: p.drive_read(11), grounded: !!p.drive_read(13), ceiling: p.drive_read(21) };
}
function runTo(p, predicate, inspect = () => {}) {
  for (let i = 0; i < 6000; i++) {
    p.drive_step(1, 0, 0, 1 / 120);
    const s = state(p); inspect(s);
    if (predicate(s)) return s;
  }
  assert.fail('The keyboard-equivalent run did not reach its exit');
}

test('upper deck and underground ramps match independent profiles and clearance', async () => {
  const p = await physics();
  for (let x = 190; x < 260; x += 1.7) {
    const t = (x - 190) / 70, f = 3 * t ** 2 - 2 * t ** 3;
    assert.ok(Math.abs(p.drive_deck(x, 20, 0) - 12 * f) < 1e-10);
    assert.ok(Math.abs(p.drive_surface(x, -105, 0) + 8 * f) < 1e-10);
    const d = (p.drive_deck(x + .0001, 20, 0) - p.drive_deck(Math.max(190, x - .0001), 20, 0)) / (x === 190 ? .0001 : .0002);
    assert.ok(Math.abs(p.drive_deck(x, 20, 1) - d) < 1e-5);
  }
  assert.equal(p.drive_ceiling(350, 20, 0), 10.9);
  assert.equal(p.drive_ceiling(350, -105, -8), -1.2);
  assert.ok(p.drive_ceiling(350, -105, 0) > 1000);
  assert.ok(p.drive_deck(NaN, 20, 0) < -1000);
  assert.ok(Number.isFinite(p.drive_ceiling(350, 20, NaN)));
});

test('the same crossing is driven above and below without a layer jump', async () => {
  const p = await physics();
  const start = ROAD_STARTS.skyway; p.drive_reset(start.x, start.y, start.yaw);
  const above = runTo(p, s => s.x >= 350);
  assert.ok(Math.abs(above.height - 12) < .1);
  assert.equal(p.drive_read(20), 12);
  const low = ROAD_STARTS.underpass; p.drive_reset(low.x, low.y, low.yaw);
  let crossing = false;
  runTo(p, s => s.y > 35, s => {
    assert.ok(Math.abs(s.height) < 1e-9);
    if (s.y > 12 && s.y < 28) { assert.equal(s.ceiling, 10.9); crossing = true; }
  });
  assert.ok(crossing);
});

test('the lower tunnel has two working ramps and a separate drivable roof', async () => {
  const p = await physics(), start = ROAD_STARTS.tunnel;
  p.drive_reset(start.x, start.y, start.yaw);
  let underground = false;
  const exit = runTo(p, s => s.x > 505 && s.grounded, s => {
    if (s.x > 300 && s.x < 400) {
      assert.ok(Math.abs(s.height + 8) < 1e-7);
      assert.equal(s.ceiling, -1.2); underground = true;
    }
  });
  assert.ok(underground && exit.height === 0);
  p.drive_reset(350, -145, Math.PI / 2);
  let roof = false;
  runTo(p, s => s.y > -80, s => {
    assert.ok(Math.abs(s.height) < 1e-9);
    roof ||= s.y > -114 && s.y < -96;
  });
  assert.ok(roof);
});

test('a low bridge slab blocks a car from the side instead of snapping it to the deck', async () => {
  const p = await physics(); p.drive_reset(480, 0, Math.PI / 2);
  for (let i = 0; i < 600; i++) p.drive_step(1, 0, 0, 1 / 120);
  assert.ok(state(p).y <= 11 && state(p).height === 0);
  assert.ok(p.drive_read(10) > 0);
});

async function geometry() {
  const p = await physics(), scene = new THREE.Scene(), saved = globalThis.document;
  globalThis.document = { createElement: () => ({ getContext: () => ({ fillRect() {}, fillText() {} }) }) };
  let result;
  try { createCity(scene); createRidge(scene, p); result = createStructures(scene, p); }
  finally { if (saved === undefined) delete globalThis.document; else globalThis.document = saved; }
  scene.updateMatrixWorld(true);
  return { p, scene, result };
}

test('actual mesh raycasts find two road levels and an open excavated entrance', async () => {
  const { p, scene } = await geometry();
  function heights(x, y) {
    const ray = new THREE.Raycaster(new THREE.Vector3(x, 100, -y), new THREE.Vector3(0, -1, 0), 0, 200);
    return ray.intersectObjects(scene.children, true).map(hit => hit.point.y);
  }
  const bridge = heights(350, 20);
  assert.ok(bridge.some(h => Math.abs(h - 12.035) < .001));
  assert.ok(bridge.some(h => Math.abs(h - .045) < .001));
  const tunnel = heights(350, -105);
  assert.ok(tunnel.some(h => Math.abs(h - .045) < .001));
  assert.ok(tunnel.some(h => Math.abs(h + 7.965) < .001));
  for (const x of [212, 232, 252, 258.5, 431.5, 448, 468, 488]) {
    const top = Math.max(...heights(x, -105));
    assert.ok(top < -.5 && Math.abs(top - p.drive_surface(x, -105, 0) - .035) < .06);
  }
});

test('upper lanes, lower crossing, tunnel and roof all have body-width collider clearance', async () => {
  const { p, result } = await geometry();
  function clear(x, y, base) {
    for (const b of result.colliders) {
      if (base >= b.top - 1e-8 || base + 1.55 <= b.bottom) continue;
      const d = Math.hypot(Math.max(Math.abs(x - b.x) - b.w, 0), Math.max(Math.abs(y - b.y) - b.h, 0));
      assert.ok(d >= 1.8, 'Blocked road at ' + [x, y, base].join(','));
    }
  }
  for (const offset of [-3, 0, 3]) {
    for (let x = 190; x <= 500; x += 2) {
      clear(x, 20 + offset, p.drive_deck(x, 20, 0));
      clear(x, -105 + offset, p.drive_surface(x, -105, 0));
    }
    for (let y = -145; y <= 70; y += 2) clear(350 + offset, y, 0);
  }
});
