import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

async function physics() {
  return (await WebAssembly.instantiate(await readFile(new URL('../public/drive.wasm', import.meta.url)), {})).instance.exports;
}
function state(p) {
  return { x: p.drive_read(0), y: p.drive_read(1), speed: p.drive_read(3),
    height: p.drive_read(11), vertical: p.drive_read(12), grounded: !!p.drive_read(13),
    pitch: p.drive_read(14), airtime: p.drive_read(16), landings: p.drive_read(17),
    impact: p.drive_read(18), best: p.drive_read(19) };
}
function launch(p) {
  p.drive_reset(190, 150, 0);
  let maximumPitch = 0;
  for (let i = 0; i < 6000; i++) {
    p.drive_step(1, 0, 0, 1 / 120); const s = state(p);
    maximumPitch = Math.max(maximumPitch, s.pitch);
    if (!s.grounded && s.x > 380) { assert.ok(maximumPitch > .4); return s; }
  }
  assert.fail('Full-throttle driving never launched from the cliff');
}

test('the authored climb is continuous, reaches 49 m, and has a real unsupported cliff', async () => {
  const p = await physics();
  for (let x = 200.1; x < 340; x += 2.7) {
    const t = (x - 200) / 140;
    assert.ok(Math.abs(p.drive_surface(x, 150, 0) - 48 * (3 * t ** 2 - 2 * t ** 3)) < 1e-10);
    const finiteDifference = (p.drive_surface(x + .0001, 150, 0) - p.drive_surface(x - .0001, 150, 0)) / .0002;
    assert.ok(Math.abs(finiteDifference - p.drive_surface(x, 150, 1)) < 1e-7);
  }
  assert.ok(p.drive_surface(379.999, 150, 0) > 48.99);
  assert.equal(p.drive_surface(380, 150, 0), 0);
  assert.equal(p.drive_surface(370, 190.001, 0), 0);
  assert.equal(p.drive_surface(NaN, 150, 0), 0);
  assert.equal(p.drive_surface(0, 0, 0), 0);
});

test('the built WASM follows free-fall equations and air controls add no tire force', async () => {
  const a = await physics(), b = await physics(), initial = launch(a), twin = launch(b);
  assert.deepEqual(initial, twin);
  for (let i = 0; i < 40; i++) {
    a.drive_step(0, 0, 0, .01); b.drive_step(1, 1, 1, .01);
  }
  const actual = state(a), duration = .4;
  assert.ok(!actual.grounded);
  assert.ok(Math.abs(actual.height - (initial.height + initial.vertical * duration - .5 * 9.81 * duration ** 2)) < 1e-8);
  assert.ok(Math.abs(actual.vertical - (initial.vertical - 9.81 * duration)) < 1e-8);
  assert.ok(Math.abs(actual.x - (initial.x + initial.speed * duration)) < 1e-8);
  for (let i = 0; i <= 20; i++) assert.equal(a.drive_read(i), b.drive_read(i));
});

test('a complete dive lands, absorbs the impact and can be reset without stale flight state', async () => {
  const p = await physics(); launch(p);
  for (let i = 0; i < 1200 && !state(p).grounded; i++) p.drive_step(1, 0, 0, 1 / 120);
  const landed = state(p);
  assert.ok(landed.grounded && landed.landings === 1 && landed.height === 0);
  assert.ok(landed.x > 415 && landed.x < 550 && landed.best > 3 && landed.impact > 20);
  const before = Array.from({ length: 21 }, (_, i) => p.drive_read(i));
  assert.equal(p.drive_step(NaN, 0, 0, .01), 0);
  assert.deepEqual(Array.from({ length: 21 }, (_, i) => p.drive_read(i)), before);
  p.drive_reset(190, 150, 0);
  const reset = state(p); assert.ok(reset.grounded && reset.height === 0 && reset.landings === 0 && reset.best === 0);
});

test('slope gravity rolls the car downhill and a vertical cliff cannot be driven up from below', async () => {
  const p = await physics(); p.drive_reset(260, 150, 0);
  for (let i = 0; i < 240; i++) p.drive_step(0, 0, 0, 1 / 120);
  assert.ok(state(p).x < 257 && state(p).speed < -2 && state(p).grounded);
  p.drive_reset(382, 150, Math.PI);
  for (let i = 0; i < 600; i++) p.drive_step(1, 0, 0, 1 / 120);
  assert.ok(state(p).x >= 380 && state(p).height === 0 && p.drive_read(10) > 0);
});
