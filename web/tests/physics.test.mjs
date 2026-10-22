import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

async function physics() {
  return (await WebAssembly.instantiate(await readFile(new URL('../public/drive.wasm', import.meta.url)), {})).instance.exports;
}

test('the built WebAssembly ABI drives, brakes, reverses and resets', async () => {
  const p = await physics();
  assert.equal(p.drive_reset(0, 0, 0), 1);
  for (let i = 0; i < 600; i++) assert.equal(p.drive_step(1, 0, 0, 1 / 120), 1);
  assert.ok(p.drive_read(0) > 35);
  assert.ok(p.drive_read(3) > 15);
  assert.equal(p.drive_read(1), 0);
  for (let i = 0; i < 1200; i++) p.drive_step(-1, 0, 0, 1 / 120);
  assert.ok(p.drive_read(3) < -3);
  assert.equal(p.drive_reset(12, 34, Math.PI / 2), 1);
  assert.equal(p.drive_read(0), 12);
  assert.equal(p.drive_read(1), 34);
  assert.equal(p.drive_read(3), 0);
});

test('handbrake causes measured rear slip and changes physical trajectory', async () => {
  const grip = await physics(), drift = await physics();
  let slip = 0, lateral = 0;
  for (let i = 0; i < 400; i++) { grip.drive_step(1, 0, 0, 1 / 120); drift.drive_step(1, 0, 0, 1 / 120); }
  for (let i = 0; i < 150; i++) {
    grip.drive_step(.35, .6, 0, 1 / 120); drift.drive_step(.35, .6, 1, 1 / 120);
    slip = Math.max(slip, Math.abs(drift.drive_read(7))); lateral = Math.max(lateral, Math.abs(drift.drive_read(4)));
  }
  assert.ok(slip > Math.PI / 12);
  assert.ok(lateral > 1);
  assert.ok(Math.hypot(grip.drive_read(0) - drift.drive_read(0), grip.drive_read(1) - drift.drive_read(1)) > 1);
});

test('the ABI rejects invalid numbers without corrupting state', async () => {
  const p = await physics();
  assert.equal(p.drive_step(NaN, 0, 0, .01), 0);
  assert.equal(p.drive_step(1, 0, 0, 10), 0);
  assert.equal(p.drive_reset(Infinity, 0, 0), 0);
  assert.equal(p.drive_contact(0, 0, 1), 0);
  for (let i = 0; i < 11; i++) assert.equal(p.drive_read(i), 0);
});
