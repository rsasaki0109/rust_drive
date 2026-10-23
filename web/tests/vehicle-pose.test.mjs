import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import * as THREE from 'three';
import { makeCar, Traffic } from '../src/city.js';
import { applyVehiclePose } from '../src/vehicle-pose.js';

function direction(object, vector) {
  object.updateWorldMatrix(true, false);
  return vector.clone().transformDirection(object.matrixWorld);
}

test('rendered headlights face the actual WASM motion at cardinal and diagonal headings', async () => {
  const p = (await WebAssembly.instantiate(
    await readFile(new URL('../public/drive.wasm', import.meta.url)), {})).instance.exports;
  const mesh = makeCar();
  for (let i = 0; i < 8; i++) {
    const yaw = i * Math.PI / 4;
    assert.equal(p.drive_reset(0, 0, yaw), 1);
    for (let j = 0; j < 120; j++) p.drive_step(1, 0, 0, 1 / 120);
    const motion = new THREE.Vector3(p.drive_read(0), 0, -p.drive_read(1)).normalize();
    applyVehiclePose(mesh, { x: p.drive_read(0), y: p.drive_read(1), yaw: p.drive_read(2) });
    const forward = direction(mesh, new THREE.Vector3(1, 0, 0));
    assert.ok(forward.dot(motion) > 1 - 1e-10, 'body disagrees with motion at yaw ' + yaw);
  }
});

test('moving ordinary cars and police face their actual movement through a turn', () => {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0 });
  traffic.spawn(false, 0xffffff, { x: 0, y: 0, to: { x: 0, y: 50 }, advance: 10 });
  traffic.setChase(true, { x: 100, y: 60, yaw: 0 });
  let turning = false;
  for (let i = 0; i < 900; i++) {
    const before = traffic.cars.map(c => ({ x: c.x, y: c.y, yaw: c.yaw }));
    traffic.update(1 / 120, i / 120, { x: 100, y: 60, yaw: 0 });
    traffic.cars.forEach((c, j) => {
      const motion = new THREE.Vector3(c.x - before[j].x, 0, before[j].y - c.y);
      if (motion.length() < 1e-8) return;
      assert.ok(direction(c.mesh, new THREE.Vector3(1, 0, 0)).dot(motion.normalize()) > 1 - 1e-10);
      turning ||= Math.abs(c.yaw - before[j].yaw) > .001;
    });
  }
  assert.ok(turning, 'exercise steering as well as straight travel');
});

test('front wheels steer left and their rolling axle stays fixed', () => {
  const mesh = makeCar();
  const front = mesh.userData.wheels.find(w => w.front);
  const rear = mesh.userData.wheels.find(w => !w.front);
  for (const distance of [0, .16, .8, 3]) {
    applyVehiclePose(mesh, { x: 0, y: 0, yaw: 0, steering: .3, distance });
    const steeredForward = direction(front.root, new THREE.Vector3(1, 0, 0));
    assert.ok(steeredForward.z < -.29, 'positive steering points north/left');
    const axle = direction(front.wheel, new THREE.Vector3(0, 1, 0));
    const expectedAxle = direction(front.root, new THREE.Vector3(0, 0, 1));
    assert.ok(axle.dot(expectedAxle) > 1 - 1e-10, 'rolling must not wobble the axle');
    assert.equal(rear.root.rotation.y, 0);
  }
  // At the bottom of the unsteered tire, forward travel requires backward rim motion.
  applyVehiclePose(mesh, { x: 0, y: 0, yaw: 0, distance: 0 });
  const bottom = rear.wheel.localToWorld(new THREE.Vector3(0, 0, .32));
  applyVehiclePose(mesh, { x: 0, y: 0, yaw: 0, distance: .000001 });
  mesh.updateMatrixWorld(true);
  const rolled = rear.wheel.localToWorld(new THREE.Vector3(0, 0, .32));
  assert.ok(Math.abs(rolled.x - bottom.x + .000001) < 1e-10, 'rim rolls against travel');
});
