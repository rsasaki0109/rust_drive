import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createFleetPlaces } from '../src/fleet-places.js';
import { cityConditions, FACILITIES } from '../src/city-life.js';
import { TaxiFleet } from '../src/fleet.js';
import { roadGraph } from '../src/urban-roads.js';
import { STREETS } from '../src/city.js';

function world() {
  const scene = new THREE.Scene(), saved = globalThis.document;
  globalThis.document = { createElement: () => ({ getContext: () => ({ fillRect() {}, fillText() {} }) }) };
  try { return { scene, places: createFleetPlaces(scene) }; }
  finally { if (saved === undefined) delete globalThis.document; else globalThis.document = saved; }
}

test('the physical door panels clear the actual walking agent at both building entrances', () => {
  for (const origin of ['office', 'nightlife']) {
    const { scene, places } = world(), graph = roadGraph(STREETS, true);
    const f = new TaxiFleet(graph, [], { plans: [{ id: 'door-' + origin, purpose: 'commute', origin, destination: 'homes',
      release: 0, walkSpeed: 1, patience: 60, dwell: 4, offset: 2 }] });
    let crossing = 0;
    for (let i = 0; i < 150; i++) {
      f.update(.05); places.update(f.snapshot()); scene.updateMatrixWorld(true);
      const p = f.people[0], doorway = FACILITIES[origin];
      if (Math.abs(p.y - doorway.door.y) < .38) {
        crossing++;
        const person = new THREE.Box3(new THREE.Vector3(p.x - .28, .1, -p.y - .28), new THREE.Vector3(p.x + .28, 1.95, -p.y + .28));
        for (const panel of places.doors[origin]) assert.ok(!new THREE.Box3().setFromObject(panel.mesh).intersectsBox(person));
      }
    }
    assert.ok(crossing >= 4, 'must actually cross the doorway, not just test an open mesh');
    assert.ok(f.events.some(e => e.type === 'building-exit' && e.building === origin));
  }
});

test('the rendered train follows its physical state and retracts its door panels', () => {
  const { scene, places } = world();
  const base = { doors: { office: { open: 0 }, nightlife: { open: 1 } }, clock: cityConditions('night', 40),
    train: { x: 24.98, doorOpen: 1, lastService: true } };
  places.update(base); scene.updateMatrixWorld(true);
  assert.equal(places.trainRoot.position.x, 24.98);
  assert.equal(places.trainRoot.visible, true);
  const passenger = new THREE.Box3(new THREE.Vector3(24.98 + 8 - .28, 1.05, -183.5 - .28),
    new THREE.Vector3(24.98 + 8 + .28, 2.95, -183.5 + .28));
  // Train meshes covering the boarding doorway must leave a clear capsule.
  for (const child of places.trainRoot.children.filter(c => c.isMesh))
    assert.ok(!new THREE.Box3().setFromObject(child).intersectsBox(passenger), 'boarding doorway must be a real opening');
  places.update({ ...base, train: null });
  assert.equal(places.trainRoot.visible, false);
});
