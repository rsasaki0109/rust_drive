import * as THREE from 'three';

export const FRACTURE_ENERGY_J = 90000;

// Spending impact energy is done by Rust. A failed fracture leaves the collider solid.
export function fractureContact(drive, barrier, nx, ny) {
  if (!barrier.breakaway || barrier.broken) return false;
  if (!drive.drive_break_barrier(nx, ny, barrier.energy)) return false;
  barrier.fracture();
  return true;
}

export function createCrashGuardrails(scene, drive) {
  const box = new THREE.BoxGeometry(1, 1, 1);
  const steel = new THREE.MeshStandardMaterial({ color: 0xd8ded7, metalness: .65, roughness: .4 });
  const post = new THREE.MeshStandardMaterial({ color: 0x59676b, metalness: .6, roughness: .6 });
  const reflector = new THREE.MeshBasicMaterial({ color: 0xf8aa55 });
  const colliders = [], fragments = [];
  function part(parent, material, x, y, z, w, h, d) {
    const m = new THREE.Mesh(box, material); m.position.set(x, y, z); m.scale.set(w, h, d); m.castShadow = true; parent.add(m); return m;
  }
  for (const y of [144, 150, 156]) {
    const x = 379.2, base = drive.drive_surface(x, y, 0), group = new THREE.Group();
    group.position.set(x, base, -y); scene.add(group);
    for (const side of [-1, 1]) part(group, post, 0, .63, side * 2.85, .20, 1.26, .20);
    part(group, steel, 0, .62, 0, .22, .28, 5.9);
    part(group, steel, 0, 1.04, 0, .22, .28, 5.9);
    for (const side of [-1, 1]) part(group, reflector, -.14, 1.04, side * 1.8, .04, .16, .30);
    const pieces = Array.from({ length: 6 }, (_, i) => {
      const m = part(scene, i % 3 ? steel : post, x, base + .9, -y + (i - 2.5), .24, .24, .94);
      m.visible = false;
      const p = { mesh: m, vx: 0, vy: 0, vz: 0, age: 0 }; fragments.push(p); return p;
    });
    const collider = {
      x, y, w: .16, h: 3, bottom: base - .25, top: base + 1.3,
      breakaway: true, broken: false, energy: FRACTURE_ENERGY_J,
      fracture() {
        collider.broken = true; group.visible = false;
        const yaw = drive.drive_read(2), forward = drive.drive_read(3), lateral = drive.drive_read(4);
        const vx = Math.cos(yaw) * forward - Math.sin(yaw) * lateral;
        const vy = Math.sin(yaw) * forward + Math.cos(yaw) * lateral;
        pieces.forEach((p, i) => {
          p.mesh.visible = true; p.mesh.position.set(x + .15, base + .8 + i % 2 * .3, -y + i - 2.5);
          p.mesh.rotation.set(0, 0, 0); p.vx = vx * .35 + 1 + i * .28; p.vy = vy * .35 + (i - 2.5) * .8; p.vz = 2 + i * .45; p.age = 0;
        });
      },
      restore() { collider.broken = false; group.visible = true; pieces.forEach(p => { p.mesh.visible = false; }); },
    };
    colliders.push(collider);
  }
  return {
    colliders,
    count: () => colliders.filter(b => b.broken).length,
    reset: () => colliders.forEach(b => b.restore()),
    update(dt) {
      for (const p of fragments) {
        if (!p.mesh.visible) continue;
        p.age += dt; p.mesh.position.x += p.vx * dt; p.mesh.position.z -= p.vy * dt;
        p.mesh.position.y += p.vz * dt - .5 * 9.81 * dt * dt; p.vz -= 9.81 * dt;
        p.mesh.rotation.x += dt * 2.4; p.mesh.rotation.z += dt * 1.7;
        const floor = drive.drive_surface(p.mesh.position.x, -p.mesh.position.z, 0) + .15;
        if (p.mesh.position.y < floor) { p.mesh.position.y = floor; p.vz = Math.abs(p.vz) > .5 ? -p.vz * .15 : 0; p.vx *= Math.exp(-dt * 3); p.vy *= Math.exp(-dt * 3); }
      }
    },
  };
}
