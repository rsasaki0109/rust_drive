import * as THREE from 'three';
import { districtAt } from './regions.js';
import { createDistrictScenery } from './scenery.js';

export const STREETS = [-150, -100, -50, 0, 50, 100, 150];
export const LIMIT = 184;
const box = new THREE.BoxGeometry(1, 1, 1);
const palette = [0x66737b, 0xb5a296, 0x53656b, 0xa6a89f, 0x7b7e81, 0x998c7e];
const matte = (color, roughness = .8) => new THREE.MeshStandardMaterial({ color, roughness });
let seed = 73621;
export function random() { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; }
export const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
export const nearestStreet = value => Math.max(-150, Math.min(150, Math.round(value / 50) * 50));
export function phase(axis, x, y, time) {
  const clock = (time + (Math.round(x / 50) + Math.round(y / 50)) * 3 + 10000) % 18;
  const north = clock < 9;
  return (axis === 'y') === north ? clock % 9 > 7.5 ? 'yellow' : 'green' : 'red';
}

function mesh(parent, geometry, material, x, y, z, sx = 1, sy = 1, sz = 1) {
  const m = new THREE.Mesh(geometry, material);
  m.position.set(x, y, z); m.scale.set(sx, sy, sz); parent.add(m); return m;
}
function instances(parent, geometry, material, rows) {
  const m = new THREE.InstancedMesh(geometry, material, rows.length);
  const t = new THREE.Object3D();
  rows.forEach((r, i) => { t.position.set(r.x, r.y, r.z); t.scale.set(r.w, r.h, r.d); t.rotation.set(0, r.yaw || 0, 0); t.updateMatrix(); m.setMatrixAt(i, t.matrix); if (r.color !== undefined) m.setColorAt(i, new THREE.Color(r.color)); });
  m.instanceMatrix.needsUpdate = true; parent.add(m); return m;
}

export function createCity(scene) {
  const colliders = [], windows = [], bodies = [], roofs = [], curbs = [], paint = [], lamps = [];
  mesh(scene, box, matte(0x53635b), 0, -.42, 0, 700, .5, 700).receiveShadow = true;
  const districts = createDistrictScenery(scene); colliders.push(...districts.colliders);
  const asphalt = matte(0x333e47);
  for (const street of STREETS) {
    mesh(scene, box, asphalt, street, -.11, 0, 18, .2, 380).receiveShadow = true;
    mesh(scene, box, asphalt, 0, -.10, -street, 380, .2, 18).receiveShadow = true;
    for (let p = -180; p < 181; p += 6) {
      if (STREETS.some(s => Math.abs(p - s) < 11)) continue;
      paint.push({ x: street, y: .015, z: -p, w: .14, h: .02, d: 3.2 });
      paint.push({ x: p, y: .017, z: -street, w: 3.2, h: .02, d: .14 });
      for (const side of [-1, 1]) {
        paint.push({ x: street + side * 4.5, y: .015, z: -p, w: .1, h: .02, d: 2.5 });
        paint.push({ x: p, y: .017, z: -street + side * 4.5, w: 2.5, h: .02, d: .1 });
      }
    }
  }
  for (let ix = 0; ix < 6; ix++) for (let iy = 0; iy < 6; iy++) {
    const cx = -125 + 50 * ix, cy = -125 + 50 * iy;
    if (districtAt(cx, cy).id !== 'downtown') continue;
    curbs.push({ x: cx, y: .04, z: -cy, w: 31.8, h: .08, d: 31.8 });
    for (let building = 0; building < 3; building++) {
      const width = building === 2 ? 23 : 10.5, depth = building === 2 ? 11 : 12;
      const x = cx + (building === 0 ? -6.5 : building === 1 ? 6.5 : 0);
      const y = cy + (building === 2 ? 7 : -6.2);
      const height = 5 + Math.floor(random() * 8) * 3.2;
      bodies.push({ x, y: height / 2 + .2, z: -y, w: width, h: height, d: depth, color: palette[(ix + iy + building) % palette.length] });
      roofs.push({ x, y: height + .38, z: -y, w: width + .4, h: .35, d: depth + .4 });
      roofs.push({ x: x + 1, y: height + 1, z: -y, w: 2.7, h: 1, d: 2.1 });
      colliders.push({ x, y, w: width / 2, h: depth / 2 });
      for (let z = 2; z < height - .8; z += 3.2) for (let k = -width / 2 + 1.4; k < width / 2 - .5; k += 2.5) {
        for (const side of [-1, 1]) windows.push({ x: x + k, y: z, z: -y + side * (depth / 2 + .025), w: 1.4, h: 1.7, d: .04, color: random() < .27 ? 0xe7bb7e : 0x608693 });
      }
      for (let z = 2; z < height - .8; z += 3.2) for (const side of [-1, 1]) for (const dz of [-3.5, 0, 3.5]) windows.push({ x: x + side * (width / 2 + .025), y: z, z: -y + dz, w: .04, h: 1.7, d: 1.4, color: 0x64828b });
      if (building === 2 && (ix + iy) % 2 === 0) {
        const names = ['NITE MARKET', 'DRIVING CLUB', 'COFFEE / 24', 'AUTO SERVICE'];
        const canvas = document.createElement('canvas'); canvas.width = 512; canvas.height = 96;
        const c = canvas.getContext('2d'); c.fillStyle = '#172c31'; c.fillRect(0, 0, 512, 96); c.fillStyle = (ix + iy) % 4 ? '#ffc58a' : '#85e3de'; c.font = 'bold 37px sans-serif'; c.textAlign = 'center'; c.fillText(names[(ix + iy) % 4], 256, 61);
        const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace;
        mesh(scene, box, new THREE.MeshStandardMaterial({ map: texture, emissiveMap: texture, emissive: 0xffffff, emissiveIntensity: .65 }), x, 3.2, -y + depth / 2 + .08, width * .9, 1.8, .12);
      }
    }
    for (const side of [-1, 1]) {
      const x = cx + side * 14.5, z = -cy + side * 14.5;
      lamps.push({ x, y: 2.9, z, w: .10, h: 5.8, d: .10 });
      roofs.push({ x: x - side * .4, y: 5.8, z, w: 1.2, h: .16, d: .45 });
      mesh(scene, box, new THREE.MeshBasicMaterial({ color: 0xffd8a4 }), x - side * .4, 5.70, z, .85, .05, .3);
    }
  }
  instances(scene, box, matte(0x909b9b), curbs).receiveShadow = true;
  instances(scene, box, matte(0xffffff), bodies);
  instances(scene, box, matte(0x46545c), roofs);
  instances(scene, box, new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: .3, metalness: .2, emissive: 0x201e19, emissiveIntensity: .2 }), windows);
  instances(scene, box, matte(0xd0c9ae), paint);
  instances(scene, box, matte(0x47535c), lamps);
  // Paired signal heads at each intersection, driven by the same phase as traffic.
  const signals = [], poles = [], heads = [], lenses = [], crosswalks = [];
  for (const x of STREETS) for (const y of STREETS) {
    for (const axis of ['x', 'y']) {
      const px = x + (axis === 'y' ? 8.1 : -10), py = y + (axis === 'y' ? -10 : -8.1);
      poles.push({ x: px, y: 2.5, z: -py, w: .13, h: 5, d: .13 });
      heads.push({ x: px, y: 4.7, z: -py, w: .50, h: 1.35, d: .42 });
      const indices = [];
      for (let j = 0; j < 3; j++) { indices.push(lenses.length); lenses.push({ x: px, y: 5.10 - j * .4, z: -py + .24, w: .14, h: .14, d: .10, color: 0x10171a }); }
      signals.push({ x, y, axis, indices });
    }
    // Crosswalk paint stays within the junction approaches.
    for (const side of [-1, 1]) for (let k = -6; k <= 6; k += 2) {
      crosswalks.push({ x: x + k, y: .02, z: -y + side * 10.5, w: 1.2, h: .025, d: 2 });
      crosswalks.push({ x: x + side * 10.5, y: .025, z: -y + k, w: 2, h: .025, d: 1.2 });
    }
  }
  instances(scene, box, matte(0x424f55), poles);
  instances(scene, box, matte(0xc6ccbf), crosswalks);
  instances(scene, box, matte(0x1b252b), heads);
  const signalMesh = instances(scene, new THREE.SphereGeometry(1, 7, 5), new THREE.MeshBasicMaterial({ color: 0xffffff }), lenses);
  let oldSecond = -1;
  const colors = { red: new THREE.Color(0xff5c48), yellow: new THREE.Color(0xffc66d), green: new THREE.Color(0x74efbf), off: new THREE.Color(0x152425) };
  return { colliders, buildingCount: bodies.length + districts.buildingCount, signals: signals.length / 2, update(time) {
    const second = Math.floor(time * 2); if (second === oldSecond) return; oldSecond = second;
    for (const s of signals) { const color = phase(s.axis, s.x, s.y, time); s.indices.forEach((i, j) => signalMesh.setColorAt(i, color === ['red', 'yellow', 'green'][j] ? colors[color] : colors.off)); }
    signalMesh.instanceColor.needsUpdate = true;
  } };
}

const tireMat = matte(0x10171a), trim = matte(0x17242c), glass = new THREE.MeshStandardMaterial({ color: 0x486675, metalness: .5, roughness: .18 });
const wheelGeo = new THREE.CylinderGeometry(.32, .32, .22, 10);
export function makeCar(color = 0xf0eee3, police = false, detailed = false) {
  const car = new THREE.Group(), paint = new THREE.MeshStandardMaterial({ color, metalness: .25, roughness: .3 });
  const body = mesh(car, box, paint, 0, .65, 0, 4.15, .65, 1.78); body.castShadow = true;
  mesh(car, box, glass, -.22, 1.17, 0, 1.95, .60, 1.56);
  mesh(car, box, paint, -.26, 1.49, 0, 1.7, .10, 1.58);
  mesh(car, box, trim, .86, 1.0, 0, 1.6, .07, 1.74);
  mesh(car, box, trim, 0, .43, 0, 4.3, .16, 1.84);
  if (detailed) {
    for (const side of [-1, 1]) { mesh(car, box, trim, 0, .72, side * .90, 2.6, .20, .035); mesh(car, box, trim, .12, 1.18, side * .79, .10, .63, .04); }
    mesh(car, box, trim, -1.75, 1.1, 0, .25, .12, 1.9);
  }
  const wheels = [];
  for (const x of [-1.28, 1.25]) for (const side of [-1, 1]) {
    const root = new THREE.Group(); root.position.set(x, .35, side * .91); car.add(root);
    const wheel = new THREE.Mesh(wheelGeo, tireMat); wheel.rotation.x = Math.PI / 2; root.add(wheel);
    wheels.push({ root, wheel, front: x > 0 });
  }
  const head = new THREE.MeshBasicMaterial({ color: 0xffe6b2 }), tail = new THREE.MeshBasicMaterial({ color: 0xff544b });
  for (const side of [-1, 1]) { mesh(car, box, head, 2.09, .73, side * .58, .035, .20, .43); mesh(car, box, tail, -2.09, .74, side * .59, .035, .18, .42); }
  let beacons = [];
  if (police) {
    mesh(car, box, trim, -.25, 1.61, 0, .35, .12, 1.35);
    beacons = [-1, 1].map(side => mesh(car, box, new THREE.MeshBasicMaterial({ color: side < 0 ? 0x329bff : 0xff5046 }), -.25, 1.75, side * .43, .34, .18, .55));
  }
  car.userData = { wheels, beacons };
  return car;
}

export class Traffic {
  constructor(scene) {
    this.scene = scene; this.cars = []; this.active = false;
    const colors = [0xdf815b, 0xb0c8be, 0x6b9fbd, 0xd4bd72, 0x6c787b, 0xb3a6b9];
    for (let i = 0; i < 20; i++) this.spawn(false, colors[i % colors.length]);
  }
  spawn(police, color = 0xd9e3e4, near = null) {
    const gx = near ? nearestStreet(near.x) : STREETS[Math.floor(random() * STREETS.length)];
    const gy = near ? nearestStreet(near.y) : STREETS[Math.floor(random() * STREETS.length)];
    const node = { x: gx, y: gy };
    const c = { mesh: makeCar(color, police), x: gx, y: gy, yaw: random() * Math.PI * 2, speed: 0, desired: 7 + random() * 7, police, node, previous: null, next: null, distance: 0 };
    this.choose(c, near);
    const direction = Math.atan2(c.next.y - gy, c.next.x - gx);
    c.x += Math.cos(direction) * (police ? 13 : 10 + random() * 25) + Math.sin(direction) * 3.8;
    c.y += Math.sin(direction) * (police ? 13 : 10 + random() * 25) - Math.cos(direction) * 3.8;
    c.yaw = direction; this.scene.add(c.mesh); this.cars.push(c); return c;
  }
  choose(c, player) {
    const n = c.node;
    let choices = [[50, 0], [-50, 0], [0, 50], [0, -50]].map(([x, y]) => ({ x: n.x + x, y: n.y + y })).filter(p => Math.abs(p.x) <= 150 && Math.abs(p.y) <= 150 && (!c.previous || p.x !== c.previous.x || p.y !== c.previous.y));
    if (!choices.length) choices = [c.previous || { x: 0, y: 0 }];
    if (c.police && player) choices.sort((a, b) => Math.hypot(a.x - player.x, a.y - player.y) - Math.hypot(b.x - player.x, b.y - player.y));
    c.next = choices[Math.floor(random() * (c.police && player ? 1 : choices.length))];
  }
  setChase(enabled, player) {
    this.active = enabled;
    for (const c of this.cars.filter(c => c.police)) this.scene.remove(c.mesh);
    this.cars = this.cars.filter(c => !c.police);
    if (enabled) for (const side of [-1, 1]) this.spawn(true, 0xe2e5df, { x: player.x + 50 * side, y: player.y + 50 });
  }
  update(dt, time, player) {
    for (const c of this.cars) {
      const heading = Math.atan2(c.next.y - c.node.y, c.next.x - c.node.x);
      const tx = c.next.x + Math.sin(heading) * 3.8, ty = c.next.y - Math.cos(heading) * 3.8;
      const distance = Math.hypot(tx - c.x, ty - c.y);
      let desired = c.police ? 17 + Math.min(5, Math.hypot(c.x - player.x, c.y - player.y) * .04) : c.desired;
      if (!c.police && distance < 23 && distance > 9 && phase(Math.abs(Math.cos(heading)) > .5 ? 'x' : 'y', c.next.x, c.next.y, time) !== 'green') desired = Math.min(desired, Math.sqrt(Math.max(0, 7 * (distance - 11))));
      for (const other of [...this.cars, player]) {
        if (other === c) continue;
        const dx = other.x - c.x, dy = other.y - c.y, front = dx * Math.cos(c.yaw) + dy * Math.sin(c.yaw), lateral = -dx * Math.sin(c.yaw) + dy * Math.cos(c.yaw);
        if (front > 0 && front < 15 && Math.abs(lateral) < 2.4) desired = Math.min(desired, Math.max(0, (front - 5) * 1.1));
      }
      c.speed += Math.max(-5 * dt, Math.min(2.3 * dt, desired - c.speed));
      const toward = Math.atan2(ty - c.y, tx - c.x);
      c.yaw += Math.max(-1.7 * dt, Math.min(1.7 * dt, wrap(toward - c.yaw)));
      c.x += Math.cos(c.yaw) * c.speed * dt; c.y += Math.sin(c.yaw) * c.speed * dt; c.distance += c.speed * dt;
      if (distance < 7) { c.previous = c.node; c.node = c.next; this.choose(c, player); }
      c.mesh.position.set(c.x, .02, -c.y); c.mesh.rotation.y = -c.yaw;
      c.mesh.userData.wheels.forEach(({ wheel }) => { wheel.rotation.z = -c.distance / .32; });
      c.mesh.userData.beacons.forEach((b, i) => { b.visible = Math.floor(time * 8) % 2 === i; });
    }
  }
}
