import * as THREE from 'three';
import { Junctions } from './junctions.js';
import { routeToStop, stopAhead, stopSpeedLimit } from './fleet-routing.js';
import { turnArc, arcTarget } from './traffic-turns.js';
import { applyVehiclePose } from './vehicle-pose.js';
import { districtAt } from './regions.js';
import { createDistrictScenery } from './scenery.js';
import { roadGraph, nodeKey, HILL_TRAFFIC, LOWER_RING } from './urban-roads.js';

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

export function createCity(scene, options = {}) {
  const colliders = [], windows = [], bodies = [], roofs = [], curbs = [], paint = [], lamps = [];
  // Eastern terrain is cut around the excavated tunnel by createRidge.
  mesh(scene, box, matte(0x53635b), 0, -.42, 0, 380, .5, 380).receiveShadow = true;
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
    if (districtAt(cx, cy).id !== 'downtown' || (options.fleet && cx === -25 && cy === 75)) continue;
    curbs.push({ x: cx, y: .04, z: -cy, w: 31.8, h: .08, d: 31.8 });
    for (let building = 0; building < 3; building++) {
      const width = building === 2 ? 23 : 10.5, depth = building === 2 ? 11 : 12;
      const x = cx + (building === 0 ? -6.5 : building === 1 ? 6.5 : 0);
      const y = cy + (building === 2 ? 7 : -6.2);
      const height = 5 + Math.floor(random() * 8) * 3.2;
      bodies.push({ x, y: height / 2 + .2, z: -y, w: width, h: height, d: depth, color: palette[(ix + iy + building) % palette.length] });
      roofs.push({ x, y: height + .38, z: -y, w: width + .4, h: .35, d: depth + .4 });
      roofs.push({ x: x + 1, y: height + 1, z: -y, w: 2.7, h: 1, d: 2.1 });
      colliders.push({ x, y, w: width / 2, h: depth / 2, top: height });
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
export function makeCar(color = 0xf0eee3, police = false, detailed = false, variant = 'sedan') {
  variant = police ? 'police' : ['sedan', 'compact', 'taxi', 'van'].includes(variant) ? variant : 'sedan';
  const compact = variant === 'compact', van = variant === 'van', taxi = variant === 'taxi';
  const length = compact ? 3.60 : 4.15, width = compact ? 1.70 : 1.78, axles = compact ? [-1.04, 1.04] : [-1.28, 1.25];
  const car = new THREE.Group(), paint = new THREE.MeshStandardMaterial({ color, metalness: .25, roughness: .3 });
  mesh(car, box, paint, 0, .65, 0, length, .65, width).castShadow = true;
  if (van) {
    mesh(car, box, paint, -.47, 1.34, 0, 2.95, 1.10, 1.75).castShadow = true;
    mesh(car, box, paint, -.47, 1.94, 0, 2.98, .10, 1.78);
    mesh(car, box, glass, 1.13, 1.28, 0, .72, .69, 1.63);
    mesh(car, box, paint, 1.09, 1.69, 0, .95, .12, 1.70);
    for (const side of [-1, 1]) mesh(car, box, trim, -.45, 1.12, side * .89, 2.65, .08, .025);
    mesh(car, box, trim, -2.09, 1.38, 0, .025, 1.0, .04);
  } else {
    mesh(car, box, glass, compact ? -.10 : -.22, compact ? 1.22 : 1.17, 0, compact ? 2.13 : 1.95, compact ? .70 : .60, width - .22);
    mesh(car, box, paint, compact ? -.12 : -.26, compact ? 1.61 : 1.49, 0, compact ? 1.92 : 1.70, .10, width - .20);
    mesh(car, box, trim, compact ? .90 : .86, 1.0, 0, compact ? 1.0 : 1.6, .07, width - .04);
  }
  mesh(car, box, trim, 0, .43, 0, length + .15, .16, width + .06);
  if (detailed) {
    for (const side of [-1, 1]) { mesh(car, box, trim, 0, .72, side * .90, 2.6, .20, .035); mesh(car, box, trim, .12, 1.18, side * .79, .10, .63, .04); }
    mesh(car, box, trim, -1.75, 1.1, 0, .25, .12, 1.9);
  }
  if (taxi) {
    const sign = new THREE.MeshBasicMaterial({ color: 0xffe5a0 });
    mesh(car, box, trim, -.25, 1.60, 0, .65, .10, .67);
    mesh(car, box, sign, -.25, 1.78, 0, .60, .26, .60);
    for (const side of [-1, 1]) mesh(car, box, trim, 0, .72, side * .90, 3.35, .13, .035);
  }
  if (police) for (const side of [-1, 1]) mesh(car, box, trim, 0, .71, side * .90, 3.55, .26, .035);
  const wheels = [];
  for (const x of axles) for (const side of [-1, 1]) {
    const root = new THREE.Group(); root.position.set(x, .35, side * (width / 2 + .02)); car.add(root);
    const wheel = new THREE.Mesh(wheelGeo, tireMat); wheel.rotation.x = Math.PI / 2; root.add(wheel);
    wheels.push({ root, wheel, front: x > 0 });
  }
  const head = new THREE.MeshBasicMaterial({ color: 0xffe6b2 }), tail = new THREE.MeshBasicMaterial({ color: 0xff544b });
  for (const side of [-1, 1]) { mesh(car, box, head, length / 2 + .015, .73, side * .58, .035, .20, .43); mesh(car, box, tail, -length / 2 - .015, .74, side * .59, .035, .18, .42); }
  let beacons = [];
  if (police) {
    mesh(car, box, trim, -.25, 1.61, 0, .35, .12, 1.35);
    beacons = [-1, 1].map(side => mesh(car, box, new THREE.MeshBasicMaterial({ color: side < 0 ? 0x329bff : 0xff5046 }), -.25, 1.75, side * .43, .34, .18, .55));
  }
  car.userData = { wheels, beacons, variant, tailMaterial: tail };
  return car;
}

// A framing diagnostic, not operational sensing: project body centres and reject
// terrain/building occlusion. The recorded PNGs remain the final visual check.
export function trafficInView(cars, camera, colliders = [], drive = null) {
  camera.updateMatrixWorld();
  const origin = { x: camera.position.x, y: -camera.position.z, z: camera.position.y };
  function intersects(a, b, box) {
    let enter = 0, leave = 1;
    for (const [key, low, high] of [
      ['x', box.x - box.w, box.x + box.w], ['y', box.y - box.h, box.y + box.h],
      ['z', box.bottom ?? -1000, box.top ?? 32],
    ]) {
      const delta = b[key] - a[key];
      if (Math.abs(delta) < 1e-8) { if (a[key] < low || a[key] > high) return false; }
      else { const t1 = (low - a[key]) / delta, t2 = (high - a[key]) / delta; enter = Math.max(enter, Math.min(t1,t2)); leave = Math.min(leave, Math.max(t1,t2)); if (enter > leave) return false; }
    }
    return enter < .995 && leave > .001;
  }
  const result = [];
  for (const c of cars) {
    const target = { x: c.x, y: c.y, z: (c.height ?? 0) + .9 };
    const distance = Math.hypot(target.x-origin.x, target.y-origin.y, target.z-origin.z);
    if (distance > 90) continue;
    const screen = new THREE.Vector3(target.x, target.z, -target.y).project(camera);
    if (screen.z < -1 || screen.z > 1 || Math.abs(screen.x) > .92 || Math.abs(screen.y) > .86) continue;
    if (colliders.some(b => !b.broken && intersects(origin,target,b))) continue;
    let covered = false;
    if (drive) for(let i=1;i<16;i++) {
      const t=i/16,x=origin.x+(target.x-origin.x)*t,y=origin.y+(target.y-origin.y)*t,z=origin.z+(target.z-origin.z)*t;
      if(drive.drive_surface(x,y,0)>z-.12){covered=true;break;}
    }
    if (!covered) result.push({ id: c.id, x: c.x, y: c.y, speed: c.speed, police: c.police, variant: c.variant ?? c.mesh.userData.variant, distance, screenX: screen.x, screenY: screen.y });
  }
  return result;
}


// Route to the closest point on a directed road edge. Comparing only endpoint
// distances sends pursuers down a short side street beside a long avenue.
function pursuitCosts(graph, from, player) {
  const distances = new Map(), direct = new Map(), incoming = new Map([...graph.keys()].map(key => [key, []]));
  let nearest = Infinity;
  for (const a of graph.values()) for (const b of a.links) {
    incoming.get(nodeKey(b)).push(a);
    const dx = b.x - a.x, dy = b.y - a.y, length = Math.hypot(dx, dy);
    const t = Math.max(0, Math.min(1, ((player.x - a.x) * dx + (player.y - a.y) * dy) / (length * length)));
    const away = Math.hypot(player.x - a.x - t * dx, player.y - a.y - t * dy);
    if (away < nearest - 1e-6) { nearest = away; distances.clear(); direct.clear(); }
    if (Math.abs(away - nearest) < 1e-6) {
      distances.set(nodeKey(a), Math.min(distances.get(nodeKey(a)) ?? Infinity, t * length));
      direct.set(nodeKey(a) + '>' + nodeKey(b), t * length);
    }
  }
  const pending = new Set(graph.keys());
  while (pending.size) {
    let key, best = Infinity;
    for (const candidate of pending) if ((distances.get(candidate) ?? Infinity) < best) { key = candidate; best = distances.get(candidate); }
    if (key === undefined) break;
    pending.delete(key);
    const destination = graph.get(key);
    for (const previous of incoming.get(key)) {
      const previousKey = nodeKey(previous), cost = best + Math.hypot(previous.x - destination.x, previous.y - destination.y);
      if (pending.has(previousKey) && cost < (distances.get(previousKey) ?? Infinity)) distances.set(previousKey, cost);
    }
  }
  return new Map(from.links.map(next => [nodeKey(next), Math.min(
    direct.get(nodeKey(from) + '>' + nodeKey(next)) ?? Infinity,
    Math.hypot(from.x - next.x, from.y - next.y) + (distances.get(nodeKey(next)) ?? Infinity)
  )]));
}

export class Traffic {
  constructor(scene, options = {}) {
    this.scene = scene; this.cars = []; this.active = false; this.nextId = 0;
    this.drive = options.drive; this.urban = Boolean(options.urban);
    this.graph = roadGraph(STREETS, this.urban);
    this.junctions = new Junctions(this.graph, { phase, lane: car => this.lane(car), drive: this.drive });
    const colors = [0xdf815b, 0xb0c8be, 0x6b9fbd, 0xd4bd72, 0x6c787b, 0xb3a6b9];
    for (let i = 0; i < (options.cityCars ?? 20); i++) this.spawn(false, colors[i % colors.length]);
    if (this.urban) {
      for (let i = 0; i < HILL_TRAFFIC.length; i++) this.spawn(false, colors[i % colors.length], HILL_TRAFFIC[i]);
      for (let i = 0; i < 8; i++) this.spawn(false, colors[(i + 2) % colors.length], LOWER_RING[i * 8]);
    }
  }
  spawn(police, color = 0xd9e3e4, near = null, pursuitTarget = near) {
    const node = near
      ? [...this.graph.values()].reduce((a, b) => Math.hypot(a.x - near.x, a.y - near.y) < Math.hypot(b.x - near.x, b.y - near.y) ? a : b)
      : this.graph.get(nodeKey({ x: STREETS[Math.floor(random() * STREETS.length)], y: STREETS[Math.floor(random() * STREETS.length)] }));
    const gx = node.x, gy = node.y;
    const variant = police ? 'police' : near?.variant === 'taxi' ? 'taxi' : ['sedan', 'compact', 'taxi', 'van'][this.cars.filter(c => !c.police).length % 4];
    const c = { id: this.nextId++, variant, serviceStop: near?.serviceStop ?? null, mesh: makeCar(variant === 'taxi' ? 0xe2bd58 : color, police, false, variant), x: gx, y: gy, yaw: random() * Math.PI * 2, speed: 0, desired: 7 + random() * 7, police, node, previous: null, next: null, distance: 0 };
    const planned = near?.to && this.graph.get(nodeKey(near.to));
    if (!police && planned && node.links.includes(planned)) c.next = planned;
    else this.choose(c, police ? pursuitTarget : null);
    const direction = Math.atan2(c.next.y - gy, c.next.x - gx);
    const initial = Number.isFinite(near?.advance) ? near.advance : police ? 13 : 10 + random() * 25;
    const limit = Math.max(2, Math.hypot(c.next.x - gx, c.next.y - gy) - 3), lane = this.lane(c);
    const positions = [0, 8, -8, 16, -16].map(offset => {
      const advance = Math.max(2, Math.min(initial + offset, limit));
      const x = gx + Math.cos(direction) * advance + Math.sin(direction) * lane;
      const y = gy + Math.sin(direction) * advance - Math.cos(direction) * lane;
      return { x, y, gap: Math.min(Infinity, ...this.cars.map(other=>Math.hypot(other.x-x,other.y-y))) };
    });
    const position = positions.find(p=>p.gap>=5.5) ?? positions.sort((a,b)=>b.gap-a.gap)[0];
    c.x = position.x; c.y = position.y;
    c.height = this.drive?.drive_surface(c.x, c.y, 0) ?? 0;
    c.yaw = direction; this.scene.add(c.mesh); this.cars.push(c); return c;
  }
  choose(c, player) {
    const n = c.node;
    if (c.serviceStop) {
      const route = routeToStop(this.graph, n, c.previous, c.serviceStop);
      if (route?.nodes.length) { c.next = route.nodes[0]; return; }
    }
    let choices = (this.graph.get(nodeKey(n))?.links ?? []).filter(p => !c.previous || nodeKey(p) !== nodeKey(c.previous));
    if (!choices.length) choices = [c.previous || { x: 0, y: 0 }];
    if (c.police && player) {
      const costs = pursuitCosts(this.graph, n, player);
      choices.sort((a, b) => (costs.get(nodeKey(a)) ?? Infinity) - (costs.get(nodeKey(b)) ?? Infinity)
        || Math.hypot(a.x - player.x, a.y - player.y) - Math.hypot(b.x - player.x, b.y - player.y));
    }
    c.next = choices[Math.floor(random() * (c.police && player ? 1 : choices.length))];
  }
  lane(c) {
    // Junction arcs stay in the through lane; the stop aim handles the curb approach.
    return this.urban && c.node.y === c.next.y && [118.5, 181.5].includes(c.node.y) ? 0 : 3.8;
  }
  setChase(enabled, player) {
    this.active = enabled;
    for (const c of this.cars.filter(c => c.police)) this.scene.remove(c.mesh);
    this.cars = this.cars.filter(c => !c.police);
    if (enabled) {
      const yaw = Number.isFinite(player.yaw) ? player.yaw : 0, c = Math.cos(yaw), s = Math.sin(yaw), used = new Set();
      // Distinct road nodes behind the player produce a real following pursuit.
      for (const gap of [32, 55]) {
        let nodes = [...this.graph.values()].filter(n => (n.x-player.x)*c+(n.y-player.y)*s < -12 && !used.has(nodeKey(n)));
        if (!nodes.length) nodes = [...this.graph.values()].filter(n => !used.has(nodeKey(n)));
        const target = { x: player.x-c*gap, y: player.y-s*gap };
        nodes.sort((a,b)=>Math.hypot(a.x-target.x,a.y-target.y)-Math.hypot(b.x-target.x,b.y-target.y));
        const node = nodes[0]; used.add(nodeKey(node)); this.spawn(true, 0xe2e5df, node, player);
      }
    }
  }
  update(dt, time, player) {
    for (const c of this.cars) {
      const fresh = c.plannedAt !== c.next, retarget = c.plannedStop !== c.serviceStop;
      const moved = Math.hypot(player.x - (c.plannedTarget?.x ?? Infinity),
                               player.y - (c.plannedTarget?.y ?? Infinity));
      if (!fresh && !retarget && (!c.police || c.arc || moved < 8)) continue;
      const turn = { id: c.id, node: c.next, previous: c.node, police: c.police, serviceStop: c.serviceStop };
      this.choose(turn, player);
      const arc = turnArc(c.node, c.next, turn.next, this.lane(c),
        this.lane({ node: c.next, next: turn.next }));
      // Refresh a moving pursuit target, retaining a bend when it is too late to brake.
      const distance = Math.hypot(c.next.x - c.x, c.next.y - c.y);
      if (fresh || !arc || distance >= 7 + Math.max(0, c.speed ** 2 - arc.speed ** 2) / 7) {
        c.plannedAt = c.next; c.plannedNext = turn.next; c.plannedArc = arc; c.plannedStop = c.serviceStop;
      }
      if (c.police) c.plannedTarget = { x: player.x, y: player.y };
    }
    this.junctions.update(dt, time, this.cars, player);
    for (const c of this.cars) {
      const heading = Math.atan2(c.next.y - c.node.y, c.next.x - c.node.x);
      const lane = this.lane(c), tx = c.next.x + Math.sin(heading) * lane, ty = c.next.y - Math.cos(heading) * lane;
      const distance = Math.hypot(tx - c.x, ty - c.y);
      const arcAim = c.arc && arcTarget(c.arc, c);
      if (c.arc && !arcAim) c.arc = null;
      const serviceAim = stopAhead(c) ? c.serviceStop : null;
      const toward = Math.atan2((arcAim?.y ?? serviceAim?.y ?? ty) - c.y, (arcAim?.x ?? serviceAim?.x ?? tx) - c.x);
      let desired = c.police ? 17 + Math.min(5, Math.hypot(c.x - player.x, c.y - player.y) * .04) : c.desired;
      // Brake for the actual bend instead of carrying cruise speed through tight alleys.
      desired = Math.min(desired, Math.max(5, (c.police ? 24 : 15) - 12 * Math.abs(wrap(toward-c.yaw))));
      if (c.plannedArc) desired = Math.min(desired,
        Math.sqrt(c.plannedArc.speed ** 2 + 7 * Math.max(0, distance - 7)));
      if (c.arc) desired = Math.min(desired, c.arc.speed);
      desired = Math.min(desired, this.junctions.limit(c), stopSpeedLimit(c));
      for (const other of [...this.cars, player]) {
        if (other === c) continue;
        const dx = other.x - c.x, dy = other.y - c.y, front = dx * Math.cos(c.yaw) + dy * Math.sin(c.yaw), lateral = -dx * Math.sin(c.yaw) + dy * Math.cos(c.yaw);
        if (front <= 0 || front >= 15 || Math.abs(lateral) >= 2.4) continue;
        // Ground traffic can differ in height on a slope while sharing one road.
        // A car on an upper deck or in flight remains a separate layer.
        if (Math.abs((other.height ?? 0) - (c.height ?? 0)) > 2.5) {
          const otherRoad = this.drive?.drive_surface(other.x, other.y, 0) ?? 0, ownRoad = this.drive?.drive_surface(c.x, c.y, 0) ?? 0;
          if (Math.abs((other.height ?? 0) - otherRoad) > 2.5 || Math.abs((c.height ?? 0) - ownRoad) > 2.5) continue;
        }
        desired = Math.min(desired, Math.max(0, (front - 5) * 1.1));
      }
      c.mesh.userData.tailMaterial?.color.setHex(desired < c.speed - .35 ? 0xff3f35 : 0x963d39);
      c.speed += Math.max(-5 * dt, Math.min((c.police ? 3.2 : 2.3) * dt, desired - c.speed));
      c.yaw += Math.max(-1.7 * dt, Math.min(1.7 * dt, wrap(toward - c.yaw)));
      c.x += Math.cos(c.yaw) * c.speed * dt; c.y += Math.sin(c.yaw) * c.speed * dt; c.distance += c.speed * dt;
      if (distance < 7 && !c.arc) {
        c.previous = c.node; c.node = c.next; c.next = c.plannedNext; c.arc = c.plannedArc;
      }
      c.height = this.drive?.drive_surface(c.x, c.y, 0) ?? 0;
      const gx = this.drive?.drive_surface(c.x, c.y, 1) ?? 0, gy = this.drive?.drive_surface(c.x, c.y, 2) ?? 0;
      applyVehiclePose(c.mesh, {
        ...c, roll: Math.atan(gy * Math.cos(c.yaw) - gx * Math.sin(c.yaw)),
        pitch: Math.atan(gx * Math.cos(c.yaw) + gy * Math.sin(c.yaw)),
        steering: Math.max(-.45, Math.min(.45, wrap(toward-c.yaw))),
      });
      c.mesh.userData.beacons.forEach((b, i) => { b.visible = Math.floor(time * 8) % 2 === i; });
    }
  }
}
