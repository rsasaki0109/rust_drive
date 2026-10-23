import * as THREE from 'three';
import { HILL_STREETS, LOWER_RING } from './urban-roads.js';
import { createCrashGuardrails } from './guardrails.js';

const box = new THREE.BoxGeometry(1, 1, 1);
const hue = [0x8d9697, 0xb5aba0, 0x647986, 0xa29180, 0x777f82, 0xc2b9a1];
export function createUrbanDistrict(scene, drive, phase) {
  const height = (x, y) => drive.drive_surface(x, y, 0);
  const colliders = [], solids = [], glass = [], glow = [], signals = [], surfaces = new Map();
  let buildingCount = 0;
  function cuboid(x, level, y, w, h, d, color, target = solids) {
    target.push({ x, y, level, w, h, d, color });
  }
  function batch(rows, material) {
    const mesh = new THREE.InstancedMesh(box, material, rows.length), t = new THREE.Object3D();
    rows.forEach((r, i) => { t.position.set(r.x, r.level, -r.y); t.rotation.set(0, 0, 0); t.scale.set(r.w, r.h, r.d); t.updateMatrix(); mesh.setMatrixAt(i, t.matrix); mesh.setColorAt(i, new THREE.Color(r.color)); });
    mesh.instanceMatrix.needsUpdate = true; mesh.receiveShadow = true; mesh.castShadow = true; scene.add(mesh); return mesh;
  }
  // Sample across the width as well as along the path, including sloped junctions.
  function ribbon(points, width, key) {
    if (!surfaces.has(key)) surfaces.set(key, { vertices: [], indices: [] });
    const { vertices, indices } = surfaces.get(key), columns = Math.max(1, Math.ceil(width / 2)), start = vertices.length / 3;
    points.forEach((p, i) => {
      const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)], length = Math.hypot(b.x - a.x, b.y - a.y);
      for (let j = 0; j <= columns; j++) {
        const offset = width * (.5 - j / columns), x = p.x - (b.y - a.y) / length * offset, y = p.y + (b.x - a.x) / length * offset;
        vertices.push(x, height(x, y) + (key === 'paint' ? .075 : key === 'pavement' ? .085 : .045), -y);
      }
      if (i) for (let j = 0; j < columns; j++) { const a = start + (i - 1) * (columns + 1) + j, b = a + 1, c = a + columns + 1, d = c + 1; indices.push(a, b, c, b, d, c); }
    });
  }
  function line(a, b, step = 2) {
    const count = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / step));
    return Array.from({ length: count + 1 }, (_, i) => ({ x: a.x + (b.x - a.x) * i / count, y: a.y + (b.y - a.y) * i / count }));
  }
  function panel(text, color, x, y, level, w, h, yaw = 0, vertical = false) {
    const canvas = document.createElement('canvas'); canvas.width = vertical ? 160 : 768; canvas.height = vertical ? 640 : 160;
    const c = canvas.getContext('2d'); c.fillStyle = '#182a34'; c.fillRect(0, 0, canvas.width, canvas.height);
    c.fillStyle = color; c.textAlign = 'center'; c.font = vertical ? 'bold 104px sans-serif' : 'bold 62px sans-serif';
    if (vertical) [...text].forEach((v, i) => c.fillText(v, 80, 120 + i * 130)); else c.fillText(text, 384, 106);
    const map = new THREE.CanvasTexture(canvas); map.colorSpace = THREE.SRGBColorSpace;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshStandardMaterial({ map, emissiveMap: map, emissive: 0xffffff, emissiveIntensity: .65, side: THREE.DoubleSide }));
    m.position.set(x, level, -y); m.rotation.y = yaw; scene.add(m);
  }
  function building(x, y, w, d, h, index, avenue = false) {
    const samples = [[x - w / 2, y - d / 2], [x + w / 2, y - d / 2], [x - w / 2, y + d / 2], [x + w / 2, y + d / 2]].map(([x, y]) => height(x, y));
    const base = Math.max(...samples) + .12, bottom = Math.min(...samples) - .25;
    cuboid(x, (base + bottom) / 2, y, w, base - bottom, d, 0x626c70);
    cuboid(x, base + h / 2, y, w, h, d, hue[index % hue.length]);
    cuboid(x, base + h + .20, y, w + .25, .40, d + .25, 0x4d5b62);
    cuboid(x - 1.2, base + h + .9, y + 1, 2.8, 1.1, 2.1, 0xa8aaa0);
    colliders.push({ x, y, w: w / 2, h: d / 2, bottom, top: base + h + 1.5 });
    for (let level = 2.7; level < h - .6; level += 3.1) for (let dx = -w / 2 + 1.6; dx < w / 2 - .7; dx += 2.6) for (const side of [-1, 1])
      cuboid(x + dx, base + level, y + side * (d / 2 + .03), 1.5, 1.6, .06, (index + Math.round(dx) + Math.round(level)) % 4 ? 0x587886 : 0xf0c58b, glass);
    if (avenue) {
      const side = y < 150 ? 1 : -1, front = y + side * (d / 2 + .10), yaw = side > 0 ? Math.PI : 0;
      panel(['坂通り / SAKA', 'COFFEE 24', '中央商店街', 'CITY HOTEL', 'RAMEN / 麺', 'BOOKS / 書店', 'TOKYO DRIVE'][index % 7], index % 2 ? '#ffd38a' : '#8de0d6', x, front, base + 3.2, w * .9, 1.65, yaw);
      panel(['珈琲', '食堂', '本屋', '旅館'][index % 4], index % 2 ? '#ffb8a0' : '#b6ddff', x + w * .35, front + side * .04, base + 8.2, 1.2, 5.4, yaw, true);
      for (const dx of [-2.2, 0, 2.2]) cuboid(x + dx, base + 1.3, front, 1.65, 2.3, .1, 0x34535d, glass);
      cuboid(x - w * .34, base + .9, front + side * .4, .7, 1.8, .7, 0xe77b59);
    }
    buildingCount++;
  }
  // A normal urban avenue, backstreets and junctions surround the existing hill.
  ribbon(line({ x: 150, y: 150 }, { x: 379.2, y: 150 }), 18, 'asphalt');
  for (const road of HILL_STREETS.slice(1)) for (let i = 1; i < road.length; i++)
    ribbon(line(road[i - 1], road[i]), road[0].y === road.at(-1).y ? 6 : 18, 'asphalt');
  for (const y of [139, 161]) ribbon(line({ x: 193, y }, { x: 377, y }), 4, 'pavement');
  const plots = [204, 220, 260, 278, 298, 316, 352];
  for (const [i, x] of plots.entries()) for (const side of [-1, 1]) {
    building(x, 150 + side * 20, 12 + i % 2, 14, 12 + (i * 7 + (side + 1) * 3) % 31, i + (side > 0 ? 2 : 0), true);
    building(x, 150 + side * 37.5, 12, 4, 5.5 + i % 4 * 2, i + 3);
  }
  for (const x of [240, 334, 370]) {
    for (const side of [-1, 1]) for (let k = -6; k <= 6; k += 2) {
      ribbon(line({ x: x + k, y: 150 + side * 10 - .8 }, { x: x + k, y: 150 + side * 10 + .8 }), 1.1, 'paint');
      ribbon(line({ x: x + side * 10 - .8, y: 150 + k }, { x: x + side * 10 + .8, y: 150 + k }), 1.1, 'paint');
    }
    // End-of-street rails stop cars from accidentally driving off the sides.
    for (const y of [110.5, 189.5]) for (let dx = -7; dx <= 7; dx += 2) {
      const z = height(x + dx, y);
      cuboid(x + dx, z + .7, y, 2, .25, .18, 0xcdd6cd);
      colliders.push({ x: x + dx, y, w: 1, h: .13, bottom: z - .2, top: z + 1.1 });
    }
  }
  for (const x of [210, 255, 295, 345]) for (const y of [139, 161]) {
    const base = height(x, y);
    cuboid(x, base + 3.2, y, .12, 6.4, .12, 0x677878);
    cuboid(x, base + 6.4, y, 2, .15, .65, 0xf2d2a1, glow);
  }
  // Elevated pedestrian crossing with stairs on the two sidewalks.
  const bridgeX = 355, bridgeBase = height(bridgeX, 150), deck = bridgeBase + 5.8;
  cuboid(bridgeX, deck - .55, 150, 3, 1.1, 26, 0x81918c);
  for (const x of [353.6, 356.4]) cuboid(x, deck + .6, 150, .12, 1.2, 26, 0xcbd3c8);
  for (const y of [139.3, 160.7]) {
    cuboid(355, bridgeBase + 2.6, y, .45, 5.2, .45, 0x71807d);
    for (let i = 0; i < 18; i++) {
      const x = 342.5 + i * .68, top = bridgeBase + (i + 1) / 18 * 5.8;
      cuboid(x, top - .13, y, .72, .26, 1.5, 0xa4b1a9);
    }
    colliders.push({ x: 348.5, y, w: 6.5, h: .85, bottom: bridgeBase, top: deck + .4 });
  }
  panel('坂通り  /  SAKA-DORI', '#d6eadb', 355, 150, deck + .6, 16, 1.5, -Math.PI / 2);
  // A winding boulevard and mixed-height blocks continue below the cliff.
  ribbon(LOWER_RING, 14, 'asphalt');
  ribbon(line({ x: 530, y: 150 }, { x: 545, y: 150 }), 18, 'asphalt');
  for (let i = 0; i < 64; i += 4) ribbon([LOWER_RING[i], LOWER_RING[i + 1]], .15, 'paint');
  const groundRoads = [LOWER_RING, line({ x: 400, y: 150 }, { x: 545, y: 150 }), line({ x: 530, y: 70 }, { x: 530, y: 150 })];
  function clearPlot(x, y, w, d) {
    return groundRoads.every(road => road.every(p => Math.hypot(Math.max(Math.abs(x - p.x) - w / 2, 0), Math.max(Math.abs(y - p.y) - d / 2, 0)) >= 11));
  }
  let site = 0;
  for (const x of [414, 435, 456, 477, 498, 519, 540, 554]) for (const y of [88, 123, 177, 216]) {
    if (!clearPlot(x, y, 12, 12) || (x > 450 && x < 514 && y < 115)) continue;
    building(x, y, 12, 12, 10 + site % 7 * 5, site++ + 2, false);
  }
  // Railway viaduct passes above the station boulevard at a separate physical level.
  cuboid(355, 9.45, 104, 370, 1.1, 8, 0x73827f);
  for (const y of [100.6, 103.4, 104.6, 107.4]) cuboid(355, 10.12, y, 370, .16, .12, 0xc2cbc8);
  for (const x of [184, 212, 240, 268, 296, 324, 352, 380, 408, 436, 464, 492, 520]) for (const y of [101.1, 106.9]) {
    if (x > 400 && !clearPlot(x, y, .7, .7)) continue;
    cuboid(x, 4.45, y, .7, 8.9, .7, 0x7e8c85);
    colliders.push({ x, y, w: .35, h: .35, bottom: 0, top: 8.9 });
  }
  cuboid(482.5, 9.45, 105, 45, 1.1, 14, 0x89958b);
  cuboid(482.5, 15.1, 105, 47, .4, 14, 0x3c6569);
  for (const x of [462, 502]) for (const y of [99, 111]) cuboid(x, 12.5, y, .25, 5, .25, 0xc7d0c2);
  panel('中央駅  CHUO STATION', '#90e6cc', 482.5, 97.8, 12.3, 22, 2.2);
  building(458, 132, 16, 8, 5, 4, false);
  function train(y, color) {
    const group = new THREE.Group(); scene.add(group);
    for (const x of [-13, 0, 13]) {
      const part = (cx, cy, cz, w, h, d, color) => { const m = new THREE.Mesh(box, new THREE.MeshStandardMaterial({ color, roughness: .45, metalness: .3 })); m.position.set(cx, cy, cz); m.scale.set(w, h, d); group.add(m); };
      part(x, 11.5, -y, 12.3, 2.6, 2.3, 0xc9d0cc);
      for (const side of [-1, 1]) {
        part(x, 11.2, -y + side * 1.17, 12.1, .35, .05, color);
        for (let i = -4; i <= 4; i += 2) part(x + i, 12, -y + side * 1.18, 1.3, .75, .04, 0x314d5a);
      }
      part(x, 13, -y, 11.5, .25, 2.2, 0x7e8e90);
    }
    return group;
  }
  const trains = [train(102, 0xe79a57), train(106, 0x65c0ad)];
  // Signals on the hill and the two boulevard junctions share the traffic controller.
  const lensRows = [];
  for (const [x, y] of [[240, 150], [334, 150], [370, 150], [435, 150], [545, 150]]) for (const axis of ['x', 'y']) {
    const px = x + (axis === 'y' ? 8.2 : -10), py = y + (axis === 'y' ? -10 : -8.2), base = height(px, py);
    cuboid(px, base + 2.6, py, .15, 5.2, .15, 0x54636a);
    cuboid(px, base + 4.7, py, .5, 1.35, .4, 0x1a2930);
    const indices = [];
    for (let j = 0; j < 3; j++) { indices.push(lensRows.length); lensRows.push({ x: px, y: py - .23, level: base + 5.1 - j * .4, w: .15, h: .15, d: .08, color: 0x14242b }); }
    signals.push({ x, y, axis, indices });
  }
  const lenses = batch(lensRows, new THREE.MeshBasicMaterial({ color: 0xffffff }));
  for (const [key, { vertices, indices }] of surfaces) {
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3)); g.setIndex(indices); g.computeVertexNormals();
    const material = new THREE.MeshStandardMaterial({ color: key === 'paint' ? 0xe8ddbc : key === 'pavement' ? 0x9b9e91 : 0x303e46, side: THREE.DoubleSide, roughness: .9, polygonOffset: true, polygonOffsetFactor: key === 'paint' ? -3 : -2, polygonOffsetUnits: -3 });
    const m = new THREE.Mesh(g, material); m.receiveShadow = true; scene.add(m);
  }
  batch(solids, new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: .75 }));
  batch(glass, new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: .3, metalness: .25, emissive: 0x322c22, emissiveIntensity: .35 }));
  if (glow.length) batch(glow, new THREE.MeshBasicMaterial({ color: 0xffffff }));
  const barriers = createCrashGuardrails(scene, drive); colliders.push(...barriers.colliders);
  const signalColors = [0xff6850, 0xffd178, 0x78e7bc], off = new THREE.Color(0x15252c);
  let previousSecond = -1;
  return {
    colliders, barriers, buildingCount, signals: signals.length / 2,
    trainX: () => trains[0].position.x,
    update(time, dt) {
      barriers.update(dt);
      trains.forEach((train, i) => { const t = (time + i * 17) % 34; train.position.x = 325 - 155 * Math.cos(Math.min(30, t) / 30 * Math.PI * 2); });
      const second = Math.floor(time * 2);
      if (second !== previousSecond) {
        previousSecond = second;
        for (const s of signals) { const p = phase(s.axis, s.x, s.y, time); s.indices.forEach((index, j) => lenses.setColorAt(index, p === ['red', 'yellow', 'green'][j] ? new THREE.Color(signalColors[j]) : off)); }
        lenses.instanceColor.needsUpdate = true;
      }
    },
  };
}
