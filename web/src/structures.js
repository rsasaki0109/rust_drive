import * as THREE from 'three';

export const ROAD_STARTS = Object.freeze({
  skyway: Object.freeze({ x: 190, y: 20, yaw: 0 }),
  underpass: Object.freeze({ x: 350, y: -55, yaw: Math.PI / 2 }),
  tunnel: Object.freeze({ x: 190, y: -105, yaw: 0 }),
});
export const STACKED_ROUTES = Object.freeze([
  { kind: 'skyway', color: '#91d4e0', points: [[160, 20], [530, 20]] },
  { kind: 'lower', color: '#b9c5bf', points: [[350, -150], [350, 70]] },
  { kind: 'tunnel', color: '#c4b2ed', points: [[160, -105], [530, -105]] },
  { kind: 'link', color: '#909e97', points: [[150, -150], [530, -150], [530, 150]] },
]);

// All road heights/clearance come from Rust. XY does not identify a road layer.
export function createStructures(scene, drive) {
  const colliders = [], batches = new Map();
  const asphalt = new THREE.MeshStandardMaterial({ color: 0x38454b, roughness: .96, side: THREE.DoubleSide });
  const concrete = new THREE.MeshStandardMaterial({ color: 0x73858a, roughness: .9, side: THREE.DoubleSide });
  const excavation = new THREE.MeshStandardMaterial({ color: 0x655d58, roughness: 1, side: THREE.DoubleSide });
  const box = new THREE.BoxGeometry(1, 1, 1);
  const floor = (x, y) => drive.drive_surface(x, y, 0);
  const deck = x => drive.drive_deck(x, 20, 0);
  function block(x, h, y, w, height, depth, color, emissive = false) {
    const key = color + ':' + emissive;
    if (!batches.has(key)) batches.set(key, { color, emissive, rows: [] });
    batches.get(key).rows.push({ x, h, y, w, height, depth });
  }
  function mesh(vertices, indices, material) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    g.setIndex(indices); g.computeVertexNormals();
    const m = new THREE.Mesh(g, material); m.receiveShadow = true; scene.add(m);
  }
  function ribbon(points, width, material, bottom = false) {
    const v = [], indices = [];
    points.forEach((p, i) => {
      const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
      const length = Math.hypot(b.x - a.x, b.y - a.y);
      const nx = -(b.y - a.y) / length * width / 2, ny = (b.x - a.x) / length * width / 2;
      v.push(p.x + nx, p.h, -p.y - ny, p.x - nx, p.h, -p.y + ny);
      if (i) { const n = i * 2; indices.push(n - 2, n - 1, n, n - 1, n + 1, n); }
    });
    mesh(v, bottom ? indices.toReversed() : indices, material);
  }
  const bridge = Array.from({ length: 156 }, (_, i) => ({ x: 190 + i * 2, y: 20, h: deck(190 + i * 2) }));
  ribbon(bridge.map(p => ({ ...p, h: p.h + .035 })), 18, asphalt);
  ribbon(bridge.map(p => ({ ...p, h: p.h - 1.1 })), 18, concrete, true);
  for (const side of [-1, 1]) {
    const v = [], indices = [];
    bridge.forEach((p, i) => {
      v.push(p.x, p.h - 1.1, -(20 + side * 9), p.x, p.h + .035, -(20 + side * 9));
      if (i) { const n = i * 2; indices.push(n - 2, n - 1, n, n - 1, n + 1, n); }
    });
    mesh(v, indices, concrete);
    const rail = bridge.map(p => ({ ...p, y: 20 + side * 8.8, h: p.h + .85 }));
    ribbon(rail, .35, concrete);
    for (let x = 194; x < 500; x += 6) {
      const h = deck(x);
      block(x, h + .6, 20 + side * 8.8, .22, 1.2, .25, 0xb9c7c5);
      colliders.push({ x, y: 20 + side * 8.8, w: 3, h: .25, bottom: h + .05, top: h + 1.25 });
    }
  }
  for (const x of [285, 405]) {
    for (const y of [13.5, 26.5]) {
      block(x, 5.45, y, 2.4, 10.9, 2.4, 0x728085);
      colliders.push({ x, y, w: 1.2, h: 1.2, bottom: 0, top: 10.9 });
      block(x, .2, y, 4, .4, 4, 0x55696d);
    }
    block(x, 10.6, 20, 3.2, .6, 18, 0x728085);
    colliders.push({ x, y: 20, w: 1.6, h: 9, bottom: 10.3, top: 10.9 });
  }
  const trench = Array.from({ length: 156 }, (_, i) => ({ x: 190 + i * 2, y: -105, h: floor(190 + i * 2, -105) }));
  ribbon(trench.map(p => ({ ...p, h: p.h + .015 })), 20, excavation);
  ribbon(trench.map(p => ({ ...p, h: p.h + .035 })), 16, asphalt);
  for (const y of [-115, -95]) {
    const v = [], indices = [];
    trench.forEach((p, i) => {
      v.push(p.x, p.h, -y, p.x, 0, -y);
      if (i) { const n = i * 2; indices.push(n - 2, n - 1, n, n - 1, n + 1, n); }
    });
    mesh(v, indices, concrete);
  }
  for (let x = 194; x < 500; x += 8) for (const y of [-115, -95]) {
    colliders.push({ x, y, w: 4, h: .12, bottom: Math.min(floor(x - 4, -105), floor(Math.min(500, x + 4), -105)) - .1, top: 0 });
  }
  // The upper roof is also a real drivable surface, distinct from the -8 m floor.
  block(345, -.6, -105, 170, 1.2, 20, 0x667c7d);
  for (const x of [260, 430]) {
    for (const y of [-116, -94]) block(x, -4, y, 2.2, 8, 2.2, 0x6f8285);
    block(x, -.6, -105, 2.4, 1.2, 24, 0x899b98);
  }
  for (let x = 270; x <= 420; x += 15) {
    for (const y of [-112, -98]) block(x, -1.3, y, 3.2, .12, .35, 0xa4e6e4, true);
  }
  for (const route of STACKED_ROUTES.filter(r => r.kind !== 'skyway' && r.kind !== 'tunnel')) {
    ribbon(route.points.map(([x, y]) => ({ x, y, h: .045 })), 18, asphalt);
  }
  for (const y of [20, -105]) {
    ribbon([{ x: 160, y, h: .035 }, { x: 190, y, h: .035 }], 18, asphalt);
    ribbon([{ x: 500, y, h: .035 }, { x: 530, y, h: .035 }], 18, asphalt);
  }
  const paint = [], paintIndices = [];
  function dash(x, y, heightAt) {
    const start = paint.length / 3;
    for (const dx of [-1.75, 0, 1.75]) paint.push(x + dx, heightAt(x + dx) + .065, -y - .09, x + dx, heightAt(x + dx) + .065, -y + .09);
    for (let i = 1; i < 3; i++) { const n = start + i * 2; paintIndices.push(n - 2, n - 1, n, n - 1, n + 1, n); }
  }
  for (let x = 195; x < 498; x += 9) { dash(x, 20, deck); dash(x, -105, xx => floor(xx, -105)); }
  mesh(paint, paintIndices, new THREE.MeshStandardMaterial({ color: 0xe5cf9f, side: THREE.DoubleSide }));
  function sign(text, x, y, h, color) {
    const canvas = document.createElement('canvas'); canvas.width = 768; canvas.height = 192;
    const c = canvas.getContext('2d'); c.fillStyle = '#142a31'; c.fillRect(0, 0, 768, 192);
    c.fillStyle = color; c.font = 'bold 64px sans-serif'; c.textAlign = 'center'; c.fillText(text, 384, 122);
    const map = new THREE.CanvasTexture(canvas); map.colorSpace = THREE.SRGBColorSpace;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(16, 4), new THREE.MeshStandardMaterial({ map, emissiveMap: map, emissive: 0xffffff, emissiveIntensity: .6, side: THREE.DoubleSide }));
    m.rotation.y = -Math.PI / 2; m.position.set(x, h + 4, -y); scene.add(m);
  }
  sign('SKYWAY / +12 m', 192, 35, 0, '#9fe6ed');
  sign('TUNNEL / -8 m', 246, -90, 0, '#d1b4f5');
  sign('LOWER ROAD', 364, -35, 0, '#c5dbd0');
  for (const batch of batches.values()) {
    const material = new THREE.MeshStandardMaterial({ color: batch.color, roughness: .85,
      emissive: batch.emissive ? batch.color : 0x000000, emissiveIntensity: batch.emissive ? 2 : 0 });
    const m = new THREE.InstancedMesh(box, material, batch.rows.length), pose = new THREE.Object3D();
    batch.rows.forEach((r, i) => {
      pose.position.set(r.x, r.h, -r.y); pose.scale.set(r.w, r.height, r.depth); pose.updateMatrix(); m.setMatrixAt(i, pose.matrix);
    });
    m.receiveShadow = true; scene.add(m);
  }
  const lamp = new THREE.PointLight(0xa8e3ed, 35, 22, 1.5); lamp.visible = false; scene.add(lamp);
  return { colliders, bridgeHeight: 12, tunnelDepth: 8, update(player) {
    lamp.visible = player.height < -3;
    if (lamp.visible) lamp.position.set(player.x + Math.cos(player.yaw) * 4, player.height + 3.8, -player.y - Math.sin(player.yaw) * 4);
  } };
}
