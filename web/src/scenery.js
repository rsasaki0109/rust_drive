import * as THREE from 'three';
import { BLOCKS, forestTrail, forestTrees } from './regions.js';

const box = new THREE.BoxGeometry(1, 1, 1);
const cylinder = new THREE.CylinderGeometry(1, 1, 1, 16);
const foliage = new THREE.IcosahedronGeometry(1, 1);
const material = color => new THREE.MeshStandardMaterial({ color, roughness: .8 });

function shape(scene, geometry, mat, x, elevation, y, width, height, depth) {
  const mesh = new THREE.Mesh(geometry, mat);
  mesh.position.set(x, elevation, -y); mesh.scale.set(width, height, depth);
  scene.add(mesh); return mesh;
}
function batch(scene, geometry, mat, rows) {
  const mesh = new THREE.InstancedMesh(geometry, mat, rows.length), transform = new THREE.Object3D();
  rows.forEach((r, i) => {
    transform.position.set(r.x, r.elevation, -r.y); transform.scale.set(r.w, r.h, r.d);
    transform.rotation.set(0, r.yaw || 0, 0); transform.updateMatrix();
    mesh.setMatrixAt(i, transform.matrix); mesh.setColorAt(i, new THREE.Color(r.color));
  });
  mesh.instanceMatrix.needsUpdate = true; mesh.receiveShadow = true; scene.add(mesh);
}
function trailSurface(scene, path, width, mat) {
  const positions = [], indices = [];
  path.forEach((p, i) => {
    const a = path[Math.max(0, i - 1)], b = path[Math.min(path.length - 1, i + 1)];
    const length = Math.hypot(b.x - a.x, b.y - a.y);
    const nx = -(b.y - a.y) / length * width / 2, ny = (b.x - a.x) / length * width / 2;
    positions.push(p.x + nx, .018, -p.y - ny, p.x - nx, .018, -p.y + ny);
    if (i) { const n = i * 2; indices.push(n - 2, n, n - 1, n - 1, n, n + 1); }
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setIndex(indices); geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(geometry, mat); mesh.receiveShadow = true; scene.add(mesh);
}
function sign(scene, label, color, x, y, height = 4, width = 12) {
  const canvas = document.createElement('canvas'); canvas.width = 512; canvas.height = 128;
  const ctx = canvas.getContext('2d'); ctx.fillStyle = '#12232b'; ctx.fillRect(0, 0, 512, 128);
  ctx.fillStyle = color; ctx.font = 'bold 42px sans-serif'; ctx.textAlign = 'center'; ctx.fillText(label, 256, 80);
  const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace;
  shape(scene, box, new THREE.MeshStandardMaterial({ map: texture, emissiveMap: texture, emissive: 0xffffff, emissiveIntensity: .5 }), x, height, y, width, 2.6, .15);
}

export function createDistrictScenery(scene) {
  const colliders = [], crates = [], details = [], trunks = [], crowns = [];
  let buildings = 0;
  const ground = { forest: material(0x496b4c), harbor: material(0x6f8589), factory: material(0x807566), plaza: material(0xa59d86) };
  const trail = new THREE.MeshStandardMaterial({ color: 0xbba67a, roughness: 1, side: THREE.DoubleSide });
  function solid(x, y, w, h, height, color) {
    crates.push({ x, y, elevation: height / 2, w, h: height, d: h, color });
    colliders.push({ x, y, w: w / 2, h: h / 2, top: height });
  }
  function tree(x, y, height, color = 0x6c986c) {
    trunks.push({ x, y, elevation: height / 2, w: .65, h: height, d: .65, color: 0x77604b });
    crowns.push({ x, y, elevation: height - .5, w: 3.1, h: 4.0, d: 3.1, color });
    colliders.push({ x, y, w: .325, h: .325, top: height });
  }
  for (const block of BLOCKS) {
    const { x: cx, y: cy, district } = block;
    if (district.id === 'downtown') continue;
    shape(scene, box, ground[district.id], cx, -.06, cy, 32, .12, 32).receiveShadow = true;
    if (district.id === 'forest') {
      const path = forestTrail(cx, cy); trailSurface(scene, path, 7.4, trail);
      for (const t of forestTrees(cx, cy)) tree(t.x, t.y, t.height, t.height % 2 ? 0x6c986c : 0x447e60);
      // Gate posts sit outside the driving corridor.
      for (const side of [-1, 1]) solid(cx - 15, cy + side * 7, .55, .55, 2.2, 0xd3c1a0);
      if (cy === 25) sign(scene, 'CANOPY PARK', district.color, cx, cy + 13, 3.5, 12);
    } else if (district.id === 'plaza') {
      shape(scene, cylinder, material(0xbdb293), cx, .012, cy, 11, .02, 11);
      const inlay = new THREE.Mesh(new THREE.RingGeometry(9.8, 10, 48), material(0x726e61));
      inlay.rotation.x = -Math.PI / 2; inlay.position.set(cx, .03, -cy); scene.add(inlay);
      // Furniture stays in the corners, leaving a clear 20 m diameter drift pad.
      const fx = cx + Math.sign(cx) * 12, fy = cy + Math.sign(cy) * 12;
      shape(scene, cylinder, material(0xd0c6ae), fx, .35, fy, 2.2, .7, 2.2);
      shape(scene, cylinder, new THREE.MeshStandardMaterial({ color: 0x79b9c4, metalness: .35, roughness: .15 }), fx, .73, fy, 1.9, .06, 1.9);
      colliders.push({ x: fx, y: fy, w: 2.2, h: 2.2, top: .8 });
      tree(cx - Math.sign(cx) * 12.5, cy + Math.sign(cy) * 12.5, 5.5, 0x8aa578);
      if (cx === 25 && cy === -25) sign(scene, 'DRIFT SQUARE', district.color, cx, cy - 14, 3.5, 13);
    } else if (district.id === 'harbor') {
      solid(cx, cy + 9, 24, 10, 7, 0x678991); buildings++;
      details.push({ x: cx, y: cy + 9, elevation: 7.12, w: 25, h: .25, d: 11, color: 0xadc2c2 });
      for (const side of [-1, 1]) {
        const x = cx + side * 7.5;
        solid(x, cy - 2, 6.2, 3, 5.2, side < 0 ? 0xb45b48 : 0x497f91);
        solid(x, cy - 7, 6.2, 3, 2.6, side < 0 ? 0xc8aa57 : 0x668d78);
        for (let i = -2; i <= 2; i++) details.push({ x: x + i * 1.05, y: cy - 3.54, elevation: 2.6, w: .10, h: 4.9, d: .06, color: 0x353f45 });
        solid(cx + side * 12.5, cy - 12, 1.2, 1.8, 18, 0xdcad53);
      }
      details.push({ x: cx, y: cy - 12, elevation: 18, w: 28, h: 1.3, d: 2, color: 0xe5b65e });
      details.push({ x: cx, y: cy - 18, elevation: 19.2, w: 1.2, h: 1, d: 18, color: 0xe5b65e });
      details.push({ x: cx, y: cy - 24, elevation: 15.4, w: .10, h: 6, d: .10, color: 0x273f49 });
      sign(scene, 'HARBOR BAY', district.color, cx, cy + 3.8, 4.2, 16);
    } else if (district.id === 'factory') {
      solid(cx - 2, cy - 7, 19, 12, 8, 0x9b7864); buildings++;
      details.push({ x: cx - 2, y: cy - 7, elevation: 8.3, w: 20, h: .6, d: 13, color: 0x5b6970 });
      for (const side of [-1, 1]) {
        const x = cx + side * 8;
        shape(scene, cylinder, material(0xb4bfbb), x, 4.5, cy + 8, 3.5, 9, 3.5);
        colliders.push({ x, y: cy + 8, w: 3.5, h: 3.5, top: 9.4 });
        shape(scene, cylinder, material(0x6c7f7e), x, 9.1, cy + 8, 3.6, .3, 3.6);
      }
      solid(cx + 12, cy - 10, 2.3, 2.3, 23, 0xb99e88);
      for (let i = 0; i < 3; i++) details.push({ x: cx + 12, y: cy - 10, elevation: 15 + i * 3, w: 2.4, h: 1, d: 2.4, color: 0xc76651 });
      details.push({ x: cx, y: cy + 8, elevation: 10.5, w: 23, h: .7, d: .7, color: 0xdaa75e });
      sign(scene, 'IRONWORKS', district.color, cx - 2, cy - .85, 5, 15);
    }
  }
  // Water, ships and distant hills are scenery outside the bounded driving map.
  shape(scene, box, new THREE.MeshStandardMaterial({ color: 0x396d80, metalness: .35, roughness: .24 }), 85, -.13, -255, 530, .12, 130);
  shape(scene, box, material(0x879598), 85, -.22, -188, 370, .4, 7);
  for (const x of [40, 135]) {
    shape(scene, box, material(0x354b57), x, 1.4, -221, 38, 4, 12);
    shape(scene, box, material(0xd0d4c4), x - 12, 5, -221, 9, 5, 10);
    shape(scene, box, material(0x9c4f42), x + 5, 4.8, -221, 18, 2.5, 9);
  }
  for (let i = 0; i < 7; i++) {
    const hill = shape(scene, new THREE.ConeGeometry(1, 1, 7), material(i % 2 ? 0x426552 : 0x50715d), -220 - (i % 3) * 22, 12 + i % 3 * 4, 10 + i * 38, 35 + i % 2 * 15, 35 + i % 3 * 10, 45);
    hill.rotation.y = i;
  }
  batch(scene, box, material(0xffffff), crates);
  batch(scene, box, material(0xffffff), details);
  batch(scene, box, material(0xffffff), trunks);
  batch(scene, foliage, material(0xffffff), crowns);
  return { colliders, buildingCount: buildings };
}
