import * as THREE from 'three';

export const RIDGE_ROUTE = Object.freeze([
  { x: 160, y: 150 }, { x: 380, y: 150 },
  { x: 460, y: 150 }, { x: 530, y: 150 },
  { x: 530, y: 70 }, { x: 160, y: 70 },
]);
export const WORLD_BOUNDS = Object.freeze({ minX: -184, maxX: 560, minY: -184, maxY: 235 });

// Geometry samples the same Rust height function used for tyre support/flight.
export function createRidge(scene, drive) {
  const surface = (x, y) => drive.drive_surface(x, y, 0);
  const rock = new THREE.MeshStandardMaterial({ color: 0x97755a, roughness: 1, side: THREE.DoubleSide });
  const top = new THREE.MeshStandardMaterial({ color: 0x8b9170, roughness: 1, side: THREE.DoubleSide });
  const road = new THREE.MeshStandardMaterial({ color: 0x434b51, roughness: .95, side: THREE.DoubleSide });
  const box = new THREE.BoxGeometry(1, 1, 1);
  function cuboid(x, height, y, w, h, d, color) {
    const m = new THREE.Mesh(box, new THREE.MeshStandardMaterial({ color, roughness: .85 }));
    m.position.set(x, height, -y); m.scale.set(w, h, d); m.receiveShadow = true; scene.add(m); return m;
  }
  function faces(vertices, indices, material) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    g.setIndex(indices); g.computeVertexNormals();
    const m = new THREE.Mesh(g, material); m.receiveShadow = true; scene.add(m);
  }
  cuboid(380, -.33, 100, 1600, .3, 1200, 0x53635b);
  cuboid(370, -.08, 135, 390, .16, 200, 0x857d64);
  const xs = Array.from({ length: 91 }, (_, i) => 200 + i * 2);
  const height = x => surface(Math.min(x, 379.999999), 150);
  const vertices = [], indices = [], walls = [], wallIndices = [];
  xs.forEach((x, i) => {
    const h = height(x);
    vertices.push(x, h, -110, x, h, -190);
    if (i) { const n = i * 2; indices.push(n - 2, n - 1, n, n - 1, n + 1, n); }
    for (const y of [110, 190]) {
      const n = walls.length / 3; walls.push(x, 0, -y, x, h, -y);
      if (i) { const p = n - 4; wallIndices.push(p, n, p + 1, p + 1, n, n + 1); }
    }
  });
  faces(vertices, indices, top); faces(walls, wallIndices, rock);
  faces([380, 0, -110, 380, 49, -110, 380, 0, -190, 380, 49, -190], [0, 1, 2, 1, 3, 2], rock);
  // Continuous climb, then a deliberately unsupported cliff edge.
  function strip(points, width, mat, offset = .035) {
    const v = [], ii = [];
    points.forEach((p, i) => {
      const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
      const length = Math.hypot(b.x - a.x, b.y - a.y), nx = -(b.y - a.y) / length * width / 2, ny = (b.x - a.x) / length * width / 2;
      const h = p.h ?? surface(p.x, p.y);
      v.push(p.x + nx, h + offset, -p.y - ny, p.x - nx, h + offset, -p.y + ny);
      if (i) { const n = i * 2; ii.push(n - 2, n - 1, n, n - 1, n + 1, n); }
    });
    faces(v, ii, mat);
  }
  strip(Array.from({ length: 116 }, (_, i) => { const x = 150 + i * 2; return { x, y: 150, h: x >= 200 ? height(x) : 0 }; }), 16, road);
  strip([{ x: 400, y: 150 }, { x: 530, y: 150 }, { x: 530, y: 70 }, { x: 150, y: 70 }], 18, road);
  // Broad, clear basin under the flight path; the return road stays at city grade.
  cuboid(475, 0, 150, 150, .05, 115, 0xa59c7d);
  for (let x = 180; x <= 372; x += 8) {
    const h = x >= 200 ? height(x) : 0;
    cuboid(x, h + .055, 150, 3.8, .04, .16, 0xe5d0a0);
  }
  for (const x of [220, 260, 300, 340, 376]) for (const side of [-1, 1]) {
    const h = height(x);
    cuboid(x, h + .6, 150 + side * 9.5, .3, 1.2, .3, 0xe5b25b);
  }
  for (let x = 410; x <= 540; x += 10) {
    cuboid(x, .035, 120, 6, .04, .25, 0xe7c275);
    cuboid(x, .035, 180, 6, .04, .25, 0xe7c275);
  }
  function sign(text, x, y, elevation) {
    const canvas = document.createElement('canvas'); canvas.width = 768; canvas.height = 192;
    const c = canvas.getContext('2d'); c.fillStyle = '#20303a'; c.fillRect(0, 0, 768, 192);
    c.fillStyle = '#ffca84'; c.font = 'bold 62px sans-serif'; c.textAlign = 'center'; c.fillText(text, 384, 121);
    const map = new THREE.CanvasTexture(canvas); map.colorSpace = THREE.SRGBColorSpace;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(20, 5), new THREE.MeshStandardMaterial({ map, emissiveMap: map, emissive: 0xffffff, emissiveIntensity: .45, side: THREE.DoubleSide }));
    m.rotation.y = -Math.PI / 2; m.position.set(x, elevation + 6, -y); scene.add(m);
  }
  sign('SKYLINE RIDGE', 197, 165, 0);
  sign('CLIFF DIVE / 49 m', 368, 165, 48);
  sign('LANDING ZONE', 472, 192, 0);
  return { climbHeight: 49 };
}
