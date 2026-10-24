import * as THREE from 'three';
import { FACILITIES, STATION_PATH, RAIL_Y } from './city-life.js';

const box = new THREE.BoxGeometry(1, 1, 1);
const material = color => new THREE.MeshStandardMaterial({ color, roughness: .75 });
function cube(parent, mat, x, y, z, w, h, d) {
  const mesh = new THREE.Mesh(box, mat); mesh.position.set(x, y, z); mesh.scale.set(w, h, d);
  mesh.castShadow = true; mesh.receiveShadow = true; parent.add(mesh); return mesh;
}
function sign(parent, text, color, x, y, z, width, height = 1.4) {
  const canvas = document.createElement('canvas'); canvas.width = 768; canvas.height = 128;
  const c = canvas.getContext('2d'); c.fillStyle = '#14252f'; c.fillRect(0, 0, 768, 128);
  c.font = 'bold 60px sans-serif'; c.textAlign = 'center'; c.fillStyle = color; c.fillText(text, 384, 87);
  const map = new THREE.CanvasTexture(canvas); map.colorSpace = THREE.SRGBColorSpace;
  return cube(parent, new THREE.MeshStandardMaterial({ map, emissiveMap: map, emissive: 0xffffff, emissiveIntensity: .8 }),
    x, y, z, width, height, .08);
}

export function createFleetPlaces(scene) {
  const doors = {}, glow = [], lights = [], glass = new THREE.MeshStandardMaterial({
    color: 0x8fc4cf, transparent: true, opacity: .35, roughness: .22, metalness: .1, side: THREE.DoubleSide });
  const frame = material(0x223943), pavement = material(0x9ca89d);
  function building(f, width, depth, height, color, title, tint) {
    const root = new THREE.Group(); root.name = f.id; root.position.set(f.door.x, 0, -f.door.y);
    root.rotation.y = f.outward === 1 ? Math.PI : 0; scene.add(root);
    cube(root, pavement, 0, .07, -depth / 2, width + .6, .14, depth + .6);
    // The ground-floor lobby is hollow, with an actual 3 m doorway.
    cube(root, material(color), 0, (height + 3.3) / 2, -depth / 2, width, height - 3.3, depth);
    for (const side of [-1, 1]) cube(root, material(color), side * width / 2, 1.65, -depth / 2, .22, 3.3, depth);
    cube(root, frame, 0, 1.65, -depth, width, 3.3, .2);
    const paneWidth = width / 2 - 1.5;
    for (const side of [-1, 1]) {
      cube(root, glass, side * (1.5 + paneWidth / 2), 1.5, 0, paneWidth, 2.9, .08);
      cube(root, frame, side * 1.5, 1.55, .03, .10, 3.1, .13);
    }
    const panels = [-1, 1].map(side => {
      const panel = new THREE.Group(); panel.position.set(side * .7, 1.4, .08);
      cube(panel, glass, 0, 0, 0, 1.4, 2.8, .08);
      for (const edge of [-1, 1]) cube(panel, frame, edge * .7, 0, .02, .06, 2.8, .1);
      root.add(panel); return { mesh: panel, side };
    });
    doors[f.id] = panels;
    cube(root, frame, -3.5, .55, -3.7, 2.7, 1.1, 1.1);
    cube(root, material(0xddc69a), 3.2, .45, -3.1, 2.3, .9, .85);
    cube(root, frame, 0, 3.15, .65, 4.3, .15, 1.5);
    for (let level = 5; level < height; level += 3) for (let x = -width / 2 + 1.5; x < width / 2 - 1; x += 2.4) {
      const mat = new THREE.MeshStandardMaterial({ color: 0x8fb6c2, emissive: tint, emissiveIntensity: .1 });
      glow.push(mat); cube(root, mat, x, level, .03, 1.5, 1.65, .06);
    }
    sign(root, title, '#' + tint.toString(16).padStart(6, '0'), 0, 4, .08, width * .91, 1.5);
    const light = new THREE.PointLight(tint, 40, 18, 2); light.position.set(0, 2.8, 1.5); root.add(light); lights.push(light);
    return root;
  }
  building(FACILITIES.office, 18, 20, 14, 0x647f8c, 'CIVIC OFFICES', 0xa9e6ef);
  const club = building(FACILITIES.nightlife, 12, 24, 13, 0x42475f, 'AFTER HOURS / CLUB', 0xff79d7);
  const neon = new THREE.MeshStandardMaterial({ color: 0xeb79d0, emissive: 0xf443c0, emissiveIntensity: .2 });
  glow.push(neon);
  for (const side of [-1, 1]) cube(club, neon, side * 5.8, 6.3, .14, .12, 8.5, .14);
  const cafe = new THREE.Group(); cafe.position.set(-17, 0, -64); scene.add(cafe);
  cube(cafe, material(0x6b6875), 0, 4, -10, 9, 8, 20);
  cube(cafe, glass, 0, 1.5, .04, 8.2, 2.5, .1);
  sign(cafe, 'LATE CAFE / 24H', '#84edda', 0, 3.2, .10, 8.5, 1.3);
  const awning = material(0x578f8d);
  cube(cafe, awning, 0, 3, .75, 9.5, .16, 1.5);
  for (const y of [61.3, 64, 66.7]) {
    const x = -11.7;
    cube(scene, frame, x, .45, -y, .8, .9, .7);
    cube(scene, material(0xd1b98e), x, .98, -y, 1.35, .12, 1.25);
  }

  const depot = new THREE.Group(); depot.position.set(-125, 0, 170); scene.add(depot);
  cube(depot, material(0x5d737a), 0, 2.5, 0, 31, 5, 15);
  sign(depot, 'CITY TAXI / DEPOT', '#f0ce83', 0, 3, -7.55, 25, 1.4);

  // North Line: a real ground railway and raised pedestrian access across the avenue.
  cube(scene, material(0x6c7e75), 0, -.18, -192, 540, .28, 40);
  cube(scene, material(0x535e64), 0, .08, -RAIL_Y, 520, .16, 5);
  const railMat = material(0xabb8bd), sleeper = material(0x605c59);
  for (const side of [-1, 1]) cube(scene, railMat, 0, .26, -RAIL_Y + side * .9, 520, .12, .10);
  for (let x = -255; x <= 255; x += 3) cube(scene, sleeper, x, .17, -RAIL_Y, .35, .15, 2.5);
  cube(scene, pavement, 25, .48, -181, 39, .96, 4.8);
  cube(scene, material(0xd6c789), 25, 1.005, -183.1, 39, .03, .22);
  cube(scene, frame, 8, 4.0, -179.2, 11, .2, 4.2);
  for (const x of [4, 12]) cube(scene, frame, x, 2.5, -178, .18, 3, .18);
  for (const x of [12, 38]) {
    const light = new THREE.PointLight(0xffe5bb, 40, 23, 2); light.position.set(x, 4.5, -180); scene.add(light); lights.push(light);
    cube(scene, frame, x, 2.7, -178, .12, 5.4, .12);
    cube(scene, new THREE.MeshBasicMaterial({ color: 0xffeac3 }), x, 5.2, -179.3, 2.0, .10, .35);
  }
  sign(scene, 'NORTH STATION / PLATFORM 1', '#9fe7e8', 25, 3.1, -176.5, 22, 1.1);
  for (let i = 1; i < STATION_PATH.length; i++) {
    const a = STATION_PATH[i - 1], b = STATION_PATH[i], dx = b.x - a.x, dy = b.y - a.y, length = Math.hypot(dx, dy);
    const nx = dy / length * 1.15, ny = -dx / length * 1.15;
    const positions = [a.x + nx, a.height, -a.y - ny, a.x - nx, a.height, -a.y + ny,
      b.x + nx, b.height, -b.y - ny, b.x - nx, b.height, -b.y + ny];
    const geometry = new THREE.BufferGeometry(); geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.setIndex([0, 1, 2, 1, 3, 2]); geometry.computeVertexNormals();
    const path = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0xc1bbb0, side: THREE.DoubleSide })); path.receiveShadow = true; scene.add(path);
    for (const side of [-1, 1]) {
      const line = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(a.x + nx * side, a.height + 1, -a.y - ny * side),
        new THREE.Vector3(b.x + nx * side, b.height + 1, -b.y - ny * side)]);
      scene.add(new THREE.Line(line, new THREE.LineBasicMaterial({ color: 0x86999b })));
    }
  }
  for (const y of [138, 162]) cube(scene, pavement, 39.2, 2.4, -y, .42, 4.8, .42);
  sign(scene, 'NORTH STATION', '#a1e5dd', 39.2, 2.4, -111.3, 5.5, .85);

  const trainRoot = new THREE.Group(); trainRoot.visible = false; scene.add(trainRoot);
  const trainDoors = [], trainPaint = material(0xdfebe5), blue = material(0x458caa), dark = material(0x23353d);
  for (const center of [-9, 9]) {
    const doorX = center < 0 ? -8 : 8;
    cube(trainRoot, dark, center, .88, 0, 16.8, .20, 3.1);
    cube(trainRoot, trainPaint, center, 3.8, 0, 16.8, .3, 3.2);
    cube(trainRoot, trainPaint, center, 2.1, -1.5, 16.8, 2.4, .10);
    cube(trainRoot, glass, center, 2.65, -1.56, 15.8, 1.2, .04);
    for (const [left, right] of [[center - 8.4, doorX - .95], [doorX + .95, center + 8.4]]) {
      cube(trainRoot, trainPaint, (left + right) / 2, 1.3, 1.52, right - left, .8, .12);
      cube(trainRoot, glass, (left + right) / 2, 2.65, 1.53, right - left, 1.7, .08);
      cube(trainRoot, blue, (left + right) / 2, 1.9, 1.59, right - left, .16, .07);
    }
    for (const end of [-1, 1]) {
      cube(trainRoot, trainPaint, center + end * 8.4, 2.15, 0, .16, 2.9, 3.05);
      cube(trainRoot, dark, center + end * 5.5, .5, 0, 2.2, .65, 2.7);
    }
    for (const side of [-1, 1]) {
      const panel = cube(trainRoot, glass, doorX + side * .475, 2.25, 1.56, .95, 2.7, .10);
      trainDoors.push({ panel, side, closedX: panel.position.x });
    }
  }
  const head = new THREE.MeshBasicMaterial({ color: 0xffedb1 });
  for (const side of [-1, 1]) cube(trainRoot, head, 17.5, 1.5, side * 1.1, .05, .25, .35);
  const lineSign = sign(trainRoot, 'NORTH LINE', '#b9eedf', 8, 3.5, 1.62, 5.5, .45);
  const lastSign = sign(trainRoot, 'LAST TRAIN', '#ffd68c', 8, 3.5, 1.63, 5.5, .45);
  return {
    doors, trainRoot,
    update(snapshot) {
      for (const [id, panels] of Object.entries(doors)) for (const { mesh, side } of panels)
        mesh.position.x = side * (.7 + snapshot.doors[id].open * 1.5);
      const night = 1 - snapshot.clock.daylight;
      for (const mat of glow) mat.emissiveIntensity = .12 + night * 2.5;
      for (const light of lights) light.intensity = 4 + night * 55;
      const train = snapshot.train; trainRoot.visible = Boolean(train);
      if (train) {
        trainRoot.position.set(train.x, 0, -RAIL_Y);
        for (const d of trainDoors) d.panel.position.x = d.closedX + d.side * train.doorOpen;
        lineSign.visible = !train.lastService; lastSign.visible = train.lastService;
      }
    },
  };
}
