import * as THREE from 'three';
import './fleet.css';
import { createCity, Traffic, STREETS, phase } from './city.js';
import { createRidge } from './ridge.js';
import { createStructures } from './structures.js';
import { createUrbanDistrict } from './urban-city.js';
import { TaxiFleet, FLEET_STOPS, PURPOSES } from './fleet.js';

const COLORS = { idle: '#aab9c4', pickup: '#ffca62', boarding: '#ffffff', occupied: '#63decb', alighting: '#c4f19a' };
const LABELS = { walking: 'WALKING TO PICKUP', waiting: 'WAITING', boarding: 'BOARDING', riding: 'ON THE WAY',
  alighting: 'GETTING OUT', leaving: 'WALKING TO DESTINATION', arrived: 'ARRIVED', cancelled: 'LEFT THE QUEUE' };
const PURPOSE = new Map(PURPOSES.map(p => [p.id, p]));
const observer = { x: -1000, y: -1000, height: 0, yaw: 0 };
const box = new THREE.BoxGeometry(1, 1, 1);
function block(parent, material, x, y, z, sx, sy, sz) {
  const mesh = new THREE.Mesh(box, material); mesh.position.set(x, y, z); mesh.scale.set(sx, sy, sz); mesh.castShadow = true; parent.add(mesh); return mesh;
}
function label(text, color, width = 256) {
  const canvas = document.createElement('canvas'); canvas.width = width; canvas.height = 64;
  const context = canvas.getContext('2d'); context.fillStyle = '#152c35'; context.fillRect(0, 0, width, 64);
  context.fillStyle = color; context.textAlign = 'center'; context.font = 'bold 26px sans-serif'; context.fillText(text, width / 2, 43);
  const map = new THREE.CanvasTexture(canvas); map.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map, depthTest: false }));
  sprite.scale.set(width / 32, 2, 1); return sprite;
}
function pedestrian(p, index) {
  const group = new THREE.Group(), purpose = PURPOSE.get(p.purpose);
  const cloth = new THREE.MeshStandardMaterial({ color: purpose.color, roughness: .9 });
  const skin = new THREE.MeshStandardMaterial({ color: [0xe8b88e, 0xb67e55, 0x875a40, 0xd49e79][index % 4] });
  const dark = new THREE.MeshStandardMaterial({ color: 0x253d4b });
  block(group, cloth, 0, 1.08, 0, .52, .7, .38);
  const head = new THREE.Mesh(new THREE.SphereGeometry(.21, 8, 6), skin); head.position.y = 1.66; group.add(head);
  const legs = [-1, 1].map(side => block(group, dark, 0, .39, side * .16, .22, .72, .20));
  const arms = [-1, 1].map(side => block(group, cloth, 0, 1.04, side * .34, .17, .65, .17));
  if (purpose.item === 'suitcase') {
    block(group, dark, -.48, .30, .36, .34, .5, .28); block(group, skin, -.31, .68, .36, .045, .4, .045);
  } else if (purpose.item === 'backpack') block(group, dark, -.30, 1.12, 0, .22, .5, .38);
  else block(group, dark, 0, .68, .45, .30, .35, .16);
  const ring = new THREE.Mesh(new THREE.RingGeometry(.45, .72, 16), new THREE.MeshBasicMaterial({ color: purpose.color, side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = .08; group.add(ring);
  group.userData = { legs, arms }; group.scale.setScalar(1.15); return group;
}

export async function bootFleet() {
  document.title = 'RustDriving · People & Taxi Fleet';
  document.getElementById('game').hidden = true;
  document.body.insertAdjacentHTML('beforeend', '<main id="fleet-app">' +
    '<header class="fleet-header"><a class="fleet-brand" href="./">R<span>∕</span></a><div><p>RUSTDRIVING / CITY LAB</p><h1>People &amp; taxi fleet<span>01</span></h1></div>' +
    '<div class="fleet-clock"><i></i><b id="fleet-clock">08:00:00</b><small>SIMULATION TIME</small></div><a class="fleet-drive" href="./">DRIVE ↗</a><button id="fleet-play" disabled>LOADING…</button></header>' +
    '<div class="fleet-layout"><aside class="fleet-sidebar"><p class="fleet-eyebrow">EVERY TRIP STARTS WITH SOMEONE</p><h2>A city full<br>of reasons to go.</h2><p class="fleet-description">People walk to the curb, request a ride<br>and decide how long they can wait.</p>' +
    '<div class="fleet-metrics" aria-label="Live trip metrics"><div><b id="fleet-waiting">0</b><span>WAITING</span></div><div><b id="fleet-riding">0</b><span>RIDING</span></div><div><b id="fleet-served">0</b><span>DELIVERED</span></div><div><b id="fleet-cancelled">0</b><span>GAVE UP</span></div></div>' +
    '<div class="fleet-section-title">PEOPLE, NOT JUST REQUESTS <span id="fleet-demand">0 TRIPS</span></div><div id="fleet-people" aria-label="Individual travel requests"></div>' +
    '<div class="fleet-purpose-key">' + PURPOSES.map(p => '<span><i style="background:' + p.color + '"></i>' + p.label + '</span>').join('') + '</div><p class="fleet-scope">Authored demand · 8 taxis · 1 rider per taxi<br>Individual walking, waiting &amp; boarding.<br>Arcade simulation, not a calibrated city model.</p></aside>' +
    '<section id="fleet-stage" aria-label="Live taxi and pedestrian city simulation"><canvas id="fleet-world" aria-label="People walking to taxis on a city street network"></canvas>' +
    '<div class="fleet-scene-heading"><span>LIVE CITY / DISPATCH</span><b>Eight taxis. Many different lives.</b></div>' +
    '<div class="fleet-start-hint" id="fleet-hint">Press START to watch people make their way.</div>' +
    '<div class="fleet-event" id="fleet-event" role="status"></div><div class="fleet-bottom"><span><i style="background:#ffca62"></i>PICKUP</span><span><i style="background:#63decb"></i>OCCUPIED</span><span><i style="background:#c4f19a"></i>DROPOFF</span><span><i style="background:#aab9c4"></i>AVAILABLE</span></div>' +
    '<div class="fleet-camera"><button id="fleet-overview" aria-pressed="true">CITY VIEW</button><button id="fleet-follow" aria-pressed="false">FOLLOW TAXI</button></div></section></div></main>');
  const $ = id => document.getElementById(id), stage = $('fleet-stage');
  let renderer;
  try {
    const response = await fetch(import.meta.env.BASE_URL + 'drive.wasm', { cache: 'no-cache' });
    if (!response.ok) throw new Error('Physics download failed: ' + response.status);
    const drive = (await WebAssembly.instantiate(await response.arrayBuffer(), {})).instance.exports;
    const scene = new THREE.Scene(); scene.background = new THREE.Color(0xcbd9dd); scene.fog = new THREE.Fog(0xcbd9dd, 260, 620);
    const camera = new THREE.PerspectiveCamera(48, 1, .2, 850);
    camera.position.set(16, 153, 105); camera.lookAt(0, 0, -48);
    renderer = new THREE.WebGLRenderer({ canvas: $('fleet-world'), antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.18;
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    scene.add(new THREE.HemisphereLight(0xdcecf6, 0x66765b, 2.5));
    const sun = new THREE.DirectionalLight(0xffefd5, 3); sun.position.set(-100, 180, 40); sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048); sun.shadow.camera.left = sun.shadow.camera.bottom = -135;
    sun.shadow.camera.right = sun.shadow.camera.top = 135; sun.shadow.camera.far = 450; sun.shadow.bias = -.001;
    sun.target.position.set(0, 0, -50); scene.add(sun, sun.target);
    const city = createCity(scene); createRidge(scene, drive);
    const structures = createStructures(scene, drive), urban = createUrbanDistrict(scene, drive, phase);
    const traffic = new Traffic(scene, { drive, urban: true, cityCars: 14 });
    const cars = FLEET_STOPS.map((stop, i) => {
      const car = traffic.spawn(false, 0xffc34b, { ...stop.from, to: stop.to, advance: 10 + i, variant: 'taxi', serviceStop: stop });
      car.desired = 8 + i % 3; return car;
    });
    traffic.update(1 / 60, 0, observer);
    const fleet = new TaxiFleet(traffic.graph, cars);
    const people = fleet.people.map((p, i) => { const mesh = pedestrian(p, i); mesh.visible = false; scene.add(mesh); return mesh; });
    const cabVisuals = fleet.taxis.map(t => {
      const ring = new THREE.Mesh(new THREE.RingGeometry(2.15, 2.6, 24), new THREE.MeshBasicMaterial({ color: COLORS.idle, side: THREE.DoubleSide }));
      ring.rotation.x = -Math.PI / 2; scene.add(ring);
      const tag = label(t.id, '#ffdb89', 96); tag.scale.set(3.8, 2.5, 1); scene.add(tag); return { ring, tag };
    });
    const stopMaterial = new THREE.MeshStandardMaterial({ color: 0xffca62 });
    for (const stop of FLEET_STOPS) {
      const level = drive.drive_surface(stop.curb.x, stop.curb.y, 0), group = new THREE.Group();
      group.position.set(stop.curb.x + stop.normal.x * .7, level, -stop.curb.y - stop.normal.y * .7);
      block(group, stopMaterial, 0, 1.3, 0, .10, 2.6, .10);
      const sign = label(stop.label, '#ffdc88'); sign.position.y = 4.4; group.add(sign); scene.add(group);
      const bay = new THREE.Mesh(new THREE.RingGeometry(2.9, 3.05, 32), new THREE.MeshBasicMaterial({ color: 0xb09d61, side: THREE.DoubleSide }));
      bay.rotation.x = -Math.PI / 2; bay.position.set(stop.x, level + .025, -stop.y); scene.add(bay);
    }
    let paused = true, following = false, selected = fleet.taxis[0].id, previous = performance.now(), accumulator = 0, hudAt = -1;
    const resize = () => { const w = stage.clientWidth, h = stage.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); };
    resize(); addEventListener('resize', resize);
    const setPaused = value => {
      paused = value; accumulator = 0; $('fleet-play').textContent = paused ? '▶ START' : 'Ⅱ PAUSE';
      $('fleet-play').setAttribute('aria-pressed', String(!paused)); $('fleet-hint').hidden = !paused;
      $('fleet-hint').textContent = fleet.time ? 'Paused · press START to continue.' : 'Press START to watch people make their way.';
    };
    $('fleet-play').disabled = false; setPaused(true);
    $('fleet-play').addEventListener('click', () => setPaused(!paused));
    $('fleet-overview').addEventListener('click', () => { following = false; $('fleet-overview').setAttribute('aria-pressed', 'true'); $('fleet-follow').setAttribute('aria-pressed', 'false'); });
    $('fleet-follow').addEventListener('click', () => { following = true; $('fleet-overview').setAttribute('aria-pressed', 'false'); $('fleet-follow').setAttribute('aria-pressed', 'true'); });
    $('fleet-people').addEventListener('click', event => { const id = event.target.closest('[data-taxi]')?.dataset.taxi; if (id) selected = id; });
    document.addEventListener('visibilitychange', () => { if (document.hidden) setPaused(true); });
    addEventListener('keydown', event => { if (event.code === 'Escape') setPaused(!paused); });
    const project = mesh => { const p = mesh.position.clone().project(camera); return mesh.visible && p.z > -1 && p.z < 1 && Math.abs(p.x) < .96 && Math.abs(p.y) < .96; };
    window.rustDrivingFleet = Object.freeze({ snapshot: () => ({
      ...fleet.snapshot(), ready: true, paused, cameraMode: following ? 'follow' : 'city',
      physics: 'rust-wasm terrain + arcade traffic', ambientCars: traffic.cars.length - cars.length,
      visiblePeople: people.filter(project).length, visibleTaxis: cars.filter(c => project(c.mesh)).length,
      bodyColors: PURPOSES.map(p => p.color),
    }) });
    function hud() {
      const snapshot = fleet.snapshot(), m = snapshot.metrics;
      $('fleet-waiting').textContent = m.waiting; $('fleet-riding').textContent = m.riding;
      $('fleet-served').textContent = m.served; $('fleet-cancelled').textContent = m.cancelled;
      $('fleet-demand').textContent = m.requested + ' TRIPS';
      $('fleet-clock').textContent = '08:' + String(Math.floor(fleet.time / 60)).padStart(2, '0') + ':' + String(Math.floor(fleet.time % 60)).padStart(2, '0');
      const rank = { boarding: 0, alighting: 1, walking: 2, waiting: 3, riding: 4, leaving: 5 };
      const active = fleet.people.filter(p => rank[p.state] !== undefined).sort((a, b) => rank[a.state] - rank[b.state] || b.release - a.release).slice(0, 3);
      $('fleet-people').innerHTML = active.length ? active.map(p => {
        const purpose = PURPOSE.get(p.purpose), elapsed = Math.max(0, fleet.time - p.release);
        return '<button class="fleet-person" data-taxi="' + (p.taxi ?? '') + '" style="--purpose:' + purpose.color + '">' +
          '<div><i></i><b>' + p.id + ' / ' + purpose.label + '</b><span>' + (p.taxi ?? '…') + '</span></div>' +
          '<p>' + p.origin.label + ' <span>→</span> ' + p.destination.label + '</p><small>' + LABELS[p.state] + '<b>' + Math.floor(elapsed) + 's</b></small>' +
          '<div class="fleet-patience"><i style="width:' + (100 * Math.min(1, elapsed / p.patience)).toFixed(1) + '%"></i></div></button>';
      }).join('') : '<p class="fleet-empty">' + (m.requested ? 'The city is between journeys.' : 'The morning is about to begin.') + '</p>';
      const event = fleet.events.filter(e => ['board', 'complete', 'cancel', 'request'].includes(e.type)).at(-1);
      const verbs = { board: 'is boarding', complete: 'arrived with', cancel: 'stopped waiting', request: 'requested a ride' };
      $('fleet-event').textContent = event ? event.person + ' · ' + PURPOSE.get(event.purpose).label + ' · ' + verbs[event.type] + (event.taxi ? ' ' + event.taxi : '') : 'Walking → waiting → pickup → ride → arrival';
    }
    function frame(now) {
      const dt = Math.min(.05, Math.max(0, (now - previous) / 1000)); previous = now;
      if (!paused) {
        accumulator += dt;
        while (accumulator >= 1 / 60) { fleet.update(1 / 60); traffic.update(1 / 60, fleet.time, observer); accumulator -= 1 / 60; }
      }
      fleet.people.forEach((p, i) => {
        const mesh = people[i]; mesh.visible = !['scheduled', 'riding', 'arrived'].includes(p.state) && !(p.state === 'cancelled' && fleet.time - p.finishedAt > 6);
        if (!mesh.visible) return;
        mesh.position.set(p.x, drive.drive_surface(p.x, p.y, 0) + .08, -p.y); mesh.rotation.y = p.yaw;
        const walking = ['walking', 'boarding', 'alighting', 'leaving', 'cancelled'].includes(p.state), swing = walking ? Math.sin(p.walked * 5) * .38 : 0;
        mesh.userData.legs.forEach((leg, j) => { leg.rotation.z = swing * (j ? 1 : -1); });
        mesh.userData.arms.forEach((arm, j) => { arm.rotation.z = swing * (j ? -1 : 1); });
      });
      fleet.taxis.forEach((t, i) => {
        const visual = cabVisuals[i], level = t.car.height ?? 0;
        visual.ring.position.set(t.car.x, level + .10, -t.car.y); visual.ring.material.color.set(COLORS[t.state]);
        visual.tag.position.set(t.car.x, level + 4, -t.car.y);
      });
      const taxi = fleet.taxis.find(t => t.id === selected) ?? fleet.taxis[0], level = taxi.car.height ?? 0;
      const destination = following ? new THREE.Vector3(taxi.car.x - 22, level + 39, -taxi.car.y + 32) : new THREE.Vector3(16, 153, 105);
      const target = following ? new THREE.Vector3(taxi.car.x, level, -taxi.car.y) : new THREE.Vector3(0, 0, -48);
      camera.position.lerp(destination, 1 - Math.exp(-dt * 3)); camera.lookAt(target);
      city.update(fleet.time); structures.update(observer); urban.update(fleet.time, paused ? 0 : dt);
      if (fleet.time - hudAt >= .25 || hudAt < 0) { hud(); hudAt = fleet.time; }
      renderer.render(scene, camera); requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
  } catch (error) {
    console.error(error); $('fleet-hint').hidden = false; $('fleet-hint').textContent = 'Unable to start. Reload in a browser with WebGL 2 and WebAssembly.';
    $('fleet-play').textContent = 'RELOAD'; $('fleet-play').disabled = false; $('fleet-play').onclick = () => location.reload();
  }
}
