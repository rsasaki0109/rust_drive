import * as THREE from 'three';
import './style.css';
import { createRidge, WORLD_BOUNDS, RIDGE_ROUTE } from './ridge.js';
import { DISTRICTS, BLOCKS, districtAt, forestTrail } from './regions.js';
import { createStructures, ROAD_STARTS, STACKED_ROUTES } from './structures.js';
import { createUrbanDistrict } from './urban-city.js';
import { URBAN_START, STATION_START, HILL_STREETS, LOWER_RING } from './urban-roads.js';
import { fractureContact } from './guardrails.js';
import { createCity, makeCar, Traffic, STREETS, nearestStreet, wrap, phase } from './city.js';

const $ = id => document.getElementById(id);
const state = { ready: false, started: false, paused: true, camera: 0, day: false, sound: false, time: 0, score: 0, combo: 0, lastDrift: 0, impact: 0, escaped: 0 };
const keys = new Set(), touch = new Set();
const player = { x: 0, y: -22, yaw: Math.PI / 2, speed: 0, lateral: 0, steering: 0, rearSlip: 0, distance: 0 };
let drive, renderer, city, structures, urban, traffic, scene, camera, car, audio, noticeTimer, smokeIndex = 0, markIndex = 0, markCount = 0, previousRear = null, spawnClock = 0, accumulator = 0;
const cameraPosition = new THREE.Vector3(), cameraTarget = new THREE.Vector3();
let cameraYaw = Math.PI / 2;

function notice(message) { $('notice').textContent = message; $('notice').classList.add('show'); clearTimeout(noticeTimer); noticeTimer = setTimeout(() => $('notice').classList.remove('show'), 2800); }
function readCar() {
  [player.x, player.y, player.yaw, player.speed, player.lateral] = [0, 1, 2, 3, 4].map(i => drive.drive_read(i));
  player.yawRate = drive.drive_read(5); player.steering = drive.drive_read(6); player.rearSlip = drive.drive_read(7); player.distance = drive.drive_read(9);
  [player.height, player.verticalSpeed, player.grounded, player.pitch, player.roll, player.airTime, player.landings, player.landingSpeed, player.bestAirTime] = [11, 12, 13, 14, 15, 16, 17, 18, 19].map(i => drive.drive_read(i));
  player.grounded = Boolean(player.grounded); player.ceiling = drive.drive_read(21);
}
function recover(initial = false, spawn = DISTRICTS[0].spawn) {
  let x = initial ? spawn.x : player.x, y = initial ? spawn.y : player.y, yaw = initial ? spawn.yaw : player.yaw;
  if (!initial && player.x > 180) { const start = districtAt(player.x, player.y).spawn; x = start.x; y = start.y; yaw = start.yaw; }
  else if (!initial) {
    const roadX = nearestStreet(x), roadY = nearestStreet(y);
    if (Math.abs(x - roadX) < Math.abs(y - roadY)) { x = roadX + 3.8; y = Math.max(-145, Math.min(145, y)); yaw = Math.PI / 2; }
    else { y = roadY - 3.8; x = Math.max(-145, Math.min(145, x)); yaw = 0; }
  }
  urban?.barriers.reset();
  drive.drive_reset(x, y, yaw); readCar(); previousRear = null; state.combo = 0; cameraYaw = yaw;
  if (!initial) {
    keys.clear(); touch.clear();
    cameraPosition.set(player.x - 14, player.height + 12, -player.y + 17);
    cameraTarget.set(player.x + 2, player.height + 1, -player.y - 3);
    notice('道路へ復帰しました');
  }
}
function pause(paused) {
  if (!state.ready) return;
  state.paused = paused; keys.clear(); touch.clear(); document.querySelectorAll('.touch button').forEach(b => b.classList.remove('down'));
  $('intro').hidden = !paused;
  $('intro-title').innerHTML = state.started ? 'TAKE A<br><em>BREATHER.</em>' : 'OWN THE<br>NEXT <em>CORNER.</em>';
  $('start').innerHTML = `${state.started ? '運転を続ける' : '街へ出る'} <span>↗</span>`;
  $('pause').textContent = paused ? '▶' : 'Ⅱ';
  if (!paused) { state.started = true; $('world').focus({ preventScroll: true }); }
}
function setChase() {
  if (!state.ready) return;
  traffic.setChase(!traffic.active, player); state.escaped = 0;
  $('chase').setAttribute('aria-pressed', String(traffic.active)); $('chase').querySelector('span').textContent = traffic.active ? 'CHASE ON' : 'CHASE OFF';
  $('heat').textContent = traffic.active ? 'PURSUIT ★★' : 'FREE DRIVE';
  notice(traffic.active ? '追跡開始。パトカーを引き離そう！' : 'フリードライブに戻りました');
}
function cycleCamera() { state.camera = (state.camera + 1) % 3; notice(['CHASE CAMERA', 'CITY VIEW', 'HOOD CAMERA'][state.camera]); }

$('district-start').addEventListener('change', event => {
  if (!state.ready || !state.paused) return;
  const district = event.target.value === 'station' ? { id: 'station', name: '中央駅前の周回道路', spawn: STATION_START } : event.target.value === 'urban' ? { id: 'urban', name: '坂のある市街地', spawn: URBAN_START } : event.target.value === 'underpass' ? { id: 'underpass', name: '高架の下', spawn: ROAD_STARTS.underpass } : DISTRICTS.find(d => d.id === event.target.value);
  if (!district) return;
  if (traffic.active) setChase();
  recover(true, district.spawn); if (district.spawn.x > 180) state.camera = 0; keys.clear(); touch.clear(); accumulator = 0; state.escaped = 0;
  cameraPosition.set(player.x - 14, player.height + 12, -player.y + 17); cameraTarget.set(player.x + 2, player.height + 1, -player.y - 3);
  notice(district.name + 'からスタート');
});
$('start').addEventListener('click', () => pause(false));
$('pause').addEventListener('click', () => pause(!state.paused));
$('camera').addEventListener('click', () => { if (state.ready) cycleCamera(); });
$('recover').addEventListener('click', () => { if (state.ready) recover(); });
$('chase').addEventListener('click', setChase);
$('day').addEventListener('click', () => { state.day = !state.day; $('day').querySelector('span').textContent = state.day ? 'DAY' : 'DUSK'; });
$('sound').addEventListener('click', () => {
  try {
    if (!audio) { const context = new AudioContext(); const engine = context.createOscillator(), gain = context.createGain(), filter = context.createBiquadFilter(); engine.type = 'sawtooth'; filter.type = 'lowpass'; filter.frequency.value = 300; gain.gain.value = 0; engine.connect(filter).connect(gain).connect(context.destination); engine.start(); audio = { context, engine, gain }; }
    audio.context.resume(); state.sound = !state.sound;
    $('sound').setAttribute('aria-pressed', String(state.sound)); $('sound').setAttribute('aria-label', state.sound ? 'エンジン音をオフ' : 'エンジン音をオン'); $('sound').querySelector('span').textContent = state.sound ? 'SOUND ON' : 'SOUND OFF';
  } catch { notice('このブラウザでは音を再生できません'); }
});

const driveKeys = ['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'];
addEventListener('keydown', event => {
  if (!state.paused && driveKeys.includes(event.code)) event.preventDefault();
  if (event.code === 'Escape' && !event.repeat) pause(!state.paused);
  if (event.code === 'Enter' && state.paused && state.ready && event.target === document.body) pause(false);
  if (!state.paused && !event.repeat) { if (event.code === 'KeyC') cycleCamera(); if (event.code === 'KeyR') recover(); }
  if (!state.paused) keys.add(event.code);
});
addEventListener('keyup', event => keys.delete(event.code));
addEventListener('blur', () => { if (state.started) pause(true); });
document.addEventListener('visibilitychange', () => { if (document.hidden && state.started) pause(true); });
for (const button of document.querySelectorAll('[data-control]')) {
  button.addEventListener('pointerdown', event => { event.preventDefault(); if (state.paused) return; button.setPointerCapture(event.pointerId); touch.add(button.dataset.control); button.classList.add('down'); });
  for (const kind of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(kind, () => { touch.delete(button.dataset.control); button.classList.remove('down'); });
}

function input() {
  return {
    throttle: Number(keys.has('KeyW') || keys.has('ArrowUp') || touch.has('gas')) - Number(keys.has('KeyS') || keys.has('ArrowDown') || touch.has('brake')),
    steering: Number(keys.has('KeyA') || keys.has('ArrowLeft') || touch.has('left')) - Number(keys.has('KeyD') || keys.has('ArrowRight') || touch.has('right')),
    hand: Number(keys.has('Space') || touch.has('hand')),
  };
}

function contact(nx, ny, penetration) {
  if (penetration <= 0) return;
  if (drive.drive_contact(nx, ny, Math.min(6, penetration))) {
    if (Math.hypot(player.speed, player.lateral) > 4) { state.impact = .55; state.combo = 0; }
    readCar();
  }
}
function hitObstacle(b, nx, ny, penetration) {
  if (b.breakaway && fractureContact(drive, b, nx, ny)) {
    readCar(); state.impact = .38; notice('ガードレール突破！');
  } else contact(nx, ny, penetration);
}
function collisions() {
  const radius = 1.8;
  for (const b of city.colliders) {
    if (b.broken) continue;
    if (player.height >= (b.top ?? 32) - .01 || player.height + 1.55 <= (b.bottom ?? -1000)) continue;
    const qx = Math.max(b.x - b.w, Math.min(b.x + b.w, player.x)), qy = Math.max(b.y - b.h, Math.min(b.y + b.h, player.y));
    const dx = player.x - qx, dy = player.y - qy, d = Math.hypot(dx, dy);
    if (d > .0001 && d < radius) hitObstacle(b, dx / d, dy / d, radius - d);
    else if (d <= .0001) { const sides = [[-1, 0, player.x - (b.x - b.w)], [1, 0, b.x + b.w - player.x], [0, -1, player.y - (b.y - b.h)], [0, 1, b.y + b.h - player.y]].sort((a, b) => a[2] - b[2]); hitObstacle(b, sides[0][0], sides[0][1], radius + sides[0][2]); }
  }
  if (player.x < WORLD_BOUNDS.minX) contact(1, 0, WORLD_BOUNDS.minX - player.x);
  if (player.x > WORLD_BOUNDS.maxX) contact(-1, 0, player.x - WORLD_BOUNDS.maxX);
  if (player.y < WORLD_BOUNDS.minY) contact(0, 1, WORLD_BOUNDS.minY - player.y);
  if (player.y > WORLD_BOUNDS.maxY) contact(0, -1, player.y - WORLD_BOUNDS.maxY);
  for (const c of traffic.cars) { if (Math.abs(player.height - (c.height ?? 0)) >= 1.9) continue; const dx = player.x - c.x, dy = player.y - c.y, d = Math.hypot(dx, dy); if (d > .01 && d < 3.3) { contact(dx / d, dy / d, 3.3 - d); c.speed *= .7; } }
}

const marks = new Float32Array(3600 * 6), smoke = [];
let markGeometry;
function prepareEffects() {
  markGeometry = new THREE.BufferGeometry(); markGeometry.setAttribute('position', new THREE.BufferAttribute(marks, 3).setUsage(THREE.DynamicDrawUsage)); markGeometry.setDrawRange(0, 0);
  const lines = new THREE.LineSegments(markGeometry, new THREE.LineBasicMaterial({ color: 0x101a20, transparent: true, opacity: .65 })); lines.frustumCulled = false; scene.add(lines);
  const canvas = document.createElement('canvas'); canvas.width = canvas.height = 64; const c = canvas.getContext('2d'), gradient = c.createRadialGradient(32, 32, 0, 32, 32, 32); gradient.addColorStop(0, 'rgba(225,226,218,.55)'); gradient.addColorStop(.5, 'rgba(225,226,218,.24)'); gradient.addColorStop(1, 'rgba(225,226,218,0)'); c.fillStyle = gradient; c.fillRect(0, 0, 64, 64);
  const map = new THREE.CanvasTexture(canvas);
  for (let i = 0; i < 64; i++) { const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map, transparent: true, depthWrite: false, opacity: 0 })); sprite.visible = false; scene.add(sprite); smoke.push({ sprite, life: 0 }); }
}
function rearPosition(side) {
  const x = player.x - Math.cos(player.yaw) * 1.28 - Math.sin(player.yaw) * side * .9, y = player.y - Math.sin(player.yaw) * 1.28 + Math.cos(player.yaw) * side * .9;
  return { x, y, height: drive.drive_support(x, y, player.height) };
}
function effects(dt, drifting) {
  for (const particle of smoke) if (particle.life > 0) { particle.life -= dt; particle.sprite.visible = particle.life > 0; particle.sprite.position.y += dt * .7; particle.sprite.position.x += dt * .2; particle.sprite.material.opacity = Math.max(0, particle.life * .5); const size = 1.2 + (1.25 - particle.life) * 2.2; particle.sprite.scale.set(size, size, 1); }
  spawnClock += dt;
  if (!drifting) { previousRear = null; return; }
  const rear = [-1, 1].map(rearPosition);
  if (previousRear) {
    for (let i = 0; i < 2; i++) { const p = previousRear[i], q = rear[i], offset = markIndex * 6; marks.set([p.x, p.height + .045, -p.y, q.x, q.height + .045, -q.y], offset); markIndex = (markIndex + 1) % 3600; markCount = Math.min(3600, markCount + 1); }
    markGeometry.attributes.position.needsUpdate = true; markGeometry.setDrawRange(0, markCount * 2);
  }
  previousRear = rear;
  if (spawnClock > .055) { spawnClock = 0; for (const p of rear) { const particle = smoke[smokeIndex++ % smoke.length]; particle.life = 1.25; particle.sprite.position.set(p.x, p.height + .45, -p.y); particle.sprite.visible = true; } }
}

function simulate(dt) {
  const wasAirborne = !player.grounded, flightTime = player.airTime;
  state.time += dt; const controls = input(); drive.drive_step(controls.throttle, controls.steering, controls.hand, dt); readCar(); collisions(); traffic.update(dt, state.time, player);
  if (wasAirborne && player.grounded && flightTime > .3) { state.score += Math.floor(flightTime * 100); state.impact = Math.min(.6, player.landingSpeed / 60); notice('着地！ AIR ' + flightTime.toFixed(1) + 's');
    for (let i = 0; i < 8; i++) { const p = smoke[smokeIndex++ % smoke.length], a = i * Math.PI / 4; p.life = .65; p.sprite.visible = true; p.sprite.position.set(player.x + Math.cos(a) * 1.3, player.height + .35, -player.y + Math.sin(a) * 1.3); }
   }
  const speed = Math.hypot(player.speed, player.lateral), angle = Math.atan2(Math.abs(player.lateral), Math.max(.1, Math.abs(player.speed)));
  const drifting = player.grounded && speed > 4 && angle > .12 && Math.abs(player.rearSlip) > .10;
  if (drifting) { state.combo += speed * angle * dt * 18; state.lastDrift = state.time; }
  else if (state.combo > 0 && state.time - state.lastDrift > 1.3) { state.score += Math.floor(state.combo); state.combo = 0; }
  effects(dt, drifting); $('drift').classList.toggle('active', drifting || state.combo > 0); $('combo').textContent = Math.floor(state.combo).toLocaleString(); $('angle').textContent = `SLIP ${Math.round(angle * 180 / Math.PI)}°  /  HOLD THE LINE`;
  if (traffic.active) {
    const nearest = Math.min(...traffic.cars.filter(c => c.police).map(c => Math.hypot(c.x - player.x, c.y - player.y)));
    state.escaped = nearest > 80 ? state.escaped + dt : 0;
    if (state.escaped > 12) { setChase(); state.score += 1000; notice('追跡を振り切った！ +1000'); }
  }
  state.impact = Math.max(0, state.impact - dt);
}

function updateCamera(dt) {
  cameraYaw += wrap(player.yaw - cameraYaw) * (1 - Math.exp(-dt * 4));
  const c = Math.cos(cameraYaw), s = Math.sin(cameraYaw), height = player.height || 0, eastern = player.x > 180;
  const roof = player.ceiling, enclosed = roof < 1000, belowGrade = height < -.5;
  let target, destination;
  if (!state.started) { destination = new THREE.Vector3(player.x - 14, height + 12, -player.y + 17); target = new THREE.Vector3(player.x + 2, height + 1, -player.y - 3); }
  else if (state.camera === 2) { destination = new THREE.Vector3(player.x + Math.cos(player.yaw) * .8, height + 1.45, -player.y - Math.sin(player.yaw) * .8); target = new THREE.Vector3(player.x + Math.cos(player.yaw) * 15, height + 1.35 + Math.sin(player.pitch) * 15, -player.y - Math.sin(player.yaw) * 15); }
  else if (enclosed || belowGrade) {
    destination = new THREE.Vector3(player.x - c * 10 + s * 1.2, height + 3.3, -player.y + s * 10 + c * 1.2);
    target = new THREE.Vector3(player.x + c * 5, height + 1.1, -player.y - s * 5);
  } else if (state.camera === 1) {
    // An oblique panorama reveals the hill/deck profile instead of flattening it.
    // Stay above the avenue: a side view can put tall shops between the camera and car.
    const urbanView = eastern && player.x < 380 && player.y > 110 && player.y < 190;
    destination = urbanView ? new THREE.Vector3(player.x - 45, height + 45, -player.y + 4) : eastern ? new THREE.Vector3(player.x - 55, height + 55, -player.y + 75) : new THREE.Vector3(player.x + 3, height + 57, -player.y + 15);
    target = new THREE.Vector3(player.x + (eastern ? 20 : 0), height + (urbanView ? 3 : eastern ? 5 : 0), -player.y);
  } else { const street = eastern && player.y > 110 && player.y < 190 && player.grounded; const reach = (street ? 13 : eastern ? 22 : 10) + Math.min(eastern ? 9 : 5, Math.abs(player.speed) * .12); destination = new THREE.Vector3(player.x - c * reach + s * 1.8, height + (street ? 6 : eastern ? 12 : 5.1) + Math.min(2, Math.abs(player.speed) * .06), -player.y + s * reach + c * 1.8); target = new THREE.Vector3(player.x + c * 3, height + 1.0, -player.y - s * 3); }
  if (belowGrade) destination.z = Math.max(96.4, Math.min(113.6, destination.z));
  destination.y = Math.max(destination.y, drive.drive_support(destination.x, -destination.z, height) + .7);
  const cameraRoof = drive.drive_ceiling(destination.x, -destination.z, height);
  destination.y = Math.min(destination.y, cameraRoof - .5, roof - .5);
  const blend = state.camera === 2 ? 1 : 1 - Math.exp(-dt * 7);
  cameraPosition.lerp(destination, blend); cameraTarget.lerp(target, blend);
  cameraPosition.y = Math.min(cameraPosition.y, roof - .5, drive.drive_ceiling(cameraPosition.x, -cameraPosition.z, height) - .5);
  camera.position.copy(cameraPosition); camera.lookAt(cameraTarget);
}

const mapContext = $('map').getContext('2d');
function drawMap() {
  const ctx = mapContext, ridge = player.x > 180, scale = ridge ? .36 : .48;
  const ox = ridge ? -52 : 90, oy = 90;
  ctx.clearRect(0, 0, 180, 180); ctx.fillStyle = '#12212b'; ctx.fillRect(0, 0, 180, 180);
  for (const block of BLOCKS) {
    ctx.fillStyle = block.district.color + '55'; ctx.fillRect(ox + (block.x - 16) * scale, oy - (block.y + 16) * scale, 32 * scale, 32 * scale);
    if (block.district.id === 'forest') {
      ctx.strokeStyle = '#c5bc92'; ctx.lineWidth = 2; ctx.beginPath();
      forestTrail(block.x, block.y).forEach((p, i) => ctx[i ? 'lineTo' : 'moveTo'](ox + p.x * scale, oy - p.y * scale)); ctx.stroke();
    }
  }
  ctx.strokeStyle = '#6a7a8266'; ctx.lineWidth = 18 * scale;
  for (const street of STREETS) {
    ctx.beginPath(); ctx.moveTo(ox + street * scale, oy + 180 * scale); ctx.lineTo(ox + street * scale, oy - 180 * scale);
    ctx.moveTo(ox - 180 * scale, oy - street * scale); ctx.lineTo(ox + 180 * scale, oy - street * scale); ctx.stroke();
  }
  if (ridge) {
    ctx.fillStyle = '#b3966c88'; ctx.fillRect(ox + 200 * scale, oy - 190 * scale, 180 * scale, 80 * scale);
    ctx.strokeStyle = '#ffbf78'; ctx.lineWidth = 5; ctx.beginPath();
    RIDGE_ROUTE.forEach((p, i) => ctx[i ? 'lineTo' : 'moveTo'](ox + p.x * scale, oy - p.y * scale)); ctx.stroke();
    ctx.setLineDash([2, 3]); ctx.strokeStyle = '#fff1c9'; ctx.beginPath(); ctx.moveTo(ox + 380 * scale, oy - 110 * scale); ctx.lineTo(ox + 380 * scale, oy - 190 * scale); ctx.stroke(); ctx.setLineDash([]);
    for (const route of STACKED_ROUTES) {
      ctx.strokeStyle = route.color; ctx.lineWidth = route.kind === 'skyway' ? 4 : 2;
      ctx.setLineDash(route.kind === 'tunnel' ? [3, 3] : []); ctx.beginPath();
      route.points.forEach(([x, y], i) => ctx[i ? 'lineTo' : 'moveTo'](ox + x * scale, oy - y * scale)); ctx.stroke();
    }
    ctx.setLineDash([]); ctx.strokeStyle = '#a8d9ce'; ctx.lineWidth = 2;
    for (const road of [...HILL_STREETS, LOWER_RING]) {
      ctx.beginPath(); road.forEach((p, i) => ctx[i ? 'lineTo' : 'moveTo'](ox + p.x * scale, oy - p.y * scale)); ctx.stroke();
    }
    ctx.strokeStyle = '#e6b173'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(ox + 170 * scale, oy - 104 * scale); ctx.lineTo(ox + 540 * scale, oy - 104 * scale); ctx.stroke();
    ctx.fillStyle = '#ffbf78'; ctx.font = '9px monospace'; ctx.fillText('CITY / METRO / SKYWAY', 8, 16);
  } else { ctx.fillStyle = '#ffbf78'; ctx.font = '8px monospace'; ctx.fillText('RIDGE →', 133, 12); }
  for (const c of traffic.cars) { ctx.fillStyle = c.police ? '#ff7e6a' : '#a3b3b7'; ctx.fillRect(ox + c.x * scale - 1, oy - c.y * scale - 1, c.police ? 4 : 2, c.police ? 4 : 2); }
  ctx.save(); ctx.translate(ox + player.x * scale, oy - player.y * scale); ctx.rotate(-player.yaw); ctx.fillStyle = player.grounded ? '#ffbd73' : '#fff1c9';
  ctx.beginPath(); ctx.moveTo(6, 0); ctx.lineTo(-4, -3.5); ctx.lineTo(-2, 0); ctx.lineTo(-4, 3.5); ctx.closePath(); ctx.fill(); ctx.restore();
}

function telemetry() {
  const district = districtAt(player.x, player.y);
  $('district-name').textContent = district.label;
  $('district-index').textContent = String(DISTRICTS.indexOf(district) + 1).padStart(2, '0');
  $('district-current').textContent = district.name; $('district-current').style.color = district.color;
  $('district-dot').style.backgroundColor = district.color;
  const speed = Math.hypot(player.speed, player.lateral) * 3.6;
  $('speed').textContent = Math.round(speed); $('gear').textContent = player.speed < -.5 ? 'R' : speed < 1 ? 'N' : String(Math.min(5, Math.floor(speed / 27) + 1)); $('rev').style.width = `${Math.min(100, speed % 27 / 27 * 75 + (speed > 1 ? 20 : 0))}%`; $('score').textContent = String(state.score).padStart(5, '0');
  $('elevation').textContent = 'ALT ' + Math.round(player.height) + ' m · ' + (player.grounded ? (player.height < -1 ? 'UNDERGROUND' : player.height > 8 && player.y < 40 ? 'UPPER ROAD' : 'GROUND') : 'AIR ' + player.airTime.toFixed(1) + ' s');
  $('elevation').classList.toggle('airborne', !player.grounded);
  $('impact').style.opacity = String(Math.min(1, state.impact * 1.7));
  $('status').textContent = traffic.active ? '追跡中。距離をとって振り切ろう。' : '自由に走ろう。Spaceでドリフト。';
  if (audio) { audio.engine.frequency.setTargetAtTime(32 + Math.abs(player.speed) * 4.3 + Math.abs(player.lateral) * 2, audio.context.currentTime, .1); audio.gain.gain.setTargetAtTime(state.sound && !state.paused ? .025 : 0, audio.context.currentTime, .1); }
}

async function boot() {
  try {
    const response = await fetch(`${import.meta.env.BASE_URL}drive.wasm`, { cache: 'no-cache' }); if (!response.ok) throw new Error(`Physics download: ${response.status}`);
    const result = await WebAssembly.instantiate(await response.arrayBuffer(), {}); drive = result.instance.exports;
    scene = new THREE.Scene(); scene.background = new THREE.Color(0x968d91); scene.fog = new THREE.Fog(0x968d91, 85, 295);
    camera = new THREE.PerspectiveCamera(58, innerWidth / innerHeight, .15, 650);
    renderer = new THREE.WebGLRenderer({ canvas: $('world'), antialias: true, powerPreference: 'high-performance' }); renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); renderer.setSize(innerWidth, innerHeight); renderer.outputColorSpace = THREE.SRGBColorSpace; renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.15;
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    const ambient = new THREE.HemisphereLight(0xd6d9ed, 0x435345, 2.0); scene.add(ambient);
    const sun = new THREE.DirectionalLight(0xffc18a, 2.5); sun.position.set(-45, 75, 25); sun.castShadow = true; sun.shadow.mapSize.set(1024, 1024); sun.shadow.camera.left = sun.shadow.camera.bottom = -25; sun.shadow.camera.right = sun.shadow.camera.top = 25; sun.shadow.camera.near = 1; sun.shadow.camera.far = 160; sun.shadow.bias = -.001; scene.add(sun, sun.target);
    city = createCity(scene); createRidge(scene, drive); structures = createStructures(scene, drive);
    urban = createUrbanDistrict(scene, drive, phase);
    city.colliders.push(...structures.colliders, ...urban.colliders);
    city.buildingCount += urban.buildingCount; city.signals += urban.signals;
    traffic = new Traffic(scene, { drive, urban: true, cityCars: 24 }); car = makeCar(0xf2ede2, false, true); scene.add(car); prepareEffects(); recover(true);
    cameraPosition.set(player.x - 14, 12, -player.y + 17); cameraTarget.set(player.x + 2, 1, -player.y - 3);
    state.ready = true; $('district-start').disabled = false; $('start').disabled = false; $('start').innerHTML = '街へ出る <span>↗</span>';
    // Small read-only diagnostic snapshot for smoke tests; controls stay on UI events.
    window.rustDriving = { snapshot: () => ({ ...player, ...state, physics: 'rust-wasm', carHeight: car.position.y, cameraHeight: camera.position.y, cameraX: camera.position.x, cameraY: -camera.position.z, district: districtAt(player.x, player.y).id, districtCount: DISTRICTS.length, trafficCars: traffic.cars.length, buildings: city.buildingCount, intersections: city.signals, smokeParticles: smoke.filter(p => p.life > 0).length, skidSegments: markCount, cameraMode: state.camera, chase: traffic.active, keysHeld: keys.size + touch.size, brokenGuardrails: urban.barriers.count(), trainX: urban.trainX(), urbanBuildings: urban.buildingCount, elevatedTraffic: traffic.cars.filter(c => c.height > 2).length }) };
    addEventListener('resize', () => { camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
    $('world').addEventListener('webglcontextlost', event => { event.preventDefault(); pause(true); $('error').hidden = false; $('error').textContent = '描画の接続が切れました。ページを再読み込みしてください。'; $('start').disabled = true; });
    let previous = performance.now();
    function frame(now) {
      const dt = Math.min(.05, Math.max(0, (now - previous) / 1000)); previous = now;
      if (!state.paused) { accumulator += dt; while (accumulator >= 1 / 120) { simulate(1 / 120); accumulator -= 1 / 120; } }
      else { accumulator = 0; if (!state.started) { state.time += dt; traffic.update(dt, state.time, player); } effects(dt, false); }
      car.position.set(player.x, player.height + .02, -player.y); car.rotation.set(player.roll, -player.yaw, player.pitch, 'YXZ'); car.visible = state.camera !== 2 || !state.started;
      for (const wheel of car.userData.wheels) { wheel.root.rotation.y = wheel.front ? -player.steering : 0; wheel.wheel.rotation.z = -player.distance / .32; }
      city.update(state.time); structures.update(player); urban.update(state.time, state.paused ? 0 : dt); updateCamera(dt); drawMap(); telemetry();
      const sky = state.day ? 0xb9d1dd : 0x968d91; scene.background.setHex(sky); scene.fog.color.setHex(sky); ambient.intensity = state.day ? 2.5 : 1.7; sun.intensity = state.day ? 3.1 : 2.3; sun.color.setHex(state.day ? 0xfff1d2 : 0xffbb80); sun.position.set(player.x - 45, player.height + 75, -player.y + 25); sun.target.position.set(player.x, player.height, -player.y);
      renderer.render(scene, camera); requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
  } catch (error) {
    console.error(error); $('error').hidden = false; $('error').textContent = '起動できませんでした。WebGL 2とWebAssemblyに対応したブラウザで、ページを再読み込みしてください。'; $('start').innerHTML = '再読み込み'; $('start').disabled = false; $('start').onclick = () => location.reload();
  }
}
boot();
