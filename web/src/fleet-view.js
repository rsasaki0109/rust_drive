import * as THREE from 'three';
import './fleet.css';
import { phase } from './city.js';
import { createFleetWorld, spawnFleetCab, FLEET_OBSERVER } from './fleet-world.js';
import { TaxiFleet, FLEET_STOPS, PURPOSES } from './fleet.js';
import { FleetLidar, ContactEvaluator, visiblePerson } from './fleet-lidar.js';
import { createStormVisuals } from './fleet-storm-view.js';
import { CITY_WEATHER } from './city-weather.js';
import { makeStandbyStop } from './fleet-standby.js';
import { cityConditions, PERIODS } from './city-life.js';
import { createFleetPlaces } from './fleet-places.js';

const COLORS = { 'off-duty': '#52636c', idle: '#aab9c4', repositioning: '#72c8ff', standby: '#72c8ff', pickup: '#ffca62', boarding: '#ffffff', occupied: '#63decb', alighting: '#c4f19a' };
const LABELS = { exiting: 'LEAVING THE BUILDING', entering: 'ENTERING THE BUILDING', returning: 'RETURNING INSIDE',
  'exiting-train': 'LEAVING THE TRAIN', 'walking-from-train': 'WALKING FROM THE STATION', 'to-train': 'WALKING TO THE TRAIN',
  'waiting-train': 'WAITING ON THE PLATFORM', 'boarding-train': 'BOARDING THE TRAIN', walking: 'WALKING TO PICKUP', waiting: 'WAITING', boarding: 'BOARDING', riding: 'ON THE WAY',
  alighting: 'GETTING OUT', leaving: 'WALKING TO DESTINATION', arrived: 'ARRIVED', cancelled: 'LEFT THE QUEUE' };
const PURPOSE = new Map(PURPOSES.map(p => [p.id, p]));
const observer = FLEET_OBSERVER;
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
  const cloth = new THREE.MeshStandardMaterial({ color: [0x5c88ad, 0xc47e77, 0x68a08e, 0x9d83ba, 0xbca66e, 0x617986, 0xbf88a6][index % 7], roughness: .9 });
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
  group.userData = { legs, arms }; group.scale.setScalar(.94 + index % 5 * .035); return group;
}

export async function bootFleet() {
  const requestedPeriod = new URLSearchParams(location.search).get('period');
  const period = Object.hasOwn(PERIODS, requestedPeriod) ? requestedPeriod : 'morning';
  const query=new URLSearchParams(location.search),lab=query.get('lab')==='1';
  document.title = lab?'RustDriving · Storm Taxi Lab':'RustDriving · City Life';
  document.getElementById('game').hidden = true;
  document.body.insertAdjacentHTML('beforeend', '<main id="fleet-app">' +
    '<header class="fleet-header"><a class="fleet-brand" href="./">R<span>∕</span></a><div><p>RUSTDRIVING / CITY LAB</p><h1>'+(lab?'Storm taxi operations':'People, trains &amp; taxis')+'<span>03</span></h1></div>' +
    '<div class="fleet-clock"><i></i><b id="fleet-clock">08:00:00</b><small>CITY CLOCK · 15×</small></div><label class="fleet-period-label"><span>SCENARIO / START TIME</span><select id="fleet-period" aria-label="City time of day">' + Object.entries(PERIODS).map(([key, value]) => '<option value="' + key + '"' + (key === period ? ' selected' : '') + '>' + value.label + '</option>').join('') + '</select></label><a class="fleet-drive" href="./">DRIVE ↗</a><button id="fleet-play" disabled>LOADING…</button></header>' +
    '<div class="fleet-layout"><aside class="fleet-sidebar"><p class="fleet-eyebrow">EVERY TRIP STARTS WITH SOMEONE</p><h2>A city that<br>keeps moving.</h2><p class="fleet-description">People leave buildings, catch trains<br>and call for a ride home.</p>' +
    '<div class="fleet-metrics" aria-label="Live trip metrics"><div><b id="fleet-waiting">0</b><span>WAITING</span></div><div><b id="fleet-riding">0</b><span>RIDING</span></div><div><b id="fleet-served">0</b><span>DELIVERED</span></div><div><b id="fleet-cancelled">0</b><span>GAVE UP</span></div></div>' +
    '<div class="fleet-lab-entry">'+(lab?'STORM TAXI LAB':'<a href="?mode=fleet&lab=1">WEATHER + AUTONOMOUS TAXIS ↗</a>')+'</div>'+
    (lab?'<div class="fleet-weather-panel"><label>WEATHER<select id="fleet-weather" aria-label="City weather">'+Object.entries(CITY_WEATHER).map(([key,p])=>'<option value="'+key+'">'+p.label+'</option>').join('')+'</select></label><label>STRENGTH <span id="fleet-strength-value">50%</span><input id="fleet-strength" type="range" min="0" max="100" value="50" aria-label="Weather strength"></label><label class="fleet-reserve-label"><input id="fleet-reserve" type="checkbox" checked> DEPLOY RESERVE CABS</label><label>DISPATCH<select id="fleet-dispatch" aria-label="Dispatch strategy"><option value="distance">ROAD DISTANCE</option><option value="eta">TRAVEL TIME + FORECAST</option></select></label><p id="fleet-dispatch-status">Nearest road distance</p><p id="fleet-storm-policy">Weather reserve stands by.</p><div class="fleet-ops-metrics"><span>PICKUP AVG <b id="fleet-pickup-avg">0s</b></span><span>EMPTY TRAVEL <b id="fleet-empty-distance">0m</b></span><span>P95 PICKUP <b id="fleet-pickup-p95" title="95th percentile of observed pickup waits">—</b></span><span>SENSOR BRAKING <b id="fleet-braking">0s</b></span><span>CONTACTS <b id="fleet-contacts">0</b></span></div></div>':'')+
    '<div class="fleet-standby-control"><label for="fleet-standby">EMPTY TAXIS</label><select id="fleet-standby" aria-label="Empty taxi strategy"><option value="demand">FOLLOW DEMAND</option><option value="last">WAIT AT LAST STOP</option></select><p id="fleet-standby-status" role="status">Standby locations follow the city.</p></div>' +
    '<p class="fleet-rail-status" id="fleet-rail">NORTH LINE / INBOUND</p><div class="fleet-section-title">PEOPLE, NOT JUST REQUESTS <span id="fleet-demand">0 TRIPS</span></div><div id="fleet-people" aria-label="Individual travel requests"></div>' +
    '<div class="fleet-purpose-key">' + PURPOSES.map(p => '<span><i style="background:' + p.color + '"></i>' + p.label + '</span>').join('') + '</div><p class="fleet-scope"><b id="fleet-supply">8 CABS ON DUTY</b> · 1 RIDER PER TAXI<br>Fictional city · local simulation<br>No accounts · no saved progress.</p></aside>' +
    '<section id="fleet-stage" aria-label="Live taxi and pedestrian city simulation"><canvas id="fleet-world" aria-label="People walking to taxis on a city street network"></canvas>' +
    '<div class="fleet-scene-heading"><span id="fleet-view-label">LIVE CITY / DISPATCH</span><b id="fleet-view-title">From the office to the curb.</b></div>' +
    (lab?'<div class="fleet-lidar-hud"><b id="fleet-lidar-title">T01 · PLANAR LIDAR</b><p id="fleet-lidar-status">Waiting for measured returns</p><select id="fleet-inspect" aria-label="Inspect taxi"></select><button id="fleet-lidar-toggle" aria-pressed="true">HIDE POINTS</button></div>':'')+
    '<button class="fleet-rate" id="fleet-rate" aria-pressed="false">1× SPEED</button><div class="fleet-start-hint" id="fleet-hint">Press START to watch people make their way.</div>' +
    '<div class="fleet-event" id="fleet-event" role="status"></div><div class="fleet-bottom"><span><i style="background:#ffca62"></i>PICKUP</span><span><i style="background:#63decb"></i>OCCUPIED</span><span><i style="background:#c4f19a"></i>DROPOFF</span><span><i style="background:#72c8ff"></i>STANDBY</span><span><i style="background:#aab9c4"></i>AVAILABLE</span></div>' +
    '<div class="fleet-camera"><button id="fleet-overview" aria-pressed="false">CITY</button><button id="fleet-office" aria-pressed="true">OFFICE</button><button id="fleet-nightlife" aria-pressed="false">NIGHTLIFE</button><button id="fleet-station" aria-pressed="false">STATION</button><button id="fleet-follow" aria-pressed="false">FOLLOW</button>'+(lab?'<button id="fleet-road" aria-pressed="false">ROAD</button>':'')+'</div></section></div></main>');
  const $ = id => document.getElementById(id), stage = $('fleet-stage');
  let renderer;
  try {
    const response = await fetch(import.meta.env.BASE_URL + 'drive.wasm', { cache: 'no-cache' });
    if (!response.ok) throw new Error('Physics download failed: ' + response.status);
    const drive = (await WebAssembly.instantiate(await response.arrayBuffer(), {})).instance.exports;
    $('fleet-period').addEventListener('change', event => { const url = new URL(location.href); url.searchParams.set('period', event.target.value); location.assign(url); });
    const scene = new THREE.Scene(); scene.background = new THREE.Color(0xcbd9dd); scene.fog = new THREE.Fog(0xcbd9dd, 260, 620);
    const camera = new THREE.PerspectiveCamera(48, 1, .2, 850);
    camera.position.set(-12, 9, -2); camera.lookAt(-25, 1.4, 13);
    renderer = new THREE.WebGLRenderer({ canvas: $('fleet-world'), antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.18;
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    const ambient = new THREE.HemisphereLight(0xdcecf6, 0x66765b, 2.5); scene.add(ambient);
    const sun = new THREE.DirectionalLight(0xffefd5, 3); sun.position.set(-100, 180, 40); sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024); sun.shadow.camera.left = sun.shadow.camera.bottom = -135;
    sun.shadow.camera.right = sun.shadow.camera.top = 135; sun.shadow.camera.far = 450; sun.shadow.bias = -.001;
    sun.target.position.set(0, 0, -50); scene.add(sun, sun.target);
    const { city, structures, urban, instrumentColliders, traffic } = createFleetWorld(scene, drive);
    const cars = [], spawnCab = (index, arriving = false) => {
      const car = spawnFleetCab(traffic, index, arriving); cars.push(car); return car;
    };
    for (let i = 0; i < cityConditions(period, 0).taxiSupply; i++) spawnCab(i);
    traffic.update(1 / 60, 0, observer);
    const standby = new URLSearchParams(location.search).get('standby') !== 'off';
    const fleet = new TaxiFleet(traffic.graph, cars, { period, standby, dispatch: lab && query.get('dispatch') === 'eta' ? 'eta' : 'distance' }), places = createFleetPlaces(scene);
    const lidar=lab?new FleetLidar(drive,instrumentColliders):null, contacts=lab?new ContactEvaluator():null, storm=lab?createStormVisuals(scene):null;
    let showLidar=true;
    if(lab) {
      $('fleet-dispatch').value = fleet.dispatchPolicy;
      $('fleet-dispatch').addEventListener('change', event => {
        fleet.setDispatchPolicy(event.target.value); hudAt = -1;
        const url = new URL(location.href);
        if (fleet.dispatchPolicy === 'eta') url.searchParams.set('dispatch', 'eta');
        else url.searchParams.delete('dispatch');
        history.replaceState(null, '', url);
      });
      const weather=Object.hasOwn(CITY_WEATHER,query.get('weather'))?query.get('weather'):'clear';
      const amount=Number(query.get('strength')??50),strength=Number.isFinite(amount)?Math.max(0,Math.min(100,amount)):50;
      $('fleet-weather').value=weather;$('fleet-strength').value=String(strength);$('fleet-strength-value').textContent=strength+'%';
      fleet.setWeather(weather,strength/100);fleet.setReserveEnabled(query.get('reserve')!=='off');$('fleet-reserve').checked=fleet.reserveEnabled;
      const setWeather=()=>{const value=Number($('fleet-strength').value);fleet.setWeather($('fleet-weather').value,value/100);$('fleet-strength-value').textContent=value+'%';hudAt=-1;
        const url=new URL(location.href);url.searchParams.set('weather',fleet.weather);url.searchParams.set('strength',String(value));history.replaceState(null,'',url);};
      $('fleet-weather').addEventListener('change',setWeather);$('fleet-strength').addEventListener('input',setWeather);
      $('fleet-reserve').addEventListener('change',()=>{fleet.setReserveEnabled($('fleet-reserve').checked);hudAt=-1;
        const url=new URL(location.href);if(fleet.reserveEnabled)url.searchParams.delete('reserve');else url.searchParams.set('reserve','off');history.replaceState(null,'',url);});
      $('fleet-inspect').addEventListener('change',event=>{selected=event.target.value;selectView(view==='road'?'road':'follow');hudAt=-1;});
      $('fleet-lidar-toggle').addEventListener('click',()=>{showLidar=!showLidar;$('fleet-lidar-toggle').textContent=showLidar?'HIDE POINTS':'SHOW POINTS';$('fleet-lidar-toggle').setAttribute('aria-pressed',String(showLidar));});
    }
    $('fleet-standby').value = standby ? 'demand' : 'last';
    $('fleet-standby').addEventListener('change', event => {
      fleet.setStandbyEnabled(event.target.value === 'demand'); hudAt = -1;
      const url = new URL(location.href);
      if (fleet.standbyEnabled) url.searchParams.delete('standby'); else url.searchParams.set('standby', 'off');
      history.replaceState(null, '', url);
    });
    const people = new Map(), cabVisuals = new Map();
    const makeCabVisual = t => {
      const ring = new THREE.Mesh(new THREE.RingGeometry(2.10, 2.23, 24), new THREE.MeshBasicMaterial({ color: COLORS.idle, side: THREE.DoubleSide }));
      ring.rotation.x = -Math.PI / 2; scene.add(ring);
      const tag = label(t.id, '#ffdb89', 96); tag.scale.set(3.8, 2.5, 1); scene.add(tag); return { ring, tag };
    };
    const stopMaterial = new THREE.MeshStandardMaterial({ color: 0xffca62 }), stopLabels = [];
    for (const stop of FLEET_STOPS) {
      const level = drive.drive_surface(stop.curb.x, stop.curb.y, 0), group = new THREE.Group();
      group.position.set(stop.curb.x + stop.dx * 13 + stop.normal.x * 1.35, level, -stop.curb.y - stop.dy * 13 - stop.normal.y * 1.35);
      block(group, stopMaterial, 0, 1.3, 0, .10, 2.6, .10);
      const sign = label(stop.label, '#ffdc88'); sign.position.y = 4.0; group.add(sign); stopLabels.push(sign); scene.add(group);
      const points = [[-4, -1.2], [4, -1.2], [4, 1.2], [-4, 1.2]].map(([along, across]) =>
        new THREE.Vector3(stop.x + stop.dx * along + stop.normal.x * across, level + .04, -stop.y - stop.dy * along - stop.normal.y * across));
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      scene.add(new THREE.LineLoop(geometry, new THREE.LineBasicMaterial({ color: 0xb6a578 })));
      const bay = makeStandbyStop(stop);
      const standbyPoints = [[-3.6, -1.2], [3.6, -1.2], [3.6, 1.2], [-3.6, 1.2]].map(([along, across]) =>
        new THREE.Vector3(bay.x + bay.dx * along + bay.normal.x * across, level + .045, -bay.y - bay.dy * along - bay.normal.y * across));
      scene.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(standbyPoints),
        new THREE.LineBasicMaterial({ color: 0x72aacc })));
    }
    let paused = true, playbackRate = 1, view = lab?'follow':period === 'night' ? 'nightlife' : 'office', selected = fleet.taxis[0].id, previous = performance.now(), accumulator = 0, hudAt = -1;
    const resize = () => { const w = stage.clientWidth, h = stage.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); };
    resize(); addEventListener('resize', resize);
    const setPaused = value => {
      paused = value; accumulator = 0; $('fleet-play').textContent = paused ? '▶ START' : 'Ⅱ PAUSE';
      $('fleet-play').setAttribute('aria-pressed', String(!paused)); $('fleet-hint').hidden = !paused;
      $('fleet-hint').textContent = fleet.time ? 'Paused · press START to continue.' : 'Press START to watch people make their way.';
    };
    $('fleet-play').disabled = false; setPaused(true);
    $('fleet-play').addEventListener('click', () => setPaused(!paused));
    $('fleet-rate').addEventListener('click', () => { playbackRate = playbackRate === 1 ? 4 : 1; $('fleet-rate').textContent = playbackRate + '× SPEED'; $('fleet-rate').setAttribute('aria-pressed', String(playbackRate === 4)); });
    const selectView = id => {
      view = id;
      for (const [mode, button] of Object.entries({ city: 'overview', office: 'office', nightlife: 'nightlife', station: 'station', follow: 'follow', ...(lab?{road:'road'}:{}) }))
        $('fleet-' + button).setAttribute('aria-pressed', String(mode === id));
      const titles = { city: ['LIVE CITY / DISPATCH', 'Different places. Different journeys.'],
        office: ['OFFICE / DOOR TO CURB', 'From the lobby to the pickup.'],
        nightlife: ['NIGHTLIFE / AFTER HOURS', 'When the trains stop, the taxis keep going.'],
        station: ['NORTH LINE / PLATFORM 1', 'From the platform to the pickup.'],
        follow: ['LIVE TAXI / ON THE ROAD', 'One ride through a moving city.'], road:['SENSOR-GUIDED TAXI','A ride through changing weather.'] };
      $('fleet-view-label').textContent = titles[id][0]; $('fleet-view-title').textContent = titles[id][1];
    };
    for (const [mode, button] of Object.entries({ city: 'overview', office: 'office', nightlife: 'nightlife', station: 'station', follow: 'follow', ...(lab?{road:'road'}:{}) }))
      $('fleet-' + button).addEventListener('click', () => selectView(mode));
    selectView(view);
    $('fleet-people').addEventListener('click', event => { const id = event.target.closest('[data-taxi]')?.dataset.taxi; if (id) selected = id; });
    document.addEventListener('visibilitychange', () => { if (document.hidden) setPaused(true); });
    addEventListener('keydown', event => { if (event.code === 'Escape') setPaused(!paused); });
    const project = mesh => { const p = mesh.position.clone().project(camera); return mesh.visible && p.z > -1 && p.z < 1 && Math.abs(p.x) < .96 && Math.abs(p.y) < .96; };
    window.rustDrivingFleet = Object.freeze({ snapshot: () => ({
      ...fleet.snapshot(), lab:lab?{...lidar.snapshot(fleet.taxis.find(t=>t.id===selected)),contacts:contacts.snapshot(),showLidar}:null, ready: true, paused, playbackRate, cameraMode: view, selectedTaxi: selected,
      lighting: { sky: scene.background.getHex(), ambient: ambient.intensity, sunlight: sun.intensity },
      physics: 'rust-wasm terrain + arcade traffic', ambientCars: traffic.cars.length - cars.length,
      visiblePeople: [...people.values()].filter(project).length, visibleTaxis: fleet.taxis.filter(t => t.onDuty && project(t.car.mesh)).length,
      purposeColors: PURPOSES.map(p => p.color),
    }) });
    function hud() {
      const snapshot = fleet.snapshot(), m = snapshot.metrics;
      $('fleet-waiting').textContent = m.waiting; $('fleet-riding').textContent = m.riding;
      $('fleet-served').textContent = m.served; $('fleet-cancelled').textContent = m.cancelled;
      $('fleet-demand').textContent = m.requested + ' TRIPS';
      const seconds = Math.floor(snapshot.clock.minutes * 60);
      $('fleet-clock').textContent = [Math.floor(seconds / 3600), Math.floor(seconds / 60) % 60, seconds % 60].map(n => String(n).padStart(2, '0')).join(':');
      $('fleet-supply').textContent = m.onDuty + ' CABS ON DUTY';
      if(lab){
        $('fleet-storm-policy').textContent=m.onDuty+' ON DUTY / '+m.targetSupply+' TARGET · '+m.weatherRequests+' WEATHER REQUESTS';
        $('fleet-pickup-avg').textContent=Math.round(m.meanWait)+'s';$('fleet-empty-distance').textContent=(m.emptyDistance/1000).toFixed(2)+'km';
        $('fleet-braking').textContent=Math.round(lidar.brakeSeconds)+'s';$('fleet-contacts').textContent=contacts.events.length;
        if($('fleet-inspect').options.length!==fleet.taxis.length)$('fleet-inspect').innerHTML=fleet.taxis.map(t=>'<option value="'+t.id+'">'+t.id+'</option>').join('');
        $('fleet-inspect').value=selected;
        const selectedCab=fleet.taxis.find(t=>t.id===selected)??fleet.taxis[0],sensor=selectedCab.car.sensor;
        const pickup = snapshot.dispatch.pickups.find(p => p.taxi === selectedCab.id);
        $('fleet-dispatch-status').textContent = fleet.dispatchPolicy === 'distance' ? 'Nearest road distance' :
          pickup ? selectedCab.id + (pickup.remainingSeconds > 0 ? ' · PICKUP EST. ' + Math.ceil(pickup.remainingSeconds) + 's' : ' · PICKUP ETA OVERDUE') :
          'Travel-time dispatch · forecast standby';
        $('fleet-pickup-p95').textContent = m.pickupP95 === null ? '—' : Math.round(m.pickupP95) + 's';
        $('fleet-pickup-p95').title = '95th percentile of ' + m.pickupWaitSamples + ' observed pickup waits. Pending/cancelled requests are reported separately.';
        $('fleet-lidar-title').textContent=selectedCab.id+' · PLANAR LIDAR / 45m';
        $('fleet-lidar-status').textContent=sensor?.ready?sensor.points.length+' RETURNS · '+sensor.detections.length+' CLUSTERS · LIMIT '+Math.round(sensor.limit*3.6)+'km/h · SPEED '+Math.round(selectedCab.car.speed*3.6)+'km/h':'Awaiting a fresh scan · holding';
      }
      $('fleet-standby-status').textContent = snapshot.standby.enabled ?
        m.repositioning + ' heading to stands · ' + m.standby + ' ready' : 'Empty cabs wait at their last stop.';
      $('fleet-standby-status').title = 'Actual empty travel: ' + Math.round(m.emptyDistance) +
        ' m; standby travel: ' + Math.round(m.repositionDistance) + ' m';
      const train = snapshot.train;
      $('fleet-rail').textContent = train ? (train.lastService ? 'LAST TRAIN' : 'NORTH LINE') + ' / ' +
        ({ approach: 'INBOUND', dwell: 'DOORS OPEN', depart: 'DEPARTING' })[train.phase] :
        snapshot.clock.railOpen ? 'NORTH LINE / NEXT SERVICE SOON' : 'NORTH LINE / CLOSED UNTIL 05:00';
      const rank = { exiting: 0, boarding: 1, alighting: 2, 'exiting-train': 0, 'walking-from-train': 2, 'boarding-train': 1,
        walking: 3, waiting: 5, riding: 4, leaving: 6, entering: 5, returning: 5, 'to-train': 5, 'waiting-train': 6 };
      const active = fleet.people.filter(p => rank[p.state] !== undefined).sort((a, b) => ((a.origin.id === view || (view === 'station' && a.origin.id === 'station')) ? -2 : 0) - ((b.origin.id === view || (view === 'station' && b.origin.id === 'station')) ? -2 : 0) || rank[a.state] - rank[b.state] || b.release - a.release).slice(0, 3);
      $('fleet-people').innerHTML = active.length ? active.map(p => {
        const purpose = PURPOSE.get(p.purpose), elapsed = Math.max(0, fleet.time - (p.requestAt ?? p.release));
        return '<button class="fleet-person' + (p.taxi === selected ? ' selected' : '') + '" data-taxi="' + (p.taxi ?? '') + '" style="--purpose:' + purpose.color + '">' +
          '<div><i></i><b>' + p.id + ' / ' + purpose.label + '</b><span>' + (p.taxi ?? '…') + '</span></div>' +
          '<p>' + p.origin.label + ' <span>→</span> ' + p.destination.label + '</p><small>' + LABELS[p.state] + '<b>' + Math.floor(elapsed) + 's</b></small>' +
          '<div class="fleet-patience"><i style="width:' + (100 * Math.min(1, elapsed / p.patience)).toFixed(1) + '%"></i></div></button>';
      }).join('') : '<p class="fleet-empty">' + (m.requested ? 'The city is between journeys.' : 'The morning is about to begin.') + '</p>';
      const event = fleet.events.filter(e => ['board', 'complete', 'cancel', 'request', 'building-exit', 'train-alight', 'train-board', 'missed-last-train'].includes(e.type)).at(-1);
      const verbs = { 'building-exit': 'left the building', 'train-alight': 'left the train', 'train-board': 'boarded the train', 'missed-last-train': 'missed the last train', board: 'is boarding', complete: 'arrived with', cancel: 'stopped waiting', request: 'requested a ride' };
      $('fleet-event').textContent = event ? event.person + ' · ' + PURPOSE.get(event.purpose).label + ' · ' + verbs[event.type] + (event.taxi ? ' ' + event.taxi : '') : 'Walking → waiting → pickup → ride → arrival';
    }
    function frame(now) {
      const dt = Math.min(.05, Math.max(0, (now - previous) / 1000)); previous = now;
      if (!paused) {
        accumulator += dt * playbackRate;
        while (accumulator >= 1 / 60) {
          fleet.update(1 / 60);
          while (fleet.taxis.length < fleet.targetSupply()) fleet.addTaxi(spawnCab(fleet.taxis.length, true));
          if(lab)lidar.update(fleet,traffic,1/60);
          traffic.update(1 / 60, fleet.time, observer);
          if(lab)contacts.update(fleet,traffic,instrumentColliders); accumulator -= 1 / 60;
        }
      }
      fleet.people.forEach((p, i) => {
        const visible = visiblePerson(p,fleet.time);
        let mesh = people.get(p.id);
        if (!mesh && visible) { mesh = pedestrian(p, i); people.set(p.id, mesh); scene.add(mesh); }
        if (!mesh) return;
        mesh.visible = visible; if (!visible) return;
        mesh.position.set(p.x, p.height + .08, -p.y);
        mesh.rotation.y = p.state === 'waiting' ? Math.atan2(-p.origin.normal.y, -p.origin.normal.x) : p.yaw;
        const moving = !['waiting', 'waiting-train'].includes(p.state), swing = moving ? Math.sin(p.walked * 5) * .38 : 0;
        mesh.userData.legs.forEach((leg, j) => { leg.rotation.z = swing * (j ? 1 : -1); });
        mesh.userData.arms.forEach((arm, j) => { arm.rotation.z = p.state === 'waiting' && j ? -1.4 + Math.sin(fleet.time * 3 + i) * .15 : swing * (j ? -1 : 1); });
      });
      fleet.taxis.forEach(t => {
        if (!cabVisuals.has(t.id)) cabVisuals.set(t.id, makeCabVisual(t));
        const visual = cabVisuals.get(t.id), level = t.car.height ?? 0;
        visual.ring.visible = t.onDuty; visual.tag.visible = t.onDuty && (view === 'city' || view === 'follow');
        visual.ring.position.set(t.car.x, level + .10, -t.car.y); visual.ring.material.color.set(COLORS[t.standbyStop && !t.person ? 'standby' : t.state]);
        visual.tag.position.set(t.car.x, level + 4, -t.car.y);
      });
      const taxi = fleet.taxis.find(t => t.id === selected) ?? fleet.taxis[0], level = taxi.car.height ?? 0;
      const views = {
        city: [new THREE.Vector3(16, 153, 105), new THREE.Vector3(0, 0, -48)],
        office: [new THREE.Vector3(-11, 9, -2), new THREE.Vector3(-25, 1.3, 13)],
        nightlife: [new THREE.Vector3(-9, 11, -48), new THREE.Vector3(-29, 2, -64)],
        station: [new THREE.Vector3(25, 18, -155), new THREE.Vector3(25, 1.5, -185)],
        follow: [new THREE.Vector3(taxi.car.x - 22, level + 39, -taxi.car.y + 32), new THREE.Vector3(taxi.car.x, level, -taxi.car.y)],
        road:[new THREE.Vector3(taxi.car.x-Math.cos(taxi.car.yaw)*12,level+6,-taxi.car.y+Math.sin(taxi.car.yaw)*12),new THREE.Vector3(taxi.car.x+Math.cos(taxi.car.yaw)*8,level+1.1,-taxi.car.y-Math.sin(taxi.car.yaw)*8)],
      };
      const [destination, target] = views[view];
      camera.position.lerp(destination, 1 - Math.exp(-dt * 3)); camera.lookAt(target);
      const pixelScale = sprite => {
        const world = sprite.getWorldPosition(new THREE.Vector3());
        return 2 * world.distanceTo(camera.position) * Math.tan(camera.fov * Math.PI / 360) / stage.clientHeight;
      };
      for (const sign of stopLabels) { const scale = pixelScale(sign); sign.scale.set(96 * scale, 22 * scale, 1); sign.visible = view === 'city' || view === 'follow'; }
      for (const { tag } of cabVisuals.values()) { const scale = pixelScale(tag); tag.scale.set(38 * scale, 22 * scale, 1); }
      const clock = cityConditions(period, fleet.time), day = clock.daylight;
      if(lab)storm.update(fleet.weather,fleet.weatherStrength,fleet.time,taxi.car,taxi.car.sensor,showLidar);
      const sky = new THREE.Color(0x122336).lerp(new THREE.Color(0xcbd9dd), day);
      if(lab&&fleet.weather!=='clear')sky.lerp(new THREE.Color(fleet.weather==='snow'?0xb7c6d2:0x526b7d),fleet.weatherStrength*.6);
      scene.fog.near=lab&&fleet.weather!=='clear'?45:260;scene.fog.far=lab&&fleet.weather!=='clear'?620-fleet.weatherStrength*(fleet.weather==='snow'?410:340):620;
      scene.background.copy(sky); scene.fog.color.copy(sky); ambient.intensity = .82 + day * 1.68;
      sun.intensity = .06 + day * 2.94; sun.color.setHex(day < .5 ? 0xffb27c : 0xffefd5);
      const sunAngle = (clock.hour - 6) * Math.PI / 12;
      sun.position.set(Math.cos(sunAngle) * 160, Math.max(12, Math.sin(sunAngle) * 180), Math.sin(sunAngle * .7) * 80);
      sun.castShadow = day > .08;
      places.update({ doors: fleet.doors, clock, train: fleet.line?.train ?? null });
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
