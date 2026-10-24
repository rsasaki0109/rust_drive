// Render-free execution of the production world, traffic, people and Rust LiDAR.
// Only canvas text painting is omitted. No agent pose, served counter or clock
// is advanced outside the production fixed-step update.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync, execFileSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import * as THREE from 'three';
import { createFleetWorld, spawnFleetCab, FLEET_OBSERVER } from '../src/fleet-world.js';
import { TaxiFleet, createDemand } from '../src/fleet.js';
import { cityConditions } from '../src/city-life.js';
import { FleetLidar, ContactEvaluator } from '../src/fleet-lidar.js';
import { percentile } from '../src/fleet-dispatch.js';

const { values: args } = parseArgs({ options: {
  weather: { type: 'string', default: 'clear' }, output: { type: 'string', default: 'web/test-results/fleet-eta/clear.json' },
  worker: { type: 'boolean', default: false }, seed: { type: 'string', default: '2027' },
  period: { type: 'string', default: 'morning' }, policy: { type: 'string', default: 'distance' },
} });
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const key = e => e.person + ':' + e.leg;
const source = process.env.RUSTDRIVING_SOURCE_SHA ?? execFileSync('git', ['rev-parse', 'HEAD'], { encoding: 'utf8' }).trim();
const wasm = await readFile(new URL('../public/drive.wasm', import.meta.url));
const files = ['web/src/fleet-dispatch.js', 'web/src/fleet-world.js', 'web/src/fleet.js', 'web/src/fleet-routing.js',
  'web/src/fleet-standby.js', 'web/src/fleet-lidar.js', 'web/src/city.js', 'web/src/city-life.js',
  'web/src/junctions.js', 'web/src/traffic-turns.js', 'web/src/urban-roads.js', 'web/src/urban-city.js',
  'web/src/scenery.js', 'web/src/ridge.js', 'web/src/structures.js', 'web/src/vehicle-pose.js',
  'web/scripts/benchmark-fleet-eta.mjs', 'web/scripts/summarize-fleet-eta.py', 'web/tests/fleet_eta_smoke.py', 'web/src/fleet-view.js', 'web/package-lock.json', 'integrations/playground/Cargo.lock'];
const sourceSha256 = Object.fromEntries(await Promise.all(files.map(async path => [path, hash(await readFile(path))])));
assert.ok(['clear', 'rain', 'snow'].includes(args.weather));

async function run() {
  assert.ok(['distance', 'eta'].includes(args.policy));
  assert.ok(['morning', 'night'].includes(args.period));
  const seed = Number(args.seed); assert.ok(Number.isSafeInteger(seed) && seed >= 0);
  const drive = (await WebAssembly.instantiate(wasm, {})).instance.exports;
  // The same canvas-label omission is used in the existing native world tests.
  globalThis.document = { createElement: () => ({ getContext: () => ({ fillRect() {}, fillText() {} }) }) };
  const scene = new THREE.Scene();
  const { traffic, instrumentColliders } = createFleetWorld(scene, drive);
  delete globalThis.document;
  const supply = cityConditions(args.period, 0).taxiSupply;
  const cars = Array.from({ length: supply }, (_, i) => spawnFleetCab(traffic, i));
  traffic.update(1 / 60, 0, FLEET_OBSERVER);
  const plans = createDemand(seed, 24, args.period);
  const initial = traffic.cars.map(c => ({ id: c.id, x: c.x, y: c.y, yaw: c.yaw, speed: c.speed,
    distance: c.distance, desired: c.desired, node: [c.node.x, c.node.y], next: [c.next.x, c.next.y] }));
  const input = { seed, period: args.period, weather: args.weather, strength: .7,
    seconds: 300, dt: 1 / 60, supply, plans, initial, colliders: instrumentColliders.map(b =>
      Object.fromEntries(['x','y','w','h','yaw','bottom','top','broken'].filter(k => b[k] !== undefined).map(k => [k,b[k]]))) };
  const f = new TaxiFleet(traffic.graph, cars, { seed, period: args.period, plans, dispatch: args.policy });
  f.setReserveEnabled(false); f.setWeather(args.weather, .7);
  const lidar = new FleetLidar(drive, instrumentColliders), contacts = new ContactEvaluator(), samples = [];
  for (let i = 0; i < 300 * 60; i++) {
    f.update(1 / 60);
    assert.equal(f.taxis.length, supply);
    lidar.update(f, traffic, 1 / 60);
    traffic.update(1 / 60, f.time, FLEET_OBSERVER);
    contacts.update(f, traffic, instrumentColliders);
    if (i % 600 === 0) {
      assert.ok(f.taxis.every(t => [t.car.x, t.car.y, t.car.speed, t.car.distance].every(Number.isFinite)));
      const owners = f.taxis.filter(t => t.person).map(t => key({ person: t.person.id, leg: t.person.leg }));
      assert.equal(owners.length, new Set(owners).size);
      samples.push({ time: f.time, taxis: f.taxis.map(t => ({ id: t.id, x: t.car.x, y: t.car.y,
        speed: t.car.speed, metres: t.car.distance, state: t.state, rider: t.person?.id ?? null })),
        contactEpisodes: contacts.events.length, lidarScans: lidar.samples });
    }
  }
  const snapshot = f.snapshot(), requests = new Map(), boarded = new Map(), completed = new Set(), cancelled = new Set();
  // Independently reconstruct outcomes from actual timestamped lifecycle events.
  for (const e of snapshot.events) {
    const id = key(e);
    if (e.type === 'request') { assert.ok(!requests.has(id)); requests.set(id, e); }
    if (e.type === 'board') {
      assert.ok(requests.has(id) && !boarded.has(id) && !cancelled.has(id));
      assert.ok(e.speed < .25 && e.time >= requests.get(id).time); boarded.set(id, e);
    }
    if (e.type === 'complete') { assert.ok(boarded.has(id) && !completed.has(id)); completed.add(id); }
    if (e.type === 'cancel') { assert.ok(requests.has(id) && !boarded.has(id)); cancelled.add(id); }
  }
  const waits = [...boarded].map(([id, e]) => e.time - requests.get(id).time);
  const waiting = [...requests.keys()].filter(id => !boarded.has(id) && !cancelled.has(id));
  assert.equal(snapshot.metrics.requested, requests.size);
  assert.equal(snapshot.metrics.served, completed.size);
  assert.equal(snapshot.metrics.cancelled, cancelled.size);
  assert.equal(snapshot.metrics.pickupWaitSamples, waits.length);
  assert.equal(snapshot.metrics.pickupP95, percentile(waits, .95));
  assert.equal(requests.size, completed.size + (boarded.size - completed.size) + cancelled.size + waiting.length);
  const metrics = { ...snapshot.metrics, requests: requests.size, boarded: boarded.size,
    completed: completed.size, unpickedAtEnd: waiting.length, ridingOrAlightingAtEnd: boarded.size - completed.size,
    cancelledFraction: requests.size ? cancelled.size / requests.size : null,
    unpickedFraction: requests.size ? (requests.size - boarded.size) / requests.size : null,
    contactEpisodes: contacts.events.length, sensorBrakeSeconds: lidar.brakeSeconds,
    staleSensorSeconds: lidar.staleSeconds, lidarScans: lidar.samples };
  return { sourceCommit: source, sourceSha256, wasmSha256: hash(wasm), policy: args.policy,
    inputSha256: hash(JSON.stringify(input)), inputJson: JSON.stringify(input), demandPlanSha256: hash(JSON.stringify(plans)),
    initialWorldSha256: hash(JSON.stringify({ initial, instrumentColliders })),
    realizedRequestsSha256: hash(JSON.stringify([...requests.values()])), seed, period: args.period,
    weather: args.weather, supply, seconds: 300, dt: 1 / 60, metrics, waits,
    events: snapshot.events, contacts: contacts.snapshot(), samples, travel: f.roadTimes.snapshot(f.time),
    etaErrors: snapshot.dispatch.etaErrors,
    scope: 'Production arcade city, signals, people and Wasm sensor/brake. Finite 24-person authored plans; live rail/weather demand generation disabled. Labels not rendered. Forecast priors uncalibrated. Not full research DrivingPipeline or real-weather validation.' };
}
const output = resolve(args.output); await mkdir(dirname(output), { recursive: true });
if (args.worker) {
  const result = await run();
  await writeFile(output, JSON.stringify(result, null, 2) + '\n');
  console.log('ETA CASE', JSON.stringify({ policy: result.policy, seed: result.seed, period: result.period,
    weather: result.weather, metrics: result.metrics }));
} else {
  const cases = [];
  for (const period of ['morning', 'night']) for (const seed of [2027, 4093]) for (const policy of ['distance', 'eta']) {
    const path = resolve(dirname(output), 'cases', args.weather + '-' + period + '-' + seed + '-' + policy + '.json');
    const run = spawnSync(process.execPath, [fileURLToPath(import.meta.url), '--worker', '--weather', args.weather,
      '--period', period, '--seed', String(seed), '--policy', policy, '--output', path],
    { encoding: 'utf8', maxBuffer: 2 * 1024 * 1024, timeout: 15 * 60 * 1000 });
    if (run.stdout) process.stdout.write(run.stdout);
    if (run.status !== 0) { if (run.stderr) process.stderr.write(run.stderr); throw new Error('Actual fleet case failed: ' + path); }
    cases.push(JSON.parse(await readFile(path, 'utf8')));
  }
  const comparisons = [];
  for (const period of ['morning', 'night']) for (const seed of [2027, 4093]) {
    const base = cases.find(c => c.period === period && c.seed === seed && c.policy === 'distance');
    const candidate = cases.find(c => c.period === period && c.seed === seed && c.policy === 'eta');
    assert.equal(base.inputSha256, candidate.inputSha256);
    assert.equal(base.wasmSha256, candidate.wasmSha256);
    assert.deepEqual(base.sourceSha256, candidate.sourceSha256);
    comparisons.push({ period, seed, weather: args.weather,
      sameInput: true, sameRealizedRequests: base.realizedRequestsSha256 === candidate.realizedRequestsSha256,
      delta: Object.fromEntries(['completed', 'boarded', 'pickupP95', 'emptyDistance', 'cancelled', 'contactEpisodes']
        .map(k => [k, base.metrics[k] === null || candidate.metrics[k] === null ? null : candidate.metrics[k] - base.metrics[k]])) });
  }
  await writeFile(output, JSON.stringify({ sourceCommit: source, sourceSha256, wasmSha256: hash(wasm),
    protocol: { seeds: [2027, 4093], periods: ['morning', 'night'], weather: args.weather, seconds: 300,
      plans: 24, onlineOnlyTelemetry: true, reserve: false, policies: ['distance', 'eta'],
      selection: 'Parameters and held-out seed list frozen before the first study run. All outcomes retained.' },
    comparisons, cases }, null, 2) + '\n');
}
