import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import * as THREE from 'three';
import { createCity, Traffic, STREETS, phase } from '../src/city.js';
import { createRidge } from '../src/ridge.js';
import { createStructures } from '../src/structures.js';
import { createUrbanDistrict } from '../src/urban-city.js';
import { roadGraph, nodeKey } from '../src/urban-roads.js';
import { fractureContact } from '../src/guardrails.js';

async function physics() {
  return (await WebAssembly.instantiate(await readFile(new URL('../public/drive.wasm', import.meta.url)), {})).instance.exports;
}
async function world() {
  const p = await physics(), scene = new THREE.Scene(), saved = globalThis.document;
  globalThis.document = { createElement: () => ({ getContext: () => ({ fillRect() {}, fillText() {} }) }) };
  let city, urban, structures;
  try { city = createCity(scene); createRidge(scene, p); structures = createStructures(scene, p); urban = createUrbanDistrict(scene, p, phase); }
  finally { if (saved === undefined) delete globalThis.document; else globalThis.document = saved; }
  scene.updateMatrixWorld(true);
  return { p, scene, urban, colliders: [...city.colliders, ...structures.colliders, ...urban.colliders], buildings: city.buildingCount + urban.buildingCount };
}
test('directed backstreets and station loop remain reachable from the city grid', () => {
  const graph = roadGraph(STREETS, true), seen = new Set(), stack = [graph.get('0:0')];
  while (stack.length) { const p = stack.pop(), key = nodeKey(p); if (seen.has(key)) continue; seen.add(key); stack.push(...p.links); }
  assert.equal(seen.size, graph.size);
  for(const a of graph.values())for(const b of a.links)
    assert.ok(b.links.some(q=>nodeKey(q)!==nodeKey(a)), 'a traffic approach must have a forward exit: '+nodeKey(a)+' -> '+nodeKey(b));
  assert.ok(graph.size > 95);
  for (const key of ['370:150', '334:181.5', '240:118.5', '545:150', '435:150']) assert.ok(seen.has(key), key);
  assert.equal([...graph.values()].filter(n => n.signal).length, 54);
  for (const p of graph.values()) if ([118.5, 181.5].includes(p.y)) for (const q of p.links) if (q.y === p.y)
    assert.ok(p.y === 181.5 ? q.x < p.x : q.x > p.x, 'one-way alley');
});
test('actual urban road meshes follow the Rust slope and retain overhead clearance', async () => {
  const { p, scene, buildings } = await world();
  assert.equal(buildings, 107);
  for (const [x, y] of [[210,150],[270,150],[310,150],[350,150],[334,120]]) {
    const h = p.drive_surface(x,y,0), ray = new THREE.Raycaster(new THREE.Vector3(x,h+1,-y), new THREE.Vector3(0,-1,0),0,2);
    const hits = ray.intersectObjects(scene.children,true);
    assert.ok(hits.some(a => Math.abs(a.point.y-h-.045)<.04), 'missing supported asphalt at '+[x,y]);
  }
  assert.equal(p.drive_deck(350,104,0),10);
  assert.ok(Math.abs(p.drive_ceiling(355,150,48)-52.7)<1e-8);
});
test('full-scene body-width sweeps keep every new traffic edge clear', async () => {
  const { p, colliders } = await world(), graph = roadGraph(STREETS,true);
  for (const a of graph.values()) for (const b of a.links) {
    if (Math.max(a.x,b.x)<180) continue;
    const length=Math.hypot(b.x-a.x,b.y-a.y), nx=-(b.y-a.y)/length, ny=(b.x-a.x)/length;
    const alley=a.y===b.y && [118.5,181.5].includes(a.y);
    for(let d=0;d<=length;d+=1.5) for(const lane of alley?[0]:[-3.8,3.8]) {
      const x=a.x+(b.x-a.x)*d/length+nx*lane,y=a.y+(b.y-a.y)*d/length+ny*lane,base=p.drive_surface(x,y,0);
      for(const c of colliders) {
        if(c.breakaway||base>=c.top-.01||base+1.55<=(c.bottom??-1000))continue;
        const distance=Math.hypot(Math.max(Math.abs(x-c.x)-c.w,0),Math.max(Math.abs(y-c.y)-c.h,0));
        assert.ok(distance>=1.8,'blocked road '+[x,y,base,c.x,c.y]);
      }
    }
  }
});
test('urban traffic uses real sloped surfaces and independent moving vehicles', async () => {
  const {p,scene,colliders}=await world(),traffic=new Traffic(scene,{drive:p,urban:true,cityCars:20});
  assert.equal(traffic.cars.length,40);
  const start=traffic.cars.map(c=>({x:c.x,y:c.y})),player={x:-160,y:-160,yaw:0,height:0};
  let elevated=false;
  for(let i=0;i<1800;i++){traffic.update(1/30,i/30,player);for(const c of traffic.cars){
    assert.ok(Number.isFinite(c.x+c.y+c.height+c.speed));
    assert.ok(Math.abs(c.mesh.position.y-c.height-.02)<1e-8);
    assert.ok(Math.abs(c.height-p.drive_surface(c.x,c.y,0))<1e-8);
    elevated ||= c.height>10;
    if(c.x>180)for(const b of colliders){
      if(b.breakaway||c.height>=(b.top??32)-.01||c.height+1.55<=(b.bottom??-1000))continue;
      const clearance=Math.hypot(Math.max(Math.abs(c.x-b.x)-b.w,0),Math.max(Math.abs(c.y-b.y)-b.h,0));
      assert.ok(clearance>=1.8,'moving traffic clips a solid at '+[i/30,c.x,c.y,b.x,b.y,clearance]);
    }
  }}
  assert.ok(elevated);
  assert.ok(traffic.cars.filter((c,i)=>Math.hypot(c.x-start[i].x,c.y-start[i].y)>8).length>25);
  traffic.setChase(true,{x:280,y:150,yaw:0,height:30});assert.equal(traffic.cars.length,42);
  assert.ok(traffic.cars.filter(c=>c.police).every(c=>c.node.x>150));
});
test('a low-speed car is stopped by a solid guardrail; a fast car spends energy and breaks it', async () => {
  const { p, urban }=await world(), rail=urban.barriers.colliders[1];
  p.drive_reset(374,150,0);
  function hit() {
    const gap=rail.x-rail.w-p.drive_read(0);
    if(!rail.broken && gap>=0 && gap<1.8) {
      const before=p.drive_read(3);
      if(fractureContact(p,rail,-1,0))return [before,p.drive_read(3)];
      p.drive_contact(-1,0,1.8-gap);
    }
    return null;
  }
  for(let i=0;i<300;i++){p.drive_step(1,0,0,1/120);hit();}
  assert.equal(rail.broken,false);assert.ok(p.drive_read(0)<378&&p.drive_read(10)>0);
  assert.ok(p.drive_read(13)&&p.drive_read(11)>48);
  p.drive_reset(140,150,0);let impact=null,airborne=false,landed=false;
  for(let i=0;i<5000;i++){
    p.drive_step(1,0,0,1/120);impact=hit()??impact;
    airborne ||= !p.drive_read(13);
    if(airborne&&p.drive_read(13)&&p.drive_read(0)>400){landed=true;break;}
  }
  assert.ok(impact&&impact[0]>12&&impact[1]>0&&impact[1]<impact[0]);
  assert.ok(Math.abs(.5*1250*(impact[0]**2-impact[1]**2)-90000)<1e-6);
  assert.ok(airborne&&landed&&p.drive_read(11)===0);
  assert.equal(urban.barriers.count(),1);
  urban.barriers.reset();assert.equal(rail.broken,false);assert.equal(urban.barriers.count(),0);
});
test('commuter trains advance on the raised track and freeze when simulation time is held', async () => {
  const { urban }=await world();urban.update(0,0);const first=urban.trainX();
  urban.update(8,.04);assert.ok(Math.abs(urban.trainX()-first)>100);
  const x=urban.trainX();urban.update(8,0);assert.equal(urban.trainX(),x);
});
