import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import * as THREE from 'three';
import { Traffic } from '../src/city.js';
import { measureScan, rayBox, rayDisc, FleetLidar, sensorLimit, boxesOverlap } from '../src/fleet-lidar.js';

async function wasm(){return (await WebAssembly.instantiate(await readFile(new URL('../public/drive.wasm',import.meta.url)),{})).instance.exports;}
const car={x:0,y:0,yaw:0,height:0,speed:8,desired:9};
test('the first-return instrument respects occlusion, orientation and physical height',()=>{
  const a={x:10,y:0,w:1,h:2,bottom:0,top:2},b={x:20,y:0,w:2,h:2,bottom:0,top:2};
  assert.equal(rayBox(0,0,1,0,a,1.1),9);
  assert.equal(rayBox(0,0,1,0,{...a,bottom:5,top:7},1.1),Infinity);
  assert.ok(Math.abs(rayBox(0,0,1,0,{...a,yaw:Math.PI/2},1.1)-8)<1e-9);
  assert.equal(rayDisc(0,0,1,0,{x:8,y:0,radius:.5,bottom:0,top:1.7},1.1),7.5);
  const ranges=measureScan(car,[car],[],[b,a],0);
  assert.equal(ranges[180],9);
  assert.equal(measureScan(car,[car],[],[],0)[180],45,'own body is excluded');
});
test('future pedestrians do not enter acquisition and moving people do',()=>{
  const future={x:6,y:0,height:0,state:'scheduled'};
  const walking={...future,state:'walking'};
  assert.equal(measureScan(car,[car],[future],[],0)[180],45);
  assert.ok(measureScan(car,[car],[walking],[],0)[180]<6);
});
test('the production WebAssembly detects and brakes for an observed vehicle',async()=>{
  const p=await wasm(), other={...car,x:12,speed:0};
  const traffic={cars:[car,other]}, fleet={weather:'clear',weatherStrength:0,time:1,taxis:[{car:{...car}}],people:[]};
  const host=fleet.taxis[0].car;traffic.cars[0]=host;
  const lidar=new FleetLidar(p,[]);lidar.update(fleet,traffic,.05);
  assert.ok(host.sensor.ready&&host.sensor.detections.length>0);
  assert.ok(host.sensor.limit<9);
  assert.ok(host.sensor.nearest<12);
  assert.equal(sensorLimit(host,2),0,'stale frames hold braking');
  assert.ok(p.fleet_lidar_begin(15,1,0,0));assert.equal(p.fleet_lidar_finish(15,8,0,0),0);
  assert.equal(p.fleet_lidar_read(15,5),0);
});
test('weather modifies measured returns through the Rust model',async()=>{
  async function sample(weather,strength){
    const p=await wasm(), host={...car};
    const fleet={weather,weatherStrength:strength,time:1,taxis:[{car:host}],people:[]};
    const lidar=new FleetLidar(p,[{x:15,y:0,w:1,h:6,bottom:0,top:3}]);lidar.update(fleet,{cars:[host]},.05);
    return host.sensor;
  }
  const a=await sample('clear',0),b=await sample('snow',1);
  assert.equal(a.baseReturns,b.baseReturns);
  assert.notEqual(a.points.length,b.points.length);
  assert.ok(b.points.every(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)));
});
test('sensor-driven cars do not use the legacy actor-position following shortcut',()=>{
  function run(withOther){
    const t=new Traffic(new THREE.Scene(),{cityCars:0});
    const c=t.spawn(false,0xffcc44,{x:-50,y:0,to:{x:0,y:0},advance:20});
    c.speed=5;c.desired=8;c.sensorDriven=true;c.sensor={ready:true,stamp:1,limit:14};
    if(withOther){const other=t.spawn(false,0xffcc44,{x:-50,y:0,to:{x:0,y:0},advance:28});other.x=c.x+8;other.y=c.y;other.speed=0;}
    t.update(.01,1,{x:-1000,y:-1000});
    return c.speed;
  }
  assert.equal(run(true),run(false),'a hidden car affects the governor only after an acquired return');
});
test('body overlap evaluation is independent of the ray instrument',()=>{
  const a={x:0,y:0,w:2,h:1,yaw:0,bottom:0,top:1.5};
  assert.ok(boxesOverlap(a,{...a,x:3}));
  assert.equal(boxesOverlap(a,{...a,x:5}),false);
  assert.equal(boxesOverlap(a,{...a,x:0,bottom:4,top:5.5}),false);
});
