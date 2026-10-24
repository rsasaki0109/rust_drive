import test from 'node:test';
import assert from 'node:assert/strict';
import { STREETS } from '../src/city.js';
import { roadGraph,nodeKey } from '../src/urban-roads.js';
import { TaxiFleet,FLEET_STOPS,DEPOT_STOPS } from '../src/fleet.js';
import { weatherSupply } from '../src/city-weather.js';

const graph=roadGraph(STREETS,true),s=FLEET_STOPS[0];
const fixture=()=>({node:graph.get(nodeKey(s.from)),next:graph.get(nodeKey(s.to)),x:s.x,y:s.y,speed:0,distance:0,serviceStop:s});
const step=(f,t)=>{for(let n=0;n<t*20;n++)f.update(.05);};
test('storm reserves are bounded, optional and preserve the clear baseline',()=>{
  assert.equal(weatherSupply(8,'clear',0,true),8);
  assert.equal(weatherSupply(8,'rain',1,true),10);
  assert.equal(weatherSupply(12,'snow',1,true),16);
  assert.equal(weatherSupply(12,'snow',1,false),12);
  assert.throws(()=>weatherSupply(8,'fog',1,true),/Invalid/);
  const f=new TaxiFleet(graph,Array.from({length:16},fixture),{plans:[]});
  f.setWeather('snow',1);step(f,1);assert.equal(f.snapshot().metrics.onDuty,12);
  f.setReserveEnabled(false);step(f,1);assert.equal(f.snapshot().metrics.onDuty,8);
  assert.equal(new Set(DEPOT_STOPS.map(d=>d.x+':'+d.y)).size,8);
  assert.equal(new Set(f.taxis.slice(8).map(t=>t.car.serviceStop.id)).size,8);
});
test('weather demand starts inside buildings and requests only after a real exit',()=>{
  const f=new TaxiFleet(graph,[],{seed:1701});
  f.setWeather('snow',1);step(f,18);
  const weather=f.people.filter(p=>p.id.startsWith('W'));
  assert.ok(weather.length>=2);
  assert.ok(weather.every(p=>p.source==='weather-demand'&&p.walked>0));
  const requests=f.events.filter(e=>e.type==='request'&&e.source==='weather-demand');
  assert.ok(requests.length>=1);
  for(const r of requests){
    const exit=f.events.find(e=>e.type==='building-exit'&&e.person===r.person);
    assert.ok(exit&&exit.time<r.time);
  }
});
test('changing weather and reserve policy preserves active passenger ownership',()=>{
  const f=new TaxiFleet(graph,[fixture()],{plans:[{id:'P1',purpose:'commute',origin:'homes',destination:'office',release:0,walkSpeed:1,patience:90,dwell:4,offset:2}]});
  step(f,1);const owner=f.people[0].taxi;assert.ok(owner);
  f.setWeather('rain',1);f.setReserveEnabled(false);step(f,1);
  assert.equal(f.people[0].taxi,owner);assert.equal(f.taxis[0].person.id,'P1');
  f.setWeather('clear',0);step(f,1);assert.equal(f.people[0].taxi,owner);
});
