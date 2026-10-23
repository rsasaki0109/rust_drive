import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { Traffic, makeCar, trafficInView } from '../src/city.js';

test('traffic moves independently through the grid with finite bounded states', () => {
  const traffic = new Traffic(new THREE.Scene());
  assert.equal(traffic.cars.length, 20);
  assert.ok(new Set(traffic.cars.map(c => Math.round(c.desired * 10))).size >= 10);
  for (let i = 0; i < 1800; i++) traffic.update(1 / 60, i / 60, { x: 0, y: -22 });
  assert.ok(traffic.cars.filter(c => c.distance > 20).length >= 15);
  for (const c of traffic.cars) {
    assert.ok([c.x, c.y, c.yaw, c.speed].every(Number.isFinite));
    assert.ok(Math.abs(c.x) < 190 && Math.abs(c.y) < 190 && c.speed >= 0 && c.speed <= 14);
  }
});

test('pursuit cars navigate toward the player and are removed when pursuit ends', () => {
  const traffic = new Traffic(new THREE.Scene()), player = { x: 0, y: -22 };
  traffic.setChase(true, player);
  const police = traffic.cars.filter(c => c.police);
  assert.equal(police.length, 2);
  assert.ok(police.every(c => c.x < player.x), 'pursuers start behind the player');
  assert.ok(Math.hypot(police[0].x-police[1].x,police[0].y-police[1].y)>3.3, 'distinct initial road positions');
  let nearest = Infinity;
  for (let i = 0; i < 3600; i++) {
    traffic.update(1 / 60, i / 60, player);
    nearest = Math.min(nearest, ...traffic.cars.filter(c => c.police).map(c => Math.hypot(c.x - player.x, c.y - player.y)));
  }
  assert.ok(nearest < 15, `Police never approached the player: ${nearest} m`);
  traffic.setChase(false, player);
  assert.equal(traffic.cars.length, 20);
  assert.equal(traffic.active, false);
});

test('ordinary traffic has genuinely different compact, sedan, taxi and van bodies', () => {
  const traffic = new Traffic(new THREE.Scene());
  assert.deepEqual(new Set(traffic.cars.map(c=>c.variant)),new Set(['sedan','compact','taxi','van']));
  const sizes = {};
  for(const variant of ['sedan','compact','taxi','van']) {
    const car=makeCar(0xffffff,false,false,variant);car.updateMatrixWorld(true);
    sizes[variant]=new THREE.Box3().setFromObject(car).getSize(new THREE.Vector3());
    assert.equal(car.userData.wheels.length,4);
    assert.ok(car.userData.wheels.some(w=>w.front)&&car.userData.wheels.some(w=>!w.front));
  }
  assert.ok(sizes.compact.x<sizes.sedan.x-.4);
  assert.ok(sizes.van.y>sizes.sedan.y+.35);
  assert.ok(sizes.taxi.y>sizes.sedan.y+.25);
  const police=makeCar(0xffffff,true);
  assert.equal(police.userData.variant,'police');assert.equal(police.userData.beacons.length,2);
});

test('traffic framing excludes cars behind the camera, too far away, or hidden by buildings and terrain', () => {
  const camera=new THREE.PerspectiveCamera(58,16/9,.15,650);
  camera.position.set(0,6,0);camera.lookAt(30,1,0);camera.updateMatrixWorld(true);
  const vehicle=(x,y,police=false)=>({x,y,height:0,police,variant:police?'police':'sedan'});
  const cars=[vehicle(20,0),vehicle(30,10,true),vehicle(-20,0),vehicle(140,0)];
  const building={x:15,y:5,w:1,h:1,bottom:0,top:10};
  const view=trafficInView(cars,camera,[building],{drive_surface:()=>0});
  assert.equal(view.length,1);assert.equal(view[0].police,false);
  assert.ok(Number.isFinite(view[0].screenX+view[0].screenY));
  assert.equal(trafficInView([vehicle(30,0)],camera,[],{
    drive_surface:(x)=>x>10&&x<20?8:0,
  }).length,0);
});

test('following traffic brakes on a slope while ignoring a car on an upper layer', () => {
  const drive = { drive_surface: (x, _y, index) => index === 0 ? x * .5 : index === 1 ? .5 : 0 };
  function follow(extraHeight) {
    const traffic = new Traffic(new THREE.Scene(), { cityCars: 0, drive });
    const follower = traffic.spawn(false, 0xabcdef, { x: 100, y: 0, to: { x: 150, y: 0 }, advance: 10 });
    follower.speed = follower.desired = 8;
    const stopped = { x: 125, y: -3.8, height: 62.5 + extraHeight };
    assert.ok(stopped.height - follower.height > 7);
    for (let i = 0; i < 600; i++) traffic.update(1 / 120, i / 120, stopped);
    return follower;
  }
  const sameRoad = follow(0);
  assert.ok(sameRoad.x < 120.8, 'stop with clearance before the ground-level car');
  assert.ok(sameRoad.speed < .5, 'settle behind the stopped car');
  assert.ok(follow(12).x > 130, 'an upper-layer car must not block this road');
});

test('pursuit follows a long avenue toward an edge target rather than a nearer alley endpoint', () => {
  const traffic = new Traffic(new THREE.Scene(), { cityCars: 0, urban: true });
  const player = { x: 270, y: 150, yaw: 0, height: 0 };
  traffic.setChase(true, player);
  const police = traffic.cars.find(car => car.police);
  police.node = traffic.graph.get('240:150');
  police.previous = traffic.graph.get('190:150');
  traffic.choose(police, player);
  assert.deepEqual([police.next.x, police.next.y], [334, 150]);
  // The north backstreet is one way west; chasing a car east of it needs the
  // forward connector, not an illegal eastbound edge.
  police.node = traffic.graph.get('240:181.5');
  police.previous = traffic.graph.get('334:181.5');
  traffic.choose(police, { x: 320, y: 181.5 });
  assert.ok(police.node.links.includes(police.next));
  assert.ok(police.next.x <= police.node.x || police.next.y !== police.node.y);
});
