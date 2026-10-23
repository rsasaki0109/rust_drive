"""Exercise real keyboard/pointer UI against a running local production preview."""
import argparse,json,math,shutil
from pathlib import Path
from playwright.sync_api import sync_playwright
from browser_clock import advance, control_clock, wait_state

parser=argparse.ArgumentParser();parser.add_argument('--url',default='http://127.0.0.1:4173/');parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
with sync_playwright() as p:
 browser=p.chromium.launch(executable_path=shutil.which('chromium') or shutil.which('chromium-browser'),headless=True,args=['--no-sandbox','--disable-dev-shm-usage','--use-angle=swiftshader','--enable-unsafe-swiftshader'])
 page=browser.new_page(viewport={'width':1440,'height':900},device_scale_factor=1);errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 control_clock(page)
 page.goto(args.url,wait_until='networkidle');wait_state(page,'window.rustDriving?.snapshot().ready',timeout=40000);advance(page,40)
 snap=lambda:page.evaluate('window.rustDriving.snapshot()')
 before=snap();assert before['physics']=='rust-wasm' and before['buildings']==64 and before['districtCount']==8 and before['intersections']==49 and before['trafficCars']==20
 page.screenshot(path=str(args.output/'intro.png'))
 district_evidence=[]
 for district in ['downtown','harbor','forest','factory','plaza']:
  assert snap()['paused']
  page.locator('#district-start').select_option(district)
  entered=snap();assert entered['district']==district and abs(entered['speed'])<.1
  assert page.locator('#district-current').text_content()
  page.locator('#start').click();page.keyboard.down('KeyW')
  wait_state(page,'window.rustDriving.snapshot().distance>4 && window.rustDriving.snapshot().speed>1',timeout=15000)
  page.keyboard.up('KeyW');driven=snap()
  assert math.hypot(driven['x']-entered['x'],driven['y']-entered['y'])>3
  while snap()['cameraMode']!=1:page.locator('#camera').click()
  advance(page,600)
  page.screenshot(path=str(args.output/f'district-{district}.png'))
  district_evidence.append({'id':district,'distance_m':driven['distance'],'speed_mps':driven['speed']})
  page.keyboard.press('Escape')
 # Return through the same public controls; keep the existing drift/pursuit checks.
 page.locator('#district-start').select_option('downtown');page.locator('#district-start').select_option('plaza')
 while snap()['cameraMode']!=0:page.locator('#camera').click()
 page.locator('#start').click();assert snap()['started'] and not snap()['paused']
 page.keyboard.down('KeyW');wait_state(page,'window.rustDriving.snapshot().speed>10',timeout=25000)
 page.keyboard.down('KeyA');page.keyboard.down('Space')
 wait_state(page,'Math.abs(window.rustDriving.snapshot().rearSlip)>Math.PI/12 && window.rustDriving.snapshot().smokeParticles>0',timeout=45000)
 sliding=snap();maximum_slip=abs(sliding['rearSlip']);maximum_smoke=sliding['smokeParticles']
 for key in ['KeyA','Space','KeyW']:page.keyboard.up(key)
 drift=snap();assert maximum_slip>math.radians(15),maximum_slip;assert maximum_smoke>0 and drift['skidSegments']>0,(maximum_smoke,drift)
 page.screenshot(path=str(args.output/'drifting.png'))
 page.keyboard.press('KeyC');assert snap()['cameraMode']==1
 page.locator('#recover').click();assert abs(snap()['speed'])<.1
 page.locator('#chase').click();assert snap()['chase'] and snap()['trafficCars']==22
 page.locator('#day').click();assert snap()['day']
 page.locator('#sound').click();assert snap()['sound'];page.locator('#sound').click();assert not snap()['sound']
 page.keyboard.press('Escape');paused=snap();assert paused['paused'] and paused['keysHeld']==0
 advance(page,300);assert snap()['x']==paused['x'] and snap()['y']==paused['y']
 page.locator('#start').click();page.keyboard.press('KeyC');assert snap()['cameraMode']==2
 page.keyboard.press('KeyC');assert snap()['cameraMode']==0
 page.screenshot(path=str(args.output/'city-chase.png'))
 page.keyboard.down('KeyW');page.evaluate("window.dispatchEvent(new Event('blur'))");assert snap()['paused'] and snap()['keysHeld']==0;page.keyboard.up('KeyW')
 page.locator('#district-start').select_option('harbor');assert not snap()['chase'] and snap()['trafficCars']==20 and snap()['speed']==0
 mobile=browser.new_page(viewport={'width':390,'height':844},device_scale_factor=1,is_mobile=True,has_touch=True);mobile.on('pageerror',lambda e:errors.append(str(e)))
 control_clock(mobile)
 mobile.goto(args.url,wait_until='networkidle');wait_state(mobile,'window.rustDriving?.snapshot().ready',timeout=40000);advance(mobile,40);mobile.locator('#start').click()
 button=mobile.locator('[data-control=gas]');assert button.is_visible();b=button.bounding_box();mobile.mouse.move(b['x']+b['width']/2,b['y']+b['height']/2);mobile.mouse.down();wait_state(mobile,'window.rustDriving.snapshot().speed>1',timeout=15000);mobile.mouse.up();assert mobile.evaluate('window.rustDriving.snapshot().keysHeld')==0
 mobile.locator('#camera').click();assert mobile.evaluate('window.rustDriving.snapshot().cameraMode')==1
 mobile.screenshot(path=str(args.output/'mobile.png'))
 assert not errors,errors
 result={'passed':True,'districts':district_evidence,'area_change_ends_pursuit':True,'desktop':True,'mobile_pointer_controls':True,'keyboard_acceleration':True,'rear_slip_peak_degrees':maximum_slip*180/math.pi,'smoke_particles_peak':maximum_smoke,'skid_segments':drift['skidSegments'],'chase_police_cars':2,'camera_modes':3,'pause_freezes_physics':True,'blur_releases_controls':True,'reset':True,'day_toggle':True,'sound_toggle':True,'uncaught_errors':errors,'city':{'buildings':before['buildings'],'intersections':before['intersections'],'traffic':before['trafficCars']}}
 (args.output/'browser-smoke.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));browser.close()
