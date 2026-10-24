"""Operate the storm taxi lab through the real UI; never write agent state."""
import argparse
import base64
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit, parse_qsl, urlencode

from PIL import Image
from playwright.sync_api import sync_playwright
from browser_clock import advance, control_clock, wait_state
from gif_capture import encode_gif, ENCODER

ROOT=Path(__file__).resolve().parents[2]
SOURCES=['web/src/fleet-view.js','web/src/fleet.js','web/src/fleet-lidar.js','web/src/fleet-storm-view.js',
         'web/src/city-weather.js','web/src/city-life.js','web/src/city.js','web/src/fleet.css',
         'web/src/fleet-routing.js','web/src/fleet-standby.js','web/src/junctions.js','web/src/traffic-turns.js',
         'integrations/playground/Cargo.toml','integrations/playground/Cargo.lock','integrations/playground/src/lib.rs',
         'integrations/playground/src/fleet_lidar.rs','crates/sim/src/lidar_weather.rs','crates/perception/src/lib.rs',
         'crates/perception/src/clustering.rs','crates/perception/src/planar_sweep.rs','crates/core/src/lib.rs',
         'crates/core/src/planar_sweep.rs','web/tests/storm_smoke.py','web/tests/browser_clock.py','web/tests/gif_capture.py']
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def url(base,reserve=True):
    p=urlsplit(base);q=dict(parse_qsl(p.query));q.update(mode='fleet',lab='1',period='morning')
    if not reserve:q['reserve']='off'
    return urlunsplit((p.scheme,p.netloc,p.path,urlencode(q),p.fragment))

def run_case(browser,args,reserve,record=False):
    output=args.output/('reserve' if reserve else 'base')
    output.mkdir(parents=True,exist_ok=True)
    page=browser.new_page(viewport={'width':1280,'height':800},device_scale_factor=1)
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)));control_clock(page)
    snap=lambda:page.evaluate('window.rustDrivingFleet.snapshot()')
    frames_count=0;segments=[];samples=[];frame_hashes={}
    def tick(ms):
        before=snap();advance(page,ms);after=snap()
        assert not errors,errors
        assert not after['paused']
        assert abs(after['time']-before['time']-ms/1000*before['playbackRate'])<.06
        samples.append({'time':after['time'],'weather':after['weather'],'metrics':after['metrics'],
                        'samples':after['lab']['samples'],'contacts':after['lab']['contacts']['episodes'],
                        'taxis':[{k:t[k] for k in ('id','x','y','speed','state','person')} for t in after['taxis']]})
        assert all(math.isfinite(t[k]) for t in after['taxis'] for k in ('x','y','speed','distance'))
        assignments=[p['taxi'] for p in after['people'] if p['taxi']]
        assert len(assignments)==len(set(assignments))
        return after
    def until(target):
        for _ in range(math.ceil(max(0,target-snap()['time'])*2)+12):
            remaining=target-snap()['time']
            if remaining<.025:return
            tick(min(1000,max(5,round(remaining*1000/snap()['playbackRate']))))
        raise AssertionError('Bounded clock did not reach '+str(target))
    def rate(value):
        if snap()['playbackRate']!=value:page.locator('#fleet-rate').click()
        assert snap()['playbackRate']==value
    try:
        page.goto(url(args.url,reserve),wait_until='networkidle')
        wait_state(page,'window.rustDrivingFleet?.snapshot().ready',15000);advance(page,200)
        first=snap();assert first['paused'] and first['lab'] and first['time']==0
        page.locator('#fleet-play').click()
        rate(4);until(20);rate(1)
        assert snap()['lab']['samples']>100 and snap()['lab']['sensor']['ready']
        with tempfile.TemporaryDirectory(prefix='rustdriving-storm-') as temp:
            frames=Path(temp)
            for weather,camera in [('clear','follow'),('rain','road'),('snow','city')]:
                page.locator('#fleet-weather').select_option(weather)
                slider=page.locator('#fleet-strength');slider.focus()
                amount=int(slider.input_value())
                for _ in range(abs(70-amount)):slider.press('ArrowRight' if amount<70 else 'ArrowLeft')
                page.locator('#fleet-'+('overview' if camera=='city' else camera)).click()
                start=snap()['time'];first_frame=frames_count
                steps=12 if args.proof_only else 80
                for index in range(steps):
                    state=tick(250 if args.proof_only else 100)
                    if record:
                        path=frames/f'frame-{frames_count:04d}.png';page.screenshot(path=str(path))
                        frame_hashes[path.name]=sha(path)
                    elif index==steps-1:page.screenshot(path=str(output/(weather+'.png')))
                    frames_count+=1
                end=snap()['time']
                assert abs(end-start-(3 if args.proof_only else 8))<.06
                segments.append({'weather':weather,'camera':camera,'simulationStart':start,'simulationEnd':end,'firstFrame':first_frame,'frames':frames_count-first_frame})
                if record:page.screenshot(path=str(output/(weather+'.png')))
            if args.proof_only:
                print('STORM PROOF',json.dumps({'metrics':snap()['metrics'],'labSamples':snap()['lab']['samples']}),flush=True)
                return {'reserve':reserve,'proof':True}
            rate(4);until(90);rate(1)
            final=snap()
            assert final['metrics']['weatherRequests']>=2,final['metrics']
            assert final['metrics']['targetSupply']==(11 if reserve else 8)
            assert final['metrics']['onDuty']==(11 if reserve else 8)
            assert final['lab']['samples']>1000
            assert final['lab']['brakeSeconds']>0
            assert final['metrics']['requested']>10
            assert final['metrics']['served']>=1,final['metrics']
            weather_requests=[e for e in final['events'] if e['type']=='request' and e.get('source')=='weather-demand']
            for request in weather_requests:
                assert any(e['type']=='building-exit' and e['person']==request['person'] and e['time']<request['time'] for e in final['events'])
            page.locator('#fleet-play').click();stopped=snap();advance(page,400);assert snap()['time']==stopped['time']
            page.locator('#fleet-weather').select_option('rain');advance(page,100);assert snap()['time']==stopped['time']
            page.locator('#fleet-reserve').set_checked(not reserve);advance(page,100);assert snap()['weather']['reserve'] is not reserve
            page.locator('#fleet-lidar-toggle').click();advance(page,100);assert not snap()['lab']['showLidar']
            page.locator('#fleet-lidar-toggle').click()
            page.locator('#fleet-inspect').select_option('T02');advance(page,200);assert snap()['selectedTaxi']=='T02'
            page.set_viewport_size({'width':390,'height':844});advance(page,200)
            assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
            assert page.locator('#fleet-weather').is_visible() and page.locator('#fleet-reserve').is_visible()
            page.screenshot(path=str(output/'mobile.png'));assert not errors,errors
            trace={'sourceCommit':os.environ.get('RUSTDRIVING_SOURCE_SHA','uncommitted'),'reserve':reserve,'segments':segments,
                   'metrics':final['metrics'],'contacts':final['lab']['contacts'],'events':final['events'],
                   'taxis':final['taxis'],'samples':samples,'sourceSha256':{p:sha(ROOT/p) for p in SOURCES}}
            (output/'acceptance.json').write_text(json.dumps(trace,indent=2)+'\n')
            print('STORM ACCEPTANCE',json.dumps({'reserve':reserve,'metrics':final['metrics'],'contacts':final['lab']['contacts']['episodes'],'samples':final['lab']['samples']}),flush=True)
            if record:
                target=args.record
                duration=encode_gif(frames,target,1280,800,100)
                with Image.open(frames/'frame-0080.png') as image:image.save(target.with_suffix('.png'))
                receipt={**trace,'frames':frames_count,'gifSeconds':duration,'frameMilliseconds':100,'physicalPlayback':1,
                         'cityClockRate':15,'preRollSeconds':20,'frameSha256':frame_hashes,'encoder':ENCODER,
                         'wasmSha256':sha(ROOT/'web/public/drive.wasm'),'gifSha256':sha(target),'posterSha256':sha(target.with_suffix('.png')),
                         'scope':'Arcade mapped taxi routes, simulated signal reservation, planar collision-prism measurements and Rust point-cloud brake; synthetic uncalibrated weather'}
                target.with_suffix('.json').write_text(json.dumps(receipt,indent=2)+'\n')
            return {'reserve':reserve,'metrics':final['metrics'],'contacts':final['lab']['contacts']['episodes']}
    finally:page.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--record',type=Path);parser.add_argument('--proof-only',action='store_true');args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    with sync_playwright() as p:
        browser=p.chromium.launch(args=['--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-dev-shm-usage'])
        results=[run_case(browser,args,True,bool(args.record))]
        if not args.proof_only:results.append(run_case(browser,args,False))
        browser.close()
    (args.output/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
    if args.record:args.record.with_name('storm-taxi-comparison.json').write_text(json.dumps(results,indent=2)+'\n')
if __name__=='__main__':main()
