"""Exercise actual ETA-dispatch controls in the rendered production lab."""
import argparse
import base64
import json
import hashlib
import os
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit, parse_qsl, urlencode
from playwright.sync_api import sync_playwright
from browser_clock import advance, control_clock, wait_state

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    parts=urlsplit(args.url);query=dict(parse_qsl(parts.query))
    query.update(mode='fleet',lab='1',dispatch='eta',reserve='off',strength='70',period='morning')
    url=urlunsplit((parts.scheme,parts.netloc,parts.path,urlencode(query),parts.fragment))
    with sync_playwright() as engine:
        browser=engine.chromium.launch(args=['--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-dev-shm-usage'])
        page=browser.new_page(viewport={'width':1280,'height':800},device_scale_factor=1)
        errors=[];page.on('pageerror',lambda e:errors.append(str(e)));control_clock(page)
        page.goto(url,wait_until='networkidle')
        wait_state(page,'window.rustDrivingFleet?.snapshot().ready',15000);advance(page,200)
        snap=lambda:page.evaluate('window.rustDrivingFleet.snapshot()')
        assert snap()['dispatch']['policy']=='eta'
        page.locator('#fleet-play').click();page.locator('#fleet-rate').click()
        phases=[]
        for weather,target in [('clear',15),('rain',30),('snow',45)]:
            page.locator('#fleet-weather').select_option(weather)
            for _ in range(30):
                remaining=target-snap()['time']
                if remaining<.04:break
                advance(page,min(1000,max(10,round(remaining*250))))
            else:raise AssertionError('Bounded clock did not reach weather phase')
            s=snap()
            assert abs(s['time']-target)<.08 and s['dispatch']['policy']=='eta'
            assert s['metrics']['onDuty']==8 and s['weather']['reserve'] is False
            assert len(s['dispatch']['forecast'])==9
            path=args.output/(weather+'.png');page.screenshot(path=str(path))
            phases.append({'weather':weather,'time':s['time'],'metrics':s['metrics'],'dispatch':s['dispatch'],'contacts':s['lab']['contacts']})
        assert snap()['dispatch']['telemetrySamples']>0
        assert snap()['metrics']['pickupWaitSamples']>0
        page.locator('#fleet-play').click();before=snap()
        owners=[(t['id'],t['person'],t['x'],t['y']) for t in before['taxis']]
        page.locator('#fleet-dispatch').select_option('distance');advance(page,200)
        after=snap()
        assert after['time']==before['time'] and after['dispatch']['policy']=='distance'
        assert [(t['id'],t['person'],t['x'],t['y']) for t in after['taxis']]==owners
        page.locator('#fleet-dispatch').select_option('eta');advance(page,200)
        assert snap()['dispatch']['policy']=='eta'
        page.set_viewport_size({'width':390,'height':844});advance(page,200)
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        assert page.locator('#fleet-dispatch').is_visible()
        page.screenshot(path=str(args.output/'mobile.png'))
        assert not errors,errors
        (args.output/'acceptance.json').write_text(json.dumps({'sourceCommit':os.environ.get('RUSTDRIVING_SOURCE_SHA','uncommitted'),'wasmSha256':hashlib.sha256(Path('web/dist/drive.wasm').read_bytes()).hexdigest(),'pngSha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.glob('*.png')},'phases':phases,'pausedSwitchPreservedOwnership':True,'mobile':True},indent=2)+'\n')
        print('ETA BROWSER',json.dumps({'metrics':before['metrics'],'dispatch':before['dispatch'],'contacts':before['lab']['contacts']['episodes']}),flush=True)
        browser.close()
    for path in sorted(args.output.glob('*.png')):
        print('IMAGE_BEGIN:'+path.as_posix());print(base64.encodebytes(path.read_bytes()).decode('ascii'));print('IMAGE_END:'+path.as_posix())
if __name__=='__main__':main()
