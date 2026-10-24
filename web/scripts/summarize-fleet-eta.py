"""Audit full paired records and retain every outcome before publishing evidence."""
import argparse
import hashlib
import itertools
import json
import math
import os
import shutil
from pathlib import Path

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def request_key(e):return (e['person'],e['leg'])
def quantile(values,q):
    return sorted(values)[math.ceil(q*len(values))-1] if values else None
def require(value,message):
    if not value:raise ValueError(message)

def audit(case,source,wasm):
    require(case['sourceCommit']==source,'case source differs')
    require(case['wasmSha256']==wasm,'case Wasm differs')
    for name,digest in case['sourceSha256'].items():
        require(sha(name)==digest,'source changed: '+name)
    require(hashlib.sha256(case['inputJson'].encode()).hexdigest()==case['inputSha256'],'input receipt differs')
    spec=json.loads(case['inputJson'])
    require(spec['seconds']==300 and spec['dt']==1/60 and len(spec['plans'])==24,'protocol changed')
    require(spec['supply']==(8 if case['period']=='morning' else 12),'fleet size changed')
    requests={};boards={};completes=set();cancels=set()
    for e in case['events']:
        k=request_key(e)
        if e['type']=='request':
            require(k not in requests,'duplicate request');requests[k]=e['time']
        elif e['type']=='board':
            require(k in requests and k not in boards and k not in cancels,'invalid boarding lifecycle')
            require(e['speed']<.25 and e['time']>=requests[k],'invalid physical boarding')
            boards[k]=e['time']
        elif e['type']=='complete':
            require(k in boards and k not in completes,'invalid completion lifecycle');completes.add(k)
        elif e['type']=='cancel':
            require(k in requests and k not in boards,'invalid cancellation');cancels.add(k)
    waits=[stamp-requests[k] for k,stamp in boards.items()]
    m=case['metrics']
    require(m['requests']==len(requests) and m['boarded']==len(boards) and m['completed']==len(completes)
            and m['cancelled']==len(cancels),'outcome counts differ')
    require(m['pickupP95']==quantile(waits,.95),'P95 differs from event reconstruction')
    require(m['contactEpisodes']==case['contacts']['episodes'],'contact record differs')
    require(m['requests']==m['completed']+m['ridingOrAlightingAtEnd']+m['cancelled']+m['unpickedAtEnd'],'request conservation failed')
    require(m['lidarScans']>1000 and case['travel']['samples']>0,'actual sensing/travel absent')
    return {'seed':case['seed'],'period':case['period'],'weather':case['weather'],'policy':case['policy'],
            'supply':case['supply'],'metrics':m,'inputSha256':case['inputSha256']}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reports',type=Path,required=True);p.add_argument('--browser',type=Path,required=True)
    p.add_argument('--wasm',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    source=os.environ['RUSTDRIVING_SOURCE_SHA'];wasm=sha(args.wasm)
    reports=[];rows=[];pairs=[]
    for weather in ['clear','rain','snow']:
        path=args.reports/(weather+'.json');report=json.loads(path.read_text())
        require(report['sourceCommit']==source and report['wasmSha256']==wasm,'report identity differs')
        expected=set(itertools.product(['morning','night'],[2027,4093],['distance','eta']))
        actual={(c['period'],c['seed'],c['policy']) for c in report['cases']}
        require(len(report['cases'])==8 and actual==expected,'missing or duplicated case')
        for case in report['cases']:
            require(case['weather']==weather,'weather identity differs');rows.append(audit(case,source,wasm))
        for period,seed in itertools.product(['morning','night'],[2027,4093]):
            base=next(c for c in report['cases'] if (c['period'],c['seed'],c['policy'])==(period,seed,'distance'))
            candidate=next(c for c in report['cases'] if (c['period'],c['seed'],c['policy'])==(period,seed,'eta'))
            require(base['inputSha256']==candidate['inputSha256'],'paired inputs differ')
            pairs.append({'weather':weather,'period':period,'seed':seed,
                'sameInput':True,'sameRealizedRequests':base['realizedRequestsSha256']==candidate['realizedRequestsSha256'],
                'delta':{k:None if base['metrics'][k] is None or candidate['metrics'][k] is None
                         else candidate['metrics'][k]-base['metrics'][k]
                         for k in ['completed','pickupP95','emptyDistance','cancelled','contactEpisodes']}})
        shutil.copyfile(path,args.output/path.name);reports.append(report)
    browser=json.loads((args.browser/'acceptance.json').read_text())
    require(browser['sourceCommit']==source and browser['wasmSha256']==wasm,'browser source/Wasm differs')
    require(browser['pausedSwitchPreservedOwnership'] and browser['mobile'],'browser controls failed')
    require(sha(args.browser/'rain.png')==browser['pngSha256']['rain.png'],'browser image changed')
    shutil.copyfile(args.browser/'rain.png',args.output/'preview.png')
    aggregated={}
    for policy in ['distance','eta']:
        cases=[c for r in reports for c in r['cases'] if c['policy']==policy]
        waits=[w for c in cases for w in c['waits']]
        aggregated[policy]={'cases':len(cases),'requests':sum(c['metrics']['requests'] for c in cases),
            'completed':sum(c['metrics']['completed'] for c in cases),'boarded':len(waits),
            'pooledPickupP95':quantile(waits,.95),'cancelled':sum(c['metrics']['cancelled'] for c in cases),
            'emptyMetres':sum(c['metrics']['emptyDistance'] for c in cases),
            'contacts':sum(c['metrics']['contactEpisodes'] for c in cases)}
    result={'sourceCommit':source,'wasmSha256':wasm,'cases':24,'comparisons':12,
        'protocol':reports[0]['protocol']|{'weather':['clear','rain','snow']},
        'aggregate':aggregated,'pairs':pairs,'rows':rows,
        'interpretation':'Exploratory synthetic paired cases; pooled quantiles exclude unpicked/cancelled riders, which are retained separately. Initial plans/world match; realized curb request times may diverge through shared building doors. No universal improvement, safety, calibrated-weather or statistical-significance claim. ETA + forecast remains opt-in.'}
    (args.output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    provenance={'sourceCommit':source,'wasmSha256':wasm,'sourceSha256':reports[0]['sourceSha256'],
        'browser':browser,'fileSha256':{p.name:sha(p) for p in args.output.iterdir() if p.is_file()}}
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2,allow_nan=False)+'\n')
    print('ETA VERIFIED SUMMARY',json.dumps({'aggregate':aggregated,'pairs':pairs}),flush=True)
if __name__=='__main__':main()
