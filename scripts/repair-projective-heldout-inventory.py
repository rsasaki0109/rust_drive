#!/usr/bin/env python3
"""Complete the frozen RPY opaque inventory after its duplicate-key writer error."""
import importlib.util,json
from pathlib import Path
from projective_depth_common import ROOT,require,sha,write


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def main():
    base=ROOT/'artifacts/projective-information/heldout-rpy';data=ROOT/'data/tum-fr1-rpy-heldout';raw=data/'raw';archive=base/'archive.tgz'
    original=load('frozen_trial',ROOT/'scripts/projective-heldout.py');original.BASE=base;original.DATA=data
    f=original.frozen();require(f['dataset']=='tum-fr1-rpy-heldout','only the original frozen RPY acquisition')
    require(not (data/'manifest.json').exists() and not (base/'acquisition.json').exists(),'never overwrite successful acquisition')
    error=Path('/tmp/rustdrive-information-rpy-acquire.log').read_bytes()
    require(b"dict() got multiple values for keyword argument 'source_path'" in error,'identified acquisition-only failure')
    receipt=dict(repair_source_sha256=sha(Path(__file__).read_bytes()),frozen_design_sha256=sha((base/'design.json').read_bytes()),
        original_helper_sha256=sha((ROOT/'scripts/projective-heldout.py').read_bytes()),original_failure_sha256=sha(error),
        repair='write each original source_path once; bytes/SHA read explicitly from verified source inventory',
        images_decoded=False,numeric_reference_poses_parsed=False,selection_parameters_changed=False,estimator_or_scoring_changed=False)
    write(base/'inventory-repair-started.json',receipt)
    source=load('bounded_source',ROOT/'scripts/fetch-independent-dataset.py');source.ARCHIVE_ROOT='rgbd_dataset_freiburg1_rpy/'
    identity=source.archive_identity(archive,original.LIMITS);first=source.scan_archive(archive,limits=original.LIMITS)
    qualifier=load('metadata_only',ROOT/'scripts/qualify-rgbd-independent.py');selected=qualifier.select_metadata(first['metadata'])
    wanted={kind+'/'+frame[kind+'_file'].removeprefix(kind+'-') for frame in selected['frames'] for kind in ('depth','rgb')}
    scan=source.scan_archive(archive,wanted,limits=original.LIMITS)
    require(scan['metadata']==first['metadata'],'unchanged complete metadata')
    names=wanted|set(scan['metadata']);require({p.name for p in raw.iterdir()}=={source.local_filename(n) for n in names},'exact selected raw inventory')
    files=[]
    for name in sorted(names):
        pin=scan['selected_files'].get(name)
        if pin is None:body=scan['metadata'][name];pin=dict(bytes=len(body),sha256=sha(body))
        target=raw/source.local_filename(name);require(not target.is_symlink() and target.is_file() and target.stat().st_size==pin['bytes'],'bounded original raw file')
        require(sha(target.read_bytes())==pin['sha256'],'original archive byte identity')
        files.append(dict(file=target.name,source_path=name,bytes=pin['bytes'],sha256=pin['sha256']))
    require(source.archive_identity(archive,original.LIMITS)==identity,'unchanged original archive')
    write(data/'manifest.json',dict(dataset=f['dataset'],preregistration_sha256=receipt['frozen_design_sha256'],
        official_archive=dict(url=f['source_url'],root=f['archive_root'],published_checksum=None,**identity),camera=original.CAMERA,frames=selected['frames'],files=files))
    write(base/'acquisition.json',dict(archive=identity,metadata_only_qualification=selected['timestamps'],image_headers_read=False,pixels_decoded=False,
        numeric_reference_poses_decoded=False,selected_images=len(wanted),members=first['members'],decompressed_bytes=first['decompressed_bytes'],inventory_repair=receipt))
    original.inputs();original.frozen();print('exact original RPY inventory completed; no pixel or numerical pose decoding',flush=True)


if __name__=='__main__':main()
