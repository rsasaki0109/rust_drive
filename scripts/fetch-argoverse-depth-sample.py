#!/usr/bin/env python3
"""Bounded stream of the official AV1.1 sample; retain calibration and first 3 PLYs.

No images, labels, or poses are decoded. Selection is archive order, before any
point inspection. Raw payloads stay ignored and retain their dataset license.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import tarfile
import urllib.request

URL = 'https://argoverse.s3.amazonaws.com/datasets/av1.1/tars/tracking_sample_v1.1.tar.gz'
OFFICIAL_LINK = 'https://s3.amazonaws.com/argoverse/datasets/av1.1/tars/tracking_sample_v1.1.tar.gz'
MAX_BYTES = 2_000_000_000


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


class Counted:
    def __init__(self, stream):
        self.stream, self.count, self.digest, self.next_notice = stream, 0, hashlib.sha256(), 64*1024*1024

    def read(self, count):
        body = self.stream.read(count)
        self.count += len(body)
        if self.count > MAX_BYTES:
            raise ValueError('compressed archive byte budget exceeded')
        self.digest.update(body)
        if self.count >= self.next_notice:
            print('compressed bytes streamed:', self.count, flush=True)
            self.next_notice += 64*1024*1024
        return body


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.absolute()
    if output.exists() or output.is_symlink() or any(p.is_symlink() for p in output.parents):
        raise ValueError('fresh real output directory required')
    output.mkdir(parents=True)
    save(output/'plan.json', dict(source=URL, official_link=OFFICIAL_LINK,
        primary_page='https://www.argoverse.org/av1.html',
        selection='first 3 lidar/PC_<timestamp>.ply regular members in original archive order, and calibration from their log',
        max_compressed_bytes=MAX_BYTES, point_contents_inspected=False, image_labels_poses_decoded=False,
        license='CC BY-NC-SA 4.0; © 2018-2019 Argo AI, LLC',
        scope='Research projection/raster consistency; no independent physical depth references; not KITTI data'))
    with urllib.request.urlopen(urllib.request.Request(URL, method='HEAD'), timeout=30) as response:
        etag = response.headers.get('ETag')
        size = int(response.headers['Content-Length'])
        if response.status != 200 or size > MAX_BYTES:
            raise ValueError('unexpected archive HTTP status/size')
    save(output/'http.json', dict(url=URL, etag=etag, compressed_bytes=size))
    points, calibrations, selected_log = [], {}, None
    with urllib.request.urlopen(urllib.request.Request(URL, headers={'If-Match': etag}), timeout=45) as response:
        if response.status != 200 or response.headers.get('ETag') != etag:
            raise ValueError('archive changed between HEAD and GET')
        counted = Counted(response)
        with tarfile.open(fileobj=counted, mode='r|gz') as archive:
            for member in archive:
                path = PurePosixPath(member.name)
                if path.is_absolute() or '..' in path.parts:
                    raise ValueError('unsafe archive member path')
                is_point = bool(re.search(r'/lidar/PC_\d+\.ply$', member.name))
                is_calib = path.name == 'vehicle_calibration_info.json'
                if not member.isfile() or not (is_point or is_calib):
                    continue
                if not 0 < member.size <= 5*1024*1024:
                    raise ValueError('selected member byte bound')
                log = str(path.parent.parent if is_point else path.parent)
                if is_point and selected_log is None:
                    selected_log = log
                if is_point and (log != selected_log or len(points) == 3):
                    continue
                body = archive.extractfile(member).read(member.size+1)
                if len(body) != member.size:
                    raise ValueError('truncated selected member')
                name = path.name
                if is_calib:
                    # Only metadata is retained if calibration precedes the first PLY.
                    calibrations[log] = (member.name, body)
                else:
                    with (output/name).open('xb') as stream:
                        stream.write(body)
                    points.append(dict(archive_member=member.name, file=name,
                        bytes=len(body), sha256=hashlib.sha256(body).hexdigest()))
                    print('selected', member.name, member.size, flush=True)
                if len(points) == 3 and selected_log in calibrations:
                    break
        if len(points) != 3 or selected_log not in calibrations:
            raise ValueError('archive did not supply 3 scans and matching log calibration')
        member_name, body = calibrations[selected_log]
        with (output/'vehicle_calibration_info.json').open('xb') as stream:
            stream.write(body)
        save(output/'acquisition.json', dict(source=URL, archive_etag=etag, log=selected_log,
            selection_preserved=True, points=points,
            calibration=dict(archive_member=member_name, file='vehicle_calibration_info.json',
                             bytes=len(body), sha256=hashlib.sha256(body).hexdigest()),
            compressed_prefix_bytes=counted.count, compressed_prefix_sha256=counted.digest.hexdigest(),
            full_archive_hash_available=False, raw_inputs_decoded=False))
        print('sample selection complete:', json.dumps(points), flush=True)


if __name__ == '__main__':
    main()
