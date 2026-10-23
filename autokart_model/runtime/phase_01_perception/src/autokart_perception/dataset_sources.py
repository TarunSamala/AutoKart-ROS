"""Local FSOCO/BRT image overlap audit. Never downloads either dataset."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2

from .training import IMAGE_SUFFIXES


def _images(root):
    root=Path(root)
    if not root.is_dir():raise ValueError(f'DATASET_DIRECTORY_MISSING:{root}')
    return sorted(p for p in root.rglob('*') if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)


def _hashes(path):
    encoded=hashlib.sha256(path.read_bytes()).hexdigest()
    decoded=cv2.imread(str(path),cv2.IMREAD_COLOR)
    if decoded is None:raise ValueError(f'UNDECODABLE_IMAGE:{path}')
    pixels=hashlib.sha256(str(decoded.shape).encode()+decoded.tobytes()).hexdigest()
    return encoded,pixels


def audit_overlap(fsoco_root,brt_root):
    fsoco=_images(fsoco_root);brt=_images(brt_root)
    if not fsoco or not brt:raise ValueError('EMPTY_SOURCE_DATASET')
    index={}
    for image in fsoco:
        for digest in _hashes(image):index.setdefault(digest,image)
    matches=[]
    for image in brt:
        matched=next((index[d] for d in _hashes(image) if d in index),None)
        if matched is not None:
            matches.append({'brt':str(image),'fsoco':str(matched)})
    return {'fsoco_images_scanned':len(fsoco),'brt_images_scanned':len(brt),
            'confirmed_identical_images':len(matches),'confirmed_matches':matches,
            'brt_images_excluded_from_independent_count':len(brt),
            'policy':'BRT documentation identifies its images as FSOCO-derived; do not merge BRT images as additional independent photos. Exact/pixel hashes find confirmed matches only; resized/cropped copies require source-ID review.'}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fsoco',required=True)
    parser.add_argument('--brt',required=True)
    parser.add_argument('--output',required=True,help='New JSON report; existing files refused')
    args=parser.parse_args(argv)
    output=Path(args.output)
    if output.exists():raise FileExistsError(output)
    report=audit_overlap(args.fsoco,args.brt)
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'report':str(output),'confirmed_identical_images':report['confirmed_identical_images'],
                      'brt_images_excluded':report['brt_images_excluded_from_independent_count']}))
    return 0


if __name__=='__main__':raise SystemExit(main())
