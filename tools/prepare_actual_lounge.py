#!/usr/bin/env python3
"""Prepare the real LiteReality lounge export; ARKit poses are evaluation only."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from evaluate_trajectory import is_rigid
from prepare_datasets import digest
from storage import check_output_storage


AR_TO_CV = np.diag([1., -1., -1., 1.])


def exported_frame(frame, sample):
    """Cross-check independently serialized row-major and column-major records."""
    pose=np.asarray(frame['cameraPoseARFrame'],dtype=float).reshape(4,4)
    k=np.asarray(frame['intrinsics'],dtype=float).reshape(3,3)
    meta_pose=np.asarray(sample['worldFromCameraTransform'],dtype=float).reshape(4,4,order='F')
    meta_k=np.asarray(sample['cameraIntrinsics'],dtype=float).reshape(3,3,order='F')
    inverse=np.asarray(sample['cameraFromWorldTransform'],dtype=float).reshape(4,4,order='F')
    errors=dict(timestamp_s=abs(float(frame['time'])-sample['timestamp']),
                pose_max_abs=float(np.max(abs(pose-meta_pose))),
                intrinsics_max_abs=float(np.max(abs(k-meta_k))),
                inverse_max_abs=float(np.max(abs(inverse@meta_pose-np.eye(4)))))
    if errors['timestamp_s']>1e-7 or errors['pose_max_abs']>1e-5 or errors['intrinsics_max_abs']>1e-3 or errors['inverse_max_abs']>1e-5:
        raise ValueError(f'Independent metadata mismatch: {errors}')
    if not is_rigid(pose) or k[0,0]<=0 or k[1,1]<=0 or not np.isfinite(k).all():
        raise ValueError('Invalid exported camera')
    if not np.allclose(k[2],[0,0,1]) or abs(k[0,1])+abs(k[1,0])>1e-6:
        raise ValueError('Unsupported nonstandard intrinsic matrix')
    return pose@AR_TO_CV,k,errors


def normalize_pixels(gray,k,width=640,height=480,focal=460.):
    """Area downsample, then normalize each exposure to one fixed pinhole K."""
    sh,sw=gray.shape
    sx,sy=width/sw,height/sh
    reduced=cv2.resize(gray,(width,height),interpolation=cv2.INTER_AREA)
    small=k.copy()
    small[0,0]*=sx;small[1,1]*=sy
    small[0,2]=(k[0,2]+.5)*sx-.5
    small[1,2]=(k[1,2]+.5)*sy-.5
    yy,xx=np.indices((height,width),dtype=np.float64)
    mx=((xx-(width-1)/2)/focal*small[0,0]+small[0,2]).astype(np.float32)
    my=((yy-(height-1)/2)/focal*small[1,1]+small[1,2]).astype(np.float32)
    if mx.min() < -1e-4 or my.min() < -1e-4 or mx.max() > width-1+1e-4 or my.max() > height-1+1e-4:
        raise ValueError('Target pinhole pixels extend outside source support')
    mx=np.clip(mx,0,width-1);my=np.clip(my,0,height-1)
    return cv2.remap(reduced,mx,my,cv2.INTER_LINEAR),dict(
        x_min=float(mx.min()),x_max=float(mx.max()),y_min=float(my.min()),y_max=float(my.max()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--init-first',type=int,default=1)
    parser.add_argument('--init-second',type=int,default=2)
    args=parser.parse_args()
    check_output_storage(args.source);check_output_storage(args.output)
    if args.output.exists():raise FileExistsError(args.output)
    meta=json.loads((args.source/'metadata.json').read_text())
    samples=meta['cameraSamples'];sample_times=np.array([s['camera']['timestamp'] for s in samples])
    records=[]
    for i,path in enumerate(sorted(args.source.glob('frame_*.json'))):
        frame=json.loads(path.read_text())
        if frame['frame_index']!=i:raise ValueError('Nonconsecutive source frame index')
        si=int(np.argmin(abs(sample_times-frame['time'])))
        pose,k,check=exported_frame(frame,samples[si]['camera'])
        exported_meta=meta['frames'][i]
        if exported_meta['cameraSampleIndex']!=si or Path(exported_meta['fileName']).name!=path.with_suffix('.jpg').name:
            raise ValueError('Metadata frame/image association mismatch')
        records.append((path,frame,pose,k,si,check,samples[si]['camera']['trackingState']))
    if len(records)!=meta['imageCount'] or len(records)<3:raise ValueError('Frame count mismatch')
    times=np.array([r[1]['time'] for r in records])
    if np.any(np.diff(times)<=0):raise ValueError('Source exposure times not strictly increasing')
    if not 0<=args.init_first<args.init_second<len(records):raise ValueError('Invalid suggested pair')
    origin=float(times[0]);out=args.output;(out/'images').mkdir(parents=True)
    frames=[];poses=[];provenance=[];source_hash=hashlib.sha256();output_hash=hashlib.sha256()
    for i,(path,frame,pose,k,si,check,state) in enumerate(records):
        check_output_storage(out)
        image=path.with_suffix('.jpg');gray=cv2.imread(str(image),cv2.IMREAD_GRAYSCALE|cv2.IMREAD_IGNORE_ORIENTATION)
        sample=samples[si]['camera']
        if gray is None or gray.shape!=(sample['capturedImageHeight'],sample['capturedImageWidth']):
            raise ValueError(f'Image dimensions mismatch: {image}')
        duplicate=args.source/'images'/image.name
        if duplicate.exists() and digest(duplicate)!=digest(image):raise ValueError('Duplicate image contents differ')
        normalized,support=normalize_pixels(gray,k)
        rel=f'images/{i:06d}.pgm'
        if not cv2.imwrite(str(out/rel),normalized):raise OSError('PGM write failed')
        source_hash.update(gray.tobytes());output_hash.update(normalized.tobytes())
        t=float(frame['time']-origin);frames.append(dict(path=rel,timestamp=t))
        # The first ARKit exposure is explicitly limited(initializing).
        if state=='normal':poses.append(dict(timestamp=t,matrix=pose.tolist(),source_frame=i,tracking_state=state))
        provenance.append(dict(index=i,source_image=str(image),source_sidecar=str(path),
            source_timestamp_s=frame['time'],source_image_sha256=digest(image),
            source_sidecar_sha256=digest(path),source_K=k.tolist(),camera_sample_index=si,
            tracking_state=state,metadata_crosscheck=check,normalized_sampling_support=support))
    manifest=dict(name='actual_lounge',width=640,height=480,fx=460.,fy=460.,cx=319.5,cy=239.5,
        lens_model='pinhole',distortion=[],init_first=args.init_first,init_second=args.init_second,frames=frames)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with (out/'frames.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['timestamp','path'])
        writer.writerows((f['timestamp'],f['path']) for f in frames)
    reference=dict(role='ARKit device reference; not independent ground truth',
        pose_convention='T_world_from_CV_camera: +X right,+Y down,+Z forward',
        conversion='per-frame row-major ARKit worldFromCamera @ diag(1,-1,-1,1), exactly once',
        selection='normal ARKit tracking state only; all input images retained',
        excluded_source_frames=[i for i,r in enumerate(records) if r[6]!='normal'],poses=poses)
    (out/'reference.json').write_text(json.dumps(reference,indent=2)+'\n')
    report=dict(adapter_sha256=digest(Path(__file__)),opencv=cv2.__version__,numpy=np.__version__,source=str(args.source),name='actual_lounge',input_frames=len(frames),reference_frames=len(poses),
        source_timestamp_origin_s=origin,duration_s=float(times[-1]-times[0]),
        sample_interval_min_median_max_s=[float(np.min(np.diff(times))),float(np.median(np.diff(times))),float(np.max(np.diff(times)))],
        operation='OpenCV grayscale JPEG decode ignoring EXIF orientation; 3x INTER_AREA downsample with pixel-center K scaling; per-exposure intrinsics normalization by bilinear remap to fixed K460',
        optical_limitation='Export supplies pinhole intrinsics but no lens distortion coefficients/lookup; this is an exported-intrinsics pinhole approximation, not a newly measured lens calibration',
        source_grayscale_pixels_concatenated_sha256=source_hash.hexdigest(),
        rectified_pixels_concatenated_sha256=output_hash.hexdigest(),
        estimator_inputs='Only grayscale pixels, fixed K and source exposure times. No poses, depth, mesh, ARKit features or IMU.',
        metadata_sha256=digest(args.source/'metadata.json'),
        scan_label=json.loads((args.source/'scan_label.json').read_text()),
        source_archive_provenance=json.loads((args.source.parent/'lounge-archive-provenance.json').read_text()),
        metadata_crosscheck_max={key:max(r[5][key] for r in records) for key in records[0][5]},frames=provenance)
    (out/'provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2))


if __name__=='__main__':main()
