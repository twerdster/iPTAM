#!/usr/bin/env python3
"""Audit real lounge reference axes against image-only feature correspondences."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from check_reference_axes import median_epipolar_error
from storage import check_output_storage


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();check_output_storage(args.output)
    if args.output.exists():raise FileExistsError(args.output)
    m=json.loads((args.dataset/'manifest.json').read_text())
    reference=json.loads((args.dataset/'reference.json').read_text())
    poses={r['source_frame']:np.asarray(r['matrix']) for r in reference['poses']}
    k=np.array([[m['fx'],0,m['cx']],[0,m['fy'],m['cy']],[0,0,1.]])
    flip=np.diag([1,-1,-1,1]);results=[]
    for i in range(len(m['frames'])-1):
        result=dict(first=i,second=i+1);results.append(result)
        if i not in poses or i+1 not in poses:
            result['skipped']='ARKit state not normal';continue
        a,b=[cv2.imread(str(args.dataset/m['frames'][j]['path']),0) for j in (i,i+1)]
        p=cv2.goodFeaturesToTrack(a,600,.01,8)
        if p is None or len(p)<12:result['skipped']='fewer than12 corners';continue
        q,ok,_=cv2.calcOpticalFlowPyrLK(a,b,p,None,winSize=(21,21),maxLevel=3)
        back,okb,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,winSize=(21,21),maxLevel=3)
        keep=ok[:,0].astype(bool)&okb[:,0].astype(bool)&(np.linalg.norm(back-p,axis=2)[:,0]<.5)
        p,q=p[keep,0],q[keep,0];result['persistent_tracks']=len(p)
        if len(p)<12:result['skipped']='fewer than12 consistent tracks';continue
        cv2.setRNGSeed(42)
        _,mask=cv2.findFundamentalMat(p,q,cv2.FM_RANSAC,1,.999)
        if mask is None or mask.sum()<12:result['skipped']='fewer than12 image-only RANSAC inliers';continue
        p,q=p[mask[:,0]!=0],q[mask[:,0]!=0];result['image_geometry_inliers']=len(p)
        p=np.c_[p,np.ones(len(p))];q=np.c_[q,np.ones(len(q))]
        result['correct_cv_epipolar_median_px']=median_epipolar_error(p,q,poses[i],poses[i+1],k)
        result['wrong_unconverted_arkit_axes_median_px']=median_epipolar_error(p,q,poses[i]@flip,poses[i+1]@flip,k)
    usable=[r for r in results if 'correct_cv_epipolar_median_px' in r]
    report=dict(evaluation_only=True,reference_role=reference['role'],
        policy='All adjacent exposure pairs. GFTT600, LK forward/back<0.5px; image-only fundamental RANSAC1px. Reference never selects bootstrap or image correspondences.',
        pairs=results,usable_pairs=len(usable),
        correct_median_of_pair_medians_px=float(np.median([r['correct_cv_epipolar_median_px'] for r in usable])) if usable else None,
        wrong_median_of_pair_medians_px=float(np.median([r['wrong_unconverted_arkit_axes_median_px'] for r in usable])) if usable else None)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='pairs'},indent=2))


if __name__=='__main__':main()
