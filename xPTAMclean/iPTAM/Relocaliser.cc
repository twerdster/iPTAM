// Copyright 2008 Isis Innovation Limited
#include "Relocaliser.h"
#include "SmallBlurryImage.h"
#include "utility.h"

using namespace CVD;
using namespace std;

Relocaliser::Relocaliser(Map &map, ATANCamera &camera)
  : mMap(map),
    mCamera(camera)
{
};

SE3<> Relocaliser::BestPose()
{
  return mse3Best;
}

bool Relocaliser::AttemptRecovery(KeyFrame &kCurrent)
{
#ifdef IPTAM_MOBILE_PAPER
  if(mMap.vpKeyFrames.empty()&&mMap.vpRetainedKeyFrames.empty()) return false;
#else
  if(mMap.vpKeyFrames.empty()) return false;
#endif
  // Ensure the incoming frame has a SmallBlurryImage attached
  if(!kCurrent.pSBI)
    kCurrent.pSBI = new SmallBlurryImage(kCurrent);
  else
    kCurrent.pSBI->MakeFromKF(kCurrent);
  
  // Find the best ZMSSD match from all keyframes in map
  ScoreKFs(kCurrent);

#ifdef IPTAM_MOBILE_PAPER
  if(!mpBestKeyFrame) return false;
  KeyFrame* best=mpBestKeyFrame;
#else
  if(mnBest<0) return false;
  KeyFrame* best=mMap.vpKeyFrames[mnBest];
#endif
  // And estimate a camera rotation from a 3DOF image alignment
  pair<SE2<>, double> result_pair = kCurrent.pSBI->IteratePosRelToTarget(*best->pSBI, 6);
  if(!std::isfinite(result_pair.second)) return false;
  mse2 = result_pair.first;
  double dScore =result_pair.second;
  
  SE3<> se3KeyFramePos = best->se3CfromW;
  mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera) * se3KeyFramePos;
  
  if(dScore < Reloc2MaxScore)
    return true;
  else 
    return false;
};

// Compare current KF to all KFs stored in map by
// Zero-mean SSD
void Relocaliser::ScoreKFs(KeyFrame &kCurrent)
{
  mdBestScore = 99999999999999.9;
  mnBest = -1;
#ifdef IPTAM_MOBILE_PAPER
  mpBestKeyFrame=nullptr;
  auto score=[&](KeyFrame* keyframe) {
    if(!keyframe) return;
    if(!keyframe->pSBI) { keyframe->pSBI=new SmallBlurryImage(*keyframe);keyframe->pSBI->MakeJacs(); }
    double value=kCurrent.pSBI->ZMSSD(*keyframe->pSBI);
    if(std::isfinite(value)&&value<mdBestScore) {mdBestScore=value;mpBestKeyFrame=keyframe;}
  };
  for(auto* keyframe:mMap.vpKeyFrames) score(keyframe);
  for(auto* keyframe:mMap.vpRetainedKeyFrames) score(keyframe);
  return;
#endif
  
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      double dSSD = kCurrent.pSBI->ZMSSD(*mMap.vpKeyFrames[i]->pSBI);
      if(dSSD < mdBestScore)
	{
	  mdBestScore = dSSD;
	  mnBest = i;
	}
    }
}

