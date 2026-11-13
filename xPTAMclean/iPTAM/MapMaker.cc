// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "HomographyInit.h"

#include "vector_image_ref.h"
#include "vision.h"
#include "image_interpolate.h"

#include "SVD.h"
#include "SymEigen.h"

#include <fstream>
#include <algorithm>
#ifdef IPTAM_MOBILE_PAPER
#include "MobileConfig.h"
#include "MobileMappingBudget.h"
#include "MobileMappingGeometry.h"
#include <chrono>
#endif

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace CVD;
using namespace std;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, const ATANCamera &cam, bool threaded)
  : mbThreaded(threaded), mMap(m), mCamera(cam)
{
  mbResetRequested = false;
  mgvdWiggleScale=MapMakerWiggleScale;
  Reset();
  if(mbThreaded) start(); // This CVD::thread func starts the map-maker thread with function run()
		   //GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
  mgvdWiggleScale=MapMakerWiggleScale; // Default to 10cm between keyframes
};

void MapMaker::Reset()
{
#ifdef IPTAM_MOBILE_PAPER
  mDiagnostics=MappingDiagnostics();
  mbMobileResourceLimit=false;msMobileResourceReason.clear();
  mPartialLandmarks.clear();mRetainedPoseAnchors.clear();mPartialObservationCursor=0;
  mPendingPartialPromotions.clear();mNextPartialId=1;
#endif
  // This is only called from within the mapmaker thread...
  mRng.seed(42);
  mMap.Reset();
  mvFailureQueue.clear();
  while(!mqNewQueue.empty()) mqNewQueue.pop();
  mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes
  for(auto* k:mvpKeyFrameQueue) delete k;
  mvpKeyFrameQueue.clear();
  mbBundleRunning = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbResetDone = true;
  mbResetRequested = false;
  mbBundleAbortRequested = false;
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::run() {
    while(!shouldStop()) { ProcessOneIteration(); sleep(5); }
}

void MapMaker::ProcessPending() {
#ifdef IPTAM_MOBILE_PAPER
    if(!mbThreaded && IPTAMMobile::Config().idleMapping) {
      IPTAMMobile::ScopedMappingDeadline deadline(SteadyTimeSeconds()+std::max(0.,IPTAMMobile::Config().idleBudgetMs)/1000.);
      ProcessOneIteration();return;
    }
#endif
    if(!mbThreaded) ProcessOneIteration();
}
#ifdef IPTAM_MOBILE_PAPER
double MapMaker::SteadyTimeSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
unsigned int MapMaker::ProcessIdleWork(double deadlineSeconds,double budgetMs) {
  if(mbThreaded || !IPTAMMobile::Config().idleMapping || !mMap.IsGood() ||
     !std::isfinite(deadlineSeconds) || !std::isfinite(budgetMs) || budgetMs<=0 || mbMobileResourceLimit) return 0;
  const double start=SteadyTimeSeconds(),end=std::min(deadlineSeconds,start+budgetMs/1000.);
  IPTAMMobile::ScopedMappingDeadline deadline(end);
  unsigned int stages=0;
  // Cooperative soft deadline: an existing bounded bundle/refind stage is an
  // indivisible quantum. Never introduce a timer thread or race an abort flag.
  while(stages<64 && SteadyTimeSeconds()<end && !IPTAMMobile::MappingBudgetExpired() && mMap.IsGood() && !mbMobileResourceLimit) {
    if(QueueSize()==0 && mbBundleConverged_Recent && mbBundleConverged_Full && mqNewQueue.empty() && mPendingPartialPromotions.empty()) break;
    ProcessOneIteration();++stages;
  }
  const double elapsed=(SteadyTimeSeconds()-start)*1000.;
  mDiagnostics.idleStages+=stages;mDiagnostics.idleMs+=elapsed;
  if(SteadyTimeSeconds()>end && stages)++mDiagnostics.idleOverruns;
  return stages;
}
#endif

void MapMaker::ProcessOneIteration() {
    if(mbResetRequested) { Reset(); return; }
    if(!mMap.IsGood()) return;
#ifdef IPTAM_MOBILE_PAPER
    auto timed=[this](MappingDiagnostics::Stage stage,auto operation) {
      const auto start=std::chrono::steady_clock::now();operation();
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      ++mDiagnostics.stageCalls[stage];mDiagnostics.stageMs[stage]+=ms;
      mDiagnostics.maxStageMs[stage]=std::max(mDiagnostics.maxStageMs[stage],ms);
    };
    if(QueueSize() > 0) timed(MappingDiagnostics::InsertKeyframe,[this]{AddKeyFrameFromTopOfQueue();});
    else if(!mPendingPartialPromotions.empty()) timed(MappingDiagnostics::PartialPromotion,[this]{ProcessPendingPartialPromotion();});
    else if(!mbBundleConverged_Recent) timed(MappingDiagnostics::RecentBundle,[this]{BundleAdjustRecent();});
    else {
        if(IPTAMMobile::Config().cullMeasurements) {
          // The mobile paper omits historic measurement densification. Tracker
          // observations and newly triangulated pairs remain the BA inputs.
          while(!mqNewQueue.empty())mqNewQueue.pop();
          mvFailureQueue.clear();
        } else timed(MappingDiagnostics::Refind,[this]{ReFindNewlyMade();});
        if(!mbBundleConverged_Full) timed(MappingDiagnostics::FullBundle,[this]{BundleAdjustAll();});
        else if(!IPTAMMobile::Config().cullMeasurements && mRng()%20 == 0)
          timed(MappingDiagnostics::Retry,[this]{ReFindFromFailureQueue();});
    }
#else
    if(QueueSize() > 0) AddKeyFrameFromTopOfQueue();
    else if(!mbBundleConverged_Recent) BundleAdjustRecent();
    else {
        ReFindNewlyMade();
        if(!mbBundleConverged_Full) BundleAdjustAll();
        else if(mRng()%20 == 0) ReFindFromFailureQueue();
    }
#endif
    // As in the original mapper loop, retiring bad points is mandatory even
    // while a bounded/nonconverged bundle stage remains pending.
    HandleBadPoints();
#ifdef IPTAM_MOBILE_PAPER
    if(IPTAMMobile::Config().cullMeasurements || IPTAMMobile::Config().cullKeyframes)
      timed(MappingDiagnostics::Cull,[this]{CullMobileMap();});
    CheckMobileResources();
#endif
}

// Tracker calls this to demand a reset
void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
  if(!mbThreaded) Reset();
}

bool MapMaker::ResetDone()
{
  return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if 
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints()
{
  // Did the tracker see this point as an outlier more often than as an inlier?
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      MapPoint &p = *mMap.vpPoints[i];
      if(p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > p.nMEstimatorInlierCount)
	p.bBad = true;
    }
  
  // All points marked as bad will be erased - erase all records of them
  // from keyframes in which they might have been measured.
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    if(mMap.vpPoints[i]->bBad)
      {
	MapPoint *p = mMap.vpPoints[i];
	for(unsigned int j=0; j<mMap.vpKeyFrames.size(); j++)
	  {
	    KeyFrame &k = *mMap.vpKeyFrames[j];
	    if(k.mMeasurements.count(p))
	      k.mMeasurements.erase(p);
	  }
      }
  // Move bad points to the trash list.
  mMap.MoveBadPointsToTrash();
}

MapMaker::~MapMaker()
{
  mbBundleAbortRequested = true;
  stop(); // makes shouldStop() return true
  cout << "Waiting for mapmaker to die.." << endl;
  if(mbThreaded) join();
  for(auto* k:mvpKeyFrameQueue) delete k;
  mvpKeyFrameQueue.clear();
  cout << " .. mapmaker has died." << endl;
}


// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> MapMaker::ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();
  
  Matrix<4> A;
  A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A[2] = v2A[0] * PDash[2] - PDash[0];
  A[3] = v2A[1] * PDash[2] - PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}



// InitFromStereo() generates the initial match from two keyframes
// and a vector of image correspondences. Uses the 
bool MapMaker::InitFromStereo(KeyFrame &kF,
			      KeyFrame &kS,
			      vector<pair<ImageRef, ImageRef> > &vTrailMatches,
			      SE3<> &se3TrackerPose
#ifdef IPTAM_MOBILE_PAPER
                              , const SE3<>* selectedSecondFromFirst
#endif
                              )
{
  std::cout << "Initializing from " << vTrailMatches.size() << " image correspondences." << std::endl;
  mdWiggleScale = mgvdWiggleScale; // Cache this for the new map.

  mCamera.SetImageSize(kF.aLevels[0].im.size());

  
  vector<HomographyMatch> vMatches;
  for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
      HomographyMatch m;
      m.v2CamPlaneFirst = mCamera.UnProject(vTrailMatches[i].first);
      m.v2CamPlaneSecond = mCamera.UnProject(vTrailMatches[i].second);
      m.m2PixelProjectionJac = mCamera.GetProjectionDerivs();
      vMatches.push_back(m);
    }

  SE3<> se3;
  bool bGood;
  HomographyInit HomographyInit;
#ifdef IPTAM_MOBILE_PAPER
  if(selectedSecondFromFirst) {
    se3=*selectedSecondFromFirst;
    bGood=true;
    const auto rotation=se3.get_rotation().get_matrix();
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)bGood=bGood&&std::isfinite(rotation[r][c]);
    for(int i=0;i<3;++i)bGood=bGood&&std::isfinite(se3.get_translation()[i]);
  } else
#endif
  bGood = HomographyInit.Compute(vMatches, 5.0, se3);
  if(!bGood)
    {
      cout << "  Could not init from stereo pair, try again." << endl;
      return false;
    }
  
  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
  if(!std::isfinite(dTransMagn) || dTransMagn < 1e-6)
    {
      cout << "  Estimated zero baseline from stereo pair, try again." << endl;
      return false;
    }
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;

  
  KeyFrame *pkFirst = new KeyFrame();
  KeyFrame *pkSecond = new KeyFrame();
  *pkFirst = kF;
  *pkSecond = kS;
  
  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();
  
  pkSecond->bFixed = false;
  pkSecond->se3CfromW = se3;
  
  // Construct map from the stereo matches.
  PatchFinder finder;

  for(unsigned int i=0; i<vMatches.size(); i++)
    {
#ifdef IPTAM_MOBILE_PAPER
      if(mMap.vpPoints.size()>=ActivePointLimit()) break;
#endif
      MapPoint *p = new MapPoint();
      
      // Patch source stuff:
      p->pPatchSourceKF = pkFirst;
      p->nSourceLevel = 0;
      p->v3Normal_NC = makeVector( 0,0,-1);
      p->irCenter = vTrailMatches[i].first;
      p->v3Center_NC = unproject(mCamera.UnProject(p->irCenter));
      p->v3OneDownFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(0,1)));
      p->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(1,0)));
      normalize(p->v3Center_NC);
      normalize(p->v3OneDownFromCenter_NC);
      normalize(p->v3OneRightFromCenter_NC);

      // Do sub-pixel alignment on the second image
      finder.MakeTemplateCoarseNoWarp(*p);
      finder.MakeSubPixTemplate();
      finder.SetSubPixPos(vec(vTrailMatches[i].second));
      bool bGood = finder.IterateSubPixToConvergence(*pkSecond,10);
      if(!bGood)
	{ 
	  delete p; continue;
	}
      
      // Triangulate point:
      Vector<2> v2SecondPos = finder.GetSubPixPos();
      p->v3WorldPos = ReprojectPoint(se3, mCamera.UnProject(v2SecondPos), vMatches[i].v2CamPlaneFirst);
      if(!std::isfinite(p->v3WorldPos*p->v3WorldPos) || p->v3WorldPos[2] <= 0.001 || (se3*p->v3WorldPos)[2] <= 0.001)
       	{
 	  delete p; continue;
 	}
      
      p->RefreshPixelVectors();
      // Not behind map? Good, then add to map.
      p->pMMData = new MapMakerData();
      mMap.vpPoints.push_back(p);
      
      // Construct first two measurements and insert into relevant DBs:
      Measurement mFirst;
      mFirst.nLevel = 0;
      mFirst.Source = Measurement::SRC_ROOT;
      mFirst.v2RootPos = vec(vTrailMatches[i].first);
      mFirst.bSubPix = true;
      pkFirst->mMeasurements[p] = mFirst;
      p->pMMData->sMeasurementKFs.insert(pkFirst);
      
      Measurement mSecond;
      mSecond.nLevel = 0;
      mSecond.Source = Measurement::SRC_TRAIL;
      mSecond.v2RootPos = finder.GetSubPixPos();
      mSecond.bSubPix = true;
      pkSecond->mMeasurements[p] = mSecond;
      p->pMMData->sMeasurementKFs.insert(pkSecond);
    }
  
  mMap.vpKeyFrames.push_back(pkFirst);
  mMap.vpKeyFrames.push_back(pkSecond);
  if(mMap.vpPoints.size()<10) { Reset(); return false; }
  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();
  
  for(int i=0; i<5; i++) { BundleAdjustAll(); if(mbResetRequested) { Reset(); return false; } }

  // Estimate the feature depth distribution in the first two key-frames
  // (Needed for epipolar search)
  if(!RefreshSceneDepth(pkFirst) || !RefreshSceneDepth(pkSecond)) { Reset(); return false; }
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;


#ifdef IPTAM_MOBILE_PAPER
  if(!IPTAMMobile::Config().partialPoints) {
#endif
  AddSomeMapPoints(0);
  AddSomeMapPoints(3);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);
  
#ifdef IPTAM_MOBILE_PAPER
  }
#endif

  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
  
  int attempts = 0;
  while(!mbBundleConverged_Full && attempts++ < 20)
    {
      BundleAdjustAll();
      if(mbResetRequested) { Reset(); return false; }
    }
  
  if(!mbBundleConverged_Full || mMap.vpPoints.size()<10) { Reset(); return false; }
  // Rotate and translate the map so the dominant plane is at z=0:
  ApplyGlobalTransformationToMap(CalcPlaneAligner());
  mMap.bGood = true;
#ifdef IPTAM_MOBILE_PAPER
  if(IPTAMMobile::Config().partialPoints)SeedPartialPoints(*pkSecond);
  if(IPTAMMobile::Config().cullMeasurements||IPTAMMobile::Config().cullKeyframes)CullMobileMap();
#endif
  se3TrackerPose = pkSecond->se3CfromW;
  
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;
  return true; 
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt 
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame &k, int nLevel)
{
  vector<Candidate> &vCSrc = k.aLevels[nLevel].vCandidates;
  vector<Candidate> vCGood;
  vector<ImageRef> irBusyLevelPos;
  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  for(meas_it it = k.mMeasurements.begin(); it!=k.mMeasurements.end(); it++)
    {
      if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
	continue;
      irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
    }
  
  // Only keep those candidates further than 10 pixels away from busy positions.
  unsigned int nMinMagSquared = 10*10;
  for(unsigned int i=0; i<vCSrc.size(); i++)
    {
      ImageRef irC = vCSrc[i].irLevelPos;
      bool bGood = true;
      for(unsigned int j=0; j<irBusyLevelPos.size(); j++)
	{
	  ImageRef irB = irBusyLevelPos[j];
	  if((irB - irC).mag_squared() < nMinMagSquared)
	    {
	      bGood = false;
	      break;
	    }
	}
      if(bGood)
	vCGood.push_back(vCSrc[i]);
    } 
  vCSrc = vCGood;
}

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel)
{
  KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
  KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));   
  Level &l = kSrc.aLevels[nLevel];

  ThinCandidates(kSrc, nLevel);
  
  size_t added = 0;
  for(unsigned int i = 0; i<l.vCandidates.size() && mMap.vpPoints.size()<
#ifdef IPTAM_MOBILE_PAPER
      ActivePointLimit()
#else
      MaxMapPoints
#endif
      && added<MaxPointsPerLevelInsertion; i++)
    if(AddPointEpipolar(kSrc, kTarget, nLevel, i)) ++added;
};

// Rotates/translates the whole map and all keyframes
void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();
  
#ifdef IPTAM_MOBILE_PAPER
  RefreshRetainedPoses();
#endif
  SO3<> so3Rot = se3NewFromOld.get_rotation();
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      mMap.vpPoints[i]->v3WorldPos = 
	se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      mMap.vpPoints[i]->RefreshPixelVectors();
    }
}

// Applies a global scale factor to the map
void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;
  
#ifdef IPTAM_MOBILE_PAPER
  for(auto& record:mRetainedPoseAnchors)record.second.second.get_translation()*=dScale;
  RefreshRetainedPoses();
  for(auto& partial:mPartialLandmarks){partial.inverseDepth/=dScale;partial.variance/=(dScale*dScale);}
#endif
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      (*mMap.vpPoints[i]).v3WorldPos *= dScale;
      (*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
      (*mMap.vpPoints[i]).v3PixelDown_W *= dScale;
      (*mMap.vpPoints[i]).RefreshPixelVectors();
    }
}

// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so 
// just dumps it on the top of the mapmaker's queue to 
// be dealt with later, and return.
void MapMaker::AddKeyFrame(KeyFrame &k)
{
#ifdef IPTAM_MOBILE_PAPER
  CheckMobileResources();if(mbMobileResourceLimit)return;
  if(IPTAMMobile::Config().idleMapping && mvpKeyFrameQueue.size()>=2)return;
#endif
  KeyFrame *pK = new KeyFrame;
  *pK = k;
  pK->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  mvpKeyFrameQueue.push_back(pK);
  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;
}

// Mapmaker's code to handle incoming key-frames.
void MapMaker::AddKeyFrameFromTopOfQueue()
{
  if(mvpKeyFrameQueue.size() == 0)
    return;
  
  KeyFrame *pK = mvpKeyFrameQueue[0];
  mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
  pK->MakeKeyFrame_Rest();
  mMap.vpKeyFrames.push_back(pK);
  // Any measurements? Update the relevant point's measurement counter status map
  for(meas_it it = pK->mMeasurements.begin();
      it!=pK->mMeasurements.end();
      it++)
    {
      it->first->pMMData->sMeasurementKFs.insert(pK);
      it->second.Source = Measurement::SRC_TRACKER;
    }
  
  // And maybe we missed some - this now adds to the map itself, too.
  ReFindInSingleKeyFrame(*pK);
#ifdef IPTAM_MOBILE_PAPER
  if(IPTAMMobile::Config().partialPoints) {
    // Feature completion/refind may already have consumed this soft quantum.
    // Snapshot the eligible candidates before seeding and retain the promotion
    // opportunity for a fresh bounded stage; never silently drop it on expiry.
    QueuePartialPromotion(*pK);
    SeedPartialPoints(*pK);
  } else {
#endif
  AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
  AddSomeMapPoints(0);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);
#ifdef IPTAM_MOBILE_PAPER
  }
#endif
  
  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
}

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(KeyFrame &kSrc, 
				KeyFrame &kTarget, 
				int nLevel,
				int nCandidate)
{
#ifdef IPTAM_MOBILE_PAPER
  ++mDiagnostics.epipolarAttempts;
  auto reject=[this](MappingDiagnostics::Rejection reason){++mDiagnostics.rejected[reason];return false;};
#define EPIPOLAR_REJECT(reason) return reject(MappingDiagnostics::reason)
#else
#define EPIPOLAR_REJECT(reason) return false
#endif
  auto& imUnProj = mimUnprojectCache;
  if(imUnProj.size() != kSrc.aLevels[0].im.size())
    {
      imUnProj.resize(kSrc.aLevels[0].im.size());
      ImageRef ir;
      do imUnProj[ir] = mCamera.UnProject(ir);
      while(ir.next(imUnProj.size()));

    }
  
  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];
  ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);
  
  Vector<3> v3Ray_SC = unproject(mCamera.UnProject(v2RootPos));
  normalize(v3Ray_SC);
  Vector<3> v3LineDirn_TC = kTarget.se3CfromW.get_rotation() * (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);

  // Restrict epipolar search to a relatively narrow depth range
  // to increase reliability
  double dMean = kSrc.dSceneDepthMean;
  double dSigma = kSrc.dSceneDepthSigma;
  double dStartDepth = max(mdWiggleScale, dMean - dSigma);
  double dEndDepth = min(40 * mdWiggleScale, dMean + dSigma);
#ifdef IPTAM_MOBILE_PAPER
  const bool invalidLegacy=!std::isfinite(dStartDepth)||!std::isfinite(dEndDepth)||dEndDepth<=dStartDepth;
  if(IPTAMMobile::Config().repairDepthIntervals) {
    const auto interval=IPTAMMobile::EpipolarDepthInterval(dMean,dSigma,mdWiggleScale,true);
    if(interval.repaired)++mDiagnostics.repairedDepthIntervals;
    dStartDepth=interval.lower;dEndDepth=interval.upper;
    if(!interval.valid) { ++mDiagnostics.invalidDepthIntervals; EPIPOLAR_REJECT(BackwardDepth); }
  }
  mDiagnostics.lastDepthMean=dMean;mDiagnostics.lastDepthSigma=dSigma;
  mDiagnostics.lastDepthStart=dStartDepth;mDiagnostics.lastDepthEnd=dEndDepth;
  if(invalidLegacy)
    ++mDiagnostics.invalidDepthIntervals;
#endif
  
  Vector<3> v3CamCenter_TC = kTarget.se3CfromW * kSrc.se3CfromW.inverse().get_translation(); // The camera end
  Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                               // the far-away end
  Vector<3> v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                               // the far-away end

  
  if(v3RayEnd_TC[2] <= v3RayStart_TC[2])  // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
    { EPIPOLAR_REJECT(BackwardDepth); }
  if(v3RayEnd_TC[2] <= 0.0 )  { EPIPOLAR_REJECT(BehindDepth); }
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);
  
  Vector<2> v2A = project(v3RayStart_TC);
  Vector<2> v2B = project(v3RayEnd_TC);
  Vector<2> v2AlongProjectedLine = v2A-v2B;
  
  if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
    {
      cout << "v2AlongProjectedLine too small." << endl;
      EPIPOLAR_REJECT(DegenerateRay);
    }
  normalize(v2AlongProjectedLine);
  Vector<2> v2Normal;
  v2Normal[0] = v2AlongProjectedLine[1];
  v2Normal[1] = -v2AlongProjectedLine[0];
  
  double dNormDist = v2A * v2Normal;
  if(fabs(dNormDist) > mCamera.LargestRadiusInImage() )
    { EPIPOLAR_REJECT(OutsideLine); }
  
  double dMinLen = min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
  double dMaxLen = max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;

  // Find current-frame corners which might match this
  PatchFinder Finder;
  Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
  if(Finder.TemplateBad())  { EPIPOLAR_REJECT(BadTemplate); }
  
  vector<Vector<2> > &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
  vector<ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
  if(!kTarget.aLevels[nLevel].bImplaneCornersCached)
    {
      for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
	vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
      kTarget.aLevels[nLevel].bImplaneCornersCached = true;
    }
  
  int nBest = -1;
  int nBestZMSSD = Finder.mnMaxSSD + 1;
  double dMaxDistDiff = mCamera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
  double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;
  
  for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
    {
      Vector<2> v2Im = vv2Corners[i];
      double dDistDiff = dNormDist - v2Im * v2Normal;
      if(dDistDiff * dDistDiff > dMaxDistSq)	continue; // skip if not along epi line
      if(v2Im * v2AlongProjectedLine < dMinLen)	continue; // skip if not far enough along line
      if(v2Im * v2AlongProjectedLine > dMaxLen)	continue; // or too far
      int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
      if(nZMSSD < nBestZMSSD)
	{
	  nBest = i;
	  nBestZMSSD = nZMSSD;
	}
    } 
  
  if(nBest == -1)   { EPIPOLAR_REJECT(NoMatch); }   // Nothing found.
  
  //  Found a likely candidate along epipolar ray
  Finder.MakeSubPixTemplate();
  Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget,10);
  if(!bSubPixConverges)
    { EPIPOLAR_REJECT(Subpixel); }
  
  // Now triangulate the 3d point...
  Vector<3> v3New;
  v3New = kTarget.se3CfromW.inverse() *  
    ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
		   mCamera.UnProject(v2RootPos), 
		   mCamera.UnProject(Finder.GetSubPixPos()));
  
  if(!std::isfinite(v3New*v3New) || (kSrc.se3CfromW*v3New)[2] <= 0.001 || (kTarget.se3CfromW*v3New)[2] <= 0.001) { EPIPOLAR_REJECT(Cheirality); }
  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3New;
  pNew->pMMData = new MapMakerData();
  
  // Patch source stuff:
  pNew->pPatchSourceKF = &kSrc;
  pNew->nSourceLevel = nLevel;
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNew->irCenter = irLevelPos;
  pNew->v3Center_NC = unproject(mCamera.UnProject(v2RootPos));
  pNew->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromCenter_NC  = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));
  
  normalize(pNew->v3Center_NC);
  normalize(pNew->v3OneDownFromCenter_NC);
  normalize(pNew->v3OneRightFromCenter_NC);
  
  pNew->RefreshPixelVectors();
    
  mMap.vpPoints.push_back(pNew);
  mqNewQueue.push(pNew);
  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2RootPos;
  m.nLevel = nLevel;
  m.bSubPix = true;
  kSrc.mMeasurements[pNew] = m;

  m.Source = Measurement::SRC_EPIPOLAR;
  m.v2RootPos = Finder.GetSubPixPos();
  kTarget.mMeasurements[pNew] = m;
  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  pNew->pMMData->sMeasurementKFs.insert(&kTarget);
#ifdef IPTAM_MOBILE_PAPER
  ++mDiagnostics.epipolarAccepted;
#endif
#undef EPIPOLAR_REJECT
  return true;
}

double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2)
{
  Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dDist = sqrt(v3Diff * v3Diff);
  return dDist;
}

vector<KeyFrame*> MapMaker::NClosestKeyFrames(KeyFrame &k, unsigned int N)
{
  vector<pair<double, KeyFrame* > > vKFandScores;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFrames[i]));
    }
  if(N > vKFandScores.size())
    N = vKFandScores.size();
  std::stable_sort(vKFandScores.begin(), vKFandScores.end(), [](const pair<double,KeyFrame*>& a,const pair<double,KeyFrame*>& b){return a.first<b.first;});
  
  vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  return vResult;
}

KeyFrame* MapMaker::ClosestKeyFrame(KeyFrame &k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      if(dDist < dClosestDist)
	{
	  dClosestDist = dDist;
	  nClosest = i;
	}
    }
  assert(nClosest != -1);
  return mMap.vpKeyFrames[nClosest];
}

double MapMaker::DistToNearestKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
#ifdef IPTAM_MOBILE_PAPER
  if(IPTAMMobile::Config().cullKeyframes)for(auto* frame:mMap.vpRetainedKeyFrames)
    dDist=std::min(dDist,KeyFrameLinearDist(kCurrent,*frame));
#endif
  return dDist;
}

bool MapMaker::NeedNewKeyFrame(KeyFrame &kCurrent)
{
  double dDist = DistToNearestKeyFrame(kCurrent);
  dDist *= (1.0 / kCurrent.dSceneDepthMean);
  
  if(dDist > MapMakerMaxKFDistWiggleMult * mdWiggleScaleDepthNormalized)
    return true;
  return false;
}

// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll()
{
  // construct the sets of kfs/points to be adjusted:
  // in this case, all of them
  set<KeyFrame*> sAdj;
  set<KeyFrame*> sFixed;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    if(mMap.vpKeyFrames[i]->bFixed)
      sFixed.insert(mMap.vpKeyFrames[i]);
    else
      sAdj.insert(mMap.vpKeyFrames[i]);
  
  set<MapPoint*> sMapPoints;
  for(unsigned int i=0; i<mMap.vpPoints.size();i++)
    sMapPoints.insert(mMap.vpPoints[i]);
  
  BundleAdjust(sAdj, sFixed, sMapPoints, false);
}

// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecent()
{
  if(mMap.vpKeyFrames.size() < 8)  
    { // Ignore this unless map is big enough
      mbBundleConverged_Recent = true;
      return;
    }

  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);
  vector<KeyFrame*> vClosest = NClosestKeyFrames(*pkfNewest, 4);
  for(int i=0; i<4; i++)
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);
  
  // Now we find the set of features which they contain.
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter!=sAdjustSet.end();
      iter++)
    {
      map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
      for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
	sMapPoints.insert(jiter->first);
    };
  
  // Finally, add all keyframes which measure above points as fixed keyframes
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = mMap.vpKeyFrames.begin(); it!=mMap.vpKeyFrames.end(); it++)
    {
      if(sAdjustSet.count(*it))
	continue;
      bool bInclude = false;
      for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
	if(sMapPoints.count(jiter->first))
	  {
	    bInclude = true;
	    break;
	  }
      if(bInclude)
	sFixedSet.insert(*it);
    }
  
  BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, true);
}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void MapMaker::BundleAdjust(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, bool bRecent)
{
#ifdef IPTAM_MOBILE_PAPER
  if(IPTAMMobile::Config().rollingShutter)for(auto* frame:mMap.vpKeyFrames)
    if(sAdjustSet.count(frame)||sFixedSet.count(frame))RefreshMobileRollingShutter(*frame);
#endif
  Bundle b(mCamera);   // Our bundle adjuster
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;
  
  // The bundle adjuster does different accounting of keyframes and map points;
  // Translation maps are stored:
  map<MapPoint*, int> mPoint_BundleID;
  map<int, MapPoint*> mBundleID_Point;
  map<KeyFrame*, int> mView_BundleID;
  map<int, KeyFrame*> mBundleID_View;
  
  // Preserve insertion order: pointer-ordered sets/maps vary with allocator
  // layout and otherwise change numerical accumulation and replay results.
  for(auto* k:mMap.vpKeyFrames) if(sAdjustSet.count(k)) {
    bool fixed=k->bFixed;
#ifdef IPTAM_MOBILE_PAPER
    if(IPTAMMobile::Config().cullMeasurements||IPTAMMobile::Config().cullKeyframes) {
      unsigned int support=0;for(const auto& measurement:k->mMeasurements)
        if(!measurement.first->bBad&&sMapPoints.count(measurement.first))++support;
      if(!fixed&&support<3){fixed=true;++mDiagnostics.fixedLowSupportViews;}
    }
#endif
    int id=b.AddCamera(k->se3CfromW,fixed);
    mView_BundleID[k]=id; mBundleID_View[id]=k;
  }
  for(auto* k:mMap.vpKeyFrames) if(sFixedSet.count(k)) {
    int id=b.AddCamera(k->se3CfromW,true);
    mView_BundleID[k]=id; mBundleID_View[id]=k;
  }
  for(auto* p:mMap.vpPoints) if(sMapPoints.count(p)) {
    int id=b.AddPoint(p->v3WorldPos);
    mPoint_BundleID[p]=id; mBundleID_Point[id]=p;
  }
  for(auto* k:mMap.vpKeyFrames) {
    if(!mView_BundleID.count(k)) continue;
    std::vector<std::pair<int,Measurement*>> ordered;
    for(auto& measurement:k->mMeasurements) {
      auto point=mPoint_BundleID.find(measurement.first);
      if(point!=mPoint_BundleID.end()) ordered.emplace_back(point->second,&measurement.second);
    }
    std::sort(ordered.begin(),ordered.end(),[](const std::pair<int,Measurement*>& a,const std::pair<int,Measurement*>& b){return a.first<b.first;});
    for(const auto& measurement:ordered) {
      auto& m=*measurement.second;
      b.AddMeas(mView_BundleID[k],measurement.first,
#ifdef IPTAM_MOBILE_PAPER
                m.BundlePosition(),
#else
                m.v2RootPos,
#endif
                LevelScale(m.nLevel)*LevelScale(m.nLevel));
    }
  }

  // Run the bundle adjuster. This returns the number of successful iterations
  int nAccepted = b.Compute(&mbBundleAbortRequested);
#ifdef IPTAM_MOBILE_PAPER
  if(b.BudgetStopped())++mDiagnostics.budgetStops;
#endif
  if(b.StoppedAtAcceptedEstimate()) {
    ++mnRetainedSolveStops;
    msLastNumericalFailure=b.FailureReason();
    std::cerr << "Bundle bounded stop: " << msLastNumericalFailure << std::endl;
  }
  
  if(nAccepted < 0)
    {
      ++mnNumericalFailures;
      msLastNumericalFailure=b.FailureReason();
      std::cerr << "Bundle solve rejected: " << msLastNumericalFailure
                << (mMap.IsGood()?"; previous map retained.":"; initialization rejected.") << std::endl;
      // Bundle works on copies; do not apply any failed trial. Wait for a new
      // keyframe before retrying this system instead of repeating every frame.
      mbBundleRunning=false; mbBundleAbortRequested=false;
      mbBundleConverged_Recent=true; mbBundleConverged_Full=true;
      if(!mMap.IsGood()) mbResetRequested=true;
      return;
    }

  // Bundle adjustment did some updates, apply these to the map
  if(nAccepted > 0)
    {
      
      for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
	  itr!=mPoint_BundleID.end();
	  itr++)
	itr->first->v3WorldPos = b.GetPoint(itr->second);
      
      for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
	  itr!=mView_BundleID.end();
	  itr++)
	itr->first->se3CfromW = b.GetCamera(itr->second);
#ifdef IPTAM_MOBILE_PAPER
      RefreshRetainedPoses();
#endif
      // BA changes both point depths and source camera poses; refresh the
      // geometric pixel footprint before any subsequent patch warp.
      for(auto* point:mMap.vpPoints) point->RefreshPixelVectors();
      if(bRecent)
	mbBundleConverged_Recent = false;
      mbBundleConverged_Full = false;
    };
  
  if(b.Converged())
    {
      mbBundleConverged_Recent = true;
      if(!bRecent)
	mbBundleConverged_Full = true;
    }
  
  mbBundleRunning = false;
  mbBundleAbortRequested = false;
  
  // Handle outlier measurements:
  vector<pair<int,int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
  for(unsigned int i=0; i<vOutliers_PC_pair.size(); i++)
    {
      MapPoint *pp = mBundleID_Point[vOutliers_PC_pair[i].first];
      KeyFrame *pk = mBundleID_View[vOutliers_PC_pair[i].second];
      Measurement &m = pk->mMeasurements[pp];
      if(pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT)   // Is the original source kf considered an outlier? That's bad.
	pp->bBad = true;
      else
	{
	  // Do we retry it? Depends where it came from!!
	  if(m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
	    mvFailureQueue.push_back(pair<KeyFrame*,MapPoint*>(pk,pp));
	  else
	    pp->pMMData->sNeverRetryKFs.insert(pk);
	  pk->mMeasurements.erase(pp);
	  pp->pMMData->sMeasurementKFs.erase(pk);
	}
    }
}

// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in 
// TrackerData.h.
bool MapMaker::ReFind_Common(KeyFrame &k, MapPoint &p)
{
  // Failure queues may still reference points safely retained in map trash.
  // They must never acquire new measurements after retirement.
  if(p.bBad) return false;
  // abort if either a measurement is already in the map, or we've
  // decided that this point-kf combo is beyond redemption
  if(p.pMMData->sMeasurementKFs.count(&k)
     || p.pMMData->sNeverRetryKFs.count(&k))
    return false;
  
  PatchFinder Finder;
  Vector<3> v3Cam = k.se3CfromW*p.v3WorldPos;
  if(v3Cam[2] < 0.001)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  Vector<2> v2ImPlane = project(v3Cam);
  if(v2ImPlane* v2ImPlane > mCamera.LargestRadiusInImage() * mCamera.LargestRadiusInImage())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  Vector<2> v2Image = mCamera.Project(v2ImPlane);
  if(mCamera.Invalid())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  ImageRef irImageSize = k.aLevels[0].im.size();
  if(!std::isfinite(v2Image*v2Image) || v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] >= irImageSize[0] || v2Image[1] >= irImageSize[1])
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  Matrix<2> m2CamDerivs = mCamera.GetProjectionDerivs();
  Finder.MakeTemplateCoarse(p, k.se3CfromW, m2CamDerivs);
  
  if(Finder.TemplateBad())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  bool bFound = Finder.FindPatchCoarse(ir(v2Image), k, 4);  // Very tight search radius!
  if(!bFound)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  // If we found something, generate a measurement struct and put it in the map
  Measurement m;
  m.nLevel = Finder.GetLevel();
  m.Source = Measurement::SRC_REFIND;
  
  if(Finder.GetLevel() > 0)
    {
      Finder.MakeSubPixTemplate();
      if(!Finder.IterateSubPixToConvergence(k,8)) {
        p.pMMData->sNeverRetryKFs.insert(&k); return false;
      }
      m.v2RootPos = Finder.GetSubPixPos();
      m.bSubPix = true;
    }
  else
    {
      m.v2RootPos = Finder.GetCoarsePosAsVector();
      m.bSubPix = false;
    };
  
  if(k.mMeasurements.count(&p))
    {
      assert(0); // This should never happen, we checked for this at the start.
    }
  k.mMeasurements[&p] = m;
  p.pMMData->sMeasurementKFs.insert(&k);
  return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(KeyFrame &k)
{
  vector<MapPoint*> vToFind;
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    vToFind.push_back(mMap.vpPoints[i]);
  
  int nFoundNow = 0;
  for(unsigned int i=0; i<vToFind.size(); i++)
    if(ReFind_Common(k,*vToFind[i]))
      nFoundNow++;

  return nFoundNow;
};

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade()
{
  if(mqNewQueue.empty())
    return;
  int nFound = 0;
  int nBad = 0;
  while(!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0)
    {
      MapPoint* pNew = mqNewQueue.front();
      mqNewQueue.pop();
      if(pNew->bBad)
	{
	  nBad++;
	  continue;
	}
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	if(ReFind_Common(*mMap.vpKeyFrames[i], *pNew))
	  nFound++;
    }
};

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue()
{
  if(mvFailureQueue.size() == 0)
    return;
  // Keep deterministic insertion order, independent of allocator addresses.
  vector<pair<KeyFrame*, MapPoint*> >::iterator it;
  int nFound=0;
  for(it = mvFailureQueue.begin(); it!=mvFailureQueue.end(); it++)
    if(ReFind_Common(*it->first, *it->second))
      nFound++;
  
  mvFailureQueue.erase(mvFailureQueue.begin(), it);
};

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent)
{
  return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> MapMaker::CalcPlaneAligner()
{
  unsigned int nPoints = mMap.vpPoints.size();
  if(nPoints < 10)
    {
      cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
      return SE3<>();
    };
  
  int nRansacs = MapMakerPlaneAlignerRansacs;
  Vector<3> v3BestMean;
  Vector<3> v3BestNormal;
  double dBestDistSquared = 9999999999999999.9;
  
  for(int i=0; i<nRansacs; i++)
    {
      int nA = mRng()%nPoints;
      int nB = nA;
      int nC = nA;
      while(nB == nA)
	nB = mRng()%nPoints;
      while(nC == nA || nC==nB)
	nC = mRng()%nPoints;
      
      Vector<3> v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos + 
				       mMap.vpPoints[nB]->v3WorldPos + 
				       mMap.vpPoints[nC]->v3WorldPos);
      
      Vector<3> v3CA = mMap.vpPoints[nC]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3BA = mMap.vpPoints[nB]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3Normal = v3CA ^ v3BA;
      if(v3Normal * v3Normal  == 0)
	continue;
      normalize(v3Normal);
      
      double dSumError = 0.0;
      for(unsigned int i=0; i<nPoints; i++)
	{
	  Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
	  double dDistSq = v3Diff * v3Diff;
	  if(dDistSq == 0.0)
	    continue;
	  double dNormDist = fabs(v3Diff * v3Normal);
	  
	  if(dNormDist > 0.05)
	    dNormDist = 0.05;
	  dSumError += dNormDist;
	}
      if(dSumError < dBestDistSquared)
	{
	  dBestDistSquared = dSumError;
	  v3BestMean = v3Mean;
	  v3BestNormal = v3Normal;
	}
    }
  
  if(dBestDistSquared == 9999999999999999.9) return SE3<>();
  // Done the ransacs, now collect the supposed inlier set
  vector<Vector<3> > vv3Inliers;
  for(unsigned int i=0; i<nPoints; i++)
    {
      Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
      double dDistSq = v3Diff * v3Diff;
      if(dDistSq == 0.0)
	continue;
      double dNormDist = fabs(v3Diff * v3BestNormal);
      if(dNormDist < 0.05)
	vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
    }
  
  if(vv3Inliers.size()<3) return SE3<>();
  // With these inliers, calculate mean and cov
  Vector<3> v3MeanOfInliers = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    v3MeanOfInliers+=vv3Inliers[i];
  v3MeanOfInliers *= (1.0 / vv3Inliers.size());
  
  Matrix<3> m3Cov = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
      Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
      m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };
  
  // Find the principal component with the minimal variance: this is the plane normal
  SymEigen<3> sym(m3Cov);
  Vector<3> v3Normal = sym.get_evectors()[0];
  
  // Use the version of the normal which points towards the cam center
  if(v3Normal[2] > 0)
    v3Normal *= -1.0;
  
  Matrix<3> m3Rot = Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];
  
  SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;
  
  return se3Aligner;
}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
bool MapMaker::RefreshSceneDepth(KeyFrame *pKF)
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  for(auto* p:mMap.vpPoints)
    {
      if(!pKF->mMeasurements.count(p)) continue;
      MapPoint &point = *p;
      Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
      if(!std::isfinite(v3PosK*v3PosK) || v3PosK[2]<=0) continue;
      dSumDepth += v3PosK[2];
      dSumDepthSquared += v3PosK[2] * v3PosK[2];
      nMeas++;
    }
 
  if(nMeas < 3) return false;
  pKF->dSceneDepthMean = dSumDepth / nMeas;
  pKF->dSceneDepthSigma = sqrt(std::max(0.0, (dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean)));
  return std::isfinite(pKF->dSceneDepthMean) && std::isfinite(pKF->dSceneDepthSigma);
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="SaveMap")
    {
      cout << "  MapMaker: Saving the map.... " << endl;
      ofstream ofs("map.dump");
      for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
	{
	  ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
	  ofs << mMap.vpPoints[i]->nSourceLevel << endl;
	}
      ofs.close();
      
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	{
	  ostringstream ost1;
	  ost1 << "keyframes/" << i << ".jpg";
//	  img_save(mMap.vpKeyFrames[i]->aLevels[0].im, ost1.str());
	  
	  ostringstream ost2;
	  ost2 << "keyframes/" << i << ".info";
	  ofstream ofs2;
	  ofs2.open(ost2.str().c_str());
	  ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
	  ofs2.close();
	}
      cout << "  ... done saving map." << endl;
      return;
    }
  
  cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; 












