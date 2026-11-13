#include "MapMaker.h"
#ifdef IPTAM_MOBILE_PAPER
#include "MobileConfig.h"
#include "MobileMappingBudget.h"
#include "MobileMappingGeometry.h"
#include "MapPoint.h"
#include "PatchFinder.h"
#include "LevelHelpers.h"
#include "SmallMatrixOpts.h"
#include "MEstimator.h"
#include "RollingShutter.h"
#include <algorithm>
#include <cmath>
#include <queue>

namespace {
struct InformationView {KeyFrame* frame;TooN::Matrix<3> information;double utility;std::size_t order;};
std::vector<InformationView> pointInformation(MapPoint& point,const std::vector<KeyFrame*>& frames,ATANCamera camera) {
  std::vector<InformationView> result;TooN::Matrix<3> total=TooN::Zeros;
  for(std::size_t order=0;order<frames.size();++order) {
    auto* frame=frames[order];auto measurement=frame->mMeasurements.find(&point);
    if(measurement==frame->mMeasurements.end())continue;
    TooN::Matrix<3> information=TooN::Zeros;
    const auto p=frame->se3CfromW*point.v3WorldPos;
    if(std::isfinite(p*p)&&p[2]>.001) {
      const auto projected=camera.Project(TooN::makeVector(p[0]/p[2],p[1]/p[2]));
      if(!camera.Invalid()&&std::isfinite(projected*projected)) {
        const auto deriv=camera.GetProjectionDerivs();TooN::Matrix<2,3> perspective=TooN::Zeros;
        perspective[0][0]=perspective[1][1]=1/p[2];perspective[0][2]=-p[0]/(p[2]*p[2]);perspective[1][2]=-p[1]/(p[2]*p[2]);
        const TooN::Matrix<2,3> jacobian=deriv*perspective*frame->se3CfromW.get_rotation().get_matrix();
        const double scale=LevelScale(measurement->second.nLevel);
        const auto residual=(measurement->second.BundlePosition()-projected)/scale;
        const double weight=Tukey::Weight(residual*residual,16.);
        if(std::isfinite(weight)&&weight>0)information=weight/(scale*scale)*jacobian.T()*jacobian;
      }
    }
    total+=information;result.push_back({frame,information,0,order});
  }
  // Scale before determinants to avoid overflow; ordering is unchanged.
  double scale=0;for(int i=0;i<3;++i)scale=std::max(scale,std::abs(total[i][i]));
  if(scale>0&&std::isfinite(scale)) {
    const double determinant=std::max(0.,M3Det(total/scale));
    for(auto& view:result) {
      view.utility=determinant-M3Det((total-view.information)/scale);
      if(!std::isfinite(view.utility))view.utility=0;view.utility=std::max(0.,view.utility);
    }
  }
  std::stable_sort(result.begin(),result.end(),[](const InformationView& a,const InformationView& b){return a.utility>b.utility;});
  return result;
}
bool graphConnectedWithout(const std::vector<KeyFrame*>& frames,const std::vector<MapPoint*>& points,KeyFrame* omitted) {
  KeyFrame* root=nullptr;for(auto* frame:frames)if(frame!=omitted&&frame->bFixed){root=frame;break;}
  if(!root)return false;std::set<KeyFrame*> visited{root};std::queue<KeyFrame*> pending;pending.push(root);
  while(!pending.empty()) {
    auto* frame=pending.front();pending.pop();
    for(auto* point:points) {
      if(point->bBad||!point->pMMData||!point->pMMData->sMeasurementKFs.count(frame))continue;
      for(auto* neighbour:frames)if(neighbour!=omitted&&point->pMMData->sMeasurementKFs.count(neighbour)&&visited.insert(neighbour).second)pending.push(neighbour);
    }
  }
  return visited.size()==frames.size()-(omitted?1:0);
}
}

std::size_t MapMaker::ActivePointLimit() const {
  const auto& config=IPTAMMobile::Config();
  if(config.partialPoints||config.cullMeasurements||config.cullKeyframes||config.coarseTracking)
    return std::min<std::size_t>(MaxMapPoints,std::max(10,config.maxActivePoints));
  return MaxMapPoints;
}
void MapMaker::CheckMobileResources() {
  const auto& config=IPTAMMobile::Config();
  if(!config.cullKeyframes && !config.cullMeasurements && !config.partialPoints)return;
  // Stop adding work rather than invalidating source pixels or dangling queue pointers.
  if(mMap.vpRetainedKeyFrames.size()>=128 || mMap.vpPointsTrash.size()>=2000) {
    mbMobileResourceLimit=true;msMobileResourceReason=mMap.vpRetainedKeyFrames.size()>=128?
      "retained keyframe memory cap (128)":"retired point memory cap (2000)";
  }
}
void MapMaker::RefreshRetainedPoses() {
  for(const auto& record:mRetainedPoseAnchors)
    record.first->se3CfromW=record.second.second*record.second.first->se3CfromW;
}
void MapMaker::RefreshMobileRollingShutter(KeyFrame& frame) {
  const auto& config=IPTAMMobile::Config();
  if(!config.rollingShutter)return;
  IPTAMMobile::RollingShutterCalibration calibration;
  calibration.known=true;calibration.width=frame.aLevels[0].im.size().x;calibration.height=frame.aLevels[0].im.size().y;
  calibration.axis=config.readoutAxis;calibration.direction=config.readoutDirection;
  calibration.readoutSeconds=config.readoutSeconds;calibration.timestampFraction=config.exposureTimestampFraction;
  std::vector<IPTAMMobile::RollingShutterObservation> observations;
  std::vector<Measurement*> measurements;
  for(auto& entry:frame.mMeasurements) {
    auto& measurement=entry.second;
    if(entry.first->bBad)continue;
    if(!measurement.hasRawPosition) {
      // ROOT/EPIPOLAR measurements have original raw positions but no adjacent
      // video observations. They still receive the frame's fitted correction.
      measurement.v2RawRootPos=measurement.v2RootPos;measurement.hasRawPosition=true;
      if(frame.hasCaptureTimestamp)measurement.timestamp=frame.captureTimestamp;
    }
    const auto p=frame.se3CfromW*entry.first->v3WorldPos;
    if(!std::isfinite(p*p)||p[2]<=.001)continue;
    const auto projected=mCamera.Project(project(p));
    if(mCamera.Invalid()||!std::isfinite(projected*projected))continue;
    const auto derivative=mCamera.GetProjectionDerivs();TooN::Matrix<2,6> jacobian=TooN::Zeros;
    for(int axis=0;axis<6;++axis) {
      const auto motion=SE3<>::generator_field(axis,unproject(p));
      jacobian.T()[axis]=derivative*TooN::makeVector((motion[0]-p[0]*motion[2]/p[2])/p[2],(motion[1]-p[1]*motion[2]/p[2])/p[2]);
    }
    IPTAMMobile::RollingShutterObservation observation;
    observation.pixel=measurement.v2RawRootPos;observation.poseJacobian=jacobian;
    observation.timestamp=measurement.timestamp;
    observation.hasPrevious=measurement.hasPrevious;observation.hasNext=measurement.hasNext;
    observation.previousPixel=measurement.v2PreviousRootPos;observation.nextPixel=measurement.v2NextRootPos;
    observation.previousTimestamp=measurement.previousTimestamp;observation.nextTimestamp=measurement.nextTimestamp;
    observations.push_back(observation);measurements.push_back(&measurement);
  }
  const auto estimate=IPTAMMobile::RollingShutter::EstimateVelocity(observations,calibration);
  // Clear stale corrections if current geometry no longer supports the velocity.
  // Raw image measurements never change and corrections never accumulate.
  for(auto& entry:frame.mMeasurements)entry.second.rollingShutterCorrected=false;
  if(!estimate.valid)return;
  for(std::size_t i=0;i<measurements.size();++i) {
    TooN::Vector<2> corrected;std::string reason;
    if(IPTAMMobile::RollingShutter::Correct(observations[i].pixel,observations[i].poseJacobian,estimate,calibration,corrected,reason)) {
      measurements[i]->v2BundleRootPos=corrected;measurements[i]->rollingShutterCorrected=true;++mDiagnostics.bundleRSCorrections;
    }
  }
}
void MapMaker::CullMobileMap() {
  auto& config=IPTAMMobile::Config();
  // Shared-landmark adjacency lets each proposed observation removal preserve
  // connection to the fixed gauge and at least three measurements per free view.
  // When these invariants conflict with the nominal observation cap, retain the
  // bridge explicitly rather than silently disconnecting the BA graph.
  const std::size_t observationLimit=std::max(2,config.maxObservations);
  const bool needsObservationCull=config.cullMeasurements&&std::any_of(mMap.vpPoints.begin(),mMap.vpPoints.end(),[&](MapPoint* point){
    return !point->bBad&&point->pMMData&&point->pMMData->sMeasurementKFs.size()>observationLimit;
  });
  if(needsObservationCull) {
    const auto& frames=mMap.vpKeyFrames;
    std::map<KeyFrame*,std::size_t> index;
    for(std::size_t i=0;i<frames.size();++i)index[frames[i]]=i;
    std::vector<std::vector<unsigned int>> edges(frames.size(),std::vector<unsigned int>(frames.size(),0));
    std::vector<unsigned int> support(frames.size(),0);
    for(auto* point:mMap.vpPoints)if(!point->bBad&&point->pMMData) {
      std::vector<std::size_t> views;
      for(auto* frame:point->pMMData->sMeasurementKFs)if(index.count(frame)&&frame->mMeasurements.count(point)) {
        views.push_back(index[frame]);++support[index[frame]];
      }
      for(auto i:views)for(auto j:views)if(i!=j)++edges[i][j];
    }
    auto connected=[&] {
      std::size_t root=frames.size();for(std::size_t i=0;i<frames.size();++i)if(frames[i]->bFixed){root=i;break;}
      if(root==frames.size())return false;
      std::vector<bool> seen(frames.size(),false);std::queue<std::size_t> queue;queue.push(root);seen[root]=true;
      std::size_t visited=0;
      while(!queue.empty()) {const auto i=queue.front();queue.pop();++visited;
        for(std::size_t j=0;j<frames.size();++j)if(edges[i][j]&&!seen[j]){seen[j]=true;queue.push(j);}
      }
      return visited==frames.size();
    };
    bool changed=false;
    for(auto* point:mMap.vpPoints) {
      if(point->bBad||!point->pMMData||point->pMMData->sMeasurementKFs.size()<=observationLimit)continue;
      const std::size_t limit=observationLimit;
      auto ranked=pointInformation(*point,frames,mCamera);
      if(ranked.size()<=limit)continue;
      std::set<KeyFrame*> keep;
      if(index.count(point->pPatchSourceKF))keep.insert(point->pPatchSourceKF);
      for(const auto& view:ranked)if(keep.size()<limit)keep.insert(view.frame);
      for(const auto& view:ranked)if(!keep.count(view.frame)) {
        const auto i=index[view.frame];
        if(!view.frame->bFixed&&support[i]<=3){++mDiagnostics.observationSupportRetentions;continue;}
        std::vector<std::size_t> others;
        for(auto* frame:point->pMMData->sMeasurementKFs)if(frame!=view.frame&&index.count(frame)&&frame->mMeasurements.count(point))others.push_back(index[frame]);
        for(auto j:others){--edges[i][j];--edges[j][i];}
        if(!connected()) {
          for(auto j:others){++edges[i][j];++edges[j][i];}
          ++mDiagnostics.observationBridgeRetentions;continue;
        }
        --support[i];view.frame->mMeasurements.erase(point);point->pMMData->sMeasurementKFs.erase(view.frame);
        point->pMMData->sNeverRetryKFs.insert(view.frame);++mDiagnostics.culledMeasurements;changed=true;
      }
    }
    if(changed)mbBundleConverged_Full=mbBundleConverged_Recent=false;
  }
  if(!config.cullKeyframes)return;
  const std::size_t limit=std::max(3,config.maxActiveKeyframes);
  while(mMap.vpKeyFrames.size()>limit&&!mbMobileResourceLimit) {
    std::map<KeyFrame*,double> votes;for(auto* frame:mMap.vpKeyFrames)votes[frame]=0;
    for(auto* point:mMap.vpPoints) {
      if(point->bBad)continue;auto ranked=pointInformation(*point,mMap.vpKeyFrames,mCamera);
      if(ranked.size()==2)for(auto& view:ranked)votes[view.frame]+=.5;
      else {const double top=std::min<std::size_t>(4,ranked.size()),normalizer=top*(top+1)/2;
        for(std::size_t i=0;i<ranked.size()&&i<top;++i)votes[ranked[i].frame]+=(top-i)/normalizer;}
    }
    KeyFrame* selected=nullptr;double score=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i+2<mMap.vpKeyFrames.size();++i) {
      auto* frame=mMap.vpKeyFrames[i];if(frame->bFixed||!graphConnectedWithout(mMap.vpKeyFrames,mMap.vpPoints,frame))continue;
      if(votes[frame]<score){score=votes[frame];selected=frame;}
    }
    if(!selected) {mbMobileResourceLimit=true;msMobileResourceReason="no connected gauge-preserving keyframe cull available";break;}
    KeyFrame* anchor=nullptr;double distance=std::numeric_limits<double>::infinity();
    for(auto* frame:mMap.vpKeyFrames)if(frame!=selected) {
      const double candidate=KeyFrameLinearDist(*selected,*frame);
      if(candidate<distance){distance=candidate;anchor=frame;}
    }
    for(auto& record:mRetainedPoseAnchors)if(record.second.first==selected)
      record.second={anchor,record.first->se3CfromW*anchor->se3CfromW.inverse()};
    mRetainedPoseAnchors[selected]={anchor,selected->se3CfromW*anchor->se3CfromW.inverse()};
    for(auto* point:mMap.vpPoints)if(point->pMMData) {
      point->pMMData->sMeasurementKFs.erase(selected);point->pMMData->sNeverRetryKFs.erase(selected);
      if(point->pMMData->GoodMeasCount()<2)point->bBad=true;
    }
    selected->mMeasurements.clear();
    const auto pendingBefore=mPendingPartialPromotions.size();
    mPendingPartialPromotions.erase(std::remove_if(mPendingPartialPromotions.begin(),mPendingPartialPromotions.end(),[selected](const PendingPartialPromotion& task){return task.target==selected;}),mPendingPartialPromotions.end());
    mDiagnostics.deferredPartialCancelled+=pendingBefore-mPendingPartialPromotions.size();
    // Provisional candidates need an active source observation at promotion.
    // Retained pixel images serve full landmarks/relocalisation only.
    mPartialLandmarks.erase(std::remove_if(mPartialLandmarks.begin(),mPartialLandmarks.end(),[selected](const PartialLandmark& partial){return partial.source==selected;}),mPartialLandmarks.end());
    mPartialObservationCursor=mPartialLandmarks.empty()?0:mPartialObservationCursor%mPartialLandmarks.size();
    mMap.vpKeyFrames.erase(std::find(mMap.vpKeyFrames.begin(),mMap.vpKeyFrames.end(),selected));
    mMap.vpRetainedKeyFrames.push_back(selected);
    mvFailureQueue.erase(std::remove_if(mvFailureQueue.begin(),mvFailureQueue.end(),[selected](const std::pair<KeyFrame*,MapPoint*>& pair){return pair.first==selected;}),mvFailureQueue.end());
    ++mDiagnostics.culledKeyframes;mbBundleConverged_Full=mbBundleConverged_Recent=false;
    HandleBadPoints();CheckMobileResources();
  }
}

std::size_t MapMaker::PartialPointCount() const { return mPartialLandmarks.size(); }
void MapMaker::SeedPartialPoints(KeyFrame& source) {
  if(!IPTAMMobile::Config().partialPoints||mbMobileResourceLimit)return;
  double inverseMean=0;unsigned int n=0;
  // Measurement maps are keyed by allocator addresses. Sum the same owned,
  // measured landmarks in stable map insertion order so reset/allocation layout
  // cannot perturb the provisional depth prior and later patch predictions.
  for(auto* point:mMap.vpPoints) {
    if(!source.mMeasurements.count(point))continue;
    const auto p=source.se3CfromW*point->v3WorldPos;
    if(std::isfinite(p[2])&&p[2]>.001){inverseMean+=1/p[2];++n;}
  }
  if(n)inverseMean/=n;else if(source.dSceneDepthMean>0)inverseMean=1/source.dSceneDepthMean;
  if(!std::isfinite(inverseMean)||inverseMean<=0)return;
  auto candidates=source.aLevels[0].vCandidates;
  std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.dSTScore>b.dSTScore;});
  unsigned int seeded=0;
  for(const auto& candidate:candidates) {
    if(seeded>=static_cast<unsigned int>(std::max(0,IPTAMMobile::Config().maxNewCandidates))||mMap.vpPoints.size()+mPartialLandmarks.size()>=ActivePointLimit())break;
    const auto pixel=vec(candidate.irLevelPos);bool busy=false;
    for(const auto& observation:source.mMeasurements)if((observation.second.v2RootPos-pixel)*(observation.second.v2RootPos-pixel)<100){busy=true;break;}
    for(const auto& partial:mPartialLandmarks)if(partial.source==&source&&(partial.sourceRoot-pixel)*(partial.sourceRoot-pixel)<100){busy=true;break;}
    if(busy)continue;
    PartialLandmark partial;partial.id=mNextPartialId++;partial.source=&source;partial.sourceRoot=pixel;
    partial.sourceRay=unproject(mCamera.UnProject(pixel));partial.inverseDepth=inverseMean;
    mPartialLandmarks.push_back(partial);++seeded;++mDiagnostics.partialSeeded;
  }
}
void MapMaker::QueuePartialPromotion(KeyFrame& target) {
  if(!IPTAMMobile::Config().partialPoints||mPartialLandmarks.empty())return;
  if(std::find(mMap.vpKeyFrames.begin(),mMap.vpKeyFrames.end(),&target)==mMap.vpKeyFrames.end())return;
  for(const auto& task:mPendingPartialPromotions)if(task.target==&target)return;
  PendingPartialPromotion task;task.target=&target;
  const auto maximum=static_cast<std::size_t>(std::max(0,IPTAMMobile::Config().maxNewCandidates));
  for(std::size_t i=0;i<mPartialLandmarks.size()&&task.candidates.size()<maximum;++i) {
    const auto& partial=mPartialLandmarks[(mPartialObservationCursor+i)%mPartialLandmarks.size()];
    if(partial.source!=&target)task.candidates.push_back(partial.id);
  }
  if(!task.candidates.empty()){mPendingPartialPromotions.push_back(std::move(task));++mDiagnostics.deferredPartialQueued;}
}
void MapMaker::ProcessPendingPartialPromotion() {
  if(mPendingPartialPromotions.empty())return;
  auto& task=mPendingPartialPromotions.front();
  if(std::find(mMap.vpKeyFrames.begin(),mMap.vpKeyFrames.end(),task.target)==mMap.vpKeyFrames.end()) {
    mPendingPartialPromotions.pop_front();++mDiagnostics.deferredPartialCancelled;return;
  }
  // Stable candidate IDs survive ordinary observation erasures/compaction.
  // Each task snapshots at most maxNewCandidates, and expiry keeps its cursor.
  std::size_t visits=0;
  const auto maximum=static_cast<std::size_t>(std::max(0,IPTAMMobile::Config().maxNewCandidates));
  while(task.next<task.candidates.size()&&visits<maximum&&!IPTAMMobile::MappingBudgetExpired()) {
    const auto id=task.candidates[task.next];
    const auto processed=ObserveAndPromotePartialPoints(*task.target,true,id);
    if(!processed&&IPTAMMobile::MappingBudgetExpired()&&std::any_of(mPartialLandmarks.begin(),mPartialLandmarks.end(),[id](const PartialLandmark& partial){return partial.id==id;}))break;
    ++task.next;++visits;
  }
  if(task.next==task.candidates.size()) {mPendingPartialPromotions.pop_front();++mDiagnostics.deferredPartialCompleted;}
}
void MapMaker::ObservePartialPoints(KeyFrame& current) {
  if(!IPTAMMobile::Config().partialPoints||!mMap.IsGood())return;
  const double start=SteadyTimeSeconds();ObserveAndPromotePartialPoints(current,false);
  const double ms=(SteadyTimeSeconds()-start)*1000.;++mDiagnostics.stageCalls[MappingDiagnostics::Partial];
  mDiagnostics.stageMs[MappingDiagnostics::Partial]+=ms;mDiagnostics.maxStageMs[MappingDiagnostics::Partial]=std::max(mDiagnostics.maxStageMs[MappingDiagnostics::Partial],ms);
}
std::size_t MapMaker::ObserveAndPromotePartialPoints(KeyFrame& current,bool promote,std::uint64_t onlyId) {
  if(mPartialLandmarks.empty())return 0;
  const std::size_t budget=onlyId?1:std::max(0,IPTAMMobile::Config().maxNewCandidates);
  std::size_t savedCursor=mPartialObservationCursor;
  std::size_t visited=0,index=mPartialObservationCursor%mPartialLandmarks.size();
  if(onlyId) {
    auto match=std::find_if(mPartialLandmarks.begin(),mPartialLandmarks.end(),[onlyId](const PartialLandmark& partial){return partial.id==onlyId;});
    if(match==mPartialLandmarks.end())return 0;
    index=static_cast<std::size_t>(match-mPartialLandmarks.begin());
  }
  while(!mPartialLandmarks.empty()&&visited<budget&&visited<mPartialLandmarks.size()&&!IPTAMMobile::MappingBudgetExpired()) {
    index%=mPartialLandmarks.size();auto& partial=mPartialLandmarks[index];++visited;
    if(partial.source==&current){++index;continue;}
    ++partial.age;bool remove=false,matched=false;
    const auto world=partial.source->se3CfromW.inverse()*(partial.sourceRay/partial.inverseDepth);
    const auto p=current.se3CfromW*world;
    if(std::isfinite(p*p)&&p[2]>.001) {
      const auto projected=mCamera.Project(TooN::makeVector(p[0]/p[2],p[1]/p[2]));
      if(!mCamera.Invalid()&&std::isfinite(projected*projected)&&projected[0]>=0&&projected[1]>=0&&projected[0]<current.aLevels[0].im.size().x&&projected[1]<current.aLevels[0].im.size().y) {
        const auto cameraDerivs=mCamera.GetProjectionDerivs();MapPoint temporary;
        temporary.v3WorldPos=world;temporary.pPatchSourceKF=partial.source;temporary.nSourceLevel=0;temporary.irCenter=CVD::ir_rounded(partial.sourceRoot);
        temporary.v3Normal_NC=TooN::makeVector(0.,0.,-1.);
        temporary.v3Center_NC=partial.sourceRay;normalize(temporary.v3Center_NC);
        temporary.v3OneRightFromCenter_NC=unproject(mCamera.UnProject(partial.sourceRoot+TooN::makeVector(1.,0.)));normalize(temporary.v3OneRightFromCenter_NC);
        temporary.v3OneDownFromCenter_NC=unproject(mCamera.UnProject(partial.sourceRoot+TooN::makeVector(0.,1.)));normalize(temporary.v3OneDownFromCenter_NC);temporary.RefreshPixelVectors();
        PatchFinder finder;finder.SetMobileMatching(true);
        auto derivatives=cameraDerivs;
        if(!temporary.bBad&&finder.CalcSearchLevelAndWarpMatrixAtLevel(temporary,current.se3CfromW,derivatives,1)>=0) {
          finder.MakeTemplateCoarseCont(temporary);
          if(!finder.TemplateBad()&&finder.FindPatchCoarse(CVD::ir_rounded(projected),current,8)) {
            finder.MakeSubPixTemplate();finder.SetSubPixPos(finder.GetCoarsePosAsVector());
            if(finder.IterateSubPixToConvergence(current,8)) {
              const auto observed=finder.GetSubPixPos();const auto residual=observed-projected;
              const auto currentFromSource=current.se3CfromW*partial.source->se3CfromW.inverse();
              const auto dp=currentFromSource.get_rotation()*(-partial.sourceRay/(partial.inverseDepth*partial.inverseDepth));
              const auto j=cameraDerivs*(dp.slice<0,2>()-p.slice<0,2>()*(dp[2]/p[2]))/p[2];
              matched=true;partial.misses=0;
              if(!current.hasCaptureTimestamp||!std::isfinite(partial.lastUpdateTime)||partial.lastUpdateTime<current.captureTimestamp) {
                if(IPTAMMobile::UpdateInverseDepth(partial.inverseDepth,partial.variance,j[0],j[1],residual[0],residual[1],4.)) {
                  ++partial.observations;++mDiagnostics.partialUpdates;partial.lastUpdateTime=current.hasCaptureTimestamp?current.captureTimestamp:std::numeric_limits<double>::quiet_NaN();
                }
              }
              if(promote&&partial.observations>0&&mMap.vpPoints.size()<ActivePointLimit()) {
                const auto sourceRay=unproject(mCamera.UnProject(partial.sourceRoot));
                const auto targetRay=unproject(mCamera.UnProject(observed));
                auto a=sourceRay,b=currentFromSource.get_rotation().inverse()*targetRay;normalize(a);normalize(b);
                const double dot=std::max(-1.,std::min(1.,a*b));
                if(std::sqrt(std::max(0.,1-dot*dot))>2*mCamera.OnePixelDist()) {
                  const auto triangulated=partial.source->se3CfromW.inverse()*ReprojectPoint(currentFromSource,project(targetRay),project(sourceRay));
                  const auto src=partial.source->se3CfromW*triangulated,dst=current.se3CfromW*triangulated;
                  if(std::isfinite(triangulated*triangulated)&&src[2]>.001&&dst[2]>.001) {
                    const auto first=mCamera.Project(project(src));const auto second=mCamera.Project(project(dst));
                    if((first-partial.sourceRoot)*(first-partial.sourceRoot)<=16&&(second-observed)*(second-observed)<=16) {
                      auto* point=new MapPoint;point->v3WorldPos=triangulated;point->pPatchSourceKF=partial.source;point->nSourceLevel=0;point->irCenter=temporary.irCenter;
                      point->v3Normal_NC=temporary.v3Normal_NC;point->v3Center_NC=temporary.v3Center_NC;point->v3OneRightFromCenter_NC=temporary.v3OneRightFromCenter_NC;point->v3OneDownFromCenter_NC=temporary.v3OneDownFromCenter_NC;point->RefreshPixelVectors();
                      point->pMMData=new MapMakerData;
                      Measurement m;m.nLevel=0;m.bSubPix=true;m.Source=Measurement::SRC_ROOT;m.v2RootPos=partial.sourceRoot;
                      // A retained source supplies pixels but not an active BA observation.
                      if(std::find(mMap.vpKeyFrames.begin(),mMap.vpKeyFrames.end(),partial.source)!=mMap.vpKeyFrames.end()) {
                        partial.source->mMeasurements[point]=m;point->pMMData->sMeasurementKFs.insert(partial.source);
                      }
                      m.nLevel=1;m.Source=Measurement::SRC_EPIPOLAR;m.v2RootPos=observed;current.mMeasurements[point]=m;point->pMMData->sMeasurementKFs.insert(&current);
                      if(point->pMMData->GoodMeasCount()>=2&&!point->bBad){mMap.vpPoints.push_back(point);mqNewQueue.push(point);++mDiagnostics.partialPromoted;mbBundleConverged_Full=mbBundleConverged_Recent=false;remove=true;}
                      else {current.mMeasurements.erase(point);partial.source->mMeasurements.erase(point);delete point;}
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    if(!matched){++partial.misses;++mDiagnostics.partialRejected;}
    if(partial.misses>8||partial.age>90)remove=true;
    if(remove){if(index<savedCursor&&savedCursor)--savedCursor;mPartialLandmarks.erase(mPartialLandmarks.begin()+index);}else ++index;
  }
  mPartialObservationCursor=mPartialLandmarks.empty()?0:(onlyId?savedCursor:index)%mPartialLandmarks.size();
  return visited;
}
#endif
