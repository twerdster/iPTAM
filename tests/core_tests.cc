#include "ATANCamera.h"
#include "Tracker.h"
#include "HomographyInit.h"
#include "MEstimator.h"
#include "TrackerData.h"
#ifdef IPTAM_MOBILE_PAPER
#include "MobileReplaySupport.h"
#endif
#include <random>
#include <iostream>
#include <stdexcept>
#include <limits>
#include <memory>
#include <utility>
class RegressionMapMaker:public MapMaker {
 public:
  using MapMaker::MapMaker;
  void PendRecentBundle() {mbBundleConverged_Recent=false;}
  bool RecentBundleConverged() const {return mbBundleConverged_Recent;}
  bool Refind(KeyFrame& keyframe,MapPoint& point) {return ReFind_Common(keyframe,point);}
  void QueueFailedMeasurement(KeyFrame& keyframe,MapPoint& point) {mvFailureQueue.emplace_back(&keyframe,&point);}
  void RetryFailedMeasurements() {ReFindFromFailureQueue();}
};
static void mapperMaintenanceRegression() {
 ATANCamera camera("mapper-maintenance");
 camera.SetImageSize(CVD::ImageRef(640,480));
 camera.SetParameters(TooN::makeVector(400./640.,400./480.,.5,.5,0.));
 Map map; RegressionMapMaker mapper(map,camera);
 auto* source=new KeyFrame; map.vpKeyFrames.push_back(source);
 auto* bad=new MapPoint; bad->v3WorldPos=TooN::makeVector(0.,0.,2.);
 bad->pMMData=new MapMakerData; bad->pMMData->sMeasurementKFs.insert(source);
 bad->bBad=true; map.vpPoints.push_back(bad);
 Measurement measurement; measurement.nLevel=0;measurement.bSubPix=false;
 measurement.v2RootPos=TooN::makeVector(319.5,239.5);measurement.Source=Measurement::SRC_ROOT;
 source->mMeasurements[bad]=measurement;
 map.bGood=true;mapper.PendRecentBundle();mapper.ProcessPending();
 if(!mapper.RecentBundleConverged()) throw std::runtime_error("pending recent-bundle branch was not exercised");
 if(!map.vpPoints.empty() || map.vpPointsTrash.size()!=1 || map.vpPointsTrash[0]!=bad || !source->mMeasurements.empty())
  throw std::runtime_error("pending recent bundle starved bad-point cleanup");

 CVD::Image<CVD::byte> image(CVD::ImageRef(640,480));std::mt19937 rng(91);
 for(int y=0;y<480;++y)for(int x=0;x<640;++x)image[y][x]=rng()%256;
 source->MakeKeyFrame_Lite(image);
 auto* target=new KeyFrame;target->MakeKeyFrame_Lite(image);map.vpKeyFrames.push_back(target);
 CVD::ImageRef corner;bool found=false;
 for(auto candidate:source->aLevels[0].vCorners)
  if(candidate.x>80&&candidate.x<560&&candidate.y>80&&candidate.y<400) {corner=candidate;found=true;break;}
 if(!found) throw std::runtime_error("retired-point regression has no textured source corner");
 auto* point=new MapPoint;map.vpPoints.push_back(point);point->pMMData=new MapMakerData;
 auto normalized=camera.UnProject(corner);
 point->v3WorldPos=TooN::makeVector(2*normalized[0],2*normalized[1],2.);
 point->pPatchSourceKF=source;point->nSourceLevel=0;point->irCenter=corner;
 point->v3PixelRight_W=TooN::makeVector(.005,0.,0.);
 point->v3PixelDown_W=TooN::makeVector(0.,.005,0.);
 if(!mapper.Refind(*target,*point) || !target->mMeasurements.count(point))
  throw std::runtime_error("retired-point fixture cannot first refind a valid patch");
 target->mMeasurements.erase(point);point->pMMData->sMeasurementKFs.clear();
 mapper.QueueFailedMeasurement(*target,*point);
 point->bBad=true;mapper.PendRecentBundle();mapper.ProcessPending();
 if(!map.vpPoints.empty() || map.vpPointsTrash.size()!=2)
  throw std::runtime_error("retired-point fixture did not move point to map trash");
 mapper.RetryFailedMeasurements();
 if(target->mMeasurements.count(point) || !point->pMMData->sMeasurementKFs.empty())
  throw std::runtime_error("failure queue reinserted a retired point measurement");
 std::cout<<"Mapper maintenance: pending-bundle cleanup and retired-point refind regressions passed\n";
}
class RegressionTracker:public Tracker {
 public:
  using Tracker::Tracker;
  bool NumericalFailure() const {return mbNumericalFailure;}
  int ConsecutiveLostFrames() const {return mnLostFrames;}
  void AssessVisualSupport(int found,int attempted) {
   for(int level=0;level<LEVELS;++level)manMeasFound[level]=manMeasAttempted[level]=0;
   manMeasFound[0]=found;manMeasAttempted[0]=attempted;
   mnPoseInliers=found;
   AssessTrackingQuality();
  }
  TooN::Vector<6> SolveFixture(std::vector<TrackerData*>& data) {return CalcPoseUpdate(data,16.0,true);}
  void AssessGeometricSupport(const std::vector<TrackerData*>& data,const TooN::SE3<>& finalPose) {
   mse3CamFromWorld=finalPose;
   for(int level=0;level<LEVELS;++level)manMeasFound[level]=manMeasAttempted[level]=0;
   mCurrentKF.mMeasurements.clear();
   for(auto* observation:data) {
    ++manMeasAttempted[observation->nSearchLevel];
    if(!observation->bFound)continue;
    ++manMeasFound[observation->nSearchLevel];
    Measurement m;m.nLevel=observation->nSearchLevel;m.bSubPix=false;
    m.v2RootPos=observation->v2Found;m.Source=Measurement::SRC_TRACKER;
    mCurrentKF.mMeasurements[&observation->Point]=m;
   }
   UpdatePoseInlierCount(data);
   AssessTrackingQuality();
  }
  size_t FixtureMeasurementCount() const {return mCurrentKF.mMeasurements.size();}
  void RejectInvalidPoseSolve() {
   MapPoint point; TrackerData data(&point);
   data.bFound=true;
   data.m26Jacobian[0][0]=std::numeric_limits<double>::quiet_NaN();
   std::vector<TrackerData*> measurements{&data};
   auto update=CalcPoseUpdate(measurements);
   for(int i=0;i<6;++i) if(update[i]!=0.) throw std::runtime_error("invalid pose solve was applied");
   AssessTrackingQuality();
  }
};
static void poseSupportRegression() {
 ATANCamera camera("final-pose-support");
 camera.SetImageSize(CVD::ImageRef(640,480));
 camera.SetParameters(TooN::makeVector(400./640.,400./480.,.5,.5,0.));
 Map map;MapMaker mapper(map,camera);RegressionTracker tracker(CVD::ImageRef(640,480),camera,map,mapper);
 map.bGood=true;
 std::vector<std::unique_ptr<MapPoint>> points;
 std::vector<std::unique_ptr<TrackerData>> observations;
 std::vector<TrackerData*> data;
 for(int i=0;i<12;++i) {
  points.emplace_back(new MapPoint);
  points.back()->v3WorldPos=TooN::makeVector((i%4-1.5)*.15,(i/4-1)*.15,2.+.1*i);
  observations.emplace_back(new TrackerData(points.back().get()));data.push_back(observations.back().get());
 }
 auto prepare=[&](int inliers,int level=0,double outlierOffset=8.) {
  TooN::SE3<> identity;
  for(int i=0;i<12;++i) {
   auto& d=*data[i];d.ProjectAndDerivs(identity,camera);d.CalcJacobian();
   d.bFound=true;d.nSearchLevel=level;d.dSqrtInvNoise=1./(1<<level);
   d.v2Found=d.v2Image+TooN::makeVector(i<inliers?0.:outlierOffset,0.);
  }
 };
 auto preserveRawInput=[&]() {
  if(tracker.MeasurementsFound()!=12 || tracker.MeasurementsAttempted()!=12 || tracker.FixtureMeasurementCount()!=12)
   throw std::runtime_error("final-pose support changed raw match counters or mapper measurements");
  for(auto* d:data)if(!d->bFound)throw std::runtime_error("final-pose support changed a raw found flag");
 };
 for(int inliers:{9,10,0}) {
  prepare(inliers);
  if(TooN::norm(tracker.SolveFixture(data))>1e-12)
   throw std::runtime_error("zero inlier residuals or rejected outliers changed fixture pose");
  tracker.AssessGeometricSupport(data,TooN::SE3<>());
  if(tracker.PoseInliers()!=inliers || tracker.TrackingQuality()!=(inliers>=10?2:0))
   throw std::runtime_error("raw twelve-match frame ignored final robust pose support");
  if(tracker.ConsecutiveLostFrames()!=(inliers>=10?0:1))
   throw std::runtime_error("robust pose support did not control loss and recovery state");
  preserveRawInput();
 }
 // Cached projections still fit perfectly at the linearization pose. A later
 // exported pose shift invalidates all twelve; count final geometry instead.
 prepare(12);
 TooN::SE3<> shifted;shifted.get_translation()=TooN::makeVector(.05,0.,0.);
 tracker.AssessGeometricSupport(data,shifted);
 if(tracker.PoseInliers()!=0 || tracker.TrackingQuality()!=0)
  throw std::runtime_error("pose support used cached linearization projections");
 for(auto* d:data)if(TooN::norm(d->v2Found-d->v2Image)>1e-12)
  throw std::runtime_error("support counting mutated cached projections");
 preserveRawInput();
 // Six root pixels at level one are three normalized pixels, inside the
 // existing four-pixel robust support radius used by the final pose solve.
 prepare(0,1,6.);tracker.AssessGeometricSupport(data,TooN::SE3<>());
 if(tracker.PoseInliers()!=12 || tracker.TrackingQuality()!=2)
  throw std::runtime_error("pose support ignored pyramid-scaled measurement noise");
 preserveRawInput();
 tracker.StopTracking();
 if(tracker.PoseInliers()!=0)throw std::runtime_error("reset retained previous pose support");
 std::cout<<"Final pose support: actual robust solve, 9/10/0 inliers, exported pose, noise scale and raw-input preservation passed\n";
}
int main() {
#ifdef IPTAM_MOBILE_PAPER
  // This suite preserves the restoration's legacy behavior. Mobile algorithms
  // have separate geometric/image/ownership tests and full-input replay runs.
  IPTAMMobile::ApplyProfile("diagnostics");
#endif
 try {
  mapperMaintenanceRegression();
  poseSupportRegression();
  std::vector<double> firstTrajectory;
  for(int lifecycle=0;lifecycle<3;++lifecycle) {
  ATANCamera c("test");
  c.SetParameters(TooN::makeVector(0.8,1.0,0.5,0.5,0.0));
  for(int y=0;y<480;y+=16) for(int x=0;x<640;x+=16) {
   auto p=TooN::makeVector(x*1.,y*1.); auto q=c.Project(c.UnProject(p));
   if(TooN::norm(q-p)>1e-9) throw std::runtime_error("camera round trip");
  }
  bool rejected=false; try {c.SetParameters(TooN::makeVector(0.,1.,.5,.5,0.));} catch(const std::invalid_argument&) {rejected=true;}
  if(!rejected) throw std::runtime_error("invalid focal accepted");
  Map map; MapMaker mm(map,c); RegressionTracker tracker(CVD::ImageRef(640,480),c,map,mm);
  CVD::Image<CVD::byte> im(CVD::ImageRef(640,480)); im.fill(0);
  for(int i=0;i<6;++i) {if(i==0||i==2) tracker.StartTracking(); tracker.TrackFrame(im,0,false);}
  if(map.IsGood()) throw std::runtime_error("blank image initialized");
  auto pose=tracker.GetCurrentPose().ln();
  for(int i=0;i<6;++i) if(!std::isfinite(pose[i])) throw std::runtime_error("nonfinite blank pose");
  for(int n:{0,1,3,10}) {
   std::vector<double> e(n,0.); double s=Tukey::FindSigmaSquared(e);
   if(!std::isfinite(s)||s<=0||Tukey::Weight(0.,s)!=1.) throw std::runtime_error("zero residual robust scale");
  }
  HomographyInit init; std::vector<HomographyMatch> matches;
  TooN::SE3<> initPose;
  if(init.Compute(matches,5.,initPose)) throw std::runtime_error("empty homography accepted");
  tracker.SetFrameTimestamp(-1.0);
  bool duplicate=false; try {tracker.SetFrameTimestamp(-1.0);} catch(const std::invalid_argument&) {duplicate=true;}
  if(!duplicate) throw std::runtime_error("duplicate capture timestamp accepted");
  CVD::Image<CVD::byte> wrongSize(CVD::ImageRef(320,240));
  bool wrong=false; try {tracker.TrackFrame(wrongSize,0,false);} catch(const std::invalid_argument&) {wrong=true;}
  if(!wrong) throw std::runtime_error("wrong frame size accepted");
  tracker.StopTracking();
  std::mt19937 rng(42); std::vector<CVD::byte> texture(800*480);
  for(auto& b:texture) b=rng()%256;
  int good=0; size_t comparisonIndex=0;
  for(int frame=0;frame<60;++frame) {
   for(int y=0;y<480;++y) for(int x=0;x<640;++x) im[y][x]=texture[y*800+x+frame];
   if(frame==0||frame==20) tracker.StartTracking();
   tracker.SetFrameTimestamp(frame/30.);
   tracker.TrackFrame(im,0,false);
   auto observed=tracker.GetCurrentPose().ln();
   for(int j=0;j<8;++j) {
    double value=j<6?observed[j]:(j==6?tracker.TrackingQuality():tracker.MeasurementsFound());
    if(lifecycle==0) firstTrajectory.push_back(value);
    else if(std::abs(value-firstTrajectory.at(comparisonIndex))>1e-10) throw std::runtime_error("reset lifecycle replay changed");
    ++comparisonIndex;
   }
   if(frame>20 && tracker.TrackingQuality()==2 && tracker.MeasurementsFound()>=10) ++good;
  }
  std::cout<<"Synthetic plane: map="<<map.IsGood()<<" points="<<map.vpPoints.size()<<" tracked="<<good<<"/39\n";
  if(!map.IsGood() || good<35) throw std::runtime_error("synthetic plane tracking regression");
  int lost=0;
  for(auto support:{std::make_pair(1,3),std::make_pair(3,3),std::make_pair(9,9)}) {
   tracker.AssessVisualSupport(support.first,support.second);
   if(tracker.TrackingQuality()!=0 || tracker.ConsecutiveLostFrames()!=++lost)
    throw std::runtime_error("sub-ten visual support retained GOOD/keyframe eligibility or skipped lost transition");
  }
  tracker.AssessVisualSupport(10,30);
  if(tracker.TrackingQuality()!=2 || tracker.ConsecutiveLostFrames()!=0)
   throw std::runtime_error("adequate visual support did not resume GOOD tracking");
  tracker.RejectInvalidPoseSolve();
  if(!tracker.NumericalFailure() || tracker.TrackingQuality()!=0) throw std::runtime_error("invalid pose solve did not invalidate its frame");
  tracker.TrackFrame(im,0,false);
  if(tracker.NumericalFailure() || tracker.TrackingQuality()!=2 || tracker.MeasurementsFound()<10) throw std::runtime_error("one rejected solve poisoned subsequent tracking");
  im.fill(0);
  for(int i=0;i<8;++i) tracker.TrackFrame(im,0,false);
  if(tracker.TrackingQuality()!=0) throw std::runtime_error("blank occlusion accepted as good tracking");
  if(tracker.PoseInliers()!=0) throw std::runtime_error("no-search lost frame retained previous pose support");
  for(int y=0;y<480;++y) for(int x=0;x<640;++x) im[y][x]=texture[y*800+x+59];
  for(int i=0;i<8;++i) tracker.TrackFrame(im,0,false);
  if(tracker.TrackingQuality()!=2 || tracker.MeasurementsFound()<10) throw std::runtime_error("relocalization failed after exact scene return");
  std::cout<<"Core safety checks passed, including relocalization after occlusion\n";
  tracker.RejectInvalidPoseSolve();
  tracker.StopTracking();
  if(tracker.NumericalFailure()) throw std::runtime_error("reset retained a previous numerical failure");
  }
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 1;}
}
