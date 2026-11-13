#include "MapMaker.h"
#include "MapPoint.h"
#include "MobileConfig.h"
#include "MobileMappingGeometry.h"
#include "MobileMappingBudget.h"
#include "vector_image_ref.h"
#include <random>
#include <stdexcept>
#include <iostream>
#include <algorithm>
static void check(bool good,const char* text){if(!good)throw std::runtime_error(text);}
class TestMapper:public MapMaker {
public:
 using MapMaker::MapMaker;
 using MapMaker::SeedPartialPoints;
 using MapMaker::ObserveAndPromotePartialPoints;
 using MapMaker::RefreshRetainedPoses;
 using MapMaker::ApplyGlobalScaleToMap;
 using MapMaker::ApplyGlobalTransformationToMap;
 using MapMaker::BundleAdjustAll;
 using MapMaker::RefreshMobileRollingShutter;
 using MapMaker::CheckMobileResources;
 using MapMaker::QueuePartialPromotion;
 void MarkBundlesConverged(){mbBundleConverged_Recent=mbBundleConverged_Full=true;}
 auto& Anchors(){return mRetainedPoseAnchors;}
 double FirstPartialInverseDepth() const {return mPartialLandmarks.at(0).inverseDepth;}
 bool PartialsFrom(KeyFrame* frame) const {for(const auto& partial:mPartialLandmarks)if(partial.source==frame)return true;return false;}
};
static ATANCamera camera(){ATANCamera c("mapping-regression");c.SetImageSize(CVD::ImageRef(640,480));c.SetParameters(TooN::makeVector(400./640.,400./480.,.5,.5,0.));return c;}
static void pointGeometry(MapPoint& p,KeyFrame& source,ATANCamera& cam,const TooN::Vector<3>& position){
 p.v3WorldPos=position;p.pPatchSourceKF=&source;p.nSourceLevel=0;
 const auto pixel=cam.Project(TooN::project(source.se3CfromW*position));p.irCenter=CVD::ir_rounded(pixel);
 p.v3Normal_NC=TooN::makeVector(0.,0.,-1.);
 p.v3Center_NC=TooN::unproject(cam.UnProject(pixel));TooN::normalize(p.v3Center_NC);
 p.v3OneRightFromCenter_NC=TooN::unproject(cam.UnProject(pixel+TooN::makeVector(1.,0.)));TooN::normalize(p.v3OneRightFromCenter_NC);
 p.v3OneDownFromCenter_NC=TooN::unproject(cam.UnProject(pixel+TooN::makeVector(0.,1.)));TooN::normalize(p.v3OneDownFromCenter_NC);
 p.RefreshPixelVectors();p.pMMData=new MapMakerData;
}
static void observe(MapPoint& point,KeyFrame& frame,ATANCamera& cam){
 Measurement m;m.nLevel=0;m.bSubPix=true;m.Source=Measurement::SRC_TRACKER;
 m.v2RootPos=cam.Project(TooN::project(frame.se3CfromW*point.v3WorldPos));frame.mMeasurements[&point]=m;point.pMMData->sMeasurementKFs.insert(&frame);
}
static void reciprocal(const Map& map){for(auto* p:map.vpPoints){check(p->pMMData,"point metadata missing");for(auto* k:p->pMMData->sMeasurementKFs)check(k->mMeasurements.count(p),"one-sided point observation");}for(auto* k:map.vpKeyFrames)for(auto& m:k->mMeasurements)check(m.first->pMMData->sMeasurementKFs.count(k),"one-sided frame observation");}
static void cullingOwnership(){
 auto cam=camera();Map map;TestMapper mapper(map,cam);
 auto& cfg=IPTAMMobile::Config();cfg=IPTAMMobile::Settings();cfg.partialPoints=false;cfg.cullMeasurements=true;cfg.cullKeyframes=true;cfg.maxActiveKeyframes=4;cfg.maxObservations=8;
 for(int i=0;i<12;++i){auto* k=new KeyFrame;k->se3CfromW.get_translation()=TooN::makeVector(-.02*i,0.,0.);k->bFixed=i==0;k->aLevels[0].im.resize(CVD::ImageRef(640,480));k->aLevels[0].im[0][0]=i;map.vpKeyFrames.push_back(k);}
 auto* origin=map.vpKeyFrames[1];
 for(int j=0;j<18;++j){auto* p=new MapPoint;pointGeometry(*p,*map.vpKeyFrames[j%12],cam,TooN::makeVector((j%6-2.5)*.08,(j/6-1)*.1,2.+.03*j));map.vpPoints.push_back(p);for(auto* k:map.vpKeyFrames)observe(*p,*k,cam);}
 cfg.partialPoints=true;
 for(auto* k:map.vpKeyFrames){Candidate c;c.irLevelPos=CVD::ImageRef(80,80);c.dSTScore=1000.;k->aLevels[0].vCandidates.push_back(c);mapper.SeedPartialPoints(*k);}
 check(mapper.PartialPointCount()==12,"retirement lifecycle candidates not seeded");
 for(auto* k:map.vpKeyFrames)mapper.QueuePartialPromotion(*k);
 map.bGood=true;mapper.CullMobileMap();
 check(map.vpKeyFrames.size()<=4,"active keyframe cap not applied");check(map.vpRetainedKeyFrames.size()==8,"retired keyframes not owned separately");check(map.vpKeyFrames.front()->bFixed,"fixed gauge keyframe was retired");
 origin=nullptr;
 for(auto* p:map.vpPoints)if(std::find(map.vpRetainedKeyFrames.begin(),map.vpRetainedKeyFrames.end(),p->pPatchSourceKF)!=map.vpRetainedKeyFrames.end()){origin=p->pPatchSourceKF;break;}
 check(origin&&origin->aLevels[0].im.size()==CVD::ImageRef(640,480),"retained patch source pixels lost");
 check(!mapper.PartialsFrom(origin)&&mapper.PartialPointCount()<=4,"retired partial source retained unpromotable depth candidates");
 check(mapper.PendingPartialPromotionCount()<=map.vpKeyFrames.size()&&mapper.Diagnostics().deferredPartialCancelled==8,"retired KF retained a pending promotion task");
 check(std::find(map.vpRetainedKeyFrames.begin(),map.vpRetainedKeyFrames.end(),origin)!=map.vpRetainedKeyFrames.end(),"source fixture was not retired");reciprocal(map);
 check(!map.vpPoints.empty(),"all constrained points lost during culling");
 // A retired anchor may itself be retired later: all records must point directly
 // to living active poses, with no dangling chain after later BA/global changes.
 for(const auto& record:mapper.Anchors())check(std::find(map.vpKeyFrames.begin(),map.vpKeyFrames.end(),record.second.first)!=map.vpKeyFrames.end(),"retired anchor chain not flattened");
 for(auto* k:map.vpKeyFrames)k->se3CfromW.get_translation()[1]+=.3;
 mapper.RefreshRetainedPoses();
 for(const auto& record:mapper.Anchors())check(TooN::norm((record.first->se3CfromW*(record.second.second*record.second.first->se3CfromW).inverse()).ln())<1e-9,"retained pose did not follow active anchor");
 const auto before=origin->se3CfromW;mapper.ApplyGlobalScaleToMap(3.);
 check(TooN::norm(origin->se3CfromW.get_translation()-3*before.get_translation())<1e-9,"scale did not update archived anchor transform");
 TooN::SE3<> transform;transform.get_translation()=TooN::makeVector(.5,-.2,.3);const auto scaled=origin->se3CfromW;mapper.ApplyGlobalTransformationToMap(transform);
 check(TooN::norm((origin->se3CfromW*(scaled*transform.inverse()).inverse()).ln())<1e-9,"global transform did not update retained poses");
 mapper.RequestReset();check(map.vpKeyFrames.empty()&&map.vpRetainedKeyFrames.empty()&&map.vpPoints.empty()&&map.vpPointsTrash.empty(),"reset did not release all active/retired ownership");check(mapper.Anchors().empty(),"reset retained stale anchor pointers");
}
static void observationBridge(){
 auto cam=camera();Map map;TestMapper mapper(map,cam);auto& cfg=IPTAMMobile::Config();cfg=IPTAMMobile::Settings();cfg.partialPoints=false;cfg.cullKeyframes=false;cfg.maxObservations=2;
 for(int i=0;i<4;++i){auto* k=new KeyFrame;k->bFixed=i==0;k->se3CfromW.get_translation()[0]=-.05*i;map.vpKeyFrames.push_back(k);}
 // One central point is the only bridge between two independently supported
 // view pairs. A two-observation cap must keep its bridge measurements.
 auto add=[&](int id,const std::vector<int>& views,bool outlier=false){auto* p=new MapPoint;pointGeometry(*p,*map.vpKeyFrames[0],cam,TooN::makeVector(.03*id,.02*(id%3),2.+.05*id));map.vpPoints.push_back(p);for(auto i:views){observe(*p,*map.vpKeyFrames[i],cam);if(outlier&&i>=2)map.vpKeyFrames[i]->mMeasurements[p].v2RootPos[0]+=100.;}return p;};
 auto* bridge=add(0,{0,1,2,3},true);for(int i=1;i<=5;++i){add(i,{0,1});add(i+5,{2,3});}
 map.bGood=true;mapper.CullMobileMap();
 check(bridge->pMMData->GoodMeasCount()>2,"nominal cap removed a required graph bridge");check(mapper.Diagnostics().observationBridgeRetentions>0,"graph-conflicting cap exception not reported");reciprocal(map);
 // Every free frame retains at least three supports even if the utility cap
 // would otherwise leave a nearly empty camera entering BA.
 for(auto* k:map.vpKeyFrames)if(!k->bFixed)check(k->mMeasurements.size()>=3,"culling left unsupported free frame");
}
static void supportAndResourceBounds(){
 auto cam=camera();Map map;TestMapper mapper(map,cam);auto& cfg=IPTAMMobile::Config();cfg=IPTAMMobile::Settings();cfg.partialPoints=false;
 for(int i=0;i<2;++i){auto* k=new KeyFrame;k->bFixed=i==0;k->se3CfromW.get_translation()[0]=-.1*i;map.vpKeyFrames.push_back(k);}
 for(int j=0;j<2;++j){auto* p=new MapPoint;pointGeometry(*p,*map.vpKeyFrames[0],cam,TooN::makeVector(.2*j,.1*j,2.+j));map.vpPoints.push_back(p);for(auto* k:map.vpKeyFrames)observe(*p,*k,cam);}
 map.bGood=true;mapper.BundleAdjustAll();check(mapper.Diagnostics().fixedLowSupportViews>0,"two-support camera entered free bundle solve");
 for(int i=0;i<128;++i)map.vpRetainedKeyFrames.push_back(new KeyFrame);
 mapper.CheckMobileResources();check(mapper.ResourceLimitReached()&&!mapper.ResourceLimitReason().empty(),"retained memory cap did not stop new map work");
 mapper.RequestReset();check(!mapper.ResourceLimitReached()&&map.vpRetainedKeyFrames.empty(),"resource-stop reset retained ownership/state");
}
static void rollingShutterRawOwnership(){
 auto cam=camera();Map map;TestMapper mapper(map,cam);auto& cfg=IPTAMMobile::Config();cfg=IPTAMMobile::Settings();cfg.rollingShutter=true;cfg.readoutSeconds=.018;
 auto* frame=new KeyFrame;frame->aLevels[0].im.resize(CVD::ImageRef(640,480));map.vpKeyFrames.push_back(frame);
 const TooN::Vector<6> velocity=TooN::makeVector(.12,-.03,.07,.02,-.08,.03);
 std::vector<TooN::Vector<2>> raw,ideal;
 for(int y=0;y<6;++y)for(int x=0;x<8;++x){
  auto* point=new MapPoint;const double z=2.+.13*((x+2*y)%7);const auto p=TooN::makeVector((x-3.5)*.18,(y-2.5)*.16,z);
  pointGeometry(*point,*frame,cam,p);map.vpPoints.push_back(point);observe(*point,*frame,cam);
  TooN::Matrix<2,3> division=TooN::Zeros;division[0][0]=division[1][1]=400/z;division[0][2]=-400*p[0]/(z*z);division[1][2]=-400*p[1]/(z*z);
  TooN::Matrix<3,6> motion=TooN::Zeros;for(int d=0;d<3;++d)motion[d][d]=1;
  for(int d=0;d<3;++d){TooN::Vector<3> e=TooN::Zeros;e[d]=1;motion.T()[d+3]=e^p;}
  const auto flow=division*motion*velocity;auto& m=frame->mMeasurements[point];const auto clean=m.v2RootPos;
  const double dt=.018*(clean[1]/479.-.5)/(1.-flow[1]*.018/479.);
  m.v2RootPos=clean+flow*dt;m.v2RawRootPos=m.v2RootPos;m.hasRawPosition=m.hasPrevious=m.hasNext=true;
  m.timestamp=1.;m.previousTimestamp=.97;m.nextTimestamp=1.03;m.v2PreviousRootPos=m.v2RootPos-.03*flow;m.v2NextRootPos=m.v2RootPos+.03*flow;
  if(x==0&&y==0){m.hasPrevious=m.hasNext=false;m.hasRawPosition=false;}
 }
 for(auto& entry:frame->mMeasurements){raw.push_back(entry.second.v2RootPos);ideal.push_back(cam.Project(TooN::project(entry.first->v3WorldPos)));}
 for(int repeat=0;repeat<2;++repeat){mapper.RefreshMobileRollingShutter(*frame);std::size_t i=0;for(auto& entry:frame->mMeasurements){const auto& m=entry.second;check(m.rollingShutterCorrected,"mapper failed supported known rolling-shutter motion");check(TooN::norm(m.v2RootPos-raw[i])==0&&TooN::norm(m.v2RawRootPos-raw[i])==0,"BA correction overwrote original pixels");check(TooN::norm(m.BundlePosition()-ideal[i])<1e-8,"BA re-correction accumulated or failed known image warp");++i;}}
}
static void depthPriorAllocationOrder(){
 auto cam=camera();Map map;TestMapper mapper(map,cam);auto& cfg=IPTAMMobile::Config();cfg=IPTAMMobile::Settings();
 double prior=0;
 for(int pass=0;pass<2;++pass){
  auto* source=new KeyFrame;source->aLevels[0].im.resize(CVD::ImageRef(640,480));map.vpKeyFrames.push_back(source);
  std::vector<MapPoint*> allocation;for(int i=0;i<100;++i)allocation.push_back(new MapPoint);
  std::sort(allocation.begin(),allocation.end(),std::less<MapPoint*>());if(pass)std::reverse(allocation.begin(),allocation.end());
  for(int i=0;i<100;++i){auto* point=allocation[i];pointGeometry(*point,*source,cam,TooN::makeVector(0.,0.,i==0?.002:1e13));map.vpPoints.push_back(point);observe(*point,*source,cam);}
  Candidate c;c.irLevelPos=CVD::ImageRef(80,80);c.dSTScore=1000.;source->aLevels[0].vCandidates.push_back(c);
  mapper.SeedPartialPoints(*source);check(mapper.PartialPointCount()==1,"allocation-order depth-prior fixture did not seed");
  const double rho=mapper.FirstPartialInverseDepth();if(!pass)prior=rho;else check(rho==prior,"partial depth prior depends on allocator-address ordering");
  mapper.RequestReset();
 }
}
static void partialGeometry(){
 auto cam=camera();Map map;TestMapper mapper(map,cam);auto& cfg=IPTAMMobile::Config();cfg=IPTAMMobile::Settings();cfg.cullMeasurements=cfg.cullKeyframes=false;cfg.maxNewCandidates=8;
 CVD::Image<CVD::byte> first(CVD::ImageRef(640,480)),shifted(CVD::ImageRef(640,480));std::mt19937 rng(911);
 for(int y=0;y<480;++y)for(int x=0;x<640;++x)first[y][x]=rng()%256;
 for(int y=0;y<480;++y)for(int x=0;x<640;++x)shifted[y][x]=x+20<640?first[y][x+20]:0;
 auto* source=new KeyFrame;auto* target=new KeyFrame;source->MakeKeyFrame_Lite(first);target->MakeKeyFrame_Lite(shifted);source->bFixed=true;source->dSceneDepthMean=2.;target->se3CfromW.get_translation()[0]=-.1;
 source->aLevels[0].vCandidates.clear();for(int y=120;y<=360;y+=80)for(int x=160;x<=480;x+=80){Candidate c;c.irLevelPos=CVD::ImageRef(x,y);c.dSTScore=1000.;source->aLevels[0].vCandidates.push_back(c);}
 map.vpKeyFrames={source};map.bGood=true;mapper.SeedPartialPoints(*source);check(mapper.PartialPointCount()==8,"bounded partial seeding failed");
 KeyFrame current;current= *target;mapper.ObservePartialPoints(current);check(map.vpPoints.empty()&&current.mMeasurements.empty(),"provisional depth counted as regular pose support");
 mapper.AddKeyFrame(*target);delete target;target=nullptr;
 {
  IPTAMMobile::ScopedMappingDeadline expired(MapMaker::SteadyTimeSeconds()-.001);
  mapper.ProcessPending(); // Actual insertion finishes under an expired quantum.
  check(map.vpPoints.empty()&&mapper.PendingPartialPromotionCount()==1,"expired insertion lost or prematurely executed promotion task");
  mapper.ProcessPending();
  check(mapper.PendingPartialPromotionCount()==1&&map.vpPoints.empty(),"expired promotion discarded a pending opportunity");
 }
 target=map.vpKeyFrames.back();mapper.MarkBundlesConverged();
 // Pending promotion alone must keep idle scheduling alive even with converged BA.
 const auto stages=mapper.ProcessIdleWork(MapMaker::SteadyTimeSeconds()+1.,4.);
 check(stages>0&&!map.vpPoints.empty(),"fresh idle quantum failed to promote deferred actual-image depth");
 check(mapper.PendingPartialPromotionCount()==0&&mapper.Diagnostics().deferredPartialCompleted==1,"completed promotion task not retired");
 check(mapper.Diagnostics().stageCalls[MappingDiagnostics::PartialPromotion]>=1,"deferred promotion stage not observed");
 for(auto* p:map.vpPoints){check(std::abs(p->v3WorldPos[2]-2.)<.02,"promoted depth differs from known plane");check((target->se3CfromW*p->v3WorldPos)[2]>0&&p->pMMData->GoodMeasCount()==2,"promotion lacks two-view positive depth/support");}reciprocal(map);
 mapper.QueuePartialPromotion(*target);
 mapper.RequestReset();check(mapper.PartialPointCount()==0&&mapper.PendingPartialPromotionCount()==0,"reset retained provisional/pending source pointers");
 double rho=.5,var=std::numeric_limits<double>::infinity();check(!IPTAMMobile::UpdateInverseDepth(rho,var,0,0,1,1,4),"pure rotation produced false depth information");check(rho==.5&&std::isinf(var),"failed filter mutated prior");
 check(IPTAMMobile::UpdateInverseDepth(rho,var,-40,0,-2,0,4)&&std::abs(rho-.55)<1e-9&&std::abs(var-.0025)<1e-9,"one-dimensional inverse-depth EKF update incorrect");const auto previous=var;check(IPTAMMobile::UpdateInverseDepth(rho,var,-40,0,0,0,4)&&var<previous,"repeated parallax did not increase information");
}
int main(){try{cullingOwnership();observationBridge();supportAndResourceBounds();rollingShutterRawOwnership();depthPriorAllocationOrder();partialGeometry();std::cout<<"Mobile mapping: connected culling, retained pose ownership/reset, measured partial-depth promotion passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
