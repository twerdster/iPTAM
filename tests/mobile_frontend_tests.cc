#include "MobileFrontend.h"
#include "MobileConfig.h"
#include "PatchFinder.h"
#include "Tracker.h"
#include "TrackerData.h"
#include "vision.h"
#include "fast_corner.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <random>
#include <stdexcept>
using namespace CVD;
using namespace TooN;
static int checks=0;
static void require(bool pass,const char* message) {++checks;if(!pass) throw std::runtime_error(message);}
static double pattern(double x,double y) {return 120+40*std::sin(.51*x)+38*std::cos(.43*y)+30*std::sin(.27*x+.37*y);}
static Image<byte> texture(int width=320,int height=240,double dx=0,double dy=0,double gain=1,double offset=0) {
 Image<byte> image(ImageRef(width,height));
 for(int y=0;y<height;++y)for(int x=0;x<width;++x)image[y][x]=static_cast<byte>(std::max(0.,std::min(255.,gain*pattern(x-dx,y-dy)+offset)));
 return image;
}
class ProbeTracker:public Tracker {
public:using Tracker::Tracker;
 void Restart(){StopTracking();}
 bool ManualNotStarted()const{return mnInitialStage==TRAIL_TRACKING_NOT_STARTED;}
 void RunMobile(){TrackMapMobile();AssessTrackingQuality();}
 void SetRotation(double radians){mv6SBIRot=Zeros;mv6SBIRot[4]=radians;}
 void Prepare(Image<byte>& image){mCurrentKF.MakeKeyFrame_Lite(image);mse3CamFromWorld=SE3<>();}
 void Insert(){AddNewKeyFrame();}
};
int main(){try {
 require(LEVELS==5,"mobile pyramid must have five levels");
 IPTAMMobile::ResetConfig();
 require(!IPTAMMobile::Config().automaticInitialization,"fixed-pair control changed to automatic initialization");
 auto original=texture();
 IPTAMMobile::Config().coarseTracking=false;
 KeyFrame legacy;legacy.MakeKeyFrame_Lite(original);legacy.MakeKeyFrame_Rest();
 std::vector<ImageRef> expectedFast;fast_corner_detect_10(original,expectedFast,10);
 require(legacy.aLevels[0].vCorners==expectedFast,"runtime legacy control changed original FAST proposals");
 require(legacy.aLevels[4].vCorners.empty(),"runtime legacy control added fifth-level features");
 IPTAMMobile::ResetConfig();
 KeyFrame lazy;lazy.MakeKeyFrame_Lite(original,false);
 require(lazy.aLevels[0].vCorners.empty(),"tracking-only pyramid unnecessarily detected corners");
 lazy.MakeKeyFrame_Rest();require(!lazy.aLevels[0].vCandidates.empty(),"new-keyframe lazy multiscale detection failed");
 KeyFrame first;first.MakeKeyFrame_Lite(original);first.MakeKeyFrame_Rest();
 require(first.aLevels[4].im.size()==ImageRef(20,15),"fifth pyramid image dimensions wrong");
 require(!first.aLevels[0].vCandidates.empty(),"multiscale Shi-Tomasi failed on smooth textured corners");
 for(int l=0;l<3;++l) require(!first.aLevels[l].vCorners.empty(),"multiscale feature absent from a required level");
 require(first.aLevels[3].vCorners.empty()&&first.aLevels[4].vCorners.empty(),"coarse levels unexpectedly generated FAST candidates");
 size_t corners=first.aLevels[0].vCandidates.size();first.MakeKeyFrame_Rest();
 require(first.aLevels[0].vCandidates.size()==corners,"repeated keyframe completion duplicated candidates");
 KeyFrame copied=first;byte saved=copied.aLevels[4].im[5][5];first.aLevels[4].im[5][5]^=255;
 require(copied.aLevels[4].im[5][5]==saved,"fifth level copy aliases pixels");
 Image<byte> flat(ImageRef(320,240));flat.fill(128);KeyFrame empty;empty.MakeKeyFrame_Lite(flat);
 require(empty.aLevels[0].vCandidates.empty(),"flat image produced mobile corners");
 PatchFinder finder;finder.SetMobileMatching(true);
 finder.MakeTemplateCoarseNoWarp(first,1,ImageRef(80,60));
 KeyFrame target;auto moved=texture(320,240,4.,-2.,.75,20.);target.MakeKeyFrame_Lite(moved);
 // Deliberately remove all target corners: exhaustive search must still find it.
 target.aLevels[1].vCorners.clear();target.aLevels[1].vCornerRowLUT.clear();
 require(finder.FindPatchCoarse(ImageRef(160,120),target,8),"exhaustive L1 normalized SSD failed without target corners");
 Vector<2> expected=LevelZeroPos(ImageRef(82,59),1);
 require(norm(finder.GetCoarsePosAsVector()-expected)<.1,"normalized match did not recover translated patch under gain/offset");
 finder.MakeSubPixTemplate();require(finder.IterateSubPixToConvergence(target,15),"ESM translation refinement failed");
 require(norm(finder.GetSubPixPos()-expected)<.6,"ESM drifted away from known translated patch");
 auto subpixel=texture(320,240,4.6,-1.4);KeyFrame fractional;fractional.MakeKeyFrame_Lite(subpixel);
 finder.MakeTemplateCoarseNoWarp(first,1,ImageRef(80,60));
 require(finder.FindPatchCoarse(ImageRef(160,120),fractional,8),"subpixel fixture coarse match failed");
 finder.MakeSubPixTemplate();require(finder.IterateSubPixToConvergence(fractional,20),"2DOF ESM did not converge at fractional translation");
 require(norm(finder.GetSubPixPos()-(LevelZeroPos(ImageRef(80,60),1)+makeVector(4.6,-1.4)))<.6,"ESM fractional translation inaccurate");
 require(!finder.FindPatchCoarse(ImageRef(160,120),empty,8),"normalized SSD accepted textureless target");
 MapPoint point;point.pPatchSourceKF=&first;point.nSourceLevel=0;point.irCenter=ImageRef(160,120);
 point.v3WorldPos=makeVector(0.,0.,2.);point.v3PixelRight_W=makeVector(.01,0.,0.);point.v3PixelDown_W=makeVector(0.,.01,0.);
 Matrix<2> derivative=200.*Identity;SE3<> pose;
 require(finder.CalcSearchLevelAndWarpMatrixAtLevel(point,pose,derivative,1)==1&&finder.MobileSourceLevel()==1,"same-view fixed L1 should source L1");
 derivative=400.*Identity;
 require(finder.CalcSearchLevelAndWarpMatrixAtLevel(point,pose,derivative,1)==1&&finder.MobileSourceLevel()==0,"2x magnification should source L0 with target fixed L1");
 derivative=100.*Identity;
 require(finder.CalcSearchLevelAndWarpMatrixAtLevel(point,pose,derivative,1)==1&&finder.MobileSourceLevel()==2,"half magnification should source L2 with target fixed L1");
 derivative=Zeros;require(finder.CalcSearchLevelAndWarpMatrixAtLevel(point,pose,derivative,1)<0,"singular mobile warp accepted");
 Vector<6> rotation=Zeros;rotation[4]=.2;require(IPTAMMobile::RotationNeedsBlurFallback(rotation),"fast rotation did not trigger blur fallback");
 rotation[4]=.01;require(!IPTAMMobile::RotationNeedsBlurFallback(rotation),"slow rotation incorrectly triggers blur fallback");
 ATANCamera camera("mobile-test");camera.SetParameters(makeVector(200./320.,200./240.,.5,.5,0.));camera.SetImageSize(ImageRef(320,240));
 Map map;MapMaker mapper(map,camera);ProbeTracker tracker(ImageRef(320,240),camera,map,mapper);
 tracker.SetFrameTimestamp(0.);tracker.TrackFrame(original,0,false);
 require(tracker.ManualNotStarted()&&!map.IsGood(),"fixed-pair mode initialized without explicit input");
 Image<byte> planeImage(ImageRef(320,240));std::mt19937 planeRandom(31415);
 for(int y=0;y<240;++y)for(int x=0;x<320;++x)planeImage[y][x]=planeRandom()%256;
 auto* source=new KeyFrame;source->MakeKeyFrame_Lite(planeImage);map.vpKeyFrames.push_back(source);
 for(int y=48;y<=192;y+=24)for(int x=48;x<=272;x+=24) {
   auto* landmark=new MapPoint;landmark->pPatchSourceKF=source;landmark->nSourceLevel=0;landmark->irCenter=ImageRef(x,y);
   auto ray=camera.UnProject(makeVector(double(x),double(y)));
   landmark->v3WorldPos=makeVector(2*ray[0],2*ray[1],2.);
   landmark->v3PixelRight_W=makeVector(.01,0.,0.);landmark->v3PixelDown_W=makeVector(0.,.01,0.);
   map.vpPoints.push_back(landmark);
 }
 map.bGood=true;
 tracker.Prepare(planeImage);tracker.SetRotation(0.);tracker.RunMobile();
 require(!tracker.MobileBlurFallback()&&tracker.TrackingQuality()==2&&tracker.PoseInliers()>=10,"fixed-L1 mobile tracker did not solve the known plane");
 require(tracker.MeasurementsAttempted()<=IPTAMMobile::Config().maxTrackedPoints,"mobile tracker exceeded its measurement budget");
 tracker.Prepare(planeImage);tracker.SetRotation(.2);tracker.RunMobile();
 std::cout<<"Plane L2 found="<<tracker.MeasurementsFound()<<" attempted="<<tracker.MeasurementsAttempted()<<" inliers="<<tracker.PoseInliers()<<" quality="<<tracker.TrackingQuality()<<" template="<<tracker.LastTemplateRejectedCount()<<" warp="<<tracker.LastWarpRejectedCount()<<"\n";
 require(tracker.MobileBlurFallback()&&tracker.TrackingQuality()==2&&tracker.PoseInliers()>=10,"L2 fallback did not retain a genuinely supported plane pose");
 size_t queued=mapper.QueueSize();tracker.Insert();require(mapper.QueueSize()==queued,"blur fallback inserted a keyframe");
 {
   Map autoMap;MapMaker autoMapper(autoMap,camera);ProbeTracker automatic(ImageRef(320,240),camera,autoMap,autoMapper);
   IPTAMMobile::Config().automaticInitialization=true;
   automatic.SetFrameTimestamp(1.);automatic.TrackFrame(original,0,false);
   require(automatic.ManualNotStarted()&&automatic.MobileInitializationStage()=="waiting_for_first_frame_request","automatic profile ignored explicitly selected first exposure");
   automatic.StartTracking();automatic.SetFrameTimestamp(1.03);automatic.TrackFrame(original,0,false);
   require(!automatic.ManualNotStarted()&&!autoMap.IsGood(),"first-frame request failed to start automatic second-view selection");
   automatic.SetFrameTimestamp(1.06);automatic.TrackFrame(original,0,false);
   require(!autoMap.IsGood(),"pure unchanged automatic view fabricated a stereo map");
   IPTAMMobile::ResetConfig();
 }
 {
   // Exact reset/replay is an algorithm control: wall-time mapper deadlines
   // intentionally vary with machine load and are exercised in budget tests.
   IPTAMMobile::Config().idleMapping=false;
   Map sequenceMap;MapMaker sequenceMapper(sequenceMap,camera);ProbeTracker sequence(ImageRef(320,240),camera,sequenceMap,sequenceMapper);
   std::mt19937 random(42);std::uniform_real_distribution<double> frequency(-.65,.65),phase(0.,6.283185307179586);
   std::vector<Vector<3>> waves;for(int i=0;i<30;++i)waves.push_back(makeVector(frequency(random),frequency(random),phase(random)));
   std::vector<byte> pixels(400*240);
   for(int y=0;y<240;++y)for(int x=0;x<400;++x) {
     double intensity=128.;for(auto wave:waves)intensity+=9.*std::sin(wave[0]*x+wave[1]*y+wave[2]);
     pixels[y*400+x]=static_cast<byte>(std::max(0.,std::min(255.,intensity)));
   }
   std::vector<double> firstStates;
   for(int lifecycle=0;lifecycle<2;++lifecycle) {
     if(lifecycle)sequence.Restart();int good=0;size_t state=0;
     for(int frame=0;frame<55;++frame) {
       Image<byte> image(ImageRef(320,240));
       for(int y=0;y<240;++y)for(int x=0;x<320;++x)image[y][x]=pixels[y*400+x+frame];
       if(frame==0||frame==15)sequence.StartTracking();
       sequence.SetFrameTimestamp(frame/30.);sequence.TrackFrame(image,0,false);
       Vector<6> pose=sequence.GetCurrentPose().ln();require(std::isfinite(pose*pose),"mobile plane exported nonfinite pose");
       if(frame>15&&sequence.TrackingQuality()==2&&sequence.PoseInliers()>=10)++good;
       for(int component=0;component<8;++component) {
         double value=component<6?pose[component]:(component==6?sequence.TrackingQuality():sequence.MeasurementsFound());
         if(lifecycle==0)firstStates.push_back(value);
         else {
           if(!(std::abs(value-firstStates.at(state))<1e-10))
             std::cerr<<std::setprecision(17)<<"Repeat mismatch frame="<<frame<<" component="<<component<<" previous="<<firstStates.at(state)<<" observed="<<value<<" difference="<<(value-firstStates.at(state))<<"\n";
           require(std::abs(value-firstStates.at(state))<1e-10,"mobile reset/replay changed pose or visual support");
         }
         ++state;
       }
     }
     std::cout<<"Mobile fixedpair plane: GOOD="<<good<<"/39 points="<<sequenceMap.vpPoints.size()<<" keyframes="<<sequenceMap.vpKeyFrames.size()<<"\n";
     require(sequenceMap.IsGood()&&good>=35,"mobile fixedpair initialization and actual-image tracking failed");
     require(sequenceMap.vpPoints.size()<=static_cast<size_t>(IPTAMMobile::Config().maxActivePoints),"mobile plane exceeded active point limit");
     Image<byte> occlusion(ImageRef(320,240));occlusion.fill(0);
     for(int frame=55;frame<63;++frame) {sequence.SetFrameTimestamp(frame/30.);sequence.TrackFrame(occlusion,0,false);}
     require(sequence.TrackingQuality()==0&&sequence.PoseInliers()==0,"mobile occlusion fabricated visual support");
     Image<byte> returned(ImageRef(320,240));
     for(int y=0;y<240;++y)for(int x=0;x<320;++x)returned[y][x]=pixels[y*400+x+54];
     for(int frame=63;frame<71;++frame) {sequence.SetFrameTimestamp(frame/30.);sequence.TrackFrame(returned,0,false);}
     require(sequence.TrackingQuality()==2&&sequence.PoseInliers()>=10,"mobile relocalization did not recover the known scene");
   }
 }
 IPTAMMobile::ResetConfig();
 {
   Map archived;auto* old=new KeyFrame(first);old->MakeKeyFrame_Rest();old->se3CfromW.get_translation()=makeVector(.2,0.,0.);
   archived.vpRetainedKeyFrames.push_back(old);KeyFrame incoming(first);
   Relocaliser recovery(archived,camera);
   require(recovery.AttemptRecovery(incoming),"retained keyframe archive was omitted from relocalization");
   require(norm(recovery.BestPose().get_translation()-old->se3CfromW.get_translation())<1e-6,"retained keyframe recovery used an incorrect pose");
 }
 Measurement measurement;measurement.v2RootPos=makeVector(10.,20.);measurement.v2BundleRootPos=makeVector(11.,21.);
 require(norm(measurement.BundlePosition()-measurement.v2RootPos)==0.,"uncorrected BA measurement changed raw pixels");
 measurement.rollingShutterCorrected=true;
 require(norm(measurement.BundlePosition()-measurement.v2BundleRootPos)==0.&&measurement.v2RootPos[0]==10.,"RS correction overwrote raw measurement");
 std::cout<<"Mobile frontend "<<checks<<" checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<"Mobile frontend: "<<e.what()<<"\n";return 1;}}
