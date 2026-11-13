#include "PatchFinder.h"
#include "KeyFrame.h"
#include "TrackerData.h"
#include "Tracker.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

static void require(bool condition,const char *message) {
    if(!condition) throw std::runtime_error(message);
}
class SearchRegressionTracker:public Tracker {
 public:
  using Tracker::Tracker;
  int Search(TrackerData& data) {std::vector<TrackerData*> points{&data};return SearchForPoints(points,10,0);}
};
int main() {
    try {
        KeyFrame source;
        auto& image=source.aLevels[0].im;
        image.resize(CVD::ImageRef(640,480));
        for(int y=0;y<480;y++) for(int x=0;x<640;x++)
            image[y][x]=((17*x+31*y)^((x>>3)*97))&255;
        MapPoint point;
        point.pPatchSourceKF=&source;
        point.nSourceLevel=0;
        point.irCenter=CVD::ImageRef(320,240);
        point.v3WorldPos=TooN::makeVector(0.,0.,2.);
        point.v3PixelRight_W=TooN::makeVector(.005,0.,0.);
        point.v3PixelDown_W=TooN::makeVector(0.,.005,0.);
        TooN::Matrix<2> jacobian=400.*TooN::Identity;
        TooN::SE3<> pose;

        PatchFinder finder;
        finder.MakeTemplateCoarse(point,pose,jacobian);
        require(!finder.TemplateBad(),"identity warp rejected a valid textured patch");
        require(finder.ZMSSDAtPoint(image,point.irCenter)==0,"identity patch template changed source pixels");

        // A large depth change rejects the patch. Returning to the exact cached
        // view must regenerate/validate it rather than retain the rejected flag.
        TooN::SE3<> farPose;
        farPose.get_translation()=TooN::makeVector(0.,0.,20.);
        require(finder.CalcSearchLevelAndWarpMatrix(point,farPose,jacobian)<0,"far-away warp should be rejected");
        require(finder.TemplateBad(),"rejected warp did not mark template bad");
        finder.MakeTemplateCoarse(point,pose,jacobian);
        require(!finder.TemplateBad(),"return to cached valid view retained stale rejection");
        require(finder.ZMSSDAtPoint(image,point.irCenter)==0,"cached-view recovery corrupted the template");

        // The public wrapper must stop before inversion on a singular warp.
        point.v3PixelRight_W=TooN::Zeros;
        finder.MakeTemplateCoarse(point,pose,jacobian);
        require(finder.TemplateBad(),"singular public-wrapper warp was accepted");
        point.v3PixelRight_W=TooN::makeVector(std::numeric_limits<double>::quiet_NaN(),0.,0.);
        finder.MakeTemplateCoarse(point,pose,jacobian);
        require(finder.TemplateBad(),"non-finite public-wrapper warp was accepted");

        // Non-finite geometry must fail projection before any pixel conversion.
        ATANCamera camera("projection-regression");
        camera.SetParameters(TooN::makeVector(400./640.,400./480.,.5,.5,0.));
        camera.SetImageSize(CVD::ImageRef(640,480));
        TrackerData::irImageSize=CVD::ImageRef(640,480);
        TrackerData data(&point);
        point.v3WorldPos=TooN::makeVector(std::numeric_limits<double>::quiet_NaN(),0.,2.);
        data.Project(pose,camera);
        require(!data.bInImage,"NaN point geometry was reported visible");
        point.v3WorldPos=TooN::makeVector(0.,0.,std::numeric_limits<double>::infinity());
        data.Project(pose,camera);
        require(!data.bInImage,"infinite point geometry was reported visible");
        Map map; MapMaker mapper(map,camera);
        SearchRegressionTracker tracker(CVD::ImageRef(640,480),camera,map,mapper);
        point.v3WorldPos=TooN::makeVector(0.,0.,2.);
        point.v3PixelRight_W=TooN::makeVector(.005,0.,0.);
        data.ProjectAndDerivs(pose,camera);
        data.Finder.MakeTemplateCoarse(point,pose,jacobian);
        require(data.bInImage && !data.Finder.TemplateBad(),"search regression requires a previously valid candidate");
        // A pose refinement rejects a previously visible candidate but leaves its
        // last pixel coordinates cached. Searching must honour the new rejection.
        TooN::SE3<> behind; behind.get_translation()=TooN::makeVector(0.,0.,-3.);
        data.ProjectAndDerivs(behind,camera);
        require(!data.bInImage,"behind-camera pose refinement stayed visible");
        require(tracker.Search(data)==0 && !data.bSearched && tracker.MeasurementsAttempted()==0,"rejected projection was searched or counted");
        // A nonfinite linearized update can retain the old visibility flag.
        // The search boundary must reject its pixels before CVD::ir conversion.
        data.ProjectAndDerivs(pose,camera); data.CalcJacobian();
        TooN::Vector<6> invalidUpdate=TooN::Zeros;
        invalidUpdate[0]=std::numeric_limits<double>::infinity();
        data.LinearUpdate(invalidUpdate);
        require(data.bInImage && !std::isfinite(data.v2Image*data.v2Image),"nonfinite linear update fixture did not reach search boundary");
        require(tracker.Search(data)==0 && !data.bInImage && !data.bSearched && tracker.MeasurementsAttempted()==0,"nonfinite pixel candidate reached integer search");
        std::cout<<"Patch finder: valid/rejected/cached-view, singular, non-finite regressions passed\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"Patch finder regression: "<<error.what()<<'\n';return 1;
    }
}
