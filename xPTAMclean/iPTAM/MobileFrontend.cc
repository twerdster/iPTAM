#include "MobileFrontend.h"
#ifdef IPTAM_MOBILE_PAPER
#include "ShiTomasi.h"
#include "LevelHelpers.h"
#include "vector_image_ref.h"
#include <algorithm>
#include <cmath>
#include <set>
namespace IPTAMMobile {
namespace {
using CVD::ImageRef;
double Score(CVD::Image<CVD::byte>& image, ImageRef p) {
  return image.in_image_with_border(p, 6) ? FindShiTomasiScoreAtPoint(image, 3, p) : 0.;
}
bool Ascend(CVD::Image<CVD::byte>& image, ImageRef& p, double minimum) {
  for(int iteration=0; iteration<3; ++iteration) {
    ImageRef best=p; double bestScore=Score(image,p);
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
      ImageRef candidate=p+ImageRef(dx,dy); double score=Score(image,candidate);
      if(score>bestScore) { best=candidate; bestScore=score; }
    }
    if(best==p) break;
    p=best;
  }
  return Score(image,p)>=minimum;
}
}
void DetectMultiscaleCorners(KeyFrame& frame,double minimumScore) {
  for(int l=0;l<LEVELS;++l) {
    Level& level=frame.aLevels[l]; level.vCorners.clear(); level.vMaxCorners.clear();
    level.vCandidates.clear(); level.vImplaneCorners.clear(); level.bImplaneCornersCached=false;
  }
  auto& image=frame.aLevels[2].im;
  CVD::Image<double> scores(image.size()); scores.fill(0.);
  for(int y=6;y<image.size().y-6;++y) for(int x=6;x<image.size().x-6;++x)
    scores[y][x]=Score(image,ImageRef(x,y));
  std::set<std::pair<int,int>> used;
  for(int y=7;y<image.size().y-7;++y) for(int x=7;x<image.size().x-7;++x) {
    double score=scores[y][x]; if(score<minimumScore) continue;
    bool maximum=true;
    for(int dy=-1;dy<=1&&maximum;++dy) for(int dx=-1;dx<=1;++dx) {
      if(dx==0&&dy==0) continue;
      double other=scores[y+dy][x+dx];
      if(other>score || (other==score && (dy<0||(dy==0&&dx<0)))) {maximum=false;break;}
    }
    if(!maximum) continue;
    ImageRef locations[3]; locations[2]=ImageRef(x,y);
    locations[1]=CVD::ir_rounded(LevelNPos(LevelZeroPos(locations[2],2),1));
    if(!Ascend(frame.aLevels[1].im,locations[1],minimumScore)) continue;
    locations[0]=CVD::ir_rounded(LevelZeroPos(locations[1],1));
    if(!Ascend(frame.aLevels[0].im,locations[0],minimumScore)) continue;
    if(!used.insert({locations[0].y,locations[0].x}).second) continue;
    // Each level records the corresponding multiscale feature, not FAST proposals.
    for(int l=0;l<3;++l) {
      Level& level=frame.aLevels[l]; level.vCorners.push_back(locations[l]);
      Candidate candidate; candidate.irLevelPos=locations[l];
      candidate.v2RootPos=LevelZeroPos(locations[l],l); candidate.dSTScore=Score(level.im,locations[l]);
      level.vCandidates.push_back(candidate);
    }
  }
  for(int l=0;l<LEVELS;++l) {
    Level& level=frame.aLevels[l];
    std::sort(level.vCorners.begin(),level.vCorners.end(),[](ImageRef a,ImageRef b){return a.y<b.y||(a.y==b.y&&a.x<b.x);});
    level.vCorners.erase(std::unique(level.vCorners.begin(),level.vCorners.end()),level.vCorners.end());
    level.vMaxCorners=level.vCorners;
    level.vCornerRowLUT.resize(level.im.size().y);
    size_t i=0; for(int y=0;y<level.im.size().y;++y) {
      while(i<level.vCorners.size()&&level.vCorners[i].y<y) ++i;
      level.vCornerRowLUT[y]=static_cast<int>(i);
    }
  }
}
bool RotationNeedsBlurFallback(const TooN::Vector<6>& rotation) {
  double norm2=rotation.slice<3,3>()*rotation.slice<3,3>();
  // Fixed, image-only trigger: approximately seven degrees between exposures.
  return std::isfinite(norm2)&&norm2>0.12*0.12;
}
}
#endif
