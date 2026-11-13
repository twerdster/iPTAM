#include "MobileInitializer.h"
#include "HomographyInit.h"
#include "ShiTomasi.h"
#include "SVD.h"
#include "SymEigen.h"
#include "SmallMatrixOpts.h"
#include "SmallBlurryImage.h"
#include "vector_image_ref.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>

namespace IPTAMMobile {
namespace {
using namespace TooN;
using PixelPair=std::pair<Vector<2>,Vector<2>>;
constexpr double kEpsilon=1e-10;

template<int N> bool Finite(const Vector<N>& v) {
  for(int i=0;i<N;++i) if(!std::isfinite(v[i])) return false;
  return true;
}
template<int R,int C> bool Finite(const Matrix<R,C>& m) {
  for(int r=0;r<R;++r) for(int c=0;c<C;++c)
    if(!std::isfinite(m[r][c])) return false;
  return true;
}
bool FinitePose(const SE3<>& pose) {
  return Finite(pose.get_translation()) && Finite(pose.get_rotation().get_matrix());
}
double Median(std::vector<double> v) {
  if(v.empty()) return 0.0;
  const auto middle=v.begin()+v.size()/2;
  std::nth_element(v.begin(),middle,v.end()); return *middle;
}
double TukeyCost(double squared,double cutoffSquared) {
  if(!std::isfinite(squared) || squared>=cutoffSquared) return cutoffSquared/3.0;
  const double remaining=1.0-squared/cutoffSquared;
  return cutoffSquared/3.0*(1.0-remaining*remaining*remaining);
}
double TukeyWeight(double squared,double cutoffSquared) {
  if(!std::isfinite(squared) || squared>=cutoffSquared) return 0.0;
  const double remaining=1.0-squared/cutoffSquared; return remaining*remaining;
}
bool NormalizeSL3(Matrix<3>& H) {
  if(!Finite(H)) return false;
  const double determinant=M3Det(H);
  if(!std::isfinite(determinant) || std::abs(determinant)<1e-12) return false;
  H/=std::cbrt(determinant);
  return Finite(H);
}
Matrix<3> Generator(int index) {
  // Trace-free basis of sl(3). A left update exp(A)H is projectively invariant.
  Matrix<3> result=Zeros;
  switch(index) {
    case 0: result[0][2]=1; break;
    case 1: result[1][2]=1; break;
    case 2: result[0][1]=1; break;
    case 3: result[1][0]=1; break;
    case 4: result[0][0]=1; result[1][1]=-1; break;
    case 5: result[0][0]=1; result[1][1]=1; result[2][2]=-2; break;
    case 6: result[2][0]=1; break;
    case 7: result[2][1]=1; break;
  }
  return result;
}
Matrix<3> ExpSL3(const Vector<8>& increment) {
  Matrix<3> A=Zeros;
  for(int i=0;i<8;++i) A+=Generator(i)*increment[i];
  double maxRow=0.0;
  for(int i=0;i<3;++i) {
    double sum=0; for(int j=0;j<3;++j) sum+=std::abs(A[i][j]);
    maxRow=std::max(maxRow,sum);
  }
  int scaling=0;
  while(maxRow>0.5 && scaling<20) { A*=0.5; maxRow*=0.5; ++scaling; }
  Matrix<3> E=Identity, term=Identity;
  for(int i=1;i<=18;++i) { term=term*A/double(i); E+=term; }
  for(int i=0;i<scaling;++i) E=E*E;
  return E;
}
bool Project(ATANCamera& camera,const Vector<3>& point,Vector<2>& pixel,
             Matrix<2,3>* jacobian=nullptr) {
  if(!Finite(point) || point[2]<=kEpsilon) return false;
  const Vector<2> plane=makeVector(point[0]/point[2],point[1]/point[2]);
  pixel=camera.Project(plane);
  if(camera.Invalid() || !Finite(pixel)) return false;
  if(jacobian) {
    const auto cameraJacobian=camera.GetProjectionDerivs();
    Matrix<2,3> division=Zeros;
    division[0][0]=division[1][1]=1.0/point[2];
    division[0][2]=-point[0]/(point[2]*point[2]);
    division[1][2]=-point[1]/(point[2]*point[2]);
    *jacobian=cameraJacobian*division;
    if(!Finite(*jacobian)) return false;
  }
  return true;
}
bool InImage(const Vector<2>& pixel,const CVD::ImageRef& size,double border=0) {
  return Finite(pixel) && pixel[0]>=border && pixel[1]>=border &&
    pixel[0]<=size.x-1-border && pixel[1]<=size.y-1-border;
}
bool Sample(const CVD::BasicImage<CVD::byte>& image,const Vector<2>& pixel,double& value) {
  if(!InImage(pixel,image.size(),1)) return false;
  const int x=int(std::floor(pixel[0])), y=int(std::floor(pixel[1]));
  const double fx=pixel[0]-x, fy=pixel[1]-y;
  value=(1-fy)*((1-fx)*image[CVD::ImageRef(x,y)]+fx*image[CVD::ImageRef(x+1,y)])+
       fy*((1-fx)*image[CVD::ImageRef(x,y+1)]+fx*image[CVD::ImageRef(x+1,y+1)]);
  return std::isfinite(value);
}
Vector<2> ToLevel(const Vector<2>& root,int level) {
  const double scale=double(1<<level);
  return (root+makeVector(.5,.5))/scale-makeVector(.5,.5);
}
Vector<2> ToRoot(const Vector<2>& pixel,int level) {
  return (pixel+makeVector(.5,.5))*double(1<<level)-makeVector(.5,.5);
}
bool Warp(ATANCamera& firstCamera,ATANCamera& currentCamera,const Matrix<3>& H,
          const Vector<2>& firstPixel,Vector<2>& currentPixel) {
  const Vector<2> ray=firstCamera.UnProject(firstPixel);
  if(!Finite(ray)) return false;
  return Project(currentCamera,H*unproject(ray),currentPixel);
}

struct CalibratedMatch {
  Vector<2> firstPixel,currentPixel,firstRay;
};
double HomographyCost(const Matrix<3>& H,const std::vector<CalibratedMatch>& matches,
                      ATANCamera camera,double threshold) {
  double cost=0.0;
  for(const auto& match:matches) {
    Vector<2> prediction;
    if(!Project(camera,H*unproject(match.firstRay),prediction)) cost+=threshold*threshold/3;
    else { const Vector<2> r=match.currentPixel-prediction; cost+=TukeyCost(r*r,threshold*threshold); }
  }
  return cost;
}

bool RefineHomography(Matrix<3>& H,const std::vector<CalibratedMatch>& matches,
                       ATANCamera camera,double threshold,int& inliers,double& cost) {
  if(matches.size()<8 || !NormalizeSL3(H)) return false;
  // The previous frame supplies the starting H; no RANSAC/minimal-set pair search.
  // A generous initial robust cutoff shrinks to the measurement-noise cutoff.
  for(double cutoff:{threshold*4.0,threshold*2.0,threshold}) {
    for(int iteration=0;iteration<12;++iteration) {
      Matrix<8> information=Zeros; Vector<8> rhs=Zeros;
      int used=0;
      for(const auto& match:matches) {
        const Vector<3> projected=H*unproject(match.firstRay);
        Vector<2> pixel; Matrix<2,3> Jproject;
        if(!Project(camera,projected,pixel,&Jproject)) continue;
        const Vector<2> residual=match.currentPixel-pixel;
        const double weight=TukeyWeight(residual*residual,cutoff*cutoff);
        if(weight<=0) continue;
        Matrix<2,8> J;
        for(int d=0;d<8;++d) J.T()[d]=Jproject*Generator(d)*projected;
        information+=weight*J.T()*J; rhs+=weight*J.T()*residual; ++used;
      }
      if(used<8 || !Finite(information) || !Finite(rhs)) break;
      SymEigen<8> eigen(information);
      const auto values=eigen.get_evalues();
      if(!Finite(values) || values[7]<=0 || values[0]/values[7]<1e-10) break;
      const Vector<8> update=eigen.backsub(rhs);
      if(!Finite(update)) return false;
      const double previous=HomographyCost(H,matches,camera,cutoff);
      bool accepted=false;
      for(double fraction=1.0;fraction>=1.0/64.0;fraction*=.5) {
        Matrix<3> candidate=ExpSL3(update*fraction)*H;
        if(!NormalizeSL3(candidate)) continue;
        const double candidateCost=HomographyCost(candidate,matches,camera,cutoff);
        if(std::isfinite(candidateCost) && candidateCost<previous) {
          H=candidate; accepted=true; break;
        }
      }
      if(!accepted || update*update<1e-12) break;
    }
  }
  inliers=0; cost=HomographyCost(H,matches,camera,threshold);
  for(const auto& match:matches) {
    Vector<2> prediction;
    if(Project(camera,H*unproject(match.firstRay),prediction) &&
       norm_sq(match.currentPixel-prediction)<threshold*threshold) ++inliers;
  }
  return Finite(H) && std::isfinite(cost);
}

// Reuse only the original Faugeras/Lustman algebra. Unlike Compute(), this does
// not pick a single solution with a same-pair epipolar score: ambiguity persists
// as two independent 3D maps and pose estimators until subsequent images decide.
class DecompositionAccess : public HomographyInit {
public:
  std::vector<HomographyDecomposition> Decompose(const Matrix<3>& H) {
    mm3BestHomography=H; DecomposeHomography(); return mvDecompositions;
  }
};
struct Point {
  Vector<2> firstPixel;
  Vector<3> world;
};
struct Hypothesis {
  int id=-1;
  SE3<> pose;
  Vector<3> normal=Zeros;
  std::vector<Point> points;
  double condition=0.0,parallax=0.0,cost=0.0;
  int inliers=0;
  bool valid=false;
};
bool Triangulate(const Vector<2>& a,const Vector<2>& b,const SE3<>& pose,Vector<3>& X) {
  const Vector<3> first=unproject(a), second=unproject(b);
  const Vector<3> rotated=pose.get_rotation()*first, t=pose.get_translation();
  const Vector<3> u=rotated^second, v=t^second;
  const double denominator=u*u;
  if(!std::isfinite(denominator) || denominator<1e-12) return false;
  const double depth=-(u*v)/denominator;
  X=first*depth;
  const Vector<3> Y=pose*X;
  return Finite(X) && Finite(Y) && X[2]>1e-6 && Y[2]>1e-6;
}
double Angle(const Vector<3>& a,const Vector<3>& b) {
  return std::atan2(norm(a^b),a*b);
}
std::pair<Vector<3>,Vector<3>> Tangents(const Vector<3>& normal) {
  Vector<3> axis=std::abs(normal[0])<.8?makeVector(1.,0.,0.):makeVector(0.,1.,0.);
  Vector<3> a=normal^axis; a/=norm(a);
  Vector<3> b=normal^a; b/=norm(b); return {a,b};
}
Matrix<2,6> PoseJacobian(const Vector<3>& point,const Matrix<2,3>& projection) {
  Matrix<3,6> motion=Zeros;
  for(int d=0;d<3;++d) motion[d][d]=1.0;
  for(int d=0;d<3;++d) {
    Vector<3> axis=Zeros; axis[d]=1.0;
    motion.T()[d+3]=axis^point;
  }
  return projection*motion;
}
double DecompositionCondition(const Hypothesis& hypothesis,
    const std::vector<CalibratedMatch>& matches,ATANCamera camera,double cutoff) {
  Matrix<8> information=Zeros;
  const auto tangent=Tangents(hypothesis.normal);
  int used=0;
  for(const auto& match:matches) {
    const Vector<3> ray=unproject(match.firstRay);
    const double denominator=hypothesis.normal*ray;
    if(denominator<=1e-6) continue;
    const Vector<3> X=ray/denominator, Y=hypothesis.pose*X;
    Vector<2> prediction; Matrix<2,3> projection;
    if(!Project(camera,Y,prediction,&projection)) continue;
    const Vector<2> residual=prediction-match.currentPixel;
    const double weight=TukeyWeight(residual*residual,cutoff*cutoff);
    if(weight<=0) continue;
    Matrix<2,8> J;
    J.slice<0,0,2,6>()=PoseJacobian(Y,projection);
    J.T()[6]=projection*hypothesis.pose.get_rotation()*(-ray*(ray*tangent.first)/(denominator*denominator));
    J.T()[7]=projection*hypothesis.pose.get_rotation()*(-ray*(ray*tangent.second)/(denominator*denominator));
    information+=weight*J.T()*J; ++used;
  }
  if(used<8 || !Finite(information)) return 0;
  SymEigen<8> eigen(information); const auto values=eigen.get_evalues();
  if(!Finite(values) || values[7]<=0 || values[0]<=0) return 0;
  return values[0]/values[7];
}
struct PoseMeasurement {Vector<3> point; Vector<2> pixel;};
std::vector<PoseMeasurement> Associate(const Hypothesis& hypothesis,
                                     const std::vector<CalibratedMatch>& matches) {
  std::vector<PoseMeasurement> measurements;
  for(const auto& point:hypothesis.points) {
    for(const auto& match:matches) {
      if(norm_sq(point.firstPixel-match.firstPixel)<.25) {
        measurements.push_back({point.world,match.currentPixel}); break;
      }
    }
  }
  return measurements;
}
double PoseCost(const SE3<>& pose,const std::vector<PoseMeasurement>& measurements,
                ATANCamera camera,double cutoff,int* inliers=nullptr) {
  double sum=0; if(inliers) *inliers=0;
  for(const auto& m:measurements) {
    Vector<2> prediction;
    if(!Project(camera,pose*m.point,prediction)) sum+=cutoff*cutoff/3;
    else {
      const double squared=norm_sq(prediction-m.pixel);
      sum+=TukeyCost(squared,cutoff*cutoff);
      if(inliers && squared<cutoff*cutoff) ++*inliers;
    }
  }
  return sum;
}
void RefinePose(Hypothesis& hypothesis,const std::vector<CalibratedMatch>& matches,
                ATANCamera camera,double threshold,int minimumInliers) {
  const auto measurements=Associate(hypothesis,matches);
  hypothesis.valid=false; hypothesis.inliers=0;
  if(measurements.size()<std::size_t(minimumInliers)) {
    hypothesis.cost=0.0; return;
  }
  for(double cutoff:{threshold*4,threshold*2,threshold}) {
    for(int iteration=0;iteration<15;++iteration) {
      Matrix<6> information=Zeros; Vector<6> rhs=Zeros;
      int used=0;
      for(const auto& m:measurements) {
        const Vector<3> point=hypothesis.pose*m.point;
        Vector<2> prediction; Matrix<2,3> projection;
        if(!Project(camera,point,prediction,&projection)) continue;
        const Vector<2> residual=m.pixel-prediction;
        const double weight=TukeyWeight(residual*residual,cutoff*cutoff);
        if(weight<=0) continue;
        const Matrix<2,6> J=PoseJacobian(point,projection);
        information+=weight*J.T()*J; rhs+=weight*J.T()*residual; ++used;
      }
      if(used<6 || !Finite(information) || !Finite(rhs)) break;
      SymEigen<6> eigen(information); const auto values=eigen.get_evalues();
      if(!Finite(values) || values[5]<=0 || values[0]/values[5]<1e-10) break;
      const Vector<6> update=eigen.backsub(rhs);
      if(!Finite(update)) return;
      const double cost=PoseCost(hypothesis.pose,measurements,camera,cutoff);
      bool accepted=false;
      for(double fraction=1.;fraction>=1./64.;fraction*=.5) {
        const SE3<> candidate=SE3<>::exp(update*fraction)*hypothesis.pose;
        if(!FinitePose(candidate)) continue;
        if(PoseCost(candidate,measurements,camera,cutoff)<cost) {
          hypothesis.pose=candidate; accepted=true; break;
        }
      }
      if(!accepted || update*update<1e-12) break;
    }
  }
  hypothesis.cost=PoseCost(hypothesis.pose,measurements,camera,threshold,&hypothesis.inliers);
  hypothesis.valid=FinitePose(hypothesis.pose) && std::isfinite(hypothesis.cost) &&
                   hypothesis.inliers>=minimumInliers;
}
}

struct MobileInitializer::Impl {
  Options options;
  KeyFrame first;
  ATANCamera firstCamera{"mobile-initializer"};
  bool begun=false;
  double lastTimestamp=0;
  Matrix<3> H=Identity;
  std::vector<Vector<2>> features;
  std::vector<Hypothesis> hypotheses;
  std::unique_ptr<SmallBlurryImage> lastSBI;
  explicit Impl(const Options& settings):options(settings) {}
  bool ValidOptions() const {
    return options.maxFeatures>=options.minimumInliers && options.minimumInliers>=10 &&
      options.minimumConditionRatio>0 && std::isfinite(options.minimumConditionRatio) &&
      options.minimumMedianParallaxRadians>0 && std::isfinite(options.minimumMedianParallaxRadians) &&
      options.maximumReprojectionError>0 && std::isfinite(options.maximumReprojectionError) &&
      options.maximumPatchRms>0 && std::isfinite(options.maximumPatchRms) &&
      options.patchSearchRadius>=1 && options.patchSearchRadius<=20;
  }
  std::vector<Hypothesis> Decompose(const std::vector<CalibratedMatch>& matches,ATANCamera camera) {
    std::vector<Hypothesis> candidates;
    SVD<3> svd(H); const auto singular=svd.get_diagonal();
    // Repeated singular values are the pure-rotation/degenerate case. Avoid the
    // original routine's exact-equality branch and its square-root divisions.
    if(!Finite(singular) || singular[2]<=1e-8 ||
       (singular[0]-singular[2])/singular[0]<1e-5 ||
       singular[0]-singular[1]<1e-10 || singular[1]-singular[2]<1e-10) return candidates;
    DecompositionAccess algebra;
    auto decompositions=algebra.Decompose(H);
    for(std::size_t d=0;d<decompositions.size();++d) {
      const auto& decomposition=decompositions[d];
      if(decomposition.d<=0 || !Finite(decomposition.v3n) ||
         !FinitePose(decomposition.se3SecondFromFirst)) continue;
      Hypothesis hypothesis; hypothesis.id=int(d);
      hypothesis.pose=decomposition.se3SecondFromFirst;
      hypothesis.pose.get_translation()/=decomposition.d;
      hypothesis.normal=decomposition.v3n;
      const double length=norm(hypothesis.normal);
      if(!std::isfinite(length) || std::abs(length-1)>1e-5 ||
         norm(hypothesis.pose.get_translation())<1e-5) continue;
      std::vector<double> parallaxes;
      for(const auto& match:matches) {
        Vector<2> prediction;
        if(!Project(camera,H*unproject(match.firstRay),prediction) ||
           norm_sq(prediction-match.currentPixel)>=options.maximumReprojectionError*options.maximumReprojectionError)
          continue;
        const Vector<2> raySecond=camera.UnProject(match.currentPixel);
        Vector<3> X;
        if(!Finite(raySecond) || !Triangulate(match.firstRay,raySecond,hypothesis.pose,X)) continue;
        // Cheirality alone admits extreme/unstable plane intersections; plane
        // visibility also constrains the normal sign and bounded finite map.
        if(hypothesis.normal*unproject(match.firstRay)<=1e-5 || norm(X)>1e5) continue;
        hypothesis.points.push_back({match.firstPixel,X});
        parallaxes.push_back(Angle(unproject(match.firstRay),
            hypothesis.pose.get_rotation().inverse()*unproject(raySecond)));
      }
      if(hypothesis.points.size()<std::size_t(options.minimumInliers) ||
         hypothesis.points.size()<.8*matches.size()) continue;
      hypothesis.parallax=Median(parallaxes);
      hypothesis.condition=DecompositionCondition(hypothesis,matches,camera,options.maximumReprojectionError);
      hypothesis.cost=0; hypothesis.inliers=int(hypothesis.points.size());
      hypothesis.valid=hypothesis.parallax>=options.minimumMedianParallaxRadians &&
                       hypothesis.condition>=options.minimumConditionRatio;
      bool duplicate=false;
      for(const auto& existing:candidates)
        if(norm((existing.pose*hypothesis.pose.inverse()).ln())<1e-5) duplicate=true;
      if(!duplicate) candidates.push_back(hypothesis);
    }
    // Visibility admits at most the two physically meaningful motion/plane
    // solutions for a valid planar first view; never rank using a reference.
    std::stable_sort(candidates.begin(),candidates.end(),[](const Hypothesis& a,const Hypothesis& b) {
      return a.points.size()>b.points.size();
    });
    if(candidates.size()>2) candidates.resize(2);
    if(candidates.size()==2) {
      // Compare robust objectives over the same physical feature identities.
      // Fewer positive-depth points in one map must not lower its objective
      // merely by reducing its denominator.
      for(int side=0;side<2;++side) {
        auto& points=candidates[side].points;
        const auto other=candidates[1-side].points;
        points.erase(std::remove_if(points.begin(),points.end(),[&](const Point& point) {
          for(const auto& p:other) if(norm_sq(p.firstPixel-point.firstPixel)<.25) return false;
          return true;
        }),points.end());
        if(points.size()<std::size_t(options.minimumInliers)) return {};
        candidates[side].inliers=int(points.size());
      }
    }
    return candidates;
  }
  void FillHypotheses(Result& result) const {
    for(const auto& h:hypotheses) {
      HypothesisDiagnostic diagnostic;
      diagnostic.id=h.id; diagnostic.positiveDepthPoints=int(h.points.size());
      diagnostic.poseInliers=h.inliers; diagnostic.conditionRatio=h.condition;
      diagnostic.medianParallaxRadians=h.parallax; diagnostic.robustCost=h.cost;
      diagnostic.valid=h.valid; diagnostic.secondFromFirst=h.pose;
      result.hypotheses.push_back(diagnostic);
    }
  }
  Result ProcessMatches(const std::vector<PixelPair>& pixels,ATANCamera camera,double timestamp,
                        const Matrix<3>* prediction=nullptr) {
    Result result;
    if(!begun) { result.reason="first keyframe is required"; return result; }
    if(!std::isfinite(timestamp) || timestamp<=lastTimestamp) {
      result.stage="rejected"; result.reason="nonmonotonic initializer timestamp"; return result;
    }
    lastTimestamp=timestamp;
    std::vector<CalibratedMatch> matches;
    const auto size=first.aLevels[0].im.size();
    for(const auto& pair:pixels) {
      if(!InImage(pair.first,size,1) || !InImage(pair.second,size,1)) continue;
      const Vector<2> ray=firstCamera.UnProject(pair.first);
      const Vector<2> currentRay=camera.UnProject(pair.second);
      if(!Finite(ray) || !Finite(currentRay)) continue;
      // Duplicate correspondences must not manufacture information/support.
      bool duplicate=false;
      for(const auto& previous:matches)
        if(norm_sq(previous.firstPixel-pair.first)<.25 ||
           norm_sq(previous.currentPixel-pair.second)<.25) { duplicate=true; break; }
      if(!duplicate) matches.push_back({pair.first,pair.second,ray});
    }
    result.tracked=int(matches.size()); result.stage="planar_tracking";result.homography=H;
    if(matches.size()<std::size_t(options.minimumInliers)) {
      result.reason="insufficient warped-patch correspondences"; FillHypotheses(result); return result;
    }
    Matrix<3> fitted=prediction?*prediction:H;
    if(!RefineHomography(fitted,matches,camera,options.maximumReprojectionError,result.inliers,result.homographyCost) ||
       result.inliers<options.minimumInliers) {
      result.reason="incremental SL3 fit has insufficient geometric support";
      FillHypotheses(result); return result;
    }
    H=fitted; result.homography=H;
    bool created=false;
    if(hypotheses.empty()) {
      auto candidates=Decompose(matches,camera);
      bool constrained=!candidates.empty();
      for(const auto& candidate:candidates) constrained=constrained && candidate.valid;
      if(!constrained) {
        // A visible alternative is not disproved by a low condition number.
        // Keep tracking H until *all* competing decompositions are constrained.
        hypotheses=candidates; FillHypotheses(result); hypotheses.clear();
        result.reason="decomposition lacks conditioned translational parallax";
        return result;
      }
      hypotheses=candidates; created=true;
    }
    else for(auto& hypothesis:hypotheses)
      RefinePose(hypothesis,matches,camera,options.maximumReprojectionError,options.minimumInliers);
    FillHypotheses(result);
    if(hypotheses.empty()) {
      result.reason="decomposition lacks conditioned translational parallax"; return result;
    }
    result.stage="competing_poses";
    int chosen=-1;
    if(hypotheses.size()==1 && hypotheses[0].valid) chosen=0;
    else if(!created && hypotheses.size()==2) {
      // Both maps must have sufficient same-frame evidence. A hypothesis that
      // cannot be observed is not automatically the losing objective function.
      if(hypotheses[0].valid && hypotheses[1].valid) {
        const double a=hypotheses[0].cost,b=hypotheses[1].cost;
        if(a>2.0*b && a-b>1e-6) chosen=1;
        else if(b>2.0*a && b-a>1e-6) chosen=0;
      }
    }
    if(chosen<0) {
      result.reason=created?"two visible planar hypotheses require subsequent images":
                             "robust objectives have not disambiguated the two maps";
      return result;
    }
    const auto& winner=hypotheses[chosen];
    result.ready=true; result.stage="ready"; result.reason="image-supported planar hypothesis selected";
    result.chosenHypothesis=winner.id; result.secondFromFirst=winner.pose;
    result.conditionRatio=winner.condition; result.medianParallaxRadians=winner.parallax;
    // Export current, not seed-frame, correspondences consistent with the chosen
    // 3D hypothesis; mapper then re-triangulates and performs its normal BA gates.
    for(const auto& m:Associate(winner,matches)) {
      Vector<2> prediction;
      if(!Project(camera,winner.pose*m.point,prediction) ||
         norm_sq(prediction-m.pixel)>=options.maximumReprojectionError*options.maximumReprojectionError) continue;
      for(const auto& point:winner.points) if(norm_sq(point.world-m.point)<1e-16) {
        result.matches.push_back({CVD::ir_rounded(point.firstPixel),CVD::ir_rounded(m.pixel)}); break;
      }
    }
    if(result.matches.size()<std::size_t(options.minimumInliers)) {
      result.ready=false; result.stage="competing_poses";
      result.reason="selected pose has insufficient exported stereo matches";
    }
    return result;
  }
};

MobileInitializer::MobileInitializer():MobileInitializer(Options()) {}
MobileInitializer::MobileInitializer(const Options& options):impl_(new Impl(options)) {}
MobileInitializer::~MobileInitializer()=default;
void MobileInitializer::Reset() { const Options options=impl_->options; impl_.reset(new Impl(options)); }
const KeyFrame& MobileInitializer::FirstKeyFrame() const { return impl_->first; }

bool MobileInitializer::Begin(const KeyFrame& first,const ATANCamera& camera,double timestamp) {
  Reset();
  if(!impl_->ValidOptions() || !std::isfinite(timestamp) ||
     first.aLevels[0].im.size().x<64 || first.aLevels[0].im.size().y<48) return false;
  for(int level=1;level<LEVELS;++level)
    if(first.aLevels[level].im.size()!=first.aLevels[level-1].im.size()/2) return false;
  impl_->first=first; impl_->firstCamera=camera;
  impl_->lastTimestamp=timestamp;
  auto& image=impl_->first.aLevels[2].im;
  if(image.size().x<16 || image.size().y<16) return false;
  struct Corner { CVD::ImageRef pixel; double score; };
  std::vector<Corner> corners;
  for(int y=5;y<image.size().y-5;++y) for(int x=5;x<image.size().x-5;++x) {
    const double score=FindShiTomasiScoreAtPoint(image,2,CVD::ImageRef(x,y));
    if(std::isfinite(score) && score>50) corners.push_back({CVD::ImageRef(x,y),score});
  }
  std::stable_sort(corners.begin(),corners.end(),[](const Corner& a,const Corner& b) {return a.score>b.score;});
  for(const auto& corner:corners) {
    Vector<2> pixel=ToRoot(vec(corner.pixel),2);
    bool close=false;
    for(const auto& previous:impl_->features)
      if(norm_sq(pixel-previous)<12.0*12.0) {close=true;break;}
    if(close) continue;
    bool valid=true;
    // Multi-scale ST support and local ascent at L1 then L0, without FAST.
    for(int level=1;level>=0;--level) {
      auto& levelImage=impl_->first.aLevels[level].im;
      CVD::ImageRef center=CVD::ir_rounded(ToLevel(pixel,level));
      double best=0;
      for(int ascent=0;ascent<3;++ascent) {
        CVD::ImageRef bestPixel=center;
        for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
          const auto candidate=center+CVD::ImageRef(dx,dy);
          if(!levelImage.in_image_with_border(candidate,5)) continue;
          const double score=FindShiTomasiScoreAtPoint(levelImage,2,candidate);
          if(score>best) {best=score; bestPixel=candidate;}
        }
        if(bestPixel==center) break;
        center=bestPixel;
      }
      if(best<50) {valid=false;break;}
      pixel=ToRoot(vec(center),level);
    }
    if(valid && Finite(impl_->firstCamera.UnProject(pixel))) impl_->features.push_back(pixel);
    if(impl_->features.size()>=std::size_t(impl_->options.maxFeatures)) break;
  }
  impl_->begun=true;
  impl_->lastSBI.reset(new SmallBlurryImage(impl_->first));
  return true;
}

MobileInitializer::Result MobileInitializer::ProcessCorrespondences(
    const std::vector<PixelPair>& matches,const ATANCamera& camera,double timestamp) {
  return impl_->ProcessMatches(matches,camera,timestamp);
}

MobileInitializer::Result MobileInitializer::Process(const KeyFrame& current,
    const ATANCamera& camera,double timestamp) {
  if(!impl_->begun || current.aLevels[0].im.size()!=impl_->first.aLevels[0].im.size()) {
    Result result; result.reason="initializer image-size discontinuity or missing first view"; return result;
  }
  ATANCamera firstCamera=impl_->firstCamera,currentCamera=camera;
  if(!std::isfinite(timestamp) || timestamp<=impl_->lastTimestamp) {
    Result result;result.stage="rejected";result.reason="nonmonotonic initializer timestamp";return result;
  }
  Matrix<3> prediction=impl_->H;
  // The same inexpensive full-frame ESM rotation initializer used by PTAM's
  // tracker. It supplies a prediction only; warped-patch support must validate
  // the resulting homography before either state is committed.
  KeyFrame sbiFrame;
  sbiFrame.aLevels[3].im=current.aLevels[3].im;
  std::unique_ptr<SmallBlurryImage> currentSBI(new SmallBlurryImage(sbiFrame));
  if(impl_->lastSBI) {
    impl_->lastSBI->MakeJacs();
    const double before=currentSBI->ZMSSD(*impl_->lastSBI);
    const auto alignment=currentSBI->IteratePosRelToTarget(*impl_->lastSBI,6);
    if(std::isfinite(alignment.second) && alignment.second<=before) {
      const SE3<> rotation=SmallBlurryImage::SE3fromSE2(alignment.first,currentCamera);
      if(FinitePose(rotation) && norm(rotation.get_rotation().ln())<.3) {
        Matrix<3> candidate=rotation.get_rotation().get_matrix()*prediction;
        if(NormalizeSL3(candidate)) prediction=candidate;
      }
    }
  }
  const auto& source=impl_->first.aLevels[1].im;
  const auto& target=current.aLevels[1].im;
  auto trackPatches=[&](const Matrix<3>& patchPrediction) {
  std::vector<PixelPair> matches;
  // Every patch is sourced from the original first image. A temporarily missed
  // feature may be rediscovered later; no uninterrupted MiniPatch trail needed.
  for(const auto& feature:impl_->features) {
    Vector<2> center;
    if(!Warp(firstCamera,currentCamera,patchPrediction,feature,center)) continue;
    const Vector<2> centerLevel=ToLevel(center,1);
    Vector<2> xWarp,yWarp;
    if(!Warp(firstCamera,currentCamera,patchPrediction,feature+makeVector(2.,0.),xWarp) ||
       !Warp(firstCamera,currentCamera,patchPrediction,feature+makeVector(0.,2.),yWarp)) continue;
    Matrix<2> localWarp;
    localWarp.T()[0]=ToLevel(xWarp,1)-centerLevel;
    localWarp.T()[1]=ToLevel(yWarp,1)-centerLevel;
    if(!Finite(localWarp) || std::abs(M2Det(localWarp))<1e-4) continue;
    const Matrix<2> inverseWarp=M2Inverse(localWarp);
    std::array<double,64> templateValues;
    std::array<Vector<2>,64> warped;
    std::array<Vector<2>,64> templateGradients;
    bool valid=true; double mean=0;
    int index=0;
    for(int y=-4;y<4 && valid;++y) for(int x=-4;x<4;++x) {
      const Vector<2> sourcePixel=ToLevel(feature,1)+makeVector(double(x),double(y));
      Vector<2> warpPixel;
      double left,right,up,down;
      if(!Sample(source,sourcePixel,templateValues[index]) ||
         !Sample(source,sourcePixel+makeVector(-1.,0.),left) ||
         !Sample(source,sourcePixel+makeVector(1.,0.),right) ||
         !Sample(source,sourcePixel+makeVector(0.,-1.),up) ||
         !Sample(source,sourcePixel+makeVector(0.,1.),down) ||
         !Warp(firstCamera,currentCamera,patchPrediction,ToRoot(sourcePixel,1),warpPixel)) {valid=false;break;}
      templateGradients[index]=inverseWarp.T()*makeVector((right-left)*.5,(down-up)*.5);
      warped[index]=ToLevel(warpPixel,1); mean+=templateValues[index]; ++index;
    }
    if(!valid) continue;
    mean/=64;
    double variance=0; for(auto& value:templateValues) {value-=mean; variance+=value*value;}
    if(variance<64*8*8) continue;
    auto residual=[&](const Vector<2>& shift,std::array<double,64>* values) {
      std::array<double,64> intensities; double average=0;
      for(int i=0;i<64;++i) {
        if(!Sample(target,warped[i]+shift,intensities[i])) return std::numeric_limits<double>::infinity();
        average+=intensities[i];
      }
      average/=64;
      double sum=0;
      for(int i=0;i<64;++i) {
        const double error=templateValues[i]-(intensities[i]-average);
        sum+=error*error; if(values) (*values)[i]=error;
      }
      return sum/64;
    };
    Vector<2> best=Zeros; double bestCost=std::numeric_limits<double>::infinity();
    const int radius=impl_->options.patchSearchRadius;
    for(int y=-radius;y<=radius;++y) for(int x=-radius;x<=radius;++x) {
      if(x*x+y*y>radius*radius) continue;
      const Vector<2> shift=makeVector(double(x),double(y));
      const double cost=residual(shift,nullptr);
      if(cost<bestCost) {best=shift;bestCost=cost;}
    }
    if(!std::isfinite(bestCost)) continue;
    // Zero-mean, warped 2-DOF subpixel alignment. The homography supplies shape;
    // the local translation measurement then constrains the next SL3 update.
    for(int iteration=0;iteration<10;++iteration) {
      std::array<double,64> errors;
      if(!std::isfinite(residual(best,&errors))) break;
      std::array<Vector<2>,64> gradients; Vector<2> meanGradient=Zeros;
      bool gradientsValid=true;
      for(int i=0;i<64;++i) {
        double left,right,up,down;
        const Vector<2> p=warped[i]+best;
        if(!Sample(target,p+makeVector(-1.,0.),left) ||
           !Sample(target,p+makeVector(1.,0.),right) ||
           !Sample(target,p+makeVector(0.,-1.),up) ||
           !Sample(target,p+makeVector(0.,1.),down)) {gradientsValid=false;break;}
        gradients[i]=.5*(makeVector((right-left)*.5,(down-up)*.5)+templateGradients[i]);
        meanGradient+=gradients[i];
      }
      if(!gradientsValid) break;
      meanGradient/=64;
      Matrix<2> information=Zeros; Vector<2> rhs=Zeros;
      for(int i=0;i<64;++i) {
        gradients[i]-=meanGradient;
        information+=gradients[i].as_col()*gradients[i].as_row(); rhs+=gradients[i]*errors[i];
      }
      const double determinant=M2Det(information);
      if(!Finite(information) || !Finite(rhs) || determinant<1e-6) break;
      const Vector<2> update=M2Inverse(information)*rhs;
      if(!Finite(update) || norm(update)>2) break;
      const Vector<2> candidate=best+update;
      if(norm(candidate)>radius+1.5) break;
      const double cost=residual(candidate,nullptr);
      if(cost>bestCost) break;
      best=candidate;bestCost=cost;
      if(update*update<1e-4) break;
    }
    if(bestCost<=impl_->options.maximumPatchRms*impl_->options.maximumPatchRms)
      matches.push_back({feature,ToRoot(centerLevel+best,1)});
  }
  return matches;
  };
  auto matches=trackPatches(prediction);
  auto supportedFit=[&](Matrix<3>& seed,const std::vector<PixelPair>& pixels,int& inliers,double& cost) {
    std::vector<CalibratedMatch> calibrated;
    for(const auto& pair:pixels) {
      const Vector<2> ray=firstCamera.UnProject(pair.first);
      if(Finite(ray)) calibrated.push_back({pair.first,pair.second,ray});
    }
    inliers=0;cost=std::numeric_limits<double>::infinity();
    RefineHomography(seed,calibrated,currentCamera,impl_->options.maximumReprojectionError,inliers,cost);
  };
  // A low-contrast coarsest image can give an unobservable SBI rotation even
  // when fine-scale patches are well textured. Validate that prediction against
  // the retained H on actual patch consensus, before changing initializer state.
  // This is two bounded starting predictions, not a RANSAC/pair-window search.
  double predictionDifference=0;
  for(const auto& feature:impl_->features) {
    Vector<2> a,b;
    if(Warp(firstCamera,currentCamera,prediction,feature,a) &&
       Warp(firstCamera,currentCamera,impl_->H,feature,b))
      predictionDifference=std::max(predictionDifference,norm(a-b));
  }
  if(predictionDifference>1.0) {
    auto retainedMatches=trackPatches(impl_->H);
    Matrix<3> fittedPrediction=prediction,fittedRetained=impl_->H;
    int predictedInliers=0,retainedInliers=0;double predictedCost,retainedCost;
    supportedFit(fittedPrediction,matches,predictedInliers,predictedCost);
    supportedFit(fittedRetained,retainedMatches,retainedInliers,retainedCost);
    if(retainedInliers>predictedInliers ||
       (retainedInliers==predictedInliers && retainedCost<predictedCost)) {
      prediction=fittedRetained;matches=std::move(retainedMatches);
    } else prediction=fittedPrediction;
  }
  const Result result=impl_->ProcessMatches(matches,camera,timestamp,&prediction);
  if(result.inliers>=impl_->options.minimumInliers) impl_->lastSBI=std::move(currentSBI);
  return result;
}
}
