#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
namespace IPTAMMobile {
struct DepthInterval {
  double lower,upper;
  bool valid,repaired;
};
inline DepthInterval EpipolarDepthInterval(double mean,double sigma,double baseline,bool repair) {
  DepthInterval result{std::max(baseline,mean-sigma),std::min(40*baseline,mean+sigma),false,false};
  const bool finite=std::isfinite(mean)&&std::isfinite(sigma)&&std::isfinite(baseline)&&mean>0&&sigma>=0&&baseline>0;
  if(finite && repair && result.upper<=result.lower && mean+sigma>result.lower) {
    // The old fixed baseline-relative upper cap can fall below the complete
    // estimated scene distribution. Keep the image-derived interval instead;
    // image corner/search limits and both-camera cheirality remain unchanged.
    result.upper=mean+sigma;result.repaired=true;
  }
  result.valid=finite&&std::isfinite(result.lower)&&std::isfinite(result.upper)&&result.lower>0&&result.upper>result.lower;
  return result;
}
inline bool UpdateInverseDepth(double& inverseDepth,double& variance,
                               double jx,double jy,double residualX,double residualY,double pixelVariance) {
  const double information=(jx*jx+jy*jy)/pixelVariance;
  if(!std::isfinite(inverseDepth)||inverseDepth<=0||!std::isfinite(information)||information<1e-9||
     !std::isfinite(residualX)||!std::isfinite(residualY)||!std::isfinite(pixelVariance)||pixelVariance<=0||
     std::isnan(variance)||variance<=0) return false;
  const double priorInformation=std::isfinite(variance)?1/variance:0.;
  const double posteriorVariance=1/(priorInformation+information);
  const double posterior=inverseDepth+posteriorVariance*(jx*residualX+jy*residualY)/pixelVariance;
  if(!std::isfinite(posterior)||posterior<=0||!std::isfinite(posteriorVariance)||posteriorVariance<=0)return false;
  inverseDepth=posterior;variance=posteriorVariance;return true;
}
} // namespace IPTAMMobile
