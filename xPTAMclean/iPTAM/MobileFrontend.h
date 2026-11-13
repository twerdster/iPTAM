#ifndef IPTAM_MOBILE_FRONTEND_H
#define IPTAM_MOBILE_FRONTEND_H
#ifdef IPTAM_MOBILE_PAPER
#include "KeyFrame.h"
namespace IPTAMMobile {
// ISMAR 2009 §3.1: exhaustive L2 Shi–Tomasi followed by local L1/L0 ascent.
void DetectMultiscaleCorners(KeyFrame& frame, double minimumScore);
bool RotationNeedsBlurFallback(const TooN::Vector<6>& rotation);
}
#endif
#endif
