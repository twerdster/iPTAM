// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage - A small and blurry representation of an image.
// used by the relocaliser.

#ifndef __SMALLBLURRYIMAGE_H
#define __SMALLBLURRYIMAGE_H
#include "image.h"
#include "byte.h"
#include "se2.h"
#include "se3.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include "globals.h"

class SmallBlurryImage
{
 public:
  SmallBlurryImage();
  SmallBlurryImage(KeyFrame &kf, double dBlur = 2.5);
  void MakeFromKF(KeyFrame &kf, double dBlur = 2.5);
  void MakeJacs();
  double ZMSSD(SmallBlurryImage &other);
  std::pair<SE2<>,double> IteratePosRelToTarget(SmallBlurryImage &other, int nIterations = 10);
  static SE3<> SE3fromSE2(SE2<> se2, ATANCamera camera);
  
protected:
  CVD::Image<CVD::byte> mimSmall;
  CVD::Image<float> mimTemplate;
  CVD::Image<Vector<2> > mimImageJacs;
  bool mbMadeJacs;
  static CVD::ImageRef mirSize;
};



#endif









