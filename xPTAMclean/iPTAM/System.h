// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
//#include "VideoSource.h"
//#include "GLWindow2.h"

#include "image.h"
#include "rgb.h"
#include "byte.h"
#include "globals.h"
#include <vector>

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
//class ARDriver;
//class MapViewer;

class System
{
public:
  System();
  void RunOneFrame(unsigned char *bwImage,uint hnd);
	bool Finished() {return mbDone;};
	
   void SendTrackerStartSig();
    void SendTrackerKillSig();
	std::vector<float> getCurrentPose();
	
private:
  CVD::Image<CVD::byte> mimFrameBW;
  
  Map *mpMap; 
  MapMaker *mpMapMaker; 
  Tracker *mpTracker; 
  ATANCamera *mpCamera;
  //ARDriver *mpARDriver;
 // MapViewer *mpMapViewer;
  
  bool mbDone;

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

};



#endif
