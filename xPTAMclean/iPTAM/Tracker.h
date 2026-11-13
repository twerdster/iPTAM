//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "RenderCompat.h"
#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include <sstream>
#include <vector>
#include <list>
#include "globals.h"
#include <random>
#ifdef IPTAM_MOBILE_PAPER
#include "MobileInitializer.h"
#include "RollingShutter.h"
#include <memory>
#endif


class TrackerData;
struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
#ifdef IPTAM_MOBILE_PAPER
  Vector<2> mobileCurrentPosition=Zeros;
#endif
};

class Tracker
{
public:
  Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm);
  
  ~Tracker();
  void SetFrameTimestamp(double timestampSeconds); // Monotonic capture time, call before TrackFrame.
  int TrackingQuality() const { return mMap.IsGood() ? static_cast<int>(mTrackingQuality) : -1; } // -1 initializing, 0 bad, 1 poor, 2 good
  int LastVisibleCount() const { return mnVisible; }
  int LastWarpRejectedCount() const { return mnWarpRejected; }
  int LastTemplateRejectedCount() const { return mnTemplateRejected; }
  int MeasurementsFound() const;
  int MeasurementsAttempted() const;
  int PoseInliers() const { return mnPoseInliers; } // Final-pose robust support, separate from raw patch matches.
  double InitialMedianDisplacement() const;
  size_t InitialTrailCount() const { return mlTrails.size(); }

#ifdef IPTAM_MOBILE_PAPER
  bool MobileBlurFallback() const { return mbMobileBlurFallback; }
  int MobileCoarseMatches() const { return mnMobileCoarseMatches; }
  const std::string& MobileInitializationStage() const { return msMobileInitializationStage; }
  int MobileRSObservations() const { return mnMobileRSObservations; }
  int MobileRSCorrections() const { return mnMobileRSCorrections; }
#endif
  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, uint hnd,bool bDraw); 

  inline SE3<> GetCurrentPose() { return mse3CamFromWorld;}
  
  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();
    void StartTracking();
	void StopTracking();
	
protected:
  KeyFrame mCurrentKF;            // The current working frame as a keyframe struct
  
  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  ATANCamera mCamera;             // Projection model
  Relocaliser mRelocaliser;       // Relocalisation module

  CVD::ImageRef mirSize;          // Image size of whole image
  
  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
  void RenderGrid();              // Draws the reference grid

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED, 
	TRAIL_TRACKING_STARTED, 
	TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame mFirstKF;              // First of the stereo pair
  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches
  
  // Methods for tracking the map once it has been made:
#ifdef IPTAM_MOBILE_PAPER
  void TrackMapMobile();
  int TrailTrackingAdvanceMobile();
  void UpdateMobileTemporalMeasurements();
  std::unique_ptr<IPTAMMobile::MobileInitializer> mpMobileInitializer;
  bool mbMobileBlurFallback=false;
  int mnMobileCoarseMatches=0, mnMobileRSObservations=0, mnMobileRSCorrections=0;
  std::string msMobileInitializationStage="not_started";
#endif
  void TrackMap();                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<TrackerData*> &vTD, 
		      int nRange, 
		      int nFineIts);  // Finds points in the image
  Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD, 
			   double dOverrideSigma = 0.0, 
			   bool bMarkOutliers = false); // Updates pose from found points.
  void UpdatePoseInlierCount(const std::vector<TrackerData*>& measurements);
  SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
  SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  Vector<6> mv6CameraVelocity;    // Motion model
  double mdVelocityMagnitude;     // Used to decide on coarse tracking 
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?
  
  std::mt19937 mRng{42}; // Fixed seed: reproducible replay and reset.
  bool mbNumericalFailure=false;
  int mnPoseInliers=0;
  int mnVisible=0, mnWarpRejected=0, mnTemplateRejected=0;
  double mdLastKeyFrameTimestamp = -1e30;
  bool mbTimestampSet = false;
  double mdLastFrameTimestamp = 0.0;
  double mdPreviousFrameInterval = 0.0;
  double mdFrameIntervalRatio = 1.0;
  bool mbDraw;                    // Should the tracker draw anything to OpenGL?
  
  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe
  
  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;
  
  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  Vector<6> mv6SBIRot;
  bool mbUseSBIInit;
  
  // User interaction for initial tracking:
  bool mbUserPressedSpacebar;
  std::ostringstream mMessageForUser;
	GLuint textureHandle;
	GLubyte *imageData;
	
  // GUI interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;

	
};

#endif






