// Copyright 2008 Isis Innovation Limited
#include "RenderCompat.h"
#include <cstdio>
#include <unistd.h>
#include <stdexcept>



#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "TrackerData.h"
#include "globals.h"
#ifdef IPTAM_MOBILE_PAPER
#include "MobileConfig.h"
#include "MobileFrontend.h"
#endif

#include "utility.h"
//#include "gl_helpers.h"
#include "fast_corner.h"
#include "vision.h"
#include "wls.h"

#include <fstream>
#include <fcntl.h>

using namespace CVD;
using namespace std;
namespace { constexpr double kFinalPoseSigmaSquared = 16.0; }

//The following is under the assumption that we are dealing with 
//a screen that is 640 in width and 480 in height
//and that the origin is in top left corner
GLfloat spriteTexcoords[] = {
	0,0,
	0,1,
	1,0,
	1,1};

GLfloat spriteVertices[] =  {
	0,0,   
	0,480,   
	640,0, 
	640,480};

GLuint createNPOTTexture(GLuint width,GLuint height) 
{
	GLuint handle;
	glGenTextures(1, &handle);
	glBindTexture(GL_TEXTURE_2D, handle);
	glTexParameterf(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0, GL_LUMINANCE, 
				 GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, 0);
	
	return handle;
}

// The constructor mostly sets up interal reference variables
// to the other classes..
Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm) : 
mMap(m),
mMapMaker(mm),
mCamera(c),
mRelocaliser(mMap, mCamera),
mirSize(irVideoSize), textureHandle(0)
{
	mCurrentKF.bFixed = false;
	//GUI.RegisterCommand("Reset", GUICommandCallBack, this);
	//GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
	//GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
	TrackerData::irImageSize = mirSize;
	
	mpSBILastFrame = NULL;
	mpSBIThisFrame = NULL;
	
	// Most of the initialisation is done in Reset()
	Reset();
}

Tracker::~Tracker() {
    delete mpSBILastFrame;
    delete mpSBIThisFrame;
}

double Tracker::InitialMedianDisplacement() const {
    std::vector<double> movement;
    for(const auto& trail:mlTrails) movement.push_back(std::sqrt(double((trail.irCurrentPos-trail.irInitialPos).mag_squared())));
    if(movement.empty()) return 0;
    std::nth_element(movement.begin(),movement.begin()+movement.size()/2,movement.end());
    return movement[movement.size()/2];
}

int Tracker::MeasurementsFound() const {
    int n=0; for(int i=0;i<LEVELS;++i) n += manMeasFound[i]; return n;
}
int Tracker::MeasurementsAttempted() const {
    int n=0; for(int i=0;i<LEVELS;++i) n += manMeasAttempted[i]; return n;
}

void Tracker::SetFrameTimestamp(double timestamp) {
    if(!std::isfinite(timestamp)) throw std::invalid_argument("Frame timestamp must be finite");
    mdFrameIntervalRatio=1.0;
    if(mbTimestampSet) {
        double dt=timestamp-mdLastFrameTimestamp;
        if(dt<=0) throw std::invalid_argument("Frame timestamps must increase");
        if(dt>0.5) { mv6CameraVelocity=Zeros; mdMSDScaledVelocityMagnitude=0; }
        else if(mdPreviousFrameInterval>0) mdFrameIntervalRatio=std::max(0.25,std::min(4.0,dt/mdPreviousFrameInterval));
        mdPreviousFrameInterval=dt;
    }
    mdLastFrameTimestamp=timestamp; mbTimestampSet=true;
}

void Tracker::StartTracking() 
{
	mbUserPressedSpacebar=true;
}

void Tracker::StopTracking()
{
	mbUserPressedSpacebar=false;
	Reset();
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset()
{
	std::fprintf(stderr, "Tracker resetting...\n");
	mRng.seed(42);
#ifdef IPTAM_MOBILE_PAPER
  if(!mpMobileInitializer) mpMobileInitializer.reset(new IPTAMMobile::MobileInitializer);
  mpMobileInitializer->Reset(); mbMobileBlurFallback=false; mnMobileCoarseMatches=0;
  mnMobileRSObservations=mnMobileRSCorrections=0; msMobileInitializationStage="not_started";
#endif
	mbNumericalFailure = false;
	mnPoseInliers = 0;
	mbDidCoarse = false;
	mbUserPressedSpacebar = false;
	mTrackingQuality = GOOD;
	mnLostFrames = 0;
	mdMSDScaledVelocityMagnitude = 0;
	mCurrentKF.dSceneDepthMean = 1.0;
	mCurrentKF.dSceneDepthSigma = 1.0;
	mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
	mlTrails.clear();
	mCamera.SetImageSize(mirSize);
	mCurrentKF.mMeasurements.clear();
	mnLastKeyFrameDropped = -20;
	mnFrame=0;
    mbTimestampSet=false; mdLastFrameTimestamp=0; mdPreviousFrameInterval=0; mdFrameIntervalRatio=1; mdLastKeyFrameTimestamp=-1e30;
	mv6CameraVelocity = Zeros;
    mse3CamFromWorld = SE3<>();
    for(int i=0;i<LEVELS;++i) manMeasAttempted[i] = manMeasFound[i] = 0;
	mbJustRecoveredSoUseCoarse = false;
	
	// Tell the MapMaker to reset itself.. 
	// this may take some time, since the mapmaker thread may have to wait
	// for an abort-check during calculation, so sleep while waiting.
	// MapMaker will also clear the map.
	mMapMaker.RequestReset();
	while(!mMapMaker.ResetDone())
		usleep(10);
	
	std::fprintf(stderr, "Tracker reset.\n");
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
//void Tracker::TrackFrame(Image<byte> &imFrame, bool bDraw)

GLfloat transpose[]={
	0,1,0,0,
	1,0,0,0,
	0,0,1,0,
	0,0,0,1
};

void Tracker::TrackFrame(Image<byte> &imFrame, uint hnd,bool bDraw)
{
#ifdef IPTAM_MOBILE_PAPER
  mbMobileBlurFallback=false; mnMobileCoarseMatches=0; mv6SBIRot=Zeros;
  mnMobileRSObservations=mnMobileRSCorrections=0;
#endif
	mnPoseInliers = 0; // Recovery may fail without invoking TrackMap this frame.
	if(mirSize.x < 64 || mirSize.y < 64 || imFrame.size() != mirSize) throw std::invalid_argument("Frame dimensions differ from calibrated tracker dimensions");
#ifdef IPTAM_HEADLESS
    bDraw = false;
#endif
    mbDraw = bDraw;
	mMessageForUser.str("");   

	bool firstRun = mbDraw && textureHandle == 0;
	
	if (firstRun) {
		glEnable(GL_TEXTURE_2D);    
		std::fprintf(stderr, "x:%d y:%d\n",mirSize.x,mirSize.y);
		textureHandle=createNPOTTexture(mirSize.x,mirSize.y); 
		glDisable(GL_TEXTURE_2D); 

	}
	
	
	// Wipe the user message clean
	
	// Take the input video image, and convert it into the tracker's keyframe struct
	// This does things like generate the image pyramid and find FAST corners
	mCurrentKF.mMeasurements.clear();
#ifdef IPTAM_MOBILE_PAPER
  mCurrentKF.MakeKeyFrame_Lite(imFrame,false);
#else
	mCurrentKF.MakeKeyFrame_Lite(imFrame);
#endif
#ifdef IPTAM_MOBILE_PAPER
  mCurrentKF.captureTimestamp=mdLastFrameTimestamp;
  mCurrentKF.hasCaptureTimestamp=mbTimestampSet;
  mCurrentKF.mobileBlurFallback=false;
#endif
	
	// Update the small images for the rotation estimator
	static double gvdSBIBlur = TrackerRotationEstimatorBlur;
	static int gvnUseSBI = TrackerUseRotationEstimator;
	mbUseSBIInit = gvnUseSBI;
	if(!mpSBIThisFrame)
	{
		mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, gvdSBIBlur);
		mpSBILastFrame = new SmallBlurryImage(mCurrentKF, gvdSBIBlur);
	}
	else
	{
		delete  mpSBILastFrame;
		mpSBILastFrame = mpSBIThisFrame;
		mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, gvdSBIBlur);
	}
	
	// From now on we only use the keyframe struct!
	mnFrame++;
	
    
	if (mbDraw)
    {
		glBindTexture(GL_TEXTURE_2D, textureHandle);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_LUMINANCE, GL_UNSIGNED_BYTE, imFrame.data());
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrthof(480, 0, 640,0, 0, 1);
		//glOrthof(0, 480, 0, 640, 0, 1);
		
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glLoadMatrixf(transpose);
	
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glEnable(GL_TEXTURE_2D);
		
		glVertexPointer(2, GL_FLOAT, 0, spriteVertices);
		glTexCoordPointer(2, GL_FLOAT, 0, spriteTexcoords);	
		glBindTexture(GL_TEXTURE_2D, textureHandle);
		
		glColor4f(1, 1, 1, 1);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		
		
		//----
		glDisable(GL_TEXTURE_2D);
		
		//glDrawPixels(mCurrentKF.aLevels[0].im);
		if(0)//TrackerDrawFASTCorners)
		{
			GLshort *crnrs=(GLshort*)malloc(sizeof(GLushort)*2*mCurrentKF.aLevels[0].vCorners.size());
			for (int i=0; i<mCurrentKF.aLevels[0].vCorners.size(); i++) {
				crnrs[2*i]=mCurrentKF.aLevels[0].vCorners[i].x;
				crnrs[2*i+1]=mCurrentKF.aLevels[0].vCorners[i].y;
			}
			
			glColor4f(1,0,1,1);  glPointSize(1); 		
			glVertexPointer(2,GL_SHORT,0,crnrs);
			glDrawArrays(GL_POINTS, 0, mCurrentKF.aLevels[0].vCorners.size());
			free(crnrs);
		}
		
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		
		//----
    }
	
	// Decide what to do - if there is a map, try to track the map ...
	if(mMap.IsGood())
	{
		if(mnLostFrames < 3)  // .. but only if we're not lost!
		{
			if(mbUseSBIInit)
				CalcSBIRotation();
			ApplyMotionModel();       // 
			TrackMap();               //  These three lines do the main tracking work.
			UpdateMotionModel();      // 
			
			AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.
#ifdef IPTAM_MOBILE_PAPER
      UpdateMobileTemporalMeasurements();
      if(mTrackingQuality==GOOD && !mbMobileBlurFallback && IPTAMMobile::Config().partialPoints)
        mMapMaker.ObservePartialPoints(mCurrentKF);
#endif
			
			{ // Provide some feedback for the user:
				mMessageForUser << "Tracking Map, quality ";
				if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
				if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
				if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
				mMessageForUser << " Found:";
				for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
				//	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
				mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
			}
			
			// Heuristics to check if a key-frame should be added to the map:
			if(mTrackingQuality == GOOD &&
#ifdef IPTAM_MOBILE_PAPER
         !mbMobileBlurFallback &&
#endif
			   mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
			   (mbTimestampSet ? (mdLastFrameTimestamp-mdLastKeyFrameTimestamp > 0.3 && mnFrame-mnLastKeyFrameDropped > 2) : mnFrame-mnLastKeyFrameDropped > 20) &&
			   mMapMaker.QueueSize() < 3)
			{
				mMessageForUser << " Adding key-frame.";
				AddNewKeyFrame();
			};
		}
		else  // what if there is a map, but tracking has been lost?
		{
			mMessageForUser << "** Attempting recovery **.";
			if(AttemptRecovery())
			{
				TrackMap();
				AssessTrackingQuality();
#ifdef IPTAM_MOBILE_PAPER
        UpdateMobileTemporalMeasurements();
#endif
			}
		}
		if(mbDraw)
			RenderGrid();
	} 
	else // If there is no map, try to make one.
		TrackForInitialMap(); 
	
	mMapMaker.ProcessPending();
	// GUI interface
	while(!mvQueuedCommands.empty())
	{
		GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
		mvQueuedCommands.erase(mvQueuedCommands.begin());
	}
};

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit 
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
	bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
	if(!bRelocGood)
		return false;
	
	SE3<> se3Best = mRelocaliser.BestPose();
	mse3CamFromWorld = mse3StartPos = se3Best;
	mv6CameraVelocity = Zeros;

	mbJustRecoveredSoUseCoarse = true;
	return true;
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
void Tracker::RenderGrid()
{
	// The colour of the ref grid shows if the coarse stage of tracking was used
	// (it's turned off when the camera is sitting still to reduce jitter.)
	
	
	if(mbDidCoarse)
		glColor4f(.0, 0.5, 1.0, 0.6);
	else
		glColor4f(0,0,1,0.6);
	
	// The grid is projected manually, i.e. GL receives projected 2D coords to draw.
	int nHalfCells = 8;
	int nTot = nHalfCells * 2 + 1;
	Image<Vector<2,GLfloat> >  imVertices(ImageRef(nTot,nTot));
	for(int i=0; i<nTot; i++)
		for(int j=0; j<nTot; j++)
		{
			Vector<3> v3;
			v3[0] = (i - nHalfCells) * 0.1;
			v3[1] = (j - nHalfCells) * 0.1;
			v3[2] = 0.0;
			Vector<3> v3Cam = mse3CamFromWorld * v3;
			
			if(v3Cam[2] < 0.001)
				v3Cam[2] = 0.001;
			imVertices[i][j] = mCamera.Project(project(v3Cam));
		}
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(2);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	for(int i=0; i<nTot; i++)
    {
		glVertexPointer(2, GL_FLOAT, 0, &imVertices[i][0]);
		glDrawArrays(GL_LINE_STRIP, 0, nTot);
		
		glVertexPointer(2, GL_FLOAT,sizeof(GLfloat)*2*nTot, &imVertices[0][i]);
		glDrawArrays(GL_LINE_STRIP, 0, nTot);
	};
	glDisableClientState(GL_VERTEX_ARRAY);
	glLineWidth(1);
	glColor4f(1,0,0,1);
}

// GUI interface. Stuff commands onto the back of a queue so the tracker handles
// them in its own thread at the end of each frame. Note the charming lack of
// any thread safety (no lock on mvQueuedCommands).
void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
	Command c;
	c.sCommand = sCommand;
	c.sParams = sParams;
	((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}

// This is called in the tracker's own thread.
void Tracker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
	if(sCommand=="Reset")
    {
		Reset();
		return;
    }
	
	// KeyPress commands are issued by GLWindow
	if(sCommand=="KeyPress")
    {
		if(sParams == "Space")
		{
			mbUserPressedSpacebar = true;
		}
		else if(sParams == "r")
		{
			Reset();
		}
		else if(sParams == "q" || sParams == "Escape")
		{
			//GUI.ParseLine("quit");
		}
		return;
    }
	if((sCommand=="PokeTracker"))
    {
		mbUserPressedSpacebar = true;
		return;
    }
    
	
	cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
	exit(1);
}; 

// Routine for establishing the initial map. This requires two spacebar presses from the user
// to define the first two key-frames. Salient points are tracked between the two keyframes
// using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
// break it.) The salient points are stored in a list of `Trail' data structures.
// What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
void Tracker::TrackForInitialMap()
{
#ifdef IPTAM_MOBILE_PAPER
  if(IPTAMMobile::Config().automaticInitialization) {
    // This path is enabled explicitly; fixed first/second replay remains unchanged.
    double time=mbTimestampSet ? mdLastFrameTimestamp : double(mnFrame);
    if(mnInitialStage==TRAIL_TRACKING_NOT_STARTED) {
      if(!mbUserPressedSpacebar) { msMobileInitializationStage="waiting_for_first_frame_request"; return; }
      mbUserPressedSpacebar=false;
      if(mpMobileInitializer->Begin(mCurrentKF,mCamera,time)) {
        mnInitialStage=TRAIL_TRACKING_STARTED; msMobileInitializationStage="tracking_planar_view";
      } else msMobileInitializationStage="insufficient_multiscale_features";
    } else if(mnInitialStage==TRAIL_TRACKING_STARTED) {
      mbUserPressedSpacebar=false;
      auto result=mpMobileInitializer->Process(mCurrentKF,mCamera,time);
      msMobileInitializationStage=result.stage+": "+result.reason;
      if(result.ready) {
        KeyFrame first=mpMobileInitializer->FirstKeyFrame();
        if(mMapMaker.InitFromStereo(first,mCurrentKF,result.matches,mse3CamFromWorld,&result.secondFromFirst)) {
          mnInitialStage=TRAIL_TRACKING_COMPLETE; msMobileInitializationStage="initialized";
        } else msMobileInitializationStage="mapper_rejected_selected_hypothesis";
      }
    }
    mMessageForUser << "Mobile auto initialization: " << msMobileInitializationStage;
    return;
  }
#endif
	// MiniPatch tracking threshhold.
	static int gvnMaxSSD = TrackerMiniPatchMaxSSD;
	MiniPatch::mnMaxSSD = gvnMaxSSD;
	
	// What stage of initial tracking are we at?
	if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED) 
    {
		if(mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
		{
			mbUserPressedSpacebar = false;
			TrailTracking_Start();
			mnInitialStage = TRAIL_TRACKING_STARTED;
		}
		else
			mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map." << endl;
		return;
    };
	
	if(mnInitialStage == TRAIL_TRACKING_STARTED)
    {
		int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
		if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
		{
			Reset();
			return;
		}
		
		// If the user pressed spacebar here, use trails to run stereo and make the intial map..
		if(mbUserPressedSpacebar)
		{
			mbUserPressedSpacebar = false;
			vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
			for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
				vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos,
															i->irCurrentPos));
			if(mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld))
                mnInitialStage = TRAIL_TRACKING_COMPLETE;
            else {
                mnInitialStage = TRAIL_TRACKING_STARTED;
                mMessageForUser << "Initialization failed. Continue slow sideways motion and press initialize again; first frame retained.";
            }
		}
		else
			mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init." << endl;
    }
}

// The current frame is to be the first keyframe!
void Tracker::TrailTracking_Start()
{
	mCurrentKF.MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
	mFirstKF = mCurrentKF; 
	vector<pair<double,ImageRef> > vCornersAndSTScores;
	for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCandidates.size(); i++)  // Copy candidates into a trivially sortable vector
    {                                                                     // so that we can choose the image corners with max ST score
		Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
		if(!mCurrentKF.aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
			continue;
		vCornersAndSTScores.push_back(pair<double,ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
    };
	sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score
	int nToAdd = MaxInitialTrails;
	for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++)
    {
		if(!mCurrentKF.aLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize))
			continue;
		Trail t;
		t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
		t.irInitialPos = vCornersAndSTScores[i].second;
		t.irCurrentPos = t.irInitialPos;
#ifdef IPTAM_MOBILE_PAPER
    t.mobileCurrentPosition=vec(t.irCurrentPos);
#endif
		mlTrails.push_back(t);
		nToAdd--;
    }
	mPreviousFrameKF = mFirstKF;  // Always store the previous frame so married-matching can work.
}

// Steady-state trail tracking: Advance from the previous frame, remove duds.
int Tracker::TrailTracking_Advance()
{
#ifdef IPTAM_MOBILE_PAPER
  if(IPTAMMobile::Config().coarseTracking) return TrailTrackingAdvanceMobile();
#endif
	GLfloat tempTrailVertices[4];
	GLfloat tempTrailColours[8];
	
	int nGoodTrails = 0;
	if(mbDraw)
    {
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glLineWidth(2);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		//glBegin(GL_LINES);
    }
	
	MiniPatch BackwardsPatch;
	Level &lCurrentFrame = mCurrentKF.aLevels[0];
	Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];
	// Trail count is exposed in status; avoid per-frame logging.
	
	for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
		list<Trail>::iterator next = i; next++;
		
		Trail &trail = *i;
		ImageRef irStart = trail.irCurrentPos;
		ImageRef irEnd = irStart;
		bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
		if(bFound)
		{
			// Also find backwards in a married-matches check
			BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
			ImageRef irBackWardsFound = irEnd;
			bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
			if((irBackWardsFound - irStart).mag_squared() > 2)
				bFound = false;
			
			trail.irCurrentPos = irEnd;
			if(bFound) nGoodTrails++;
		}
		
		if(mbDraw)
		{
			if(!bFound)
				//glColor3f(0,1,1); // Failed trails flash purple before dying.
			{tempTrailColours[0]=0;tempTrailColours[1]=1.0; tempTrailColours[2]=1.0;tempTrailColours[3]=1.0;}
			else
				//glColor3f(1,1,0);
			{tempTrailColours[0]=1.0;tempTrailColours[1]=1.0; tempTrailColours[2]=0;tempTrailColours[3]=1.0;}
			
			//glVertex(trail.irInitialPos);
			tempTrailVertices[0]=trail.irInitialPos.x;
			tempTrailVertices[1]=trail.irInitialPos.y;
			
			if(bFound) 
				//glColor3f(1,0,0);
			{tempTrailColours[4]=1.0;tempTrailColours[5]=0; tempTrailColours[6]=0;tempTrailColours[7]=1.0;}
			else 
			{
				tempTrailColours[4]=tempTrailColours[0];
				tempTrailColours[5]=tempTrailColours[1];
				tempTrailColours[6]=tempTrailColours[2];
				tempTrailColours[7]=tempTrailColours[3];
			}
			//glVertex(trail.irCurrentPos);
			tempTrailVertices[2]=trail.irCurrentPos.x;
			tempTrailVertices[3]=trail.irCurrentPos.y;
			
			glVertexPointer(2,GL_FLOAT,0,&tempTrailVertices);
			glColorPointer(4, GL_FLOAT, 0, &tempTrailColours);
			glDrawArrays(GL_LINE_STRIP, 0, 2);
		}
		if(!bFound) // Erase from list of trails if not found this frame.
		{
			mlTrails.erase(i);
		}
		i = next;
    }
	mPreviousFrameKF = mCurrentKF;
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	return nGoodTrails;
}

// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void Tracker::TrackMap()
{
#ifdef IPTAM_MOBILE_PAPER
  if(IPTAMMobile::Config().coarseTracking) { TrackMapMobile(); return; }
#endif
  constexpr int nTrackingLevels=4; // Historical frontend has four active levels.
	// A rejected pose solve invalidates this frame, not later observations.
	mbNumericalFailure = false;
	mnPoseInliers = 0;
	if (mbUserPressedSpacebar) {
		StopTracking();
        return;
	}
	// Some accounting which will be used for tracking quality assessment:
	for(int i=0; i<nTrackingLevels; i++)
		manMeasAttempted[i] = manMeasFound[i] = 0;
	
	mnVisible=mnWarpRejected=mnTemplateRejected=0;
	// The Potentially-Visible-Set (PVS) is split into pyramid levels.
	vector<TrackerData*> avPVS[nTrackingLevels];
	for(int i=0; i<nTrackingLevels; i++)
		avPVS[i].reserve(500);
	
	// For all points in the map..
	for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
		MapPoint &p= *(mMap.vpPoints[i]);
        if(p.bBad) continue;
		// Ensure that this map point has an associated TrackerData struct.
		if(!p.pTData) p.pTData = new TrackerData(&p);   
		TrackerData &TData = *p.pTData;
		
		// Project according to current view, and if it's not in the image, skip.
		TData.Project(mse3CamFromWorld, mCamera); 
		if(!TData.bInImage)
			continue;   
		
		++mnVisible;
		// Calculate camera projection derivatives of this point.
		TData.GetDerivsUnsafe(mCamera);
		
		// And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
		TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
		if(TData.nSearchLevel == -1) { ++mnWarpRejected;
			continue; }   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
		
		// Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
		TData.bSearched = false;
		TData.bFound = false;
		avPVS[TData.nSearchLevel].push_back(&TData);
    };
	
	// Next: A large degree of faffing about and deciding which points are going to be measured!
	// First, randomly shuffle the individual levels of the PVS.
	for(int i=0; i<nTrackingLevels; i++)
		std::shuffle(avPVS[i].begin(), avPVS[i].end(), mRng);
	
	// The next two data structs contain the list of points which will next 
	// be searched for in the image, and then used in pose update.
	vector<TrackerData*> vNextToSearch;
	vector<TrackerData*> vIterationSet;
	
	// Tunable parameters to do with the coarse tracking stage:
	static unsigned int gvnCoarseMin=TrackerCoarseMin;   // Min number of large-scale features for coarse stage
	static unsigned int gvnCoarseMax=TrackerCoarseMax;   // Max number of large-scale features for coarse stage
	static unsigned int gvnCoarseRange=TrackerCoarseRange;       // Pixel search radius for coarse features
	static int gvnCoarseSubPixIts=TrackerCoarseSubPixIts; // Max sub-pixel iterations for coarse features
	static int gvnCoarseDisabled=TrackerDisableCoarse;    // Set this to 1 to disable coarse stage (except after recovery)
	static double gvdCoarseMinVel=TrackerCoarseMinVelocity;  // Speed above which coarse stage is used.
	
	unsigned int nCoarseMax = gvnCoarseMax;
	unsigned int nCoarseRange = gvnCoarseRange;
	
	mbDidCoarse = false;
	
	// Set of heuristics to check if we should do a coarse tracking stage.
	bool bTryCoarse = true;
	if(gvnCoarseDisabled || 
	   mdMSDScaledVelocityMagnitude < gvdCoarseMinVel  ||
	   nCoarseMax == 0)
		bTryCoarse = false;
	if(mbJustRecoveredSoUseCoarse)
    {
		bTryCoarse = true;
		nCoarseMax *=2;
		nCoarseRange *=2;
		mbJustRecoveredSoUseCoarse = false;
    };
	
	// If we do want to do a coarse stage, also check that there's enough high-level 
	// PV map points. We use the lowest-res two pyramid levels (nTrackingLevels-1 and nTrackingLevels-2),
	// with preference to nTrackingLevels-1.
	if(bTryCoarse && avPVS[nTrackingLevels-1].size() + avPVS[nTrackingLevels-2].size() > gvnCoarseMin )
    {
		// Now, fill the vNextToSearch struct with an appropriate number of 
		// TrackerDatas corresponding to coarse map points! This depends on how many
		// there are in different pyramid levels compared to CoarseMin and CoarseMax.
		
		if(avPVS[nTrackingLevels-1].size() <= nCoarseMax)
		{ // Fewer than CoarseMax in nTrackingLevels-1? then take all of them, and remove them from the PVS list.
			vNextToSearch = avPVS[nTrackingLevels-1];
			avPVS[nTrackingLevels-1].clear();
		}
		else
		{ // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
			for(unsigned int i=0; i<nCoarseMax; i++)
				vNextToSearch.push_back(avPVS[nTrackingLevels-1][i]);
			avPVS[nTrackingLevels-1].erase(avPVS[nTrackingLevels-1].begin(), avPVS[nTrackingLevels-1].begin() + nCoarseMax);
		}
		
		// If didn't source enough from nTrackingLevels-1, get some from nTrackingLevels-2... same as above.
		if(vNextToSearch.size() < nCoarseMax)
		{
			unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
			if(avPVS[nTrackingLevels-2].size() <= nMoreCoarseNeeded)
			{
				vNextToSearch.insert(vNextToSearch.end(), avPVS[nTrackingLevels-2].begin(), avPVS[nTrackingLevels-2].end());
				avPVS[nTrackingLevels-2].clear();
			}
			else
			{
				for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
					vNextToSearch.push_back(avPVS[nTrackingLevels-2][i]);
				avPVS[nTrackingLevels-2].erase(avPVS[nTrackingLevels-2].begin(), avPVS[nTrackingLevels-2].begin() + nMoreCoarseNeeded);
			}
		}
		// Now go and attempt to find these points in the image!
		unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, gvnCoarseSubPixIts);
		vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
		if(nFound >= gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
		{
			mbDidCoarse = true;
			for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
			{
				if(iter != 0)
				{ // Re-project the points on all but the first iteration.
					for(unsigned int i=0; i<vIterationSet.size(); i++)
						if(vIterationSet[i]->bFound)  
							vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
				}
				for(unsigned int i=0; i<vIterationSet.size(); i++)
					if(vIterationSet[i]->bFound)
						vIterationSet[i]->CalcJacobian();
				double dOverrideSigma = 0.0;
				// Hack: force the MEstimator to be pretty brutal 
				// with outliers beyond the fifth iteration.
				if(iter > 5)
					dOverrideSigma = 1.0;
				
				// Calculate and apply the pose update...
				Vector<6> v6Update = 
				CalcPoseUpdate(vIterationSet, dOverrideSigma);
				mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
			};
		}
    };
	
	// So, at this stage, we may or may not have done a coarse tracking stage.
	// Now do the fine tracking stage. This needs many more points!
	
	int nFineRange = 10;  // Pixel search range for the fine stage. 
	if(mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
		nFineRange = 5;
	
	// What patches shall we use this time? The high-level ones are quite important,
	// so do all of these, with sub-pixel refinement.
	{
		int l = nTrackingLevels - 1;
		for(unsigned int i=0; i<avPVS[l].size(); i++)
			avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
		SearchForPoints(avPVS[l], nFineRange, 8);
		for(unsigned int i=0; i<avPVS[l].size(); i++)
			vIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
	};
	
	// All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
	vNextToSearch.clear();
	for(int l=nTrackingLevels - 2; l>=0; l--)
		for(unsigned int i=0; i<avPVS[l].size(); i++)
			vNextToSearch.push_back(avPVS[l][i]);
	
	// But we haven't got CPU to track _all_ patches in the map - arbitrarily limit 
	// ourselves to 1000, and choose these randomly.
	static int gvnMaxPatchesPerFrame=TrackerMaxPatchesPerFrame;
	int nFinePatchesToUse = gvnMaxPatchesPerFrame - vIterationSet.size();
	if(nFinePatchesToUse < 0)
		nFinePatchesToUse = 0;
	if((int) vNextToSearch.size() > nFinePatchesToUse)
    {
		std::shuffle(vNextToSearch.begin(), vNextToSearch.end(), mRng);
		vNextToSearch.resize(nFinePatchesToUse); // Chop!
    };
	
	// If we did a coarse tracking stage: re-project and find derivs of fine points
	if(mbDidCoarse)
		for(unsigned int i=0; i<vNextToSearch.size(); i++)
			vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
	
	// Find fine points in image:
	SearchForPoints(vNextToSearch, nFineRange, 0);
	// And attach them all to the end of the optimisation-set.
	for(unsigned int i=0; i<vNextToSearch.size(); i++)
		vIterationSet.push_back(vNextToSearch[i]);
	
	// Again, ten gauss-newton pose update iterations.
	Vector<6> v6LastUpdate;
	v6LastUpdate = Zeros;
	for(int iter = 0; iter<10; iter++)
    {
		bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
								  // reprojection at every iteration - it really isn't necessary!
		if(iter == 0 || iter == 4 || iter == 9)
			bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
		else                            // iterations is for M-Estimator convergence rather than 
			bNonLinearIteration = false;  // linearisation effects.
		
		if(iter != 0)   // Either way: first iteration doesn't need projection update.
        {
			if(bNonLinearIteration)
			{
				for(unsigned int i=0; i<vIterationSet.size(); i++)
					if(vIterationSet[i]->bFound)
						vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
			}
			else
			{
				for(unsigned int i=0; i<vIterationSet.size(); i++)
					if(vIterationSet[i]->bFound)
						vIterationSet[i]->LinearUpdate(v6LastUpdate);
			};
		}
		
		if(bNonLinearIteration)
			for(unsigned int i=0; i<vIterationSet.size(); i++)
				if(vIterationSet[i]->bFound)
					vIterationSet[i]->CalcJacobian();
		
		// Again, an M-Estimator hack beyond the fifth iteration.
		double dOverrideSigma = 0.0;
		if(iter > 5)
			dOverrideSigma = kFinalPoseSigmaSquared;
		
		// Calculate and update pose; also store update vector for linear iteration updates.
		Vector<6> v6Update = 
		CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
		mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
		v6LastUpdate = v6Update;
    };
	UpdatePoseInlierCount(vIterationSet);
	
	if(mbDraw)
    {
		
		glPointSize(6);
		glEnable(GL_BLEND);
		glEnable(GL_POINT_SMOOTH);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		
		Vector<2,GLfloat> aPoint;
		
		for(vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
			it!= vIterationSet.rend(); 
			it++)
		{
			if(! (*it)->bFound)
				continue;
			
			aPoint[0]=(GLfloat)(*it)->v2Image[0];
			aPoint[1]=(GLfloat)(*it)->v2Image[1];
			
			glVertexPointer(2,GL_FLOAT,0,&aPoint[0]);
		    glColorPointer(3, GL_FLOAT,0, &gavLevelColors[(*it)->nSearchLevel]);
			glDrawArrays(GL_POINTS, 0, 1);			
		}
		
		glDisable(GL_BLEND);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);
    }
	
	// Update the current keyframe with info on what was found in the frame.
	// Strictly speaking this is unnecessary to do every frame, it'll only be
	// needed if the KF gets added to MapMaker. Do it anyway.
	// Export pose to current keyframe:
	mCurrentKF.se3CfromW = mse3CamFromWorld;
	
	// Record successful measurements. Use the KeyFrame-Measurement struct for this.
	mCurrentKF.mMeasurements.clear();
	for(vector<TrackerData*>::iterator it = vIterationSet.begin();
		it!= vIterationSet.end(); 
		it++)
    {
		if(! (*it)->bFound)
			continue;
		Measurement m;
		m.v2RootPos = (*it)->v2Found;
		m.nLevel = (*it)->nSearchLevel;
		m.bSubPix = (*it)->bDidSubPix; 
		mCurrentKF.mMeasurements[& ((*it)->Point)] = m;
    }
	
	// Finally, find the mean scene depth from tracked features
	{
		double dSum = 0;
		double dSumSq = 0;
		int nNum = 0;
		for(vector<TrackerData*>::iterator it = vIterationSet.begin();
			it!= vIterationSet.end(); 
			it++)
			if((*it)->bFound)
			{
				double z = (*it)->v3Cam[2];
				dSum+= z;
				dSumSq+= z*z;
				nNum++;
			};
		if(nNum > 20)
		{
			mCurrentKF.dSceneDepthMean = dSum/nNum;
			mCurrentKF.dSceneDepthSigma = sqrt(std::max(0.0, (dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean)));
		}
	}
}

// Count support at the pose actually exported, rather than the last solve's
// linearization pose or its cached projection. Keep raw match counters,
// found flags and mapper measurements unchanged.
void Tracker::UpdatePoseInlierCount(const vector<TrackerData*>& measurements)
{
    mnPoseInliers = 0;
    ATANCamera camera = mCamera;
    for(const auto* data:measurements)
    {
        if(!data || !data->bFound || !std::isfinite(data->dSqrtInvNoise) || data->dSqrtInvNoise<=0)
            continue;
        const auto point = mse3CamFromWorld * data->Point.v3WorldPos;
        if(!std::isfinite(point*point) || point[2]<=0.001)
            continue;
        const auto projected = camera.Project(project(point));
        if(camera.Invalid() || !std::isfinite(projected*projected) ||
           projected[0]<0 || projected[1]<0 || projected[0]>=mirSize.x || projected[1]>=mirSize.y)
            continue;
        const auto error = data->dSqrtInvNoise * (data->v2Found-projected);
        const double squaredError = error*error;
        if(!std::isfinite(squaredError)) continue;
        double weight = Tukey::Weight(squaredError,kFinalPoseSigmaSquared);
        if(TrackerMEstimator=="Cauchy") weight=Cauchy::Weight(squaredError,kFinalPoseSigmaSquared);
        else if(TrackerMEstimator=="Huber") weight=Huber::Weight(squaredError,kFinalPoseSigmaSquared);
        if(std::isfinite(weight) && weight>0) ++mnPoseInliers;
    }
}

// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
	int nFound = 0;
	for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
    {
		// First, attempt a search at pixel locations which are FAST corners.
		// (PatchFinder::FindPatchCoarse)
		TrackerData &TD = *vTD[i];
		// A coarse pose update can move an already selected point out of view.
		// Reject it before template work, counters, or conversion to integer pixels.
		if(!TD.bInImage || !std::isfinite(TD.v2Image[0]) || !std::isfinite(TD.v2Image[1]))
		{
			TD.bInImage = TD.bPotentiallyVisible = TD.bFound = TD.bSearched = false;
			continue;
		}
		PatchFinder &Finder = TD.Finder;
		Finder.MakeTemplateCoarseCont(TD.Point);
		if(Finder.TemplateBad())
		{
            ++mnTemplateRejected;
			TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
			continue;
		}
		manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta
		
		bool bFound = 
		Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange);
		TD.bSearched = true;
		if(!bFound) 
		{
			TD.bFound = false;
			continue;
		}
		
		TD.bFound = true;
		TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
		
		nFound++;
		manMeasFound[Finder.GetLevel()]++;
		
		// Found the patch in coarse search - are Sub-pixel iterations wanted too?
		if(nSubPixIts > 0)
		{
			TD.bDidSubPix = true;
			Finder.MakeSubPixTemplate();
			bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF, nSubPixIts);
			if(!bSubPixConverges)
			{ // If subpix doesn't converge, the patch location is probably very dubious!
				TD.bFound = false;
				nFound--;
				manMeasFound[Finder.GetLevel()]--;
				continue;
			}
			TD.v2Found = Finder.GetSubPixPos();
		}
		else
		{
			TD.v2Found = Finder.GetCoarsePosAsVector();
			TD.bDidSubPix = false;
		}
    }
	return nFound;
};

//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the Tukey MEstimator.
Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
	// Which M-estimator are we using?
	int nEstimator = 0;
	static string gvsEstimator=TrackerMEstimator;
	if(gvsEstimator == "Tukey")
		nEstimator = 0;
	else if(gvsEstimator == "Cauchy")
		nEstimator = 1;
	else if(gvsEstimator == "Huber")
		nEstimator = 2;
	else 
    {
		cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
		nEstimator = 0;
		gvsEstimator = "Tukey";
    };
	
	// Find the covariance-scaled reprojection error for each measurement.
	// Also, store the square of these quantities for M-Estimator sigma squared estimation.
	vector<double> vdErrorSquared;
	for(unsigned int f=0; f<vTD.size(); f++)
    {
		TrackerData &TD = *vTD[f];
		if(!TD.bFound)
			continue;
		TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
		vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
    };
	
	// No valid measurements? Return null update.
	if(vdErrorSquared.size() == 0)
		return makeVector( 0,0,0,0,0,0);
	
	// What is the distribution of errors?
	double dSigmaSquared;
	if(dOverrideSigma > 0)
		dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
	else
    {
		if (nEstimator == 0)
			dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
		else if(nEstimator == 1)
			dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
		else 
			dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
    }
	
	// The TooN WLSCholesky class handles reweighted least squares.
	// It just needs errors and jacobians.
	WLS<6> wls;
	wls.add_prior(100.0); // Stabilising prior
	for(unsigned int f=0; f<vTD.size(); f++)
    {
		TrackerData &TD = *vTD[f];
		if(!TD.bFound)
			continue;
		Vector<2> &v2 = TD.v2Error_CovScaled;
		double dErrorSq = v2 * v2;
		double dWeight;
		
		if(nEstimator == 0)
			dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
		else if(nEstimator == 1)
			dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
		else 
			dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
		
		// Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
		if(dWeight == 0.0)
		{
			if(bMarkOutliers)
				TD.Point.nMEstimatorOutlierCount++;
			continue;
		}
		else
			if(bMarkOutliers)
				TD.Point.nMEstimatorInlierCount++;
		
		Matrix<2,6> &m26Jac = TD.m26Jacobian;
		wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
		wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
    }
	
	wls.compute();
	auto update=wls.get_mu();
    for(int i=0;i<6;++i) if(!std::isfinite(update[i])) { mbNumericalFailure=true; return makeVector(0.,0.,0.,0.,0.,0.); }
    return update;
}


// Just add the current velocity to the current pose.
// N.b. this doesn't actually use time in any way, i.e. it assumes
// a one-frame-per-second camera. Skipped frames etc
// are not handled properly here.
void Tracker::ApplyMotionModel()
{
	mse3StartPos = mse3CamFromWorld;
	Vector<6> v6Velocity = mdFrameIntervalRatio * mv6CameraVelocity;
	if(mbUseSBIInit)
    {
		v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
		v6Velocity[0] = 0.0;
		v6Velocity[1] = 0.0;
    }
	mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
};


// The motion model is entirely the tracker's, and is kept as a decaying
// constant velocity model.
void Tracker::UpdateMotionModel()
{
	SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
	Vector<6> v6Motion = SE3<>::ln(se3NewFromOld);
	Vector<6> v6OldVel = mdFrameIntervalRatio * mv6CameraVelocity;
	
	mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
	mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);
	
	// Also make an estimate of this which has been scaled by the mean scene depth.
	// This is used to decide if we should use a coarse tracking stage.
	// We can tolerate more translational vel when far away from scene!
	Vector<6> v6 = mv6CameraVelocity;
	v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
	mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}

// Time to add a new keyframe? The MapMaker handles most of this.
void Tracker::AddNewKeyFrame()
{
#ifdef IPTAM_MOBILE_PAPER
  if(mbMobileBlurFallback) return;
#endif
	mMapMaker.AddKeyFrame(mCurrentKF);
	mnLastKeyFrameDropped = mnFrame;
    mdLastKeyFrameTimestamp = mdLastFrameTimestamp;
}

// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
void Tracker::AssessTrackingQuality()
{
	int nTotalAttempted = 0;
	int nTotalFound = 0;
	int nLargeAttempted = 0;
	int nLargeFound = 0;
	
	for(int i=0; i<LEVELS; i++)
    {
		nTotalAttempted += manMeasAttempted[i];
		nTotalFound += manMeasFound[i];
		if(i>=2) nLargeAttempted += manMeasAttempted[i];
		if(i>=2) nLargeFound += manMeasFound[i];
    }
	
	if(mbNumericalFailure || nTotalFound < 10 || mnPoseInliers < 10 || nTotalAttempted == 0)
		mTrackingQuality = BAD;
	else
    {
		double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
		double dLargeFracFound;
		if(nLargeAttempted > 10)
			dLargeFracFound = (double) nLargeFound / nLargeAttempted;
		else
			dLargeFracFound = dTotalFracFound;
		
		static double gvdQualityGood=TrackerTrackingQualityGood;
		static double gvdQualityLost=TrackerTrackingQualityLost;
		
		
		if(dTotalFracFound > gvdQualityGood)
			mTrackingQuality = GOOD;
		else if(dLargeFracFound < gvdQualityLost)
			mTrackingQuality = BAD;
		else
			mTrackingQuality = DODGY;
    }
	
	if(mTrackingQuality == DODGY)
    {
		// Further heuristics to see if it's actually bad, not just dodgy...
		// If the camera pose estimate has run miles away, it's probably bad.
		if(mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
			mTrackingQuality = BAD;
    }
	
	if(mTrackingQuality==BAD)
		mnLostFrames++;
	else
		mnLostFrames = 0;
}

string Tracker::GetMessageForUser()
{
	return mMessageForUser.str();
}

void Tracker::CalcSBIRotation()
{
	mpSBILastFrame->MakeJacs();
	pair<SE2<>, double> result_pair;
	result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
	if(!std::isfinite(result_pair.second)) { mv6SBIRot=Zeros; return; }
	SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
	mv6SBIRot = se3Adjust.ln();
}

ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here









#ifdef IPTAM_MOBILE_PAPER
void Tracker::TrackMapMobile() {
  mbNumericalFailure=false; mnPoseInliers=0; mnVisible=mnWarpRejected=mnTemplateRejected=0;
  for(int l=0;l<LEVELS;++l) manMeasAttempted[l]=manMeasFound[l]=0;
  if(mbUserPressedSpacebar) { StopTracking(); return; }
  mbMobileBlurFallback=IPTAMMobile::RotationNeedsBlurFallback(mv6SBIRot);
  mCurrentKF.mobileBlurFallback=mbMobileBlurFallback;
  const int targetLevel=mbMobileBlurFallback ? 2 : 1;
  mbDidCoarse=true; mbJustRecoveredSoUseCoarse=false;
  const int gridColumns=8,gridRows=6;
  vector<TrackerData*> cells[gridColumns*gridRows];
  for(auto* point:mMap.vpPoints) {
    if(!point||point->bBad) continue;
    if(!point->pTData) point->pTData=new TrackerData(point);
    TrackerData& data=*point->pTData; data.bFound=data.bSearched=false;
    data.Project(mse3CamFromWorld,mCamera); if(!data.bInImage) continue;
    ++mnVisible; data.GetDerivsUnsafe(mCamera);
    data.Finder.SetMobileMatching(true,mbMobileBlurFallback);
    data.nSearchLevel=data.Finder.CalcSearchLevelAndWarpMatrixAtLevel(*point,mse3CamFromWorld,data.m2CamDerivs,targetLevel);
    if(data.nSearchLevel<0) {++mnWarpRejected;continue;}
    int x=std::min(gridColumns-1,static_cast<int>(data.v2Image[0]*gridColumns/mirSize.x));
    int y=std::min(gridRows-1,static_cast<int>(data.v2Image[1]*gridRows/mirSize.y));
    cells[y*gridColumns+x].push_back(&data);
  }
  for(auto& cell:cells) std::shuffle(cell.begin(),cell.end(),mRng);
  vector<TrackerData*> measurements;
  const size_t maximum=static_cast<size_t>(std::max(0,IPTAMMobile::Config().maxTrackedPoints));
  for(size_t pass=0;measurements.size()<maximum;++pass) {
    bool added=false;
    for(auto& cell:cells) if(pass<cell.size()&&measurements.size()<maximum) { measurements.push_back(cell[pass]);added=true; }
    if(!added) break;
  }
  mnMobileCoarseMatches=SearchForPoints(measurements,4*LevelScale(targetLevel),12);
  SE3<> predicted=mse3CamFromWorld;
  if(mnMobileCoarseMatches>=10) {
    for(int iteration=0;iteration<10;++iteration) {
      for(auto* data:measurements) if(data->bFound) {
        data->ProjectAndDerivs(mse3CamFromWorld,mCamera);
        if(data->bFound) data->CalcJacobian();
      }
      Vector<6> update=CalcPoseUpdate(measurements,iteration>5?kFinalPoseSigmaSquared:0.,iteration==9);
      if(mbNumericalFailure||!std::isfinite(update*update)) {mbNumericalFailure=true;break;}
      SE3<> trial=SE3<>::exp(update)*mse3CamFromWorld;
      Vector<6> logPose=trial.ln();
      if(!std::isfinite(logPose*logPose)) {mbNumericalFailure=true;break;}
      mse3CamFromWorld=trial;
    }
  }
  if(mbNumericalFailure) mse3CamFromWorld=predicted;
  UpdatePoseInlierCount(measurements);
  mCurrentKF.se3CfromW=mse3CamFromWorld;
  mCurrentKF.mMeasurements.clear();
  double depth=0,depthSquared=0;int depthCount=0;
  for(auto* data:measurements) if(data->bFound) {
    data->ProjectAndDerivs(mse3CamFromWorld,mCamera);
    if(!data->bFound) continue;
    data->CalcJacobian();
    Measurement measurement;
    measurement.Source=Measurement::SRC_TRACKER;
    measurement.nLevel=data->nSearchLevel; measurement.bSubPix=data->bDidSubPix;
    measurement.v2RootPos=measurement.v2RawRootPos=measurement.v2BundleRootPos=data->v2Found;
    measurement.hasRawPosition=true; measurement.m26PoseJacobian=data->m26Jacobian;
    measurement.timestamp=mdLastFrameTimestamp;
    if(mbTimestampSet&&data->mobileLastObservationTimed&&data->mobileLastObservedFrame==mnFrame-1) {
      measurement.hasPrevious=true;measurement.previousTimestamp=data->mobileLastObservedTimestamp;
      measurement.v2PreviousRootPos=data->mobileLastObservedPosition;
    }
    mCurrentKF.mMeasurements[&data->Point]=measurement;
    double z=data->v3Cam[2]; if(std::isfinite(z)&&z>0) {depth+=z;depthSquared+=z*z;++depthCount;}
  }
  if(depthCount>20) {
    mCurrentKF.dSceneDepthMean=depth/depthCount;
    mCurrentKF.dSceneDepthSigma=std::sqrt(std::max(0.,depthSquared/depthCount-mCurrentKF.dSceneDepthMean*mCurrentKF.dSceneDepthMean));
  }
}

void Tracker::UpdateMobileTemporalMeasurements() {
  // Only actual consecutive successful video observations may establish timing.
  if(mTrackingQuality!=GOOD||!mbTimestampSet||mbMobileBlurFallback) return;
  for(auto& entry:mCurrentKF.mMeasurements) {
    auto* data=entry.first->pTData;
    if(!data) continue;
    data->mobileLastObservedFrame=mnFrame; data->mobileLastObservedTimestamp=mdLastFrameTimestamp;
    data->mobileLastObservationTimed=true; data->mobileLastObservedPosition=entry.second.v2RootPos;
  }
  if(!IPTAMMobile::Config().rollingShutter) return;
  IPTAMMobile::RollingShutterCalibration calibration;
  calibration.known=true;calibration.width=mirSize.x;calibration.height=mirSize.y;
  calibration.axis=IPTAMMobile::Config().readoutAxis;
  calibration.direction=IPTAMMobile::Config().readoutDirection;
  calibration.readoutSeconds=IPTAMMobile::Config().readoutSeconds;
  calibration.timestampFraction=IPTAMMobile::Config().exposureTimestampFraction;
  std::string reason;
  if(!calibration.Validate(reason)) return;
  auto update=[&](KeyFrame* keyframe) {
    if(!keyframe||!keyframe->hasCaptureTimestamp||keyframe->mobileBlurFallback) return;
    bool adjacent=false;
    for(auto& entry:mCurrentKF.mMeasurements) {
      const Measurement& current=entry.second;
      if(!current.hasPrevious||current.previousTimestamp!=keyframe->captureTimestamp) continue;
      auto previous=keyframe->mMeasurements.find(entry.first);
      if(previous==keyframe->mMeasurements.end()) continue;
      previous->second.hasNext=true;previous->second.nextTimestamp=mdLastFrameTimestamp;
      previous->second.v2NextRootPos=current.v2RootPos; adjacent=true;
    }
    if(!adjacent) return;
    vector<IPTAMMobile::RollingShutterObservation> observations;
    for(const auto& entry:keyframe->mMeasurements) {
      const auto& m=entry.second;
      if(!m.hasRawPosition||!m.hasPrevious||!m.hasNext) continue;
      IPTAMMobile::RollingShutterObservation observation;
      observation.pixel=m.v2RawRootPos;observation.poseJacobian=m.m26PoseJacobian;
      observation.timestamp=m.timestamp;observation.previousTimestamp=m.previousTimestamp;observation.nextTimestamp=m.nextTimestamp;
      observation.previousPixel=m.v2PreviousRootPos;observation.nextPixel=m.v2NextRootPos;
      observation.hasPrevious=observation.hasNext=true;
      observations.push_back(observation);
    }
    mnMobileRSObservations+=static_cast<int>(observations.size());
    auto estimate=IPTAMMobile::RollingShutter::EstimateVelocity(observations,calibration);
    if(!estimate.valid) return;
    bool changed=false;
    for(auto& entry:keyframe->mMeasurements) {
      auto& measurement=entry.second; if(!measurement.hasRawPosition) continue;
      Vector<2> corrected;
      if(IPTAMMobile::RollingShutter::Correct(measurement.v2RawRootPos,measurement.m26PoseJacobian,estimate,calibration,corrected,reason)) {
        measurement.v2BundleRootPos=corrected;measurement.rollingShutterCorrected=true;++mnMobileRSCorrections;changed=true;
      }
    }
    if(changed) mMapMaker.NotifyMobileMeasurementsChanged(*keyframe);
  };
  for(auto* keyframe:mMap.vpKeyFrames) update(keyframe);
}
#endif

#ifdef IPTAM_MOBILE_PAPER
int Tracker::TrailTrackingAdvanceMobile() {
  int good=0;
  for(auto it=mlTrails.begin();it!=mlTrails.end();) {
    const Vector<2> previous=it->mobileCurrentPosition;
    PatchFinder forward;forward.SetMobileMatching(true);
    forward.MakeMobileTemplateAtRoot(mPreviousFrameKF,1,previous);
    bool valid=!forward.TemplateBad()&&forward.FindPatchCoarse(CVD::ir_rounded(previous),mCurrentKF,8);
    Vector<2> current=previous;
    if(valid) {forward.MakeSubPixTemplate();valid=forward.IterateSubPixToConvergence(mCurrentKF,15);current=forward.GetSubPixPos();}
    if(valid) {
      PatchFinder backward;backward.SetMobileMatching(true);
      backward.MakeMobileTemplateAtRoot(mCurrentKF,1,current);
      valid=!backward.TemplateBad()&&backward.FindPatchCoarse(CVD::ir_rounded(previous),mPreviousFrameKF,8);
      if(valid) {
        backward.MakeSubPixTemplate();valid=backward.IterateSubPixToConvergence(mPreviousFrameKF,15);
        const Vector<2> error=backward.GetSubPixPos()-previous;
        valid=valid&&std::isfinite(error*error)&&error*error<=2.;
      }
    }
    if(valid) {it->mobileCurrentPosition=current;it->irCurrentPos=CVD::ir_rounded(current);++it;++good;}
    else it=mlTrails.erase(it);
  }
  mPreviousFrameKF=mCurrentKF;
  return good;
}
#endif
