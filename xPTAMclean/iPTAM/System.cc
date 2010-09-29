// Copyright 2008 Isis Innovation Limited
#include "System.h"
#import <OpenGLES/ES1/gl.h>
#import <OpenGLES/ES1/glext.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"

using namespace CVD;
using namespace std;

System::System()
{
  mpCamera = new ATANCamera("Camera");
  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(ImageRef(640,480), *mpCamera, *mpMap, *mpMapMaker);  
  mimFrameBW.resize(ImageRef(640,480));
  mbDone = false;
};

void System::SendTrackerStartSig()
{
	mpTracker->StartTracking();
}

void System::SendTrackerKillSig()
{
	mpTracker->StopTracking();
}

void System::RunOneFrame(unsigned char *bwImage,uint hnd)
{
  //while(!mbDone)
  //{
      
      // Grab video frame in black and white from videobuffer
	mimFrameBW.copy_from(BasicImage<byte>(bwImage,ImageRef(640,480)));
	
	
    //  mGLWindow.SetupViewport();
    //  mGLWindow.SetupVideoOrtho();

	//glViewport(0, 0, 320, 480);
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	glOrthof(0, 640, 480, 0, -1, 1);
//	
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
	//glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    //glClear(GL_COLOR_BUFFER_BIT);


    mpTracker->TrackFrame(mimFrameBW, hnd,1);
      
	//string sCaption = mpTracker->GetMessageForUser();
    //}
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
}








