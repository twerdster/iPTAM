/*
 * Autor : Arnaud GROSJEAN (VIDE SARL)
 * This implementation of VideoSource allows to use OpenCV as a source for the video input
 * I did so because libCVD failed getting my V4L2 device
 *
 * INSTALLATION :
 * - Copy the VideoSource_Linux_OpenCV.cc file in your PTAM directory
 * - In the Makefile:
 *	- set the linkflags to
 LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lblas -llapack -lGVars3 -lcvd -lcv -lcxcore -lhighgui
 *	- set the videosource to 
 VIDEOSOURCE = VideoSource_Linux_OpenCV.o
 * - Compile the project
 * - Enjoy !
 * 
 * Notice this code define two constants for the image width and height (OPENCV_VIDEO_W and OPENCV_VIDEO_H)
 */

#include "VideoSource.h"
#include "colourspace_convert.h"
#include "colourspaces.h"
#include "CAMSIZE.h"

using namespace CVD;
using namespace std;

using namespace cv;

int OPENCV_VIDEO_W =CAMSIZEX; 
int OPENCV_VIDEO_H =CAMSIZEY;
int sml=SMALL;

VideoSource::VideoSource()
{
	cout << "  VideoSource: Opening video source..." << endl;
	if (sml) 
	mptr = new VideoCapture("/users/aw/desktop/PTAMvids/small.AVI");	
	else
		mptr = new VideoCapture("/users/aw/desktop/PTAMvids/check_table1.AVI");	
	VideoCapture* cap = (VideoCapture*)mptr;
	if(!cap->isOpened()){
		cerr << "Unable to get the camera" << endl;
		exit(-1);
	}
	cout << "  ... got video source." << endl;
	mirSize = ImageRef(OPENCV_VIDEO_W, OPENCV_VIDEO_H);
};

ImageRef VideoSource::Size()
{ 
	return mirSize;
};

void conversionNB(Mat frame, Image<byte> &imBW){
	Mat clone = frame.clone();
	Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
	for (int i = 0; i < OPENCV_VIDEO_H; i++){
		for (int j = 0; j < OPENCV_VIDEO_W; j++){	
			imBW[i][j] = (frame_p(i,j)[0] + frame_p(i,j)[1] + frame_p(i,j)[2]) / 3;
		}
	}
	
}

void conversionRGB(Mat frame, Image<Rgb<byte> > &imRGB){
	Mat clone = frame.clone();
	Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
	for (int i = 0; i < OPENCV_VIDEO_H; i++){
		for (int j = 0; j < OPENCV_VIDEO_W; j++){	
			imRGB[i][j].red = frame_p(i,j)[2];
			imRGB[i][j].green = frame_p(i,j)[1];
			imRGB[i][j].blue = frame_p(i,j)[0];
		}
	}
}

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
	Mat frame;
	VideoCapture* cap = (VideoCapture*)mptr;
	*cap >> frame;
	conversionNB(frame, imBW);
	conversionRGB(frame, imRGB);
}


