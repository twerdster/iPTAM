

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>
#include <OpenGLES/ES1/glext.h>
									 
@interface MyVideoBuffer : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>{

	@public
	AVCaptureSession*		_session;

	CMTime previousTimestamp;
	
	Float64 videoFrameRate;
	CMVideoDimensions videoDimensions;
	CMVideoCodecType videoType;
	
	uint m_textureHandle;
	unsigned char *bwImage;
	
}

@property (nonatomic, retain) AVCaptureSession* _session;
@property (readwrite) Float64 videoFrameRate;
@property (readwrite) CMVideoDimensions videoDimensions;
@property (readwrite) CMVideoCodecType videoType;
@property (readwrite) CMTime previousTimestamp;
@property (readwrite) uint CameraTexture;

- (void)	captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection;
- (void)	renderCameraToSprite:(uint)text;
- (GLuint)	createVideoTextuerUsingWidth:(GLuint)w Height:(GLuint)h;
- (void)	resetWithSize:(GLuint)w Height:(GLuint)h;
- (void)	convertToBlackWhite:(unsigned char *) pixels;
- (void) setGLStuff:(EAGLContext*)c :(GLuint)rb :(GLuint)fb :(GLuint)bw :(GLuint)bh; 
- (void) render;
- (IBAction) pressButton;


@end
