

#import "MyVideoBuffer.h"
#import <MobileCoreServices/MobileCoreServices.h>
#import <OpenGLES/ES1/gl.h>
#import <OpenGLES/ES1/glext.h>
#import <CoreGraphics/CoreGraphics.h>

#include "System.h"
System ptam;

@implementation MyVideoBuffer

@synthesize _session;
@synthesize previousTimestamp;
@synthesize videoFrameRate;
@synthesize videoDimensions;
@synthesize videoType;
@synthesize CameraTexture=m_textureHandle;

EAGLContext *acontext;
GLuint arb,afb;
GLint abw;
GLint abh;

- (IBAction) pressButton {
	NSLog(@"Pressed screen");
	ptam.SendTrackerStartSig();
}

-(id) init
{
	if ((self = [super init]))
	{
		NSError * error;
		
		//-- Setup our Capture Session.
		self._session = [[AVCaptureSession alloc] init];
		
		[self._session beginConfiguration];
		
		//-- Set a preset session size. 
		[self._session setSessionPreset:AVCaptureSessionPreset640x480];
		
		//-- Creata a video device and input from that Device.  Add the input to the capture session.
		AVCaptureDevice * videoDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
		if(videoDevice == nil)
			return nil;
		
		//-- Add the device to the session.
		AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
		if(error)
			return nil;
		
		[self._session addInput:input];
		
		//-- Create the output for the capture session.  We want 32bit BRGA
		AVCaptureVideoDataOutput * dataOutput = [[[AVCaptureVideoDataOutput alloc] init] autorelease];
		[dataOutput setAlwaysDiscardsLateVideoFrames:YES]; // Probably want to set this to NO when we're recording//
		[dataOutput setVideoSettings:[NSDictionary dictionaryWithObject:[NSNumber numberWithInt:kCVPixelFormatType_32BGRA] forKey:(id)kCVPixelBufferPixelFormatTypeKey]]; // Necessary for manual preview
	
		// we want our dispatch to be on the main thread so OpenGL can do things with the data
		[dataOutput setSampleBufferDelegate:self queue:dispatch_get_main_queue()];
		
		
		[self._session addOutput:dataOutput];
		[self._session commitConfiguration];
		
		//-- Pre create our texture, instead of inside of CaptureOutput.
		m_textureHandle = [self createVideoTextuerUsingWidth:640 Height:480];
		
		//and create output Black and White image
		bwImage = (unsigned char *)malloc(640*480*4);
		memset(bwImage,0,640*480*4);
	}
	return self;
}

-(GLuint)createVideoTextuerUsingWidth:(GLuint)w Height:(GLuint)h
{
//	int dataSize = w * h ;
//	uint8_t* textureData = (uint8_t*)malloc(dataSize);
//	if(textureData == NULL)
//		return 0;
//	memset(textureData, 128, dataSize);
	
	GLuint handle;
	glGenTextures(1, &handle);
	glBindTexture(GL_TEXTURE_2D, handle);
	glTexParameterf(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, w, h, 0, GL_LUMINANCE, 
				 GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, 0);
	//free(textureData);
	
	return handle;
}

- (void) resetWithSize:(GLuint)w Height:(GLuint)h
{
	NSLog(@"_session beginConfiguration");
	[_session beginConfiguration];
	
	//-- Match the wxh with a preset.
	if(w == 1280 && h == 720)
	{
		[_session setSessionPreset:AVCaptureSessionPreset1280x720];
	}
	else if(w == 640)
	{
		[_session setSessionPreset:AVCaptureSessionPreset640x480];
	}
	else if(w == 480)
	{
		[_session setSessionPreset:AVCaptureSessionPresetMedium];
	}
	else if(w == 192)
	{
		[_session setSessionPreset:AVCaptureSessionPresetLow];
	}
	
	[_session commitConfiguration];
	NSLog(@"_session commitConfiguration");
}

- (void) convertToBlackWhite:(unsigned char *) pixels 
{
	memcpy(bwImage,pixels,640*480*4);
	//changing the order of access of the individual pixels massively changes the speed of the program.
	//i think this is because there are multiple and frequent cache misses when the pixels are not called in order
	unsigned int * pntrBWImage= (unsigned int *)bwImage;
	unsigned int index;
	unsigned int fourBytes;
	for (int j=0;j<480; j++)
	for (int i=0; i<640; i++) 
		{
			index=(640)*j+i;
			fourBytes=pntrBWImage[index];
			//bwImage[index]=(fourBytes<<1);//pntrBWImage[index];//(bwImage[4*index+0]+bwImage[4*index+1]+bwImage[4*index+2])/3;//(pixels[4*index]+pixels[4*index+1]+pixels[4*index+2]);
			//bwImage[index]=(fourBytes>>(1*8) & 0x000F + fourBytes>>(2*8) & 0x000F + fourBytes>>(3*8) & 0x000F)/3;
			bwImage[index]=((unsigned char)fourBytes>>(2*8)) +((unsigned char)fourBytes>>(1*8))+((unsigned char)fourBytes>>(0*8));
		}
}

CVD::Image<CVD::byte> mimFrameBW;

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
	CMTime timestamp = CMSampleBufferGetPresentationTimeStamp( sampleBuffer );
	if (CMTIME_IS_VALID( self.previousTimestamp ))
		self.videoFrameRate = 1.0 / CMTimeGetSeconds( CMTimeSubtract( timestamp, self.previousTimestamp ) );
	
	previousTimestamp = timestamp;

	CMFormatDescriptionRef formatDesc = CMSampleBufferGetFormatDescription(sampleBuffer);
	self.videoDimensions = CMVideoFormatDescriptionGetDimensions(formatDesc);
	
	CMVideoCodecType type = CMFormatDescriptionGetMediaSubType(formatDesc);
#if defined(__LITTLE_ENDIAN__)
	type = OSSwapInt32(type);
#endif
	self.videoType = type;
	
	CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
	CVPixelBufferLockBaseAddress( pixelBuffer, 0 );
	
	//-- If we haven't created the video texture, do so now.
	//	if(m_textureHandle == 0)
//	{
//m_textureHandle = [self createVideoTextuerUsingWidth:videoDimensions.width Height:videoDimensions.height];
//	}
	

	

	
	
	// This application only creates a single context which is already set current at this point.
    // This call is redundant, but needed if dealing with multiple contexts.
	[EAGLContext setCurrentContext:acontext];
	
    // This application only creates a single default framebuffer which is already bound at this point.
    // This call is redundant, but needed if dealing with multiple framebuffers.
	glBindFramebufferOES(GL_FRAMEBUFFER_OES, afb);
    glViewport(0, 0, abw, abh);
	
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
	
		
	unsigned char* linebase = (unsigned char *)CVPixelBufferGetBaseAddress( pixelBuffer );
	[self convertToBlackWhite:linebase];
	
	if (0) {
		glBindTexture(GL_TEXTURE_2D, m_textureHandle);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, videoDimensions.width, videoDimensions.height, GL_LUMINANCE, GL_UNSIGNED_BYTE, bwImage);
		[self renderCameraToSprite:m_textureHandle];
	}
	else
		ptam.RunOneFrame(bwImage,m_textureHandle);
	
	
	// This application only creates a single color renderbuffer which is already bound at this point.
    // This call is redundant, but needed if dealing with multiple renderbuffers.
	glBindRenderbufferOES(GL_RENDERBUFFER_OES, arb);
	
    [acontext presentRenderbuffer:GL_RENDERBUFFER_OES];
	
		CVPixelBufferUnlockBaseAddress( pixelBuffer, 0 );
	
}



-(void)renderCameraToSprite:(uint)text
{
	GLfloat spriteTexcoords[] = {
		1,1,   
		1,0.0f,
		0,1,   
		0.0f,0,};
	
	GLfloat spriteVertices[] =  {
		0,0,0,   
		320,0,0,   
		0,480,0, 
		320,480,0};
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrthof(0, 320, 0, 480, 0, 1);
	
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnable(GL_TEXTURE_2D);

	glVertexPointer(3, GL_FLOAT, 0, spriteVertices);
	glTexCoordPointer(2, GL_FLOAT, 0, spriteTexcoords);	
	glBindTexture(GL_TEXTURE_2D, text);


	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glBindTexture(GL_TEXTURE_2D, 0);
	
}

- (void)render
{
    // Replace the implementation of this method to do your own custom drawing
	
    }


- (void) setGLStuff:(EAGLContext*)c :(GLuint)rb :(GLuint)fb :(GLuint)bw :(GLuint)bh 
{
	acontext=c;
	arb=rb;
	afb=fb;
	abw=bw;
	abh=bh;
}

- (void)dealloc 
{
	[_session release];
	
	[super dealloc];
	free(bwImage);
}

@end
