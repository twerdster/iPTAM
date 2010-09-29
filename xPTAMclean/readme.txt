Read Me About GLVideoFrame
========================
1.0

GLVideoFrame demonstrates how to use the AVFoundation APIs to get, manipulate, and view uncompressed sample buffers captured from the camera.

GLVideoFrame runs on iPhone OS 4.0 and later devices with a video camera.

Packing List
------------
The sample contains the following items:

o Read Me.txt -- This file.
o GLVideoFrame -- An Xcode project for the sample.
o build -- Contains a pre-built binary for testing purposes.
o Classes.[h,m] -- The core AVCam code.
o main.m -- Creates the app object and the application delegate and sets up the event cycle.
o Icon.png, Icon-Small.png, iTunesArtwork -- program icon files.
o MainWindow.xib - nib file containing the program user interface objects
o MyVideoBuffer.m/.h - object that initializes the capture session and get the uncompressed frames from the camera.
o GLVideoFrame-Info.plist - program property list file.

Using the Sample
----------------
To test the sample, just build and run it and you will see a colored image animated against a background containing live camera video.

Building the Sample
-------------------
The sample was built using Xcode 3.2.3 on Mac OS X 10.6.3 with the Mac OS X 10.6 SDK.  You should be able to just open the project and choose Build from the Build menu.

How It Works
------------

GLVideoFrame displays an animated colored sprite on top of a background containing the live captured video frames. The video frames are copied into an OpenGL texture, and OpenGL is used for the rendering. More specifically, the EAGLView class wraps the CAEAGLLayer from CoreAnimation into a convenient UIView subclass. This view content is an EAGL surface used to render the OpenGL scene.

Credits and Version History
---------------------------
If you find any problems with this sample, please file a bug against it.

<http://developer.apple.com/bugreporter/>

1.0 (June 2010) was the first shipping version.

