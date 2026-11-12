# iPTAM context

## Purpose

Incomplete iOS 4-era port of Klein and Murray’s PTAM with Objective-C camera/display code and trimmed LibCVD and TooN.

## Implementation evidence

Project settings target iOS 4.1/32-bit. Source listings confirm tracker/map classes; MapMaker inherits CVD::Thread and calls start(), without proving working mapping. The original report describes initialized planar points followed by failed map matching; recovery was disabled for motion-model-only updates, with drift/scale limitations.

Key entry points: `xPTAMclean/xPTAM.xcodeproj/project.pbxproj`, `xPTAMclean/MyVideoBuffer.mm`, `xPTAMclean/Classes/ES1Renderer.m`, `xPTAMclean/iPTAM/System.cc`, `xPTAMclean/iPTAM/MapMaker.h`, `xPTAMclean/iPTAM/MapMaker.cc`, `iPTAM report.pdf`.

## Documentation state

The README explains the historical port, build settings and reported tracking failure. The pre-existing source-architecture schematic now uses component labels and notes disabled recovery. Source files and all original component notices are unchanged.

## Validation scope

Project/camera/display source and historical report inspection only; no current Xcode build, iPhone deployment, tracking test or runtime measurement.

## License boundary

PTAM, LibCVD and TooN retain distinct component notices. No single license for the entire tree or independent wrapper/project-file license was established by this review.

## Limitations

Unmaintained port, old SDK/signing/API requirements and historically nonworking closed-loop tracking. The architecture figure shows source components, not demonstrated operation.
