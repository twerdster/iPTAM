# iPTAM

[![C++ and Objective-C](https://img.shields.io/badge/code-C%2B%2B%20%7C%20Objective--C-00599C?logo=c%2B%2B&logoColor=white)](xPTAMclean)
[![Platform: iOS 4 era, Xcode](https://img.shields.io/badge/platform-iOS%204%20era%20%C2%B7%20Xcode-555555?logo=apple&logoColor=white)](xPTAMclean/xPTAM.xcodeproj)
[![Status: unmaintained](https://img.shields.io/badge/status-unmaintained-lightgrey)](#iptam)

**PTAM for the iPhone.** An attempted port of PTAM (Parallel Tracking and Mapping) by Georg Klein and
David Murray to iOS, together with trimmed copies of the LibCVD (`iCVD`) and TooN (`iTooN`) libraries
it depends on.

![Explanatory illustration of the PTAM architecture in this port: camera frames, tracking thread, mapping thread, shared map and OpenGL ES display](docs/images/iptam-architecture.svg)

*Explanatory illustration of the PTAM tracking/mapping structure, using component names from the
source files in `xPTAMclean/`. It is not a screenshot, and it does not show that the port works.*

> **NOTE:** This project is out of date and is not being maintained.

The original notes follow.

## Usage

The project can be downloaded as is and compiled in XCode. You will need a provisioned iPhone running iOS >4.0 to compile the software.The project should compile out of the box.

Please be aware that this is not  intended to be a complete project in ANY sense and needs a huge amount of work in order to further its utility as an Augmented Reality application pose estimator. Please read [Report.pdf](iPTAM%20report.pdf) for more details on the port I have attempted.

These build notes date from the iOS 4 era. The project has not been rebuilt or tested with
current Xcode or iOS versions.

## License

All files in the iPTAM folder fall under this license agreement: http://www.robots.ox.ac.uk/~gk/PTAM/download.html

### LibCVD

All files for LibCVD fall under this license agreement which should be in every file in the iCVD directory:

```
/*
        This file is part of the CVD Library.

        Copyright (C) 2005 The Authors

        This library is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 2.1 of the License, or (at your option) any later version.

        This library is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.

        You should have received a copy of the GNU Lesser General Public
        License along with this library; if not, write to the Free Software
        Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
```

If you find a file in that directory which does not contain this license heading then add this heading and abide by the terms.

### TooN

All files for TooN fall under this license agreement which should be in every file in the iTooN directory:

```
// -*- c++ -*-

//     Copyright (C) 2009 Tom Drummond (twd20@cam.ac.uk)
//
// This file is part of the TooN Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with this library; see the file COPYING.  If not, write to the Free
// Software Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307,
// USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU General Public License.
```

If you find a file in that directory which does not contain this license heading then add this heading and abide by the terms.
