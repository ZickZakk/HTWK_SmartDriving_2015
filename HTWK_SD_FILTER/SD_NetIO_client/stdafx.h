/**
 *
 * ADTF Demo Source.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: ABOEMI9 $
 * $Date: 2013-08-27 16:36:26 +0200 (Tue, 27 Aug 2013) $
 * $Revision: 40298 $
 *
 * @remarks
 *
 */
#ifndef __STD_INCLUDES_HEADER
#define __STD_INCLUDES_HEADER

#define _USE_MATH_DEFINES

#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
#include <string.h>
#include <string.h>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <strings.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <thread>
using namespace std;

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
using namespace cv;

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include <adtf_graphics.h>
using namespace adtf_graphics;

#include "Maneuver.h"
#include "Steer.h"
#include "Lane.h"

#include "IDriveAlgorithm.h"
#include "DriveAlgorithm.h"

#include "ICrossRoadDetector.h"
#include "CrossRoadDetector.h"

#include "Server.h"
#include "Client.h"

using namespace adtf;

//#ifdef WIN32
//#include <windows.h>
//#endif // WIN32

//#include <GL/gl.h>
//#include <GL/glu.h>


#endif // __STD_INCLUDES_HEADER
