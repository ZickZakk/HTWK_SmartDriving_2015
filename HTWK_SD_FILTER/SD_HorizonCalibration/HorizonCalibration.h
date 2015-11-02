/**
 * Copyright (c) 2014-2015, HTWK SmartDriving
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * AUTHORS: Silvio Feig, Denny Hecht, Andreas Kluge, Lars Kollmann, Eike Florian Petersen, Artem Pokas
 *
 */

#ifndef _IMAGE_CUTTER_H_
#define _IMAGE_CUTTER_H_

#ifdef _DEBUG
#define FILTER_NAME "SD HorizonCalibration Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.HorizonCalibration.debug"
#else
#define FILTER_NAME "SD HorizonCalibration"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.HorizonCalibration"
#endif

#define PROP_NAME_CANNY_THRESHOLD_1 "Canny_Threshold_1"
#define PROP_NAME_CANNY_THRESHOLD_2 "Canny_Threshold_2"
#define PROP_NAME_HOUGH_THRESHOLD "Hough_Threshold"

/**
* A Filter for horizon detection. The detected horizon will be safed in the log directory of ADTF.
*
* @copyright (c) Copyright 2014 SD. All rights reserved
* @author dhecht
*/
class HorizonCalibration : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// pins
	cVideoPin videoInput;
	cVideoPin horizonVideo;
	
	// video stuff
	tBool isFirstFrame;
	tBitmapFormat videoInfo;

	// Line detection
	unique_ptr<AbstractHough> lineDetection;

	// Properties
	tUInt8 thresholdCanny1;
	tUInt8 thresholdCanny2;
	tUInt8 thresholdHough;

	// horizon detection
	tInt maxCameraCalibrationFrames;
	tInt maxHorizonCalibrationFrames;
	tInt currentFrame;
	vector<tInt> horizons;
	tInt horizonPosition;

public:
	HorizonCalibration(const tChar *__info);
	virtual ~HorizonCalibration(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	tResult PropertyChanged(const char*);
	
private:
	/** 
	* Initalises the properties for the Canny-filter and the hough-transform. 
	*/ 
	tVoid initProperties();
	
	/** 
	* Creates a new video input interface with a specific name.
	*
	* @param[in] name the name of the video input interface
	* @param[in] pin a specific cVideoPin that should be create 
	*
	* @return NOERROR if successful, ERROR otherwise. 
	*/
	tResult createVideoInputPin(const tChar* name, cVideoPin &pin);
	
	/** 
	* Creates a new video output interface with a specific name.
	*
	* @param[in] name the name of the video input interface
	* @param[in] pin a specific cVideoPin that should be create 
	*
	* @return NOERROR if successful, ERROR otherwise.  
	*/
	tResult createVideoOutputPin(const tChar*, cVideoPin&);
	
	/** 
	* Copies the video input informations to an private variable.
	*
	* @param[in] format the video information
	*/
	tVoid setBitmapFormat(const tBitmapFormat *format);
	
	/** 
	* The main pipeline for image processing. 
	*
	* @param[in] mediaSample an interface for media samples i.e. images
	*
	* @return NOERROR if successful, ERROR otherwise.
	*/
	tResult processImage(IMediaSample *mediaSample);
	
	/** 
	* Prepares the image for the Hough-transform.
	* 1. converts the rgb image in grayscale image
	* 2. blurs the image to reduce the noise
	* 3. edge detection with the Canny-filter
	*
	* @param[in,out] image [in] an rgb image, [out] edge detected image (binary image)
	*/
	tVoid prepareImage(Mat &image);
	
	/** 
	* Detects lines by using the Hough-transform with the angles from -45° to 45°.
	*
	* @param[in] image an edge detected image
	* @param[out] lines a set of detected lines
	*/
	tVoid detectLines(const Mat &image, vector<SD_Line> &lines);
	
	/**  
	* Draws a set of lines in a specific image.
	*
	* @depricated because not used in this filter
	*/
	tVoid drawLinesInImage(Mat &image, vector<SD_Line> &lines);
	
	/** 
	* Draws a set of red lines in a specific image.
	*
	* @param[in, out] image [in] an rgb image, [out] an rgb image with red lines
	* @param[in] lines a set of lines
	*/
	tVoid drawLinesInImage(Mat &image, list<SD_LineExtendet> &lines);
	
	/** 
	* Draws a green horizon in a specific image.
	*
	* @param[in, out] image [in] an rgb image, [out] an rgb image with a green line
	* @param[in] horizon the coordinates of the horizon
	*/
	tVoid drawHorizonInImage(Mat &image, SD_Line &horizon);
	
	/** 
	* Searchs all lines at the lower third of the image and stores all lines left and right from the image center in a seperate list.
	*
	* @param[in] lines all detected lines from the Hough-transform
	* @param[out] leftLines a sorted list with all lines left from the image center
	* @param[out] rightLines a sorted list with all lines right from the image center
	*/
	tVoid searchLinesOfInterest(vector<SD_Line> &lines, list<SD_LineExtendet> &leftLines, list<SD_LineExtendet> &rightLines);
	
	/** 
	* Detects the horizon on the basis of the lines left and right from the image center.
	*
	* @param[in] leftLines a sorted list with all lines left from the image center
	* @param[in] rightLines a sorted list with all lines right from the image center
	*
	* @return the coordinates of the horizon
	*/
	SD_Line detectHorizon(list<SD_LineExtendet> &leftLines, list<SD_LineExtendet> &rightLines);
	
	/** 
	* Check if the given position is in the image.
	*
	* @param[in] a specific point
	*
	* @return true if the point is in the image, false otherwise
	*/
	tBool isPointInImage(const SD_Point &point);
	
	/** 
	* Calculates the y-position of the horizon from the measured values. By default 60 values will be measured.
	*
	* @return the y-position of the horizon
	*/
	tInt getHorizon();
	
	/** 
	* Saves the y-position of the horizon in the file "horizon.txt". The file is found in the log directory of ADTF.
	*/
	tVoid saveHorizonToFile();
	
	/** 
	* Updates the thresholds to newer thresholds from the UI of ADTF.
	*/
	tVoid setThresholds();
	
	/** 
	* Transmits an image to a specific video output interface.
	*
	* @param[in] image a specific image
	* @param[in] a specific cVideoPin that should send the video to an other filter
	*
	* @return NOERROR if successful, ERROR otherwise. 
	*/
	tResult transmitVideoOutput(Mat &image, cVideoPin&);
};

#endif