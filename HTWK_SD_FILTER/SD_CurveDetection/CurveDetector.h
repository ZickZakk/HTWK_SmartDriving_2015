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
#define FILTER_NAME "SD CurveDetector Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.CurveDetector.debug"
#else
#define FILTER_NAME "SD CurveDetector Release"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.CurveDetector"
#endif

#define PROP_NAME_CANNY_THRESHOLD_1 "Canny_Threshold_1"
#define PROP_NAME_CANNY_THRESHOLD_2 "Canny_Threshold_2"

const tInt NO_POINT = -22;

class CurveDetector : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// pins
	cVideoPin videoInput;
	cVideoPin rgbOutput;
		
	// video stuff
	tBool isFirstFrame;
	tBitmapFormat videoInputInfo;

	// thresholds
	tInt thresholdCanny1;
	tInt thresholdCanny2;

public:
	CurveDetector(const tChar *__info);
	virtual ~CurveDetector(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	tResult PropertyChanged(const char *name);

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
	tResult createVideoInputPin(const tChar*, cVideoPin&);
	
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
	tVoid setBitmapFormat(const tBitmapFormat*);
	
	/** 
	* The main pipeline for image processing. 
	*
	* @param[in] mediaSample an interface for media samples i.e. images
	*
	* @return NOERROR if successful, ERROR otherwise.
	*/
	tResult processImage(IMediaSample*);

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
	* 
	*
	*/
	tVoid searchLines(const Mat &image, list<SD_Point> &leftLines, list<SD_Point> &rightLines);

	/**
	*
	* @param[in] image
	* @param[in] center
	* @param[in] height
	* @param[out] left
	*/
	tVoid getLeftLane(const Mat &image, const tUInt &center, const tUInt &height, tUInt &left);

	/**
	*
	* @param[in] image
	* @param[in] center
	* @param[in] height
	* @param[out] left
	*/
	tVoid getRightLane(const Mat &image, const tUInt &center, const tUInt &height, tUInt &right);

	/**
	*
	*/
	tVoid addPointToList(const SD_Point &point, list<SD_Point> &list);

	/**
	*
	*/
	tVoid drawPointsInImage(Mat &image, list<SD_Point> &list, const Scalar &color);

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
	tResult transmitVideoOutput(Mat&, cVideoPin&);
};

#endif