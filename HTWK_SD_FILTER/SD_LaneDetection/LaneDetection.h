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

#ifndef _NEW_LANE_DETECTION_H_
#define _NEW_LANE_DETECTION_H_

#ifdef _DEBUG
#define FILTER_NAME "SD LaneDetection Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.LaneDetection.debug"
#else
#define FILTER_NAME "SD LaneDetection Release"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.LaneDetection.release"
#endif

#define PROP_NAME_HOUGH_ALGORITHM "HoughAlgorithm"
#define PROP_NAME_BINARY_THRESHOLD "Binary_Threshold"
#define PROP_NAME_CANNY_THRESHOLD_1 "Canny_Threshold_1"
#define PROP_NAME_CANNY_THRESHOLD_2 "Canny_Threshold_2"
#define PROP_NAME_HOUGH_THRESHOLD "Hough_Threshold"

class LaneDetection : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// pins
	cVideoPin videoInput;
	cVideoPin edgdeOuputPin;
	cVideoPin binaryOuputPin;
	cVideoPin linesOutputPin;
	cOutputPin glcOutput;
	
	// stuff
	tBool isFirstFrame;
	tBitmapFormat videoInfo;
	HoughAlgorithm hough;

	// properties
	tInt thresholdCanny1;
	tInt thresholdCanny2;
	tInt thresholdHough;
	tInt houghAlgorithm;

	tVoid initProperties();
	tResult createInputPin(const tChar*, cVideoPin&);
	tResult createOutputPin(const tChar*, cOutputPin&);
	tResult createVideoOutputPin(const tChar*, cVideoPin&);
	tVoid setBitmapFormat(const tBitmapFormat*);

	tResult processImage(IMediaSample*);
	tVoid prepareImage(Mat&);
	void drawLinesInImage(Mat &image, vector<SD_Line> &lines);
	tResult createAndTransmitGCL(vector<SD_Line>&);
	tResult transmitVideoOutput(Mat&, cVideoPin&);
	tVoid setThresholds();
	tVoid setHoughAlgorithm();

public:
	LaneDetection(const tChar *__info);
	virtual ~LaneDetection(void);

	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	tResult PropertyChanged(const char*);
};

#endif