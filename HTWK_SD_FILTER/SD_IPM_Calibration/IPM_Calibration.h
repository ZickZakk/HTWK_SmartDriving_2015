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

/**
* @filename
* @copyright (c) Copyright 2014 SD-Team HTWK
* @author dhecht
* @details
*/
#ifndef _IPM_Calibration_H_
#define _IPM_Calibration_H_

#ifdef _DEBUG
#define FILTER_NAME "SD IPM_Calibration Calibration Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.IPM_Calibration_calibration.debug"
#else
#define FILTER_NAME "SD IPM_Calibration Calibration"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.IPM_Calibration_calibration.release"
#endif

#define PROP_NAME_ID "ID"

class IPM_Calibration : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	tInt id;

	// pins
	cVideoPin videoInput;
	cVideoPin IPM_CalibrationVideoOutputPin;
		
	// video stuff
	tBool isFirstFrame;
	tBitmapFormat videoInputInfo;

public:
	IPM_Calibration(const tChar *__info);
	virtual ~IPM_Calibration(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	tResult PropertyChanged(const char *name);
	
private:
	tResult createInputPin(const tChar*, cVideoPin&);
	tResult createVideoOutputPin(const tChar*, cVideoPin&);
	tResult createOutputPin(const tChar *pinName, cOutputPin &pin);
	tVoid setBitmapFormat(const tBitmapFormat*);
	tResult processImage(IMediaSample*);
	tVoid drawGuidelines(Mat &image);
	tResult transmitVideoOutput(Mat&, cVideoPin&);
};

#endif