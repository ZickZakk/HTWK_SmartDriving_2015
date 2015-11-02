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
#ifndef _SteerAngleChecker_H_
#define _SteerAngleChecker_H_

#ifdef _DEBUG
#define FILTER_NAME "SD SteerAngleChecker Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.SteerAngleChecker_calibration.debug"
#else
#define FILTER_NAME "SD SteerAngleChecker"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.SteerAngleChecker_calibration.release"
#endif

class SteerAngleChecker : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// pins
	cInputPin steerAngleInput;

	// Coder description for input pins
	cObjectPtr<IMediaTypeDescription> steerAngleMediaDescription;
	cObjectPtr<IMediaDescriptionManager> descriptionManager;

public:
	SteerAngleChecker(const tChar *__info);
	virtual ~SteerAngleChecker(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	
private:
	tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);
	tResult createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	tResult getValue(IMediaSample *mediaSample, tInt &distance) const;
	tResult getValue(IMediaSample *mediaSample, tUInt16 &value) const;
};

#endif