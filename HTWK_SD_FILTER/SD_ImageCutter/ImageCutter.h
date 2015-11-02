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
#define FILTER_NAME "SD ImageCutter Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.ImageCutter.debug"
#else
#define FILTER_NAME "SD ImageCutter Release"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.ImageCutter.release"
#endif

class ImageCutter : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// pins
	cVideoPin videoInput;
	cVideoPin cuttedVideoOutputPin;
		
	// video stuff
	tBool isFirstFrame;
	tBitmapFormat videoInputInfo;
	tBitmapFormat videoOutputInfo;

public:
	ImageCutter(const tChar *__info);
	virtual ~ImageCutter(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	
private:
	tResult createInputPin(const tChar*, cVideoPin&);
	tResult createVideoOutputPin(const tChar*, cVideoPin&);
	tVoid setBitmapFormat(const tBitmapFormat*);

	Mat extractROI(const Mat& inImage);

	tResult processImage(IMediaSample*);
	tResult transmitVideoOutput(Mat&, cVideoPin&);
};

#endif //_IMAGE_CUTTER_H_