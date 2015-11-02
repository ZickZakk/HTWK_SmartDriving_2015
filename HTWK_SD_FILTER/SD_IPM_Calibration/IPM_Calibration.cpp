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
#include "stdafx.h"
#include "IPM_Calibration.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, IPM_Calibration);

IPM_Calibration::IPM_Calibration(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
	
	this->id = 1;
	SetPropertyInt(PROP_NAME_ID, this->id);
}

IPM_Calibration::~IPM_Calibration(void)
{}

tResult IPM_Calibration::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	switch (eStage)
	{
	case StageFirst:
		RETURN_IF_FAILED(createInputPin("ipmVideo", this->videoInput));
		RETURN_IF_FAILED(createVideoOutputPin("ipmCalibrationVideo", this->IPM_CalibrationVideoOutputPin));
		break;
		
	case StageNormal:
		break;

	case StageGraphReady:
		break;
	}
	
	RETURN_NOERROR;
}

tResult IPM_Calibration::createInputPin(const tChar *pinName, cVideoPin &pin)
{
	RETURN_IF_FAILED(pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult IPM_Calibration::createVideoOutputPin(const tChar *pinName, cVideoPin &pin)
{
	RETURN_IF_FAILED(pin.Create(pinName, IPin::PD_Output, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult IPM_Calibration::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult IPM_Calibration::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult IPM_Calibration::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult IPM_Calibration::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);

	if (eventCore == IPinEventSink::PE_MediaSampleReceived)
	{
		if (source == &this->videoInput)
		{
			if (this->isFirstFrame)
			{
				//Read media type
				cObjectPtr<IMediaType> type;
				RETURN_IF_FAILED(this->videoInput.GetMediaType(&type));

				cObjectPtr<IMediaTypeVideo> typeVideo;
				RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)(&typeVideo)));

				const tBitmapFormat *format = typeVideo->GetFormat();
				RETURN_IF_POINTER_NULL(format);

				//Set media type
				setBitmapFormat(format);
				this->isFirstFrame = false;
			}
			else {
				RETURN_IF_FAILED(processImage(mediaSample));
			}

		}
	}

	RETURN_NOERROR;
}

tVoid IPM_Calibration::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInputInfo.nPaletteSize = format->nPaletteSize;
	this->videoInputInfo.nPixelFormat = format->nPixelFormat;
	this->videoInputInfo.nHeight = format->nHeight;
	this->videoInputInfo.nWidth = format->nWidth;
	this->videoInputInfo.nSize = format->nSize;

	this->IPM_CalibrationVideoOutputPin.SetFormat(&this->videoInputInfo, NULL);
}

tResult IPM_Calibration::processImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		LOG_INFO(cString::Format("ID %d recieved image", this->id));
		//Receive the image
		Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*)buffer);

		Mat result = image.clone();
		mediaSample->Unlock(buffer);

		drawGuidelines(result);
		
		//Send it to the output pin
		transmitVideoOutput(result, this->IPM_CalibrationVideoOutputPin);
	}
	
	RETURN_NOERROR;
}

tVoid IPM_Calibration::drawGuidelines(Mat &image)
{
	Scalar colorFirst(255, 0, 0);
	Scalar colorSecond(0, 255, 0);
	Scalar colorThird(0, 0, 255);
	Scalar colorFourth(255, 255, 0);

	cv::line(image, cv::Point(0, 181), Point(this->videoInputInfo.nWidth, 181), colorFirst);
	cv::line(image, cv::Point(0, 223), Point(this->videoInputInfo.nWidth, 223), colorFirst);

	cv::line(image, cv::Point(0, 263), Point(this->videoInputInfo.nWidth, 263), colorSecond);
	cv::line(image, cv::Point(0, 312), Point(this->videoInputInfo.nWidth, 312), colorSecond);
	
	cv::line(image, cv::Point(0, 353), Point(this->videoInputInfo.nWidth, 353), colorThird);
	cv::line(image, cv::Point(0, 405), Point(this->videoInputInfo.nWidth, 405), colorThird);
	
	cv::line(image, cv::Point(0, 441), Point(this->videoInputInfo.nWidth, 441), colorFourth);
}

tResult IPM_Calibration::transmitVideoOutput(Mat &image, cVideoPin &pin)
{
	if (!pin.IsConnected())
	{
		RETURN_NOERROR;
	}
	
	if (image.type() != CV_8UC3)
	{
		cvtColor(image, image, CV_GRAY2RGB);
	}

	// Create new IMediaSample
	cObjectPtr<IMediaSample> sample;

	if (IS_OK(AllocMediaSample(&sample)))
	{
		LOG_INFO(cString::Format("ID %d send image", this->id));

		tTimeStamp time = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();
		RETURN_IF_FAILED(sample->Update(time, image.data, this->videoInputInfo.nSize, 0));
		RETURN_IF_FAILED(pin.Transmit(sample));
	}

	RETURN_NOERROR;
}

tResult IPM_Calibration::PropertyChanged(const char *name)
{
	this->id = GetPropertyInt(PROP_NAME_ID);

	RETURN_NOERROR;
}