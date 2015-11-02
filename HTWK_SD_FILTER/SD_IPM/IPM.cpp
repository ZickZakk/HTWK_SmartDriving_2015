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
#include "IPM.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, IPM);

IPM::IPM(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
	initProperties();
}

tVoid IPM::initProperties()
{
	// Canny threshold 1
	this->thresholdCanny1 = 100;
	SetPropertyInt(PROP_NAME_CANNY_THRESHOLD_1, this->thresholdCanny1);
    SetPropertyInt(PROP_NAME_CANNY_THRESHOLD_1 NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_CANNY_THRESHOLD_1 NSSUBPROP_MAXIMUM, 255);
    SetPropertyBool(PROP_NAME_CANNY_THRESHOLD_1 NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_CANNY_THRESHOLD_1 NSSUBPROP_ISCHANGEABLE, tTrue);
	
	// Canny threshold 2
	this->thresholdCanny2 = 150;
	SetPropertyInt(PROP_NAME_CANNY_THRESHOLD_2, this->thresholdCanny2);
    SetPropertyInt(PROP_NAME_CANNY_THRESHOLD_2 NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_CANNY_THRESHOLD_2 NSSUBPROP_MAXIMUM, 255);
    SetPropertyBool(PROP_NAME_CANNY_THRESHOLD_2 NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_CANNY_THRESHOLD_2 NSSUBPROP_ISCHANGEABLE, tTrue);
}

IPM::~IPM(void)
{}

tResult IPM::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	switch (eStage)
	{
	case StageFirst:
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createInputPin("videoInput", this->videoInput), "cant create video input");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createVideoOutputPin("ipmVideo", this->ipmVideoOutputPin), "cant create ipm out");
		break;
		
	case StageNormal:
		setThresholds();
		break;

	case StageGraphReady:
		break;
	}
	
	RETURN_NOERROR;
}

tResult IPM::createInputPin(const tChar *pinName, cVideoPin &pin)
{
	RETURN_IF_FAILED(pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult IPM::createVideoOutputPin(const tChar *pinName, cVideoPin &pin)
{
	RETURN_IF_FAILED(pin.Create(pinName, IPin::PD_Output));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult IPM::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult IPM::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult IPM::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult IPM::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
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

tVoid IPM::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInputInfo.nPaletteSize = format->nPaletteSize;
	this->videoInputInfo.nPixelFormat = format->nPixelFormat;
	this->videoInputInfo.nHeight = format->nHeight;
	this->videoInputInfo.nWidth = format->nWidth;
	this->videoInputInfo.nSize = format->nSize;

	this->ipmVideoOutputPin.SetFormat(&this->videoInputInfo, NULL);

	// Stützstellen für IPM
	// top (left, right)
	this->inputQuad[0] = Point2f(170.0f, 250.0f);
	this->inputQuad[1] = Point2f(368.0f, 250.0f);

	// bottom (left, right)
	this->inputQuad[2] = Point2f(0.0f, 300.0f);
	this->inputQuad[3] = Point2f(static_cast<float>(this->videoInputInfo.nWidth), static_cast<float>(this->videoInputInfo.nHeight));
	
	const tFloat32 MARGIN = 100.0f;

	// top (left, right)
	this->outputQuad[0] = Point2f(MARGIN, 50.0f);
	this->outputQuad[1] = Point2f(static_cast<float>(this->videoInputInfo.nWidth) - MARGIN, 50.0f);

	// bottom (left, right)
	this->outputQuad[2] = Point2f(MARGIN, 310.0f);
	this->outputQuad[3] = Point2f(static_cast<float>(this->videoInputInfo.nWidth) - MARGIN, static_cast<float>(this->videoInputInfo.nHeight - 25.0f));
	
	// this->perspective = Mat(2, 4, CV_8UC3);
	this->perspective = getPerspectiveTransform(this->inputQuad, this->outputQuad);
}

tResult IPM::processImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;
	static Mat image;
	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		//Receive the image
		image = Mat(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*)buffer).clone();
		mediaSample->Unlock(buffer);
		
		prepareImage(image);

		Mat ipm;
		warpPerspective(image, ipm, this->perspective, image.size(), INTER_LINEAR);// INTER_LANCZOS4);
		
		//Send it to the output pin
		transmitVideoOutput(ipm, this->ipmVideoOutputPin);
	}
	
	RETURN_NOERROR;
}

tVoid IPM::prepareImage(Mat &image)
{
	cvtColor(image, image, CV_RGB2GRAY);
	blur(image, image, cv::Size(5,5));
	Canny(image, image, this->thresholdCanny1, this->thresholdCanny2);
}

tResult IPM::transmitVideoOutput(Mat &image, cVideoPin &pin)
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
		tTimeStamp time = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();
		RETURN_IF_FAILED(sample->Update(time, image.data, this->videoInputInfo.nSize, 0));
		RETURN_IF_FAILED(pin.Transmit(sample));
	}

	RETURN_NOERROR;
}

tResult IPM::PropertyChanged(const char *propertyName)
{
	setThresholds();

	RETURN_NOERROR;
}

tVoid IPM::setThresholds()
{
	this->thresholdCanny1 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_1);
	this->thresholdCanny2 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_2);
}