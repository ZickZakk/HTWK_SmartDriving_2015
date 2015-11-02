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
#include "ImageCutter.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, ImageCutter);

ImageCutter::ImageCutter(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
}

ImageCutter::~ImageCutter(void)
{}

tResult ImageCutter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	switch (eStage)
	{
	case StageFirst:
		RETURN_IF_FAILED(createInputPin("videoInput", this->videoInput));
		RETURN_IF_FAILED(createVideoOutputPin("cuttedVideo", this->cuttedVideoOutputPin));
		break;
		
	case StageNormal:
		break;

	case StageGraphReady:
		break;
	}
	
	RETURN_NOERROR;
}

tResult ImageCutter::createInputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult ImageCutter::createVideoOutputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Output, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult ImageCutter::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult ImageCutter::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult ImageCutter::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult ImageCutter::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
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

tVoid ImageCutter::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInputInfo.nPaletteSize = format->nPaletteSize;
	this->videoInputInfo.nPixelFormat = format->nPixelFormat;
	this->videoInputInfo.nHeight = format->nHeight;
	this->videoInputInfo.nWidth = format->nWidth;
	this->videoInputInfo.nSize = format->nSize;

	// Output format
	this->videoOutputInfo = this->videoInputInfo;
	this->videoOutputInfo.nWidth = 640;
	this->videoOutputInfo.nHeight = 240;
	// just needed the half size, because the height has 240px instead of 480px
	this->videoOutputInfo.nSize /= 2;

	this->cuttedVideoOutputPin.SetFormat(&this->videoOutputInfo, NULL);
}

tResult ImageCutter::processImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		//Receive the image
		Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*)buffer);
		
		mediaSample->Unlock(buffer);

		static Mat cuttedImage;
		//Extract region of interest
		cuttedImage = image(Range(240, this->videoInputInfo.nHeight), Range(0, this->videoInputInfo.nWidth)).clone();
		// extractROI(image);
		
		//Send it to the output pin
		transmitVideoOutput(cuttedImage, this->cuttedVideoOutputPin);
	}
	
	RETURN_NOERROR;
}

Mat ImageCutter::extractROI(const Mat& inImage){
	Mat outImage(240, 640, inImage.type());

	for (tInt row = 240, rowOutImage = 0; row < 480 /*&& rowOutImage < 240*/; row++, rowOutImage++)
	{
		for (tInt col = 0; col < 640; col++)
		{
			outImage.at<Vec3b>(rowOutImage, col) = inImage.at<Vec3b>(row, col);
		}
	}

	return outImage;
}

tResult ImageCutter::transmitVideoOutput(Mat &image, cVideoPin &pin)
{
	if (!pin.IsConnected())
	{
		RETURN_NOERROR;
	}
	
	// Create new IMediaSample
	cObjectPtr<IMediaSample> sample;

	if (IS_OK(AllocMediaSample(&sample)))
	{
		tTimeStamp time = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();
		RETURN_IF_FAILED(sample->Update(time, image.data, this->videoOutputInfo.nSize, 0));
		RETURN_IF_FAILED(pin.Transmit(sample));
	}

	RETURN_NOERROR;
}
