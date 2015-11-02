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

#include "stdafx.h"
#include "CurveDetector.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, CurveDetector);

CurveDetector::CurveDetector(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
	initProperties();
}

tVoid CurveDetector::initProperties()
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

CurveDetector::~CurveDetector(void)
{
}

tResult CurveDetector::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	switch (eStage)
	{
	case StageFirst:
		RETURN_IF_FAILED(createVideoInputPin("videoInput", this->videoInput));
		RETURN_IF_FAILED(createVideoOutputPin("rgbVideo", this->rgbOutput));
		break;
		
	case StageNormal:
		setThresholds();
		break;

	case StageGraphReady:
		break;
	}
	
	RETURN_NOERROR;
}

tResult CurveDetector::createVideoInputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult CurveDetector::createVideoOutputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Output, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult CurveDetector::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult CurveDetector::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult CurveDetector::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult CurveDetector::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);

	if (eventCore == IPinEventSink::PE_MediaSampleReceived)
	{
		if (source == &this->videoInput)
		{
			if (this->isFirstFrame)
			{
				cObjectPtr<IMediaType> type;
				RETURN_IF_FAILED(this->videoInput.GetMediaType(&type));

				cObjectPtr<IMediaTypeVideo> typeVideo;
				RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)(&typeVideo)));

				const tBitmapFormat *format = typeVideo->GetFormat();
				RETURN_IF_POINTER_NULL(format);

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

tResult CurveDetector::PropertyChanged(const char *propertyName)
{
	setThresholds();

	RETURN_NOERROR;
}

tVoid CurveDetector::setThresholds()
{
	this->thresholdCanny1 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_1);
	this->thresholdCanny2 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_2);
}

tVoid CurveDetector::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInputInfo.nPaletteSize = format->nPaletteSize;
	this->videoInputInfo.nPixelFormat = format->nPixelFormat;
	this->videoInputInfo.nHeight = format->nHeight;
	this->videoInputInfo.nWidth = format->nWidth;
	this->videoInputInfo.nSize = format->nSize;

	this->rgbOutput.SetFormat(format, NULL);
}

tResult CurveDetector::processImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*)buffer);
		mediaSample->Unlock(buffer);

		Mat result = image.clone();
		prepareImage(result);

		list<SD_Point> left, right;
		searchLines(result, left, right);

		drawPointsInImage(result, left, Scalar(0, 0, 255));
		drawPointsInImage(result, right, Scalar(0, 0, 255));

		transmitVideoOutput(result, this->rgbOutput);
	}
	
	RETURN_NOERROR;
}

tVoid CurveDetector::prepareImage(Mat &image)
{
	cvtColor(image, image, CV_RGB2GRAY);
	blur(image, image, cv::Size(5,5));
	Canny(image, image, this->thresholdCanny1, this->thresholdCanny2);
}

tVoid CurveDetector::searchLines(const Mat &image, list<SD_Point> &leftLines, list<SD_Point> &rightLines)
{
	tUInt center = this->videoInputInfo.nWidth / 2;
	tUInt left = NO_POINT;
	tUInt right = NO_POINT;
	SD_Point point;

	// das gesamte Bild durchsuchen
	for (tUInt height = this->videoInputInfo.nHeight - 1; height > 0; height--)
	{
		getLeftLane(image, center, height, left);
		getRightLane(image, center, height, right);

		point.x = left;
		point.y = height;
		addPointToList(point, leftLines);

		point.x = right;
		addPointToList(point, rightLines);

		if (left != NO_POINT && right != NO_POINT)
		{
			center = (right - left) / 2;
		}
	}
}

tVoid CurveDetector::getLeftLane(const Mat &image, const tUInt &center, const tUInt &height, tUInt &left)
{
	// linke Bildhälfte durchsuchen
	for (tInt leftSide = center; leftSide > 0; leftSide--)
	{

#ifdef _DEBUG
		int test = static_cast<tUInt>(image.at<uchar>(height, leftSide));
		
		if (test > 0)
		{
			LOG_INFO(cString::Format("%d", test));
		}
#endif

		if (static_cast<tUInt>(image.at<uchar>(height, leftSide)) == 255)
		{
			left = leftSide;
			return;
		}
	}

	left = NO_POINT;
}

tVoid CurveDetector::getRightLane(const Mat &image, const tUInt &center, const tUInt &height, tUInt &right)
{
	// rechte Bildhälfte durchsuchen
	for (tInt rightSide = center; rightSide < this->videoInputInfo.nWidth; rightSide++)
	{
		if (static_cast<tUInt>(image.at<uchar>(height, rightSide)) == 255)
		{
			right = rightSide;
			return;
		}
	}

	right = NO_POINT;
}

tVoid CurveDetector::addPointToList(const SD_Point &point, list<SD_Point> &list)
{
	if (point.x != NO_POINT)
	{
		list.push_front(point);
	}
}

tVoid CurveDetector::drawPointsInImage(Mat &image, list<SD_Point> &list, const Scalar &color)
{
	if (image.type() != CV_8UC3)
	{
		cvtColor(image, image, CV_GRAY2RGB);
	}

	for (auto point = list.begin(); point != list.end(); ++point)
	{
		circle(image, Point(point->x, point->y), 1, Scalar(255, 0, 0));
	}
}

tResult CurveDetector::transmitVideoOutput(Mat &image, cVideoPin &pin)
{
	if (!pin.IsConnected())
	{
		RETURN_NOERROR;
	}
	
	if (image.type() != CV_8UC3)
	{
		cvtColor(image, image, CV_GRAY2RGB);
	}

	cObjectPtr<IMediaSample> sample;

	if (IS_OK(AllocMediaSample(&sample)))
	{
		tTimeStamp time = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();
		RETURN_IF_FAILED(sample->Update(time, image.data, this->videoInputInfo.nSize, 0));
		RETURN_IF_FAILED(pin.Transmit(sample));
	}

	RETURN_NOERROR;
}
