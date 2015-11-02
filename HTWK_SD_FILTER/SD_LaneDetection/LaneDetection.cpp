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
#include "LaneDetection.h"
#include <opencv2/imgproc/types_c.h>
using namespace cv;

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, LaneDetection);

LaneDetection::LaneDetection(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
	// this->hough.resetStrategie(SUBSAMPLING_IMAGE_WIDTH);
	
	initProperties();
}

tVoid LaneDetection::initProperties()
{
	// Hough algorithm
	this->houghAlgorithm = 1;
	SetPropertyInt(PROP_NAME_HOUGH_ALGORITHM, this->houghAlgorithm);
	SetPropertyStr(PROP_NAME_HOUGH_ALGORITHM NSSUBPROP_VALUELISTNOEDIT, "1@Default|2@FAST|3@OpenCvWrapper");
	SetPropertyStr(PROP_NAME_HOUGH_ALGORITHM NSSUBPROP_DESCRIPTION, "Choose a hough transform algorithm");
	SetPropertyBool(PROP_NAME_HOUGH_ALGORITHM NSSUBPROP_ISCHANGEABLE, tTrue);

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

	// Hough threshold
	this->thresholdHough = 90;
	SetPropertyInt(PROP_NAME_HOUGH_THRESHOLD, 90);
    SetPropertyInt(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_MAXIMUM, 255);
    SetPropertyBool(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
}

LaneDetection::~LaneDetection(void)
{}

tResult LaneDetection::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	switch (eStage)
	{
	case StageFirst:
		// video in
		RETURN_IF_FAILED(createInputPin("videoInput", this->videoInput));
		
		// glc out
		RETURN_IF_FAILED(createOutputPin("glcOutput", this->glcOutput));

		// video output pins
		RETURN_IF_FAILED(createVideoOutputPin("edgeVideoOutput", this->edgdeOuputPin));
		RETURN_IF_FAILED(createVideoOutputPin("binaryVideouOutput", this->binaryOuputPin));
		RETURN_IF_FAILED(createVideoOutputPin("lanesOutputVideo", this->linesOutputPin));
		break;
		
	case StageNormal:
		setThresholds();
		setHoughAlgorithm();
		break;

	case StageGraphReady:
		break;
	}
	
	RETURN_NOERROR;
}

tResult LaneDetection::createInputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult LaneDetection::createOutputPin(const tChar *pinName, cOutputPin &pin)
{
	cObjectPtr<IMediaType> typeStruct;
	RETURN_IF_FAILED(AllocMediaType((tVoid**) &typeStruct, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, NULL));
	RETURN_IF_FAILED(pin.Create(pinName, typeStruct, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult LaneDetection::createVideoOutputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Output, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult LaneDetection::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult LaneDetection::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult LaneDetection::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult LaneDetection::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
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

tVoid LaneDetection::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInfo.nPaletteSize = format->nPaletteSize;
	this->videoInfo.nPixelFormat = format->nPixelFormat;
	this->videoInfo.nHeight = format->nHeight;
	this->videoInfo.nWidth = format->nWidth;
	this->videoInfo.nSize = format->nSize;

	// Set input and output format
	this->edgdeOuputPin.SetFormat(&this->videoInfo, NULL);
	this->binaryOuputPin.SetFormat(&this->videoInfo, NULL);
	this->linesOutputPin.SetFormat(&this->videoInfo, NULL);
}

tResult LaneDetection::processImage(IMediaSample *mediaSample)
{
	cObjectPtr<IMediaSample> rgbImage;
	const tVoid *buffer;

	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		Mat image(Size(this->videoInfo.nWidth, this->videoInfo.nHeight), CV_8UC3, (char*)buffer);
		mediaSample->Unlock(buffer);
		
		Mat resultImage(Size(this->videoInfo.nWidth, this->videoInfo.nHeight), CV_8UC3, (char*)buffer);
		// prepare image with blur and canny
		prepareImage(resultImage);
		
		auto startTime = cHighResTimer::GetTime();
		// detect lanes
		vector<SD_Line> lines;
		this->hough.getLines(resultImage, this->thresholdHough, lines);
	
		auto endTime = cHighResTimer::GetTime(); 
		LOG_INFO(cString::Format("time for lane detection: %lldms", (endTime - startTime) / 1000)); // convert time from µs to ms
		LOG_INFO(cString::Format("detected %d lines", lines.size()));
		
		// draw the lines in an mat image
		if (this->linesOutputPin.IsConnected())
		{
			drawLinesInImage(image, lines);
			transmitVideoOutput(image, this->linesOutputPin);
		}

		if (this->glcOutput.IsConnected())
		{
			return createAndTransmitGCL(lines);
		}
	}
	
	RETURN_NOERROR;
}

tVoid LaneDetection::prepareImage(Mat &image)
{
	cvtColor(image, image, CV_RGB2GRAY);
	// threshold wird nicht verwendet, da das rauschen zu stark ist
	// threshold(image, image, this->thresholdBinary, 255.0, 0);
	// blur, damit das Rauschen im Bild geringer ist
	cv::blur(image, image, cv::Size(5,5));
	transmitVideoOutput(image, this->binaryOuputPin);

	Canny(image, image, this->thresholdCanny1, this->thresholdCanny2);
	transmitVideoOutput(image, this->edgdeOuputPin);
}

void LaneDetection::drawLinesInImage(Mat &image, vector<SD_Line> &lines)
{
	for (vector<SD_Line>::iterator it = lines.begin(); it != lines.end(); ++it)
	{
		line(image, cv::Point(it->x1, it->y1), cv::Point(it->x2, it->y2), cv::Scalar(0, 0, 255), 2, 8);
	}
}

tResult LaneDetection::transmitVideoOutput(Mat &image, cVideoPin &pin)
{
	if (!pin.IsConnected())
	{
		RETURN_NOERROR;
	}
	
	// Das Bild muss zurück in ein RGB-Bild konvertiert werden, da sonst das VideoDisplay nichts anzeigt
	if (cString::Compare(pin.GetName(), "lanesOutputVideo"))
	{
		cvtColor(image, image, CV_GRAY2RGB);
	}
	
	// erstelle neuen IMediaSample
	cObjectPtr<IMediaSample> sample;
	
	if (IS_OK(AllocMediaSample(&sample)))
	{
		tTimeStamp time = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();
		RETURN_IF_FAILED(sample->Update(time, image.data, this->videoInfo.nWidth * this->videoInfo.nHeight * 3, 0));
		RETURN_IF_FAILED(pin.Transmit(sample));
	}

	RETURN_NOERROR;
}

tResult LaneDetection::createAndTransmitGCL(vector<SD_Line> &lines)
{
	cObjectPtr<IMediaSample> sample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&sample));
	RETURN_IF_FAILED(sample->AllocBuffer(2 * 8192));

	sample->SetTime(_clock->GetStreamTime());
	tUInt32 *brush;
	RETURN_IF_FAILED(sample->WriteLock((tVoid**) &brush));

	// Farbe für Linie einstellen
	// FGCOL = ForeGround COLor
	cGCLWriter::StoreCommand(brush, GCL_CMD_FGCOL, cColor(0, 0, 255).GetRGBA());

	for (vector<SD_Line>::iterator it = lines.begin(); it != lines.end(); ++it)
	{
		static tResult errorCode;
		LOG_INFO(cString::Format("Koordinaten: (%d, %d), (%d, %d)", it->x1, it->y1, it->x2, it->y2));
		
		if (IS_FAILED(errorCode = cGCLWriter::StoreCommand(brush, GCL_CMD_DRAWLINE, it->x1, it->y1, it->x2, it->y2)))
		{
			LOG_ERROR(cString::Format("Fehler beim zeichnen der Line %d\n"));
			cGCLWriter::StoreCommand(brush, GCL_CMD_END);
			sample->Unlock(brush);

			RETURN_ERROR(errorCode);
		}
	}

	cGCLWriter::StoreCommand(brush, GCL_CMD_END);
	sample->Unlock(brush);

	RETURN_IF_FAILED(this->glcOutput.Transmit(sample));
	RETURN_NOERROR;
}

tResult LaneDetection::PropertyChanged(const char *propertyName)
{
	setThresholds();

	if (cString::Compare(propertyName, PROP_NAME_HOUGH_ALGORITHM))
	{
#ifdef _DEBUG
		LOG_INFO("changed hough algorithm");
#endif
		setHoughAlgorithm();
	}

	RETURN_NOERROR;
}

tVoid LaneDetection::setThresholds()
{
	this->thresholdCanny1 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_1);
	this->thresholdCanny2 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_2);
	this->thresholdHough = GetPropertyInt(PROP_NAME_HOUGH_THRESHOLD);
}

tVoid LaneDetection::setHoughAlgorithm()
{
	this->houghAlgorithm = GetPropertyInt(PROP_NAME_HOUGH_ALGORITHM);

	switch (this->houghAlgorithm)
	{
	case 1:
		this->hough.resetStrategie(HOUGH_DEFAULT);
		break;

	case 2:
		this->hough.resetStrategie(HOUGH_FAST);
		break;

	case 3:
		this->hough.resetStrategie(HOUGH_OPENCV);
		break;
	}
}