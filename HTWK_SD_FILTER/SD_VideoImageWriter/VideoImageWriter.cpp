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
#include "VideoImageWriter.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_VIDEO_IMAGE_WRITER, VideoImageWriter);

VideoImageWriter::VideoImageWriter(const char *__info) : cFilter(__info)
{
	this->frameCounter = 1;
	this->isFirstFrame = true;

	initProperties();
}

tVoid VideoImageWriter::initProperties()
{
	// Canny threshold 1
	this->maxFrame = 120;
	SetPropertyInt(PROP_NAME_FRAMES, this->maxFrame);
    SetPropertyInt(PROP_NAME_FRAMES NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_FRAMES NSSUBPROP_MAXIMUM, 500);
    SetPropertyBool(PROP_NAME_FRAMES NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_FRAMES NSSUBPROP_ISCHANGEABLE, tTrue);

	this->isShotRgbImageActive = true;
	SetPropertyBool(PROP_NAME_SHOT_RGB_IMAGE, this->isShotRgbImageActive);
    SetPropertyBool(PROP_NAME_SHOT_RGB_IMAGE NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_SHOT_RGB_IMAGE NSSUBPROP_ISCHANGEABLE, tTrue);
}

VideoImageWriter::~VideoImageWriter(void)
{}

tResult VideoImageWriter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	switch (eStage)
	{
	case StageFirst:
		RETURN_IF_FAILED(createInputPin("videoInput", this->videoInput));
		break;
		
	case StageNormal:
		setThresholds();
		break;

	case StageGraphReady:
        // cv::namedWindow(cv::String("ObstacleDetectionWindow"), cv::WINDOW_NORMAL);
		// cv::startWindowThread();
		break;
	}
	
	RETURN_NOERROR;
}

tResult VideoImageWriter::createInputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult VideoImageWriter::createVideoOutputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Output, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult VideoImageWriter::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult VideoImageWriter::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult VideoImageWriter::Shutdown(tInitStage eStage, __exception)
{
	if (eStage == StageFirst)
	{
		cv::destroyWindow("ObstacleDetectionWindow");
	}

	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult VideoImageWriter::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
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
				this->frameCounter++;

				if (this->maxFrame == this->frameCounter)
				{
					writeImageToHdd(mediaSample);

				}
				// displayImage(mediaSample);
			}

		}
	}

	RETURN_NOERROR;
}

tVoid VideoImageWriter::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInputInfo.nPaletteSize = format->nPaletteSize;
	this->videoInputInfo.nPixelFormat = format->nPixelFormat;
	this->videoInputInfo.nHeight = format->nHeight;
	this->videoInputInfo.nWidth = format->nWidth;
	this->videoInputInfo.nSize = format->nSize;
}

tResult VideoImageWriter::writeImageToHdd(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		//Receive the image
		static Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*)buffer);
		
		imwrite("C:\\Users\\Denny\\Desktop\\rgb.png", image);
		cvtColor(image, image, CV_RGB2GRAY);
		imwrite("C:\\Users\\Denny\\Desktop\\gray.png", image);
		blur(image, image, cv::Size(5,5));
		imwrite("C:\\Users\\Denny\\Desktop\\blur.png", image);
		Canny(image, image, 100.0, 150.0);

		// imwrite("/home/odroid/Desktop/aufnahmen/photo.png", image);
		imwrite("C:\\Users\\Denny\\Desktop\\edge.png", image);

		LOG_INFO("Shot a photo!");
		mediaSample->Unlock(buffer);
	}
	else
	{
		LOG_WARNING("Cant lock image");
	}

	RETURN_NOERROR;
}


tResult VideoImageWriter::displayImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		//Receive the image
		static Mat image;
		image = Mat(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*)buffer);

		cvtColor(image, image, CV_RGB2GRAY);
		blur(image, image, cv::Size(5,5));
		Canny(image, image, 100.0, 150.0);


		if(searchVerticalLine(image, cv::Point(70, 350), 100, 50, 5))
		{
			LOG_INFO("parking parallel");
		}
		else
		{
			LOG_INFO("parking cross");
		}

		if((!searchVerticalLine(image, cv::Point(1, 270), 150, 9, 3))
		    && searchVerticalLine(image, cv::Point(1, 280), 90, 9, 3))
		{
			LOG_INFO("left access");
		}


		cv::imshow(cv::String("ObstacleDetectionWindow"), image);
		imwrite("/home/odroid/Desktop/aufnahmen/parking.png", image);

		mediaSample->Unlock(buffer);
	}

	RETURN_NOERROR;
}

bool VideoImageWriter::searchVerticalLine(const cv::Mat& ipmImage, const cv::Point& topLeftOfSearchArea, int width, int height, int numberOfSamplingPoints)
{
	int distanceSamplingPoints = height / numberOfSamplingPoints;
	int startAtRow = distanceSamplingPoints / 2;

	cv::Rect roi = cv::Rect(topLeftOfSearchArea.x, topLeftOfSearchArea.y, width, height);
	cv::Mat roiIpmImage = ipmImage(roi);

	int foundSamplingPoint = 0;
	for (int row=startAtRow; row<roiIpmImage.rows; row+=distanceSamplingPoints)
	{
		for(int col=0; col<roiIpmImage.cols; col++)
		{
			uchar grayPixel = roiIpmImage.at<uchar>(row, col);

			if (grayPixel>200)
			{
				foundSamplingPoint++;
				break;
			}
			roiIpmImage.at<uchar>(row, col) = 255;
		}
	}

	return (foundSamplingPoint >= numberOfSamplingPoints-1);
}


tResult VideoImageWriter::PropertyChanged(const char *propertyName)
{
	setThresholds();

	RETURN_NOERROR;
}

tVoid VideoImageWriter::setThresholds()
{
	this->maxFrame = GetPropertyInt(PROP_NAME_FRAMES);
	this->isShotRgbImageActive = GetPropertyBool(PROP_NAME_SHOT_RGB_IMAGE);
}
