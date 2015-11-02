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

#include <list>
#include <fstream>
using namespace std;

#include "stdafx.h"
#include "HorizonCalibration.h"
using namespace smartdriving;

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, HorizonCalibration);

HorizonCalibration::HorizonCalibration(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
	this->lineDetection.reset(new HorizonDetector());
	this->maxCameraCalibrationFrames = 30;
	this->maxHorizonCalibrationFrames = 90;
	this->currentFrame = 0;
	this->horizonPosition = 0;

	initProperties();
}

tVoid HorizonCalibration::initProperties()
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

	// Hough threshold
	this->thresholdHough = 75;
	SetPropertyInt(PROP_NAME_HOUGH_THRESHOLD, this->thresholdHough);
    SetPropertyInt(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_MAXIMUM, 255);
    SetPropertyBool(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_HOUGH_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
}

HorizonCalibration::~HorizonCalibration(void)
{}

tResult HorizonCalibration::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	switch (eStage)
	{
	case StageFirst:
		RETURN_IF_FAILED(createVideoInputPin("videoInput", this->videoInput));
		RETURN_IF_FAILED(createVideoOutputPin("horizonVideo", this->horizonVideo));
		break;
		
	case StageNormal:
		setThresholds();
		break;

	case StageGraphReady:
		break;
	}
	
	RETURN_NOERROR;
}

tResult HorizonCalibration::createVideoInputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult HorizonCalibration::createVideoOutputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Output, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult HorizonCalibration::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult HorizonCalibration::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult HorizonCalibration::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult HorizonCalibration::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
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

tVoid HorizonCalibration::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInfo.nPaletteSize = format->nPaletteSize;
	this->videoInfo.nPixelFormat = format->nPixelFormat;
	this->videoInfo.nHeight = format->nHeight;
	this->videoInfo.nWidth = format->nWidth;
	this->videoInfo.nSize = format->nSize;
	
	this->horizonVideo.SetFormat(&this->videoInfo, NULL);
}

tResult HorizonCalibration::processImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (IS_OK(mediaSample->Lock(&buffer)))
	{
		Mat image(Size(this->videoInfo.nWidth, this->videoInfo.nHeight), CV_8UC3, (char*)buffer);
		
		mediaSample->Unlock(buffer);
		
		static Mat result; 
		SD_Line horizon;
		vector<SD_Line> lines;
		list<SD_LineExtendet> leftLines, rightLines;
		
		result = image.clone();
		prepareImage(result);

		detectLines(result, lines);
		searchLinesOfInterest(lines, leftLines, rightLines);
		horizon = detectHorizon(leftLines, rightLines);

		if (this->currentFrame == maxHorizonCalibrationFrames)
		{
			imwrite("realLane.png", image);
			LOG_INFO(cString::Format("Horizon detected at y=%d", getHorizon()));
			saveHorizonToFile();
		}

		drawLinesInImage(image, leftLines);
		drawLinesInImage(image, rightLines);
		drawHorizonInImage(image, horizon);

		transmitVideoOutput(image, this->horizonVideo);
	}
	
	RETURN_NOERROR;
}

tVoid HorizonCalibration::prepareImage(Mat &image)
{
	cvtColor(image, image, CV_RGB2GRAY);
	cv::blur(image, image, cv::Size(5,5));
	Canny(image, image, this->thresholdCanny1, this->thresholdCanny2);
}

tVoid HorizonCalibration::detectLines(const Mat &image, vector<SD_Line> &lines)
{
	this->lineDetection->setImage(image);
	this->lineDetection->transform();
	lines = this->lineDetection->getLines(this->thresholdHough);
}

tVoid HorizonCalibration::searchLinesOfInterest(vector<SD_Line> &lines, list<SD_LineExtendet> &leftLines, list<SD_LineExtendet> &rightLines)
{
	// Grenzwert für das untere Drittel
	tInt heightThreshold = this->videoInfo.nHeight - (this->videoInfo.nHeight / 3);
	tInt widthCenter = this->videoInfo.nWidth / 2;
		
	static SD_Line intersectBottom;
	intersectBottom.x1 = 0;
	intersectBottom.y1 = this->videoInfo.nHeight;
	intersectBottom.x2 = this->videoInfo.nWidth;
	intersectBottom.y2 = this->videoInfo.nHeight;

	static SD_Line intersectLeft;
	intersectLeft.x1 = 0;
	intersectLeft.y1 = heightThreshold;
	intersectLeft.x2 = 0;
	intersectLeft.y2 = this->videoInfo.nHeight;

	static SD_Line intersectRight;
	intersectRight.x1 = this->videoInfo.nWidth;
	intersectRight.y1 = heightThreshold;
	intersectRight.x2 = this->videoInfo.nWidth;
	intersectRight.y2 = this->videoInfo.nHeight;

	for (vector<SD_Line>::iterator line = lines.begin(); line != lines.end(); ++line)
	{
		static SD_Point point;
		static SD_LineExtendet lineOfInterest;

		// Gucken, ob die Linie den Boden schneidet
		if (getLineIntersection(intersectBottom, *line, point))
		{
			if (point.y == this->videoInfo.nHeight && point.x >= 0 && point.x <= 700)
			{
				// prüfen, ob die Linie die linke Bildhälfte schneidet
				if (point.x <= widthCenter)
				{
					lineOfInterest.line = *line;
					lineOfInterest.intersection = point;
					lineOfInterest.videoHeight = this->videoInfo.nHeight;

					leftLines.push_back(lineOfInterest);
				}
				else
				{
					lineOfInterest.line = *line;
					lineOfInterest.intersection = point;
					lineOfInterest.videoHeight = this->videoInfo.nHeight;

					rightLines.push_back(lineOfInterest);
				}
			}
		}
		// Gucken, ob die Linie den linken Rand schneidet
		else if (getLineIntersection(intersectLeft, *line, point))
		{
			if (point.y >= heightThreshold && point.y <= this->videoInfo.nHeight && point.x == 0)
			{
				// Linie der Liste für links hinzufügen
				lineOfInterest.line = *line;
				lineOfInterest.intersection = point;
				lineOfInterest.videoHeight = this->videoInfo.nHeight;

				leftLines.push_back(lineOfInterest);
			}
		}
		else if (getLineIntersection(intersectRight, *line, point))
		{
			if (point.y >= heightThreshold && point.y <= this->videoInfo.nHeight && point.x == this->videoInfo.nWidth)
			{
				// Linie der rechten Liste hinzufügen
				lineOfInterest.line = *line;
				lineOfInterest.intersection = point;
				lineOfInterest.videoHeight = this->videoInfo.nHeight;

				rightLines.push_back(lineOfInterest);
			}
		}
	}

	// Listen sortieren, um die weiterverarbeitung zu vereinfachen
	leftLines.sort(sortLeftList);
	rightLines.sort(sortRightList);
}

tVoid HorizonCalibration::drawLinesInImage(Mat &image, vector<SD_Line> &lines)
{
	for (vector<SD_Line>::iterator it = lines.begin(); it != lines.end(); ++it)
	{
		line(image, cv::Point(it->x1, it->y1), cv::Point(it->x2, it->y2), cv::Scalar(0, 0, 255), 2, 8);
	}
}

tVoid HorizonCalibration::drawLinesInImage(Mat &image, list<SD_LineExtendet> &lines)
{
	for (auto it = lines.begin(); it != lines.end(); ++it)
	{
		line(image, cv::Point(it->line.x1, it->line.y1), cv::Point(it->line.x2, it->line.y2), cv::Scalar(0, 0, 255), 2, 8);
	}
}

tVoid HorizonCalibration::drawHorizonInImage(Mat &image, SD_Line &horizon)
{
	line(image, cv::Point(horizon.x1, horizon.y1), cv::Point(horizon.x2, horizon.y2), cv::Scalar(0, 255, 0), 2, 8);
}

SD_Line HorizonCalibration::detectHorizon(list<SD_LineExtendet> &leftLines, list<SD_LineExtendet> &rightLines)
{
	static SD_Line horizon;
	static SD_Point point;

	for (auto left = leftLines.begin(); left != leftLines.end(); ++left)
	{
		for (auto right = rightLines.begin(); right != rightLines.end(); ++right)
		{
			this->currentFrame++;

			if (getLineIntersection(left->line, right->line, point))
			{
				// Prüfen, ob die Kamera kalibriert ist
				if (this->currentFrame > this->maxCameraCalibrationFrames && this->currentFrame <= this->maxHorizonCalibrationFrames)
				{
					if (isPointInImage(point))
					{
						this->horizons.push_back(point.y);
					}
					else
					{
						break;
					}
				}

				horizon.x1 = 0;
				horizon.y1 = point.y;
				horizon.x2 = this->videoInfo.nWidth;
				horizon.y2 = point.y;

				return horizon;
			}
			else
			{
				horizon.x1 = 0;
				horizon.y1 = this->videoInfo.nHeight;
				horizon.x2 = this->videoInfo.nWidth;
				horizon.y2 = this->videoInfo.nHeight;
				
				return horizon;
			}
		}
	}

	horizon.x1 = 0;
	horizon.y1 = this->videoInfo.nHeight;
	horizon.x2 = this->videoInfo.nWidth;
	horizon.y2 = this->videoInfo.nHeight;

	return horizon;
}

tBool HorizonCalibration::isPointInImage(const SD_Point &point)
{
	return point.x >= 0 && point.x < this->videoInfo.nWidth && point.y >= 0 && point.y < this->videoInfo.nHeight;
}

tInt HorizonCalibration::getHorizon()
{
	for (auto horizon = this->horizons.begin(); horizon != this->horizons.end(); ++horizon)
	{
		this->horizonPosition += *horizon;
	}

	return this->horizonPosition /= this->horizons.size();
}

tVoid HorizonCalibration::saveHorizonToFile()
{
	FILE *horizonSaver;
	horizonSaver = fopen("horizon.txt", "w");
	fprintf(horizonSaver, "%d\n", this->horizonPosition);
	fclose(horizonSaver);
}

tResult HorizonCalibration::transmitVideoOutput(Mat &image, cVideoPin &pin)
{
	if (!pin.IsConnected())
	{
		RETURN_NOERROR;
	}
	
	// erstelle neuen IMediaSample
	cObjectPtr<IMediaSample> sample;

	if (IS_OK(AllocMediaSample(&sample)))
	{
		tTimeStamp time = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();
		RETURN_IF_FAILED(sample->Update(time, image.data, this->videoInfo.nSize, 0));
		RETURN_IF_FAILED(pin.Transmit(sample));
	}

	RETURN_NOERROR;
}

tResult HorizonCalibration::PropertyChanged(const char *propertyName)
{
	setThresholds();

	RETURN_NOERROR;
}

tVoid HorizonCalibration::setThresholds()
{
	this->thresholdCanny1 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_1);
	this->thresholdCanny2 = GetPropertyInt(PROP_NAME_CANNY_THRESHOLD_2);
	this->thresholdHough = GetPropertyInt(PROP_NAME_HOUGH_THRESHOLD);
}
