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

CrossRoadDetector::CrossRoadDetector()
{
	initExpectedRoadMarkPositions();
}

CrossRoadDetector::~CrossRoadDetector()
{

}

void CrossRoadDetector::initExpectedRoadMarkPositions()
{
	stopLinePostion = cv::Point(250, 459);

	minYOppositeAccess = 180;
	leftMinRoadEdgeOppositeAccess = cv::Point(110, minYOppositeAccess);
	medianStripMinOppositeAccess = cv::Point(320, minYOppositeAccess);
	rightMinRoadEdgeOppositeAccess = cv::Point(530, minYOppositeAccess);

	minXLeftAccess = 15;
	minXRightAccess = 400;

	topMinYRoadEdgeSideAccess = 287;
	medianStripMinYSideAccess = 325;
	bottomMinYRoadEdgeSideAccess = 400;
}

bool CrossRoadDetector::searchStopLine(const cv::Mat &ipmImage)
{
	return searchHorizontalLine(ipmImage, stopLinePostion, 25, 20);
}

bool CrossRoadDetector::hasOppositeAccess(const cv::Mat& ipmImage)
{
	int numberOfLinesDetected = 0;
	if (searchVerticalLine(ipmImage, leftMinRoadEdgeOppositeAccess))
	{
		numberOfLinesDetected++;
	}
	if (searchVerticalLine(ipmImage, medianStripMinOppositeAccess))
	{
		numberOfLinesDetected++;
	}
	if (searchVerticalLine(ipmImage, rightMinRoadEdgeOppositeAccess))
	{
		numberOfLinesDetected++;
	}

	return (numberOfLinesDetected >= 2);
}


bool CrossRoadDetector::hasRightAccess(const cv::Mat& ipmImage)
{
	return hasVerticalAccess(ipmImage, minXRightAccess);
}


bool CrossRoadDetector::hasLeftAccess(const cv::Mat& ipmImage)
{
	if((!searchVerticalLine(ipmImage, cv::Point(1, 270), 150, 9, 3))
	    && searchVerticalLine(ipmImage, cv::Point(1, 280), 90, 9, 3))
	{
		return true;
	}
	
	return false;
}


bool CrossRoadDetector::hasVerticalAccess(const cv::Mat& ipmImage, int minx)
{
	cv::Point roadEdgeTopMin(minx, topMinYRoadEdgeSideAccess);
	cv::Point medianStripMin(minx, medianStripMinYSideAccess);
	cv::Point roadEdgeBottom(minx, bottomMinYRoadEdgeSideAccess);

	int numberOfLinesDetected = 0;
	if (searchHorizontalLine(ipmImage, roadEdgeTopMin, 35, 10))
	{
		numberOfLinesDetected++;
	}
	if (searchHorizontalLine(ipmImage, medianStripMin))
	{
		numberOfLinesDetected++;
	}
	if (searchHorizontalLine(ipmImage, roadEdgeBottom))
	{
		numberOfLinesDetected++;
	}

	return (numberOfLinesDetected >= 2);

}


bool CrossRoadDetector::searchHorizontalLine(const cv::Mat &ipmImage, const cv::Point& topLeftOfSearchArea, int width, int height, int numberOfSamplingPoints)
{
	int distanceSamplingPoints = width / numberOfSamplingPoints;
	int startAtCol = distanceSamplingPoints / 2;

	cv::Rect roi = cv::Rect(topLeftOfSearchArea.x, topLeftOfSearchArea.y, width, height);
	cv::Mat roiIpmImage = ipmImage(roi);

	int foundSamplingPoint = 0;
	for (int col=startAtCol; col<roiIpmImage.cols; col+=distanceSamplingPoints)
	{
		for(int row = roiIpmImage.rows - 1; row > 0; row--)
		{
			static int pixel;
			pixel = roiIpmImage.at<uchar>(row, col);

			if (isWhite(pixel))
			{
				foundSamplingPoint++;
				break;
			}
			// roiIpmImage.at<cv::Vec3b>(row, col)[2] = 0xff;
		}
	}

	return (foundSamplingPoint >= numberOfSamplingPoints-1);
}


bool CrossRoadDetector::searchVerticalLine(const cv::Mat& ipmImage, const cv::Point& topLeftOfSearchArea, int width, int height, int numberOfSamplingPoints)
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

			if (grayPixel > 200)
			{
				foundSamplingPoint++;
				break;
			}
		}
	}

	return (foundSamplingPoint >= numberOfSamplingPoints-1);
}


bool CrossRoadDetector::isWhite(const cv::Vec3b colorPixel, int thresholdWhite)
{
	return colorPixel[0] > thresholdWhite
		&& colorPixel[1] > thresholdWhite
		&& colorPixel[2] > thresholdWhite;
}

bool CrossRoadDetector::isWhite(const int &pixel, int thresholdWhite)
{
	return pixel > thresholdWhite;
}
