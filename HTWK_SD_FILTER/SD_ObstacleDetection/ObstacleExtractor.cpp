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
#include "ObstacleExtractor.h"
#include "ObstacleDescription.h"

#include <stack>
#include <iostream>
#include <iomanip>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ObstacleExtractor::ObstacleExtractor()
{
	numberOfSurfaces = 0;
}

std::vector<ObstacleDescription>& ObstacleExtractor::process(const cv::Mat& image)
{
	allocateImageIfNecessary(sourceImage, image.size(), sourceImage.type());
	clearImage(sourceImage);
	image.copyTo(sourceImage);

	allocateImageIfNecessary(labeledImage, sourceImage.size(), sourceImage.type());
	clearImage(labeledImage);

	obstacleDescriptions.clear();

	numberOfSurfaces = 0;
	int obstacleId = 1;
	for (int r=0; r<sourceImage.rows; ++r)
	{
		for (int c=0;c<sourceImage.cols; ++c)
		{
			cv::Point currentPoint(c,r);

			if (!isLabeled(sourceImage, currentPoint)) { continue; }
			if (isLabeled(labeledImage, currentPoint)) { continue; }
			
			ObstacleDescription obstacleDescription(obstacleId);
			floodFill(currentPoint, sourceImage, labeledImage, obstacleDescription);
			obstacleDescriptions.push_back(obstacleDescription);

			obstacleId+=1;
			LOG_INFO(cString::Format("##> obstacle ->"));
		}
	}
	numberOfSurfaces = obstacleId;

	return obstacleDescriptions;
}

void ObstacleExtractor::allocateImageIfNecessary(cv::Mat& image, cv::Size size, int type)
{
	image.create(size, type);
}

void ObstacleExtractor::clearImage(cv::Mat& image)
{
	image = cv::Mat::zeros(image.rows, image.cols, image.type());
}

bool ObstacleExtractor::isLabeled(const cv::Mat& image, const cv::Point& p)
{
	if (image.at<ushort>(p.y, p.x) == 0)
	{
		return false;
	}
	return true;
}

void ObstacleExtractor::floodFill(const cv::Point& start,
								const cv::Mat& sourceImage, cv::Mat& labeledImage,
								ObstacleDescription& obstacleDescription)
{
	unsigned int obstacleId = obstacleDescription.id;

	std::stack<cv::Point> stack;
	stack.push(start);

	while (!stack.empty())
	{
		cv::Point currentPoint = stack.top(); stack.pop();
		
		obstacleDescription.add(currentPoint);

		labeledImage.at<ushort>(currentPoint.y, currentPoint.x) = obstacleId;

		cv::Point topLeft(currentPoint.x-1, currentPoint.y-1);
		checkLabelAndPush(topLeft, sourceImage, labeledImage, stack, obstacleId);

		cv::Point top(currentPoint.x, currentPoint.y-1);
		checkLabelAndPush(top, sourceImage, labeledImage, stack, obstacleId);

		cv::Point topRight(currentPoint.x+1, currentPoint.y-1);
		checkLabelAndPush(topRight, sourceImage, labeledImage, stack, obstacleId);

		cv::Point right(currentPoint.x+1, currentPoint.y);
		checkLabelAndPush(right, sourceImage, labeledImage, stack, obstacleId);

		cv::Point left(currentPoint.x-1, currentPoint.y);
		checkLabelAndPush(left, sourceImage, labeledImage, stack, obstacleId);

		cv::Point bottomLeft(currentPoint.x-1, currentPoint.y+1);
		checkLabelAndPush(bottomLeft, sourceImage, labeledImage, stack, obstacleId);

		cv::Point bottom(currentPoint.x, currentPoint.y+1);
		checkLabelAndPush(bottom, sourceImage, labeledImage, stack, obstacleId);

		cv::Point bottomRight(currentPoint.x+1, currentPoint.y+1);
		checkLabelAndPush(bottomRight, sourceImage, labeledImage, stack, obstacleId);
	}
}

void ObstacleExtractor::checkLabelAndPush(const cv::Point& point,
									     const cv::Mat& sourceImage, cv::Mat& labeledImage,
										 std::stack<cv::Point>& stack,
										 int obstacleId)
{
	if (isLabeled(sourceImage, point)
			&& !isLabeled(labeledImage, point)
			&& isInImage(sourceImage, point))
	{
		stack.push(point);
		labeledImage.at<ushort>(point.y, point.x) = obstacleId;
	}
}

bool ObstacleExtractor::isInImage(const cv::Mat& image, const cv::Point& p)
{
	return ((p.y < image.rows && p.y >= 0) && (p.x < image.cols && p.x >= 0));
}

unsigned int ObstacleExtractor::getNumberOfSurfaces()
{
	return numberOfSurfaces;
}

cv::Mat& ObstacleExtractor::getLabeledImage()
{
	return labeledImage;
}
