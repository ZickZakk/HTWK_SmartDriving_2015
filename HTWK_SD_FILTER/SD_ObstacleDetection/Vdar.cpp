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
#include "Vdar.h"

#include "cmath"

Vdar::~Vdar()
{

}

void Vdar::setReferenceBackground(const cv::Mat& referenceBackground)
{
	groundRemover.setBackground(referenceBackground);
}


bool Vdar::isObstacleAtPosInDepthImg(const cv::Point& point, const cv::Mat& depthImage)
{
	extractBoundRects(depthImage);

	for (int i=0; i<(int)boundRects.size(); i++)
	{
		if ((point.x >= boundRects[i].tl().x) && (point.y >= boundRects[i].tl().y)
			&& (point.x <= boundRects[i].br().x) && (point.y <= boundRects[i].br().y))
		{
			return true;
		}
	}

	return false;
}


bool Vdar::isObstacleInAreaInDepthImg(const cv::Rect& searchArea, const cv::Mat& depthImage)
{
	extractBoundRects(depthImage);

	cv::Rect area(cv::Point((320+searchArea.tl().x), 480-searchArea.tl().y),
				  cv::Point((320+searchArea.br().x), 480-searchArea.br().y));

	for (int i=0; i<(int)boundRects.size(); i++)
	{
		// if (RectA.X1 < RectB.X2 && RectA.X2 > RectB.X1
		//	   && RectA.Y1 < RectB.Y2 && RectA.Y2 > RectB.Y1) 
		if (area.tl().x < boundRects[i].br().x && area.br().x > boundRects[i].tl().x
		 && area.tl().y < boundRects[i].br().y && area.br().y > boundRects[i].tl().y)
		{
			return true;
		}
	}

	return false;
}


void Vdar::getObstaclesInAreaInDepthImg(const cv::Rect& searchArea, const cv::Mat& depthImage, std::vector<cv::Rect>& obstacles)
{
	obstacles.clear();
	extractBoundRects(depthImage);

	cv::Rect area(cv::Point((320+searchArea.tl().x), 480-searchArea.tl().y),
				  cv::Point((320+searchArea.br().x), 480-searchArea.br().y));

	for (int i=0; i<(int)boundRects.size(); i++)
	{
		// if (RectA.X1 < RectB.X2 && RectA.X2 > RectB.X1
		//	   && RectA.Y1 < RectB.Y2 && RectA.Y2 > RectB.Y1) 
		if (area.tl().x < boundRects[i].br().x && area.br().x > boundRects[i].tl().x
		 && area.tl().y < boundRects[i].br().y && area.br().y > boundRects[i].tl().y)
		{
			cv::Rect boundRectWorld(cv::Point(boundRects[i].tl().x - 320, 480-boundRects[i].tl().y),
									cv::Point(boundRects[i].br().x - 320, 480-boundRects[i].br().y));
			obstacles.push_back(boundRectWorld);	
		}
	}
}


void Vdar::getPositionsOfObstaclesInAreaInDepthImg(const cv::Rect& searchArea, const cv::Mat& depthImage, std::vector<cv::Point>& positions)
{
	positions.clear();
	extractBoundRects(depthImage);

	cv::Rect area(cv::Point((320+searchArea.tl().x), 480-searchArea.tl().y),
				  cv::Point((320+searchArea.br().x), 480-searchArea.br().y));

	for (int i=0; i<(int)boundRects.size(); i++)
	{
		// if (RectA.X1 < RectB.X2 && RectA.X2 > RectB.X1
		//	   && RectA.Y1 < RectB.Y2 && RectA.Y2 > RectB.Y1) 
		if (area.tl().x < boundRects[i].br().x && area.br().x > boundRects[i].tl().x
		 && area.tl().y < boundRects[i].br().y && area.br().y > boundRects[i].tl().y)
		{
			cv::Point obstaclePosition(((boundRects[i].br().x-320) + (boundRects[i].tl().x-320))/2, 480 - boundRects[i].br().y);
			positions.push_back(obstaclePosition);
		}
	}
}


void Vdar::getDistancesOfObstaclesInAreaInDepthImg(const cv::Rect& searchArea, const cv::Mat& depthImage, std::vector<int>& distances)
{
	distances.clear();
	extractBoundRects(depthImage);

	cv::Rect area(cv::Point((320+searchArea.tl().x), 480-searchArea.tl().y),
				  cv::Point((320+searchArea.br().x), 480-searchArea.br().y));

	for (int i=0; i<(int)boundRects.size(); i++)
	{
		// if (RectA.X1 < RectB.X2 && RectA.X2 > RectB.X1
		//	   && RectA.Y1 < RectB.Y2 && RectA.Y2 > RectB.Y1) 
		if (area.tl().x < boundRects[i].br().x && area.br().x > boundRects[i].tl().x
		 && area.tl().y < boundRects[i].br().y && area.br().y > boundRects[i].tl().y)
		{
			int obstacleDistance = sqrt(pow(((boundRects[i].br().x-320) + (boundRects[i].tl().x-320))/2, 2) + pow(480 - boundRects[i].br().y, 2));
			distances.push_back(obstacleDistance);
		}
	}
}


void Vdar::extractBoundRects(const cv::Mat& depthImage)
{
	processingImage = Scalar(0,0,0);
	processingImage = denoiser.process(depthImage);

	try
	{
		processingImage = groundRemover.process(processingImage);
	} catch (const char* msg) {
		LOG_INFO(msg);
		LOG_INFO(cString::Format("height image = %d, size of vertLineMedian = %d",
					depthImage.rows, groundRemover.getVerLineMedianSize()));
	}

	try
	{
		birdsEyeView = birdsEyeViewer.process(processingImage);
	} catch (const char* msg) {
		LOG_INFO(msg);
	}

	boundRects.clear();
	try
	{
		boundRects = connectedComponentFinder.process(birdsEyeView);
	} catch (const char* msg) {
		LOG_INFO(msg);
		LOG_INFO(cString::Format("ConnectedComponentFinder srcImage depth = %d", processingImage.depth()));
	}
}


void Vdar::setIsDebugActive(const bool &isDebugActive)
{

}
