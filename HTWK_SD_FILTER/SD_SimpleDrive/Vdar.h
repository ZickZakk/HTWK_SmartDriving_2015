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

#include "NoiseRemover.h"
#include "GroundRemover.h"
#include "BirdsEyeViewer.h"
#include "ConnectedComponentFinder.h"

#ifndef _OBSTACLEDECTION_H_
#define _OBSTACLEDECTION_H_

class Vdar : public IObstacleDetection
{
public:
	void setReferenceBackground(const cv::Mat& referenceBackgound);

	bool isObstacleAtPosInDepthImg(const cv::Point& point, const cv::Mat& depthImage);
	bool isObstacleInAreaInDepthImg(const cv::Rect& area, const cv::Mat& depthImage);
	void getObstaclesInAreaInDepthImg(const cv::Rect& are, const cv::Mat& depthImage, std::vector<cv::Rect>& boundRects);

	void getPositionsOfObstaclesInAreaInDepthImg(const cv::Rect& searchArea, const cv::Mat& depthImage, std::vector<cv::Point>& positions) override;
	void getDistancesOfObstaclesInAreaInDepthImg(const cv::Rect& searchArea, const cv::Mat& depthImage, std::vector<int>& distances) override;

	void setIsDebugActive(const bool &isDebugActive);

	virtual ~Vdar();

private:
	std::vector<cv::Rect> boundRects;

	cv::Mat referenceDepthImage;
	cv::Mat processingImage;
	cv::Mat birdsEyeView;

	NoiseRemover denoiser;
	GroundRemover groundRemover;
	BirdsEyeViewer birdsEyeViewer;
	ConnectedComponentFinder connectedComponentFinder;

	void extractBoundRects(const cv::Mat& depthImage);
};

#endif
