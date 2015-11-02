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

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ICrossRoadDetector.h"


class CrossRoadDetector : public ICrossRoadDetector
{
public:
	bool searchStopLine(const cv::Mat &ipmImage);

	bool hasOppositeAccess(const cv::Mat& ipmImage);
	bool hasRightAccess(const cv::Mat& ipmImage);
	bool hasLeftAccess(const cv::Mat& ipmImage);

	CrossRoadDetector();
	virtual ~CrossRoadDetector();
		
private:

	cv::Point stopLinePostion;

	int minYOppositeAccess;
	cv::Point leftMinRoadEdgeOppositeAccess;
	cv::Point medianStripMinOppositeAccess;
	cv::Point rightMinRoadEdgeOppositeAccess;

	int minXLeftAccess;
	int minXRightAccess;

	int topMinYRoadEdgeSideAccess;
	int medianStripMinYSideAccess;
	int bottomMinYRoadEdgeSideAccess;


	void initExpectedRoadMarkPositions();

	bool hasVerticalAccess(const cv::Mat& ipmImage, int minx);
	bool searchHorizontalLine(const cv::Mat &ipmImage, const cv::Point& topLeftOfSearchArea,
			int width=25, int height=40, int numberOfSamplingPoints=5);
	bool searchVerticalLine(const cv::Mat& ipmImage, const cv::Point& topLeftOfSearchArea,
			int width=40, int height=25, int numberOfSamplingPoints=5);
	bool isWhite(const cv::Vec3b colorPixel, int thresholdWhite=150);
	bool isWhite(const int &pixel, int thresholdWhite=150);
};
