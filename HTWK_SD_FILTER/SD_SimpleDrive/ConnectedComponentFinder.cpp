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
#include "ConnectedComponentFinder.h"

ConnectedComponentFinder::ConnectedComponentFinder()
{
	thresh = 0;
	max_thresh = 255;
}


std::vector<cv::Rect>& ConnectedComponentFinder::process(const cv::Mat& imSrc)
{
	if (!isSupportedImageType(imSrc))
	{
		throw "ConnectedComponentFinder - Invalid Image Type";
	}

	try
	{
		threshold_output.create(imSrc.size(), imSrc.type());
		threshold(imSrc, threshold_output, thresh, max_thresh, cv::THRESH_BINARY);

		contours.clear();
		hierarchy.clear();
		findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		/// Approximate contours to polygons + get bounding rects and circles
		std::vector<std::vector<cv::Point> > contours_poly(contours.size());
		boundRects.clear();
		for(int i=0; i<(int)contours.size(); i++)
		{
			approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);

			cv::Rect boundRect = boundingRect(cv::Mat(contours_poly[i]));
			boundRects.push_back(boundRect);
		}
	}
	catch( cv::Exception& e )
	{
		const char* err_msg = e.what();
		throw err_msg;
	}

	return boundRects;
}


void ConnectedComponentFinder::setThreshold(uchar value)
{
	thresh = value;
}


uchar ConnectedComponentFinder::getThreshold()
{
	return thresh;
}


bool ConnectedComponentFinder::isSupportedImageType(const cv::Mat& imSrc)
{
	return (imSrc.depth() == CV_8U || imSrc.depth() == CV_16S || imSrc.depth() == CV_32F);
}
