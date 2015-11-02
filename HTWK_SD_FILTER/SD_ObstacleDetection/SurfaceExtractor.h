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

#include <opencv2/core/core.hpp>
#include <stack>

#include "SurfaceDescription.h"

#ifndef _SURFACE_EXTRACTOR_H_
#define _SURFACE_EXTRACTOR_H_

class SurfaceExtractor
{
	public:
		std::vector<SurfaceDescription>& process(const cv::Mat& imSrc);
//		void getSurfaceDescriptionById(int label, SurfaceDescription& surfaceDescription);
//		void getLargestSurface(SurfaceDescription& descriptionLargestSurface);

		unsigned int getNumberOfSurfaces();
		cv::Mat& getLabeledImage();

		SurfaceExtractor();

	private:
		std::vector<SurfaceDescription> surfaceDescriptions;

		cv::Mat labeledImage;
		cv::Mat sourceImage;

		unsigned int numberOfSurfaces;

		void clearImage(cv::Mat& image);
		void allocateImageIfNecessary(cv::Mat& image, cv::Size size, int type);
		void floodFill(const cv::Point& start,
				const cv::Mat& sourceImage, cv::Mat& labeledImage,
				SurfaceDescription& surfaceDescription);
		void checkLabelAndPush(const cv::Point& position,
				const cv::Mat& sourceImage, cv::Mat& labeledImage,
				std::stack<cv::Point>& stack,
				int surfaceId);
		bool isLabeled(const cv::Mat& im, const cv::Point& p);
		bool isInImage(const cv::Mat& im, const cv::Point& p);
};

#endif /* _SURFACE_EXTRACTOR_H_ */
