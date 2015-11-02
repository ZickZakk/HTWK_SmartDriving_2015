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
#include "NoiseRemover.h"

NoiseRemover::NoiseRemover() : trimLevel(2)
{
	mask = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
}


NoiseRemover::~NoiseRemover() {}

cv::Mat NoiseRemover::getMask() const
{
	return mask;
}


void NoiseRemover::setTrimLevel(tUInt8 n)
{
	trimLevel = n;
}


tUInt8 NoiseRemover::getTrimLevel() const
{
	return trimLevel;
}


cv::Mat& NoiseRemover::process(const cv::Mat& image)
{
	imRes.create(image.rows, image.cols, image.type());

	cv::dilate(image, imRes, mask);
	cv::erode(imRes, imRes, mask);

	for (int i=trimLevel; i>0; --i) {
		cv::dilate(imRes, imRes, mask);
		cv::dilate(imRes, imRes, mask);
		cv::erode(imRes, imRes, mask);
	}

	return imRes;
}
