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


GroundRemover::GroundRemover() : threshold(5000)
{

}


GroundRemover::~GroundRemover()
{

}


void GroundRemover::setBackground(const cv::Mat &imgSrc)
{
	int rows = imgSrc.rows;
	int cols = imgSrc.cols;

	vertLineMedian.clear();
	vertLineMedian.resize(rows);

	for (int r=0; r<rows; r++)
	{
		std::vector<ushort> line;
		for (int c=0; c<cols; c++)
		{
			ushort intensity = imgSrc.at<ushort>(r, c);
			if (0 != intensity)
			{
				line.push_back(intensity);
			}
		}

		std::sort(line.begin(), line.end());

		vertLineMedian.at(r) = line.at(line.size()/2);
	}
}


cv::Mat& GroundRemover::process(const cv::Mat &imSrc)
{
	int nrow = imSrc.rows;
	int ncol = imSrc.cols;

	if (!isValidSize(imSrc))
	{
		throw "GroundRemover - invalid image size";
	}

	if (!isValidType(imSrc))
	{
		throw "GoundRemover - invalid type";
	}


	imRes.create(imSrc.rows, imSrc.cols, imSrc.type());

	for (int r=0; r<nrow; r++)
	{
		for (int c=0; c<ncol; c++)
		{
			ushort intensity = imSrc.at<ushort>(r,c);

			// if current intensity lower than threshold value
			// remove current intensity
			if ((vertLineMedian.at(r) - intensity) < threshold)
			{
				imRes.at<ushort>(r,c) = 0;
			}
			else
			{
				imRes.at<ushort>(r,c) = intensity;
			}
		}
	}

	return imRes;
}


bool GroundRemover::isValidSize(const cv::Mat &imSrc)
{
	return imSrc.rows == (int)vertLineMedian.size();
}


bool GroundRemover::isValidType(const cv::Mat& imSrc)
{
	return imSrc.type() == CV_16U;
}


int GroundRemover::getVerLineMedianSize()
{
	return (int)vertLineMedian.size();
}
