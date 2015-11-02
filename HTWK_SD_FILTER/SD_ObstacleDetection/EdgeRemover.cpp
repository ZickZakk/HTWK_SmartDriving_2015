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
#include "EdgeRemover.h"

EdgeRemover::EdgeRemover() : thresholdH(1000), thresholdN(1)
{

}

EdgeRemover::~EdgeRemover()
{

}

bool EdgeRemover::distance(int xc, int xi)
{
	if (abs(xc - xi) > thresholdH)
	{
		return true;
	}
	
	return false;
}


cv::Mat& EdgeRemover::process(const cv::Mat &imSrc)
{
	int nrow = imSrc.rows;
	int ncol = imSrc.cols;

	imRes.create(imSrc.rows, imSrc.cols, imSrc.type());

	for (int r=1; r<nrow-1; r++)
	{
		for (int c=1; c<ncol-1; c++)
		{
			int xc = imSrc.at<ushort>(r,c);
			int nGreatDistance = 0;

			if (distance(xc, imSrc.at<ushort>(r, c-1)))
			{
				nGreatDistance += 1;
			}

			if (distance(xc, imSrc.at<ushort>(r-1, c)))
			{
				nGreatDistance += 1;
			}

			if (distance(xc, imSrc.at<ushort>(r+1, c)))
			{
				nGreatDistance += 1;
			}

			if (distance(xc, imSrc.at<ushort>(r, c+1)))
			{
				nGreatDistance += 1;
			}

			if (nGreatDistance >= thresholdN)
			{
				imRes.at<ushort>(r,c) = 0;
			}
			else
			{
				imRes.at<ushort>(r,c) = imSrc.at<ushort>(r,c);
			}
		}
	}

	return imRes;
}


