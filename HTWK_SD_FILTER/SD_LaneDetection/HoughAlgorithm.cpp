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

HoughAlgorithm::HoughAlgorithm(void)
{
	setStrategy(HOUGH_DEFAULT);
}

HoughAlgorithm::HoughAlgorithm(const Hough &strategy)
{
	setStrategy(strategy);
}

HoughAlgorithm::~HoughAlgorithm(void)
{
	this->hough.release();
}

void HoughAlgorithm::resetStrategie(const Hough &strategy)
{
	setStrategy(strategy);
}

void HoughAlgorithm::setStrategy(const Hough &strategy)
{
	this->strategy = strategy;

	switch (strategy)
	{
	case HOUGH_DEFAULT:
		this->hough.reset(new HoughDefault());
		break;

	case HOUGH_FAST:
		this->hough.reset(new HoughFast());
		break;
		
	default:
		this->hough.reset(new HoughDefault());
		break;
	}
}

void HoughAlgorithm::getLines(cv::Mat &image, const int &threshold, vector<SD_Line> &lines)
{
	if (this->strategy == HOUGH_OPENCV)
	{
		vector<Vec2f> cvLines;
		HoughLines(image, cvLines, 1, CV_PI/180, 100, 0, 0 );

		for(size_t i = 0; i < cvLines.size(); i++ )
		{
			float rho = cvLines[i][0];
			float theta = cvLines[i][1];

			double a = cos(theta);
			double b = sin(theta);
			double x0 = a * rho; 
			double y0 = b * rho;

			static SD_Line line;
			line.x1 = cvRound(x0 + 1000 * (-b));
			line.y1 = cvRound(y0 + 1000 * (a));
			line.x2 = cvRound(x0 - 1000 * (-b));
			line.y2 = cvRound(y0 - 1000 * (a));
			
			lines.push_back(line);
		}

	}
	else
	{
		this->hough->setImage(image);
		this->hough->transform();
		lines = this->hough->getLines(threshold);
	}
}