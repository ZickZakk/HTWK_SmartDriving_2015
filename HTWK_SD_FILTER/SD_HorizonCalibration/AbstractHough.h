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

#ifndef _HOUGH_H_
#define _HOUGH_H_

#include "SmartDrivingUtilities.h"
using namespace smartdriving;

namespace smartdriving
{
	class AbstractHough
	{
	private:
		int houghSpaceHeight;
		int houghSpaceWidth;

		double imageCenterX;
		double imageCenterY;

	public:
		AbstractHough();
		virtual ~AbstractHough();
		
	public:
		virtual void transform(void) = 0;
		vector<SD_Line> getLines(const int &threshold);
		void setImage(const cv::Mat&);

	private:
		bool isLocalMaximaFound(int&, int&, const int&);
		const int getHoughSpaceIndex(const int&, const int&) const;
		bool isPositionInImage(const pair<int, int>&, const int&, const int&) const;
		int getYCoord(const int&, const int&, const int&);
		int getXCoord(const int&, const int&, const int&);

	protected:
		unique_ptr<unsigned int[]> houghSpace;
		cv::Mat image;

		int imageWidth;
		int imageHeight;
		double houghSpaceHeightRelative;

		void initHough(void);
		double getDistance(const int&, const int&, const int&) const;
	};
}

#endif /* HOUGH_H_ */