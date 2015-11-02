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
#include <opencv2/core/core.hpp>

#ifndef _SURFACE_DESCRIPTION_H_
#define _SURFACE_DESCRIPTION_H_

class SurfaceDescription
{
	public:
		unsigned short id;
		unsigned int size;
		cv::Point topLeft;
		cv::Point bottomRight;
		cv::Point minimalIntensity;
		unsigned short minimalIntensityValue;

		SurfaceDescription(unsigned short surfaceId, unsigned int surfaceSize,
				cv::Point tL, cv::Point bR, cv::Point mI, unsigned short mIV)
		{
			id = surfaceId;
			size = surfaceSize;
			topLeft = tL;
			bottomRight=bR;
			minimalIntensity=mI;
			minimalIntensityValue=mIV;
		}

		SurfaceDescription(unsigned short surfaceId, unsigned int surfaceSize)
		{
			id = surfaceId;
			size = surfaceSize;
			topLeft = cv::Point(0xFFFF, 0xFFFF);
			bottomRight = cv::Point(0, 0);
			minimalIntensity = cv::Point(0,0);
			minimalIntensityValue = 0xFFFF;
		}

		SurfaceDescription(unsigned short surfaceId)
		{
			id = surfaceId;
			size = 0;
			topLeft = cv::Point(0xFFFF, 0xFFFF);
			bottomRight = cv::Point(0, 0);
			minimalIntensity = cv::Point(0,0);
			minimalIntensityValue = 0xFFFF;
		}

		SurfaceDescription()
		{
			id = 0;
			size = 0;
			topLeft = cv::Point(0xFFFF, 0xFFFF);
			bottomRight = cv::Point(0, 0);
			minimalIntensity = cv::Point(0,0);
			minimalIntensityValue = 0xFFFF;
		}

		void add(cv::Point currentPoint, unsigned short currentIntensityValue);
};
#endif /* _SURFACE_DESCRIPTION_H_ */
