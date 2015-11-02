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

#include "SurfaceDescription.h"

void SurfaceDescription::add(cv::Point currentPoint, unsigned short currentIntensityValue)
{
	if (currentIntensityValue <= minimalIntensityValue)
	{
		minimalIntensity.x = currentPoint.x;
		minimalIntensity.y = currentPoint.y;
		minimalIntensityValue = currentIntensityValue;
	}

	unsigned short minrow = topLeft.y;
	unsigned short mincol = topLeft.x;
	unsigned short maxrow = bottomRight.y;
	unsigned short maxcol = bottomRight.x;

	if (currentPoint.y < minrow)
	{
		minrow = currentPoint.y;
	}
	if (currentPoint.y > maxrow)
	{
		maxrow = currentPoint.y;
	}
	if (currentPoint.x < mincol)
	{
		mincol = currentPoint.x;
	}
	if (currentPoint.x > maxcol)
	{
		maxcol = currentPoint.x;
	}

	topLeft.y = minrow;
	topLeft.x = mincol;
	bottomRight.y = maxrow;
	bottomRight.x = maxcol;
}
