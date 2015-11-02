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

#include <cmath>

#include "CoordinateConverter.h"


CoordinateConverter::CoordinateConverter()
	: screenHeight(480), screenWidth(640), horizontalFieldOfView(58.0f), verticalFieldOfView(45.0f)
{
	setScreenCenter();
	setCoeffXAndCoeffY();
}


CoordinateConverter::CoordinateConverter(int scrHghtX, int scrWdthY, float hFV, float vFV)
	: screenHeight(scrHghtX), screenWidth(scrWdthY), horizontalFieldOfView(hFV), verticalFieldOfView(vFV)
{
	setScreenCenter();
	setCoeffXAndCoeffY();
}


void CoordinateConverter::setScreenCenter()
{
	screenCenterX = screenWidth / 2;
	screenCenterY = screenHeight / 2;
}


void CoordinateConverter::setCoeffXAndCoeffY()
{
	float xzFactor = (2.0f * tan(degToRad(horizontalFieldOfView/2.0f)));
	float yzFactor = (2.0f * tan(degToRad(verticalFieldOfView/2.0f)));

	f_x = static_cast<float>(screenWidth) / xzFactor;
	f_y = static_cast<float>(screenHeight) / yzFactor;
}


void CoordinateConverter::depthToWorld(int depthX, int depthY, int depthZ, float& worldX, float& worldY, float& worldZ)
{
	worldZ = rawDepthToMeters(depthZ);

	worldX = worldZ * static_cast<float>(screenCenterX - depthX) / f_x;
	worldY = worldZ * static_cast<float>(screenCenterY - depthY) / f_y;
}


float CoordinateConverter::rawDepthToMeters(int rawDepth)
{
	float rawDepthF = static_cast<double>(rawDepth); 

	if (rawDepth <= 0xFFFF)
	{ 
		// following values acquired during aadc xtion depth sensor calibration
		float depth = 3.33776840956e-05f * rawDepthF + -0.0674169620271f;
		return (depth);
	}

	return 0.0f;
}


float CoordinateConverter::degToRad(float degValue)
{
	return degValue * M_PI / 180.0f;
}
