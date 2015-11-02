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

PI_Regler::PI_Regler()
{
	this->isDebugActive = false;
}

PI_Regler::~PI_Regler()
{
}

void PI_Regler::setMinRightLanePosition(const int &minRightLanePosition) 
{
	this->minRightLanePosition = minRightLanePosition;
}

void PI_Regler::setMaxRightLanePosition(const int &maxRightLanePosition) 
{
	this->maxRightLanePosition = maxRightLanePosition;
}

void PI_Regler::setMinLeftLanePosition(const int &minLeftLanePosition) 
{
	this->minLeftLanePosition = minLeftLanePosition;
}

void PI_Regler::setMaxLeftLanePosition(const int &maxLeftLanePosition) 
{
	this->maxLeftLanePosition = maxLeftLanePosition;
}

void PI_Regler::prepareImage(cv::Mat &image)
{
	cvtColor(image, image, CV_RGB2GRAY);
	blur(image, image, cv::Size(5,5));
	Canny(image, image, 100.0, 150.0);
}

bool PI_Regler::getRightLanePosition(const Mat &image, tInt &xPosition)
{
	static tInt startPositionOnX;
	startPositionOnX = 380;

	static tInt counter;
	counter = 0;

	xPosition = 0;

	for (tInt width = startPositionOnX; width < image.cols; width++)
	{
		for (tInt height = 440; height < 460; height++)
		{
			if (foundWhitePixel(image, width, height))
			{
				xPosition += width;
				counter++;
			}
		}
	}

	if (counter == 0)
	{
		return false;
	}

	xPosition /= counter;

	return true;
}

bool PI_Regler::getLeftLanePosition(const Mat &image, tInt &yPosition)
{
	static tInt counter;
	counter = 0;

	yPosition = 0;

	for (tInt width = 15; width < 35; width++)
	{
		for (tInt height = 479; height >= 0; height--)
		{
			if (foundWhitePixel(image, width, height))
			{
				yPosition += height;
				counter++;
				break;
			}
		}
	}

	if (counter == 0)
	{
		return false;
	}

	yPosition /= counter;

	return true;
}

bool PI_Regler::foundWhitePixel(const Mat &image, const tInt &width, const tInt &height) const
{
	if (static_cast<tInt>(image.at<uchar>(height, width)) > 200)
	{
		return true;
	}

	return false;
}

void PI_Regler::calculateSteeringAngle(const Lane &lane, const int &position, int &steeringAngle) 
{
	if (hasToSteerLeft(lane, position, steeringAngle))
	{
		setSteerAngle(0, steeringAngle);
	}
	else if (hasToSteerRight(lane, position, steeringAngle))
	{
		setSteerAngle(0, steeringAngle);
	}
	else
	{
		LOG_WARNING(cString::Format("LANE_%s prev: %d curr: %d steeringAngle: %d", (lane == 0 ? "Left" : "Right"), (lane == 0 ? this->yPositionBefore : this->xPositionBefore), position, steeringAngle));
	}

	if (lane == LANE_RIGHT)
	{
		this->xPositionBefore = position;
	}
	else
	{
		this->yPositionBefore = position;
	}
}

bool PI_Regler::hasNotToSteer(const Lane &lane, const tInt &position, tInt &steerAngle) const
{
	static tUInt differenceCoordinates;
	static tUInt distanceToLane;

	if (lane == LANE_RIGHT)
	{
		getDifference(this->xPositionBefore, position, differenceCoordinates);
		getDifference(position, this->minRightLanePosition, distanceToLane);

		return (isPositionInToleranceRange(lane, this->xPositionBefore) && isPositionInToleranceRange(lane, position)) || 
			(differenceCoordinates >= this->smoothCurveValue && distanceToLane >= this->smoothCurveValue);
	}
	else
	{
		getDifference(this->yPositionBefore, position, differenceCoordinates);
		getDifference(position, this->minLeftLanePosition, distanceToLane);

		return (isPositionInToleranceRange(lane, this->yPositionBefore) && isPositionInToleranceRange(lane, position)) || 
			(differenceCoordinates >= this->smoothCurveValue && distanceToLane >= this->smoothCurveValue);
	}
}

bool PI_Regler::hasToSteerLeft(const Lane &lane, const tInt &position, tInt &steerAngle)
{
	static tInt value;
	static tInt difference;

	if (lane == LANE_RIGHT)
	{
		// Bild 1
		if ((position >= this->xPositionBefore) && 
			isPositionLowerThanToleranceRange(lane, position) && 
			isPositionLowerThanToleranceRange(lane, this->xPositionBefore))
		{
			difference = abs(this->maxRightLanePosition - position);

			if ((difference <= 5) && steerAngle <= 0)
			{
				steerAngle *= -1;
			}

			return true;
		}
		// Bild 2 und 3
		else if ((isPositionInToleranceRange(lane, position) && isPositionLowerThanToleranceRange(lane, this->xPositionBefore)) ||
			(isPositionHigherThanToleranceRange(lane, position) && isPositionLowerThanToleranceRange(lane, position)) ||
			((this->xPositionBefore < position) && isPositionHigherThanToleranceRange(lane, this->xPositionBefore) && isPositionHigherThanToleranceRange(lane, position)))
		{
			// lenkwinkel ist noch nicht links
			if (steerAngle >= 0)
			{
				difference = (this->xPositionBefore - position);
				smoothCurve(difference, steerAngle);
				steerAngle *= -1;
			}
			else
			{
				difference = (this->maxRightLanePosition - position);
				smoothCurve(difference, value);
				value *= -1;
				steerAngle += value;
			}

			return true;
		}
	}
	else
	{
		return (isPositionInToleranceRange(lane, position) && isPositionLowerThanToleranceRange(lane, this->yPositionBefore)) || 
			isPositionLowerThanToleranceRange(lane, position) ||
			(isPositionLowerThanToleranceRange(lane, position) && isPositionInToleranceRange(lane, this->yPositionBefore));
	}

	return false;
}

bool PI_Regler::hasToSteerRight(const Lane &lane, const tInt &position, tInt &steerAngle)
{
	static tInt value;
	static tInt difference;

	if (lane == LANE_RIGHT)
	{
		// Bild 2 und 3
		if ((isPositionLowerThanToleranceRange(lane, position) && isPositionHigherThanToleranceRange(lane, this->xPositionBefore)) || 
			(isPositionInToleranceRange(lane, position) && isPositionHigherThanToleranceRange(lane, this->xPositionBefore)) ||
			((position < this->xPositionBefore) && isPositionLowerThanToleranceRange(lane, position) && isPositionLowerThanToleranceRange(lane, position)))
		{
			// lenkwinkel ist noch nicht rechts
			if (steerAngle <= 0)
			{
				difference = (this->xPositionBefore - position);
				smoothCurve(difference, steerAngle);
			}
			else
			{
				difference = (this->minRightLanePosition - position);
				smoothCurve(difference, value);
				steerAngle += value;
			}

			return true;
		}
		// Bild 5
		else if ((position <= this->xPositionBefore) && 
			isPositionHigherThanToleranceRange(lane, position) && 
			isPositionHigherThanToleranceRange(lane, this->xPositionBefore))
		{
			difference = abs(this->maxRightLanePosition - position);

			if ((difference <= 5) && steerAngle <= 0)
			{
				steerAngle *= -1;
			}

			return true;
		}
	}
	else
	{
		return (isPositionInToleranceRange(lane, position) && isPositionHigherThanToleranceRange(lane, this->yPositionBefore)) || 
			isPositionHigherThanToleranceRange(lane, position) ||
			(isPositionHigherThanToleranceRange(lane, position) && isPositionInToleranceRange(lane, this->yPositionBefore));
	}

	return false;
}

void PI_Regler::getDifference(const tInt &positionBefore, const tInt &positionCurrent, tUInt difference) const
{
	difference = abs(positionBefore - positionCurrent);
}

void PI_Regler::smoothCurve(const tInt &difference, tInt &steerAngle)
{
	static const tUInt value1 = 0;
	static const tUInt value2 = 1;
	static const tUInt value3 = 2;
	static const tUInt value4 = 3;
	static const tUInt value5 = 4;

	static tUInt tmpDifference;
	tmpDifference = abs(difference);

	if (tmpDifference >= value1 && tmpDifference < value1 + this->smoothCurveValue)
	{
		steerAngle = 1;
	}
	else if (tmpDifference >= value2 && tmpDifference < value2 + this->smoothCurveValue)
	{
		steerAngle = 2;
	}
	else if (tmpDifference >= value3 && tmpDifference < value3 + this->smoothCurveValue)
	{
		steerAngle = 3;
	}
	else if (tmpDifference >= value4 && tmpDifference < value4 + this->smoothCurveValue)
	{
		steerAngle = 4;
	}
	else if (tmpDifference >= value5)
	{
		steerAngle = 5;
	}
}

bool PI_Regler::isPositionInToleranceRange(const Lane &lane, const tInt &position) const
{	
	if (lane == LANE_RIGHT)
	{
		return position >= minRightLanePosition && position <= maxRightLanePosition;
	}
	else
	{
		return position >= minLeftLanePosition && position <= maxLeftLanePosition;
	}
}

bool PI_Regler::isPositionLowerThanToleranceRange(const Lane &lane, const tInt &position) const
{	
	if (lane == LANE_RIGHT)
	{
		return position < minRightLanePosition;
	}
	else
	{
		return position < minLeftLanePosition;
	}
}

bool PI_Regler::isPositionHigherThanToleranceRange(const Lane &lane, const tInt &position) const
{
	if (lane == LANE_RIGHT)
	{
		return position > maxRightLanePosition;
	}
	else
	{
		return position > maxLeftLanePosition;
	}
}

void PI_Regler::getSteerAngle(const Lane &lane, const tInt &position, tInt &value) const
{
	static tInt difference;
	
	if (lane == LANE_RIGHT)
	{
		getDifference(position, this->minRightLanePosition, difference);
	}
	else
	{
		getDifference(position, this->maxLeftLanePosition, difference);
	}

	if (difference >= 10)
	{
		value = 5;
	}
	else if (difference >= 7)
	{
		value = 4;
	}
	else if (difference >= 4)
	{
		value = 2;
	}
	else 
	{
		value = 1;
	}
}

void PI_Regler::setSteerAngle(const tInt &value, tInt &steerAngle) const
{
	static const tInt MIN_STEER_ANGLE = -20;
	static const tInt MAX_STEER_ANGLE = 20;

	steerAngle += value;

	if (steerAngle <= MIN_STEER_ANGLE)
	{
		steerAngle = MIN_STEER_ANGLE;		
	}
	else if (steerAngle >= MAX_STEER_ANGLE)
	{
		steerAngle = MAX_STEER_ANGLE;
	}
}

void PI_Regler::setIsDebugActive(const bool &isDebugActive)
{
	this->isDebugActive = isDebugActive;
}

void PI_Regler::setSmoothCurveValue(const unsigned int smoothCurveValue)
{
	this->smoothCurveValue = smoothCurveValue;
}