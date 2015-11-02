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

DriveAlgorithm::DriveAlgorithm()
{
	this->isDebugActive = false;
	this->isLeftLaneDetected = false;
	this->isRightLaneDetected = false;
}

DriveAlgorithm::~DriveAlgorithm()
{

}

void DriveAlgorithm::setMinRightLanePosition(const int &minRightLanePosition) 
{
	this->minRightLanePosition = minRightLanePosition;
}

void DriveAlgorithm::setMaxRightLanePosition(const int &maxRightLanePosition) 
{
	this->maxRightLanePosition = maxRightLanePosition;
}

void DriveAlgorithm::setMinLeftLanePosition(const int &minLeftLanePosition) 
{
	this->minLeftLanePosition = minLeftLanePosition;
}

void DriveAlgorithm::setMaxLeftLanePosition(const int &maxLeftLanePosition) 
{
	this->maxLeftLanePosition = maxLeftLanePosition;
}

void DriveAlgorithm::prepareImage(cv::Mat &image)
{
	cvtColor(image, image, CV_RGB2GRAY);
	blur(image, image, cv::Size(5,5));
	Canny(image, image, 100.0, 150.0);
}

bool DriveAlgorithm::getRightLanePosition(const Mat &image, tInt &xPosition)
{
	static tInt startPositionOnX;
	startPositionOnX = 380;

	static tInt counter;
	counter = 0;

	xPosition = 0;

	/*for (tInt width = startPositionOnX; width < image.cols; width++)
	{
		for (tInt height = 440; height < 460; height++)
		{
			if (foundWhitePixel(image, width, height))
			{
				xPosition += width;
				counter++;
			}
		}
	}*/
	for (tInt height = 440; height < 460; height++)
	{
		for (tInt width = startPositionOnX; width < image.cols; width++)
		{

			if (foundWhitePixel(image, width, height))
			{
				xPosition += width;
				counter++;
				break;
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

bool DriveAlgorithm::getLeftLanePosition(const Mat &image, tInt &yPosition)
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

bool DriveAlgorithm::getLeftLanePositionFromX(const Mat &image, tInt &steeringAngle)
{
	for (int height = 460; height < 480; height++)
	{
		for (int width = 300; width >= 0; width--)
		{
			if (foundWhitePixel(image, width, height))
			{
				steeringAngle = 15;
				return true;
			}
		}
	}

	return false;
}

bool DriveAlgorithm::foundWhitePixel(const Mat &image, const tInt &width, const tInt &height) const
{
	if (static_cast<tInt>(image.at<uchar>(height, width)) > 200)
	{
		return true;
	}

	return false;
}

void DriveAlgorithm::calculateSteeringAngle(const Lane &lane, const int &position, int &steeringAngle)
{
	static tInt tmpSteerAngle;

	if (lane == LANE_RIGHT)
	{
		tmpSteerAngle = position - this->xPositionBefore;
	}
	else if (lane == LANE_LEFT)
	{
		tmpSteerAngle = position - this->yPositionBefore;
	}
	else
	{
		tmpSteerAngle = position - this->straightPositionBefore;
	}

	static tInt steerValue;

	if (hasNotToSteer(lane, position))
	{
		if (lane == LANE_LEFT)
		{
			if (isDebugActive)
			{
				LOG_INFO("lenke nicht");
				LOG_INFO(cString::Format("yPrev: %d, yCurr: %d, diff: %d", this->yPositionBefore, position, tmpSteerAngle));
			}
		}

		if (isDebugActive)
		{
			LOG_INFO("lenke nicht :P");
			if (lane == LANE_LEFT)
			{
				LOG_INFO("lenke nicht");
				LOG_INFO(cString::Format("yPrev: %d, yCurr: %d, diff: %d", this->yPositionBefore, position, tmpSteerAngle));
			}
			else
			{
				LOG_INFO(cString::Format("xPrev: %d, xCurr: %d, diff: %d", this->xPositionBefore, position, tmpSteerAngle));
			}
		}
	}
	else if (hasToSteerLeft(lane, position))
	{
		if (lane == LANE_LEFT)
		{
			if (isDebugActive)
			{
				LOG_INFO("lenke links");
				LOG_INFO(cString::Format("yPrev: %d, yCurr: %d, diff: %d", this->yPositionBefore, position, tmpSteerAngle));
			}
		}

		if (isDebugActive)
		{
			LOG_INFO("lenke links");
			if (lane == LANE_LEFT)
			{
				LOG_INFO(cString::Format("yPrev: %d, yCurr: %d, diff: %d", this->yPositionBefore, position, tmpSteerAngle));
			}
			else
			{
				LOG_INFO(cString::Format("xPrev: %d, xCurr: %d, diff: %d", this->xPositionBefore, position, tmpSteerAngle));
			}
		}

		calculateOptimalSteeringAngle(STEER_LEFT, tmpSteerAngle, steerValue);
		setNewSteerAngle(steeringAngle, steerValue);
	}
	else if (hasToSteerRight(lane, position))
	{
		if (isDebugActive)
		{
			LOG_INFO("lenke rechts");
			if (lane == LANE_LEFT)
			{
				LOG_INFO(cString::Format("yPrev: %d, yCurr: %d, diff: %d", this->yPositionBefore, position, tmpSteerAngle));
			}
			else
			{
				LOG_INFO(cString::Format("xPrev: %d, xCurr: %d, diff: %d", this->xPositionBefore, position, tmpSteerAngle));
			}
		}
		
		calculateOptimalSteeringAngle(STEER_RIGHT, tmpSteerAngle, steerValue);
		setNewSteerAngle(steeringAngle, steerValue);
	}
	else
	{
		if (isDebugActive)
		{
			LOG_WARNING("Dont know to steer left, right or not to steer");

			if (lane == LANE_LEFT)
			{
				LOG_INFO(cString::Format("yPrev: %d, yCurr: %d, diff: %d", this->yPositionBefore, position, tmpSteerAngle));
			}
			else
			{
				LOG_INFO(cString::Format("xPrev: %d, xCurr: %d, diff: %d", this->xPositionBefore, position, tmpSteerAngle));
			}
		}
	}
	
	if (lane == LANE_RIGHT)
	{
		this->xPositionBefore = position;
	}
	else if (lane == LANE_LEFT)
	{
		this->yPositionBefore = position;
	}
	else
	{
		this->straightPositionBefore = position;
	}
}

bool DriveAlgorithm::hasNotToSteer(const Lane &lane, const tInt &position) const
{
	if (lane == LANE_RIGHT)
	{
		return (isPositionInToleranceRange(lane, this->xPositionBefore) && isPositionInToleranceRange(lane, position)) || 
			(this->xPositionBefore < position && isPositionLowerThanToleranceRange(lane, this->xPositionBefore) && isPositionLowerThanToleranceRange(lane, position)) ||
			(this->xPositionBefore > position && isPositionHigherThanToleranceRange(lane, this->xPositionBefore) && isPositionHigherThanToleranceRange(lane, position));
	}
	else if (lane == LANE_LEFT)
	{
		return position <= this->yPositionBefore && 
			(isPositionLowerThanToleranceRange(lane, this->yPositionBefore) || isPositionInToleranceRange(lane, this->yPositionBefore)) && 
			isPositionLowerThanToleranceRange(lane, position);	
	}
	else
	{
		return (isPositionInToleranceRange(lane, this->straightPositionBefore) && isPositionInToleranceRange(lane, position)) || 
			(this->straightPositionBefore < position && isPositionLowerThanToleranceRange(lane, this->straightPositionBefore) && isPositionLowerThanToleranceRange(lane, position)) ||
			(this->straightPositionBefore > position && isPositionHigherThanToleranceRange(lane, this->straightPositionBefore) && isPositionHigherThanToleranceRange(lane, position));
	}
}

bool DriveAlgorithm::hasToSteerLeft(const Lane &lane, const tInt &position) const
{
	if (lane == LANE_RIGHT)
	{
		return position <= this->xPositionBefore && 
			(isPositionLowerThanToleranceRange(lane, this->xPositionBefore) || isPositionInToleranceRange(lane, this->xPositionBefore)) && 
			isPositionLowerThanToleranceRange(lane, position);
	}
	else if(lane == LANE_LEFT)
	{
		return (isPositionInToleranceRange(lane, this->yPositionBefore) && isPositionInToleranceRange(lane, position)) || 
			(this->yPositionBefore < position && isPositionLowerThanToleranceRange(lane, this->yPositionBefore) && isPositionLowerThanToleranceRange(lane, position)) ||
			(this->yPositionBefore > position && isPositionHigherThanToleranceRange(lane, this->yPositionBefore) && isPositionHigherThanToleranceRange(lane, position));
	}
	else
	{
		return position <= this->straightPositionBefore && 
			(isPositionLowerThanToleranceRange(lane, this->straightPositionBefore) || isPositionInToleranceRange(lane, this->straightPositionBefore)) && 
			isPositionLowerThanToleranceRange(lane, position);
	}
}

bool DriveAlgorithm::hasToSteerRight(const Lane &lane, const tInt &position) const
{
	if (lane == LANE_RIGHT)
	{
		return this->xPositionBefore <= position && 
			(isPositionHigherThanToleranceRange(lane, this->xPositionBefore) || isPositionInToleranceRange(lane, this->xPositionBefore)) && 
			isPositionHigherThanToleranceRange(lane, position);
	}
	else if (lane == LANE_LEFT)
	{
		return this->yPositionBefore <= position && 
			(isPositionHigherThanToleranceRange(lane, this->yPositionBefore) || isPositionInToleranceRange(lane, this->yPositionBefore)) && 
			isPositionHigherThanToleranceRange(lane, position);
	}
	else 
	{
		return this->straightPositionBefore <= position && 
			(isPositionHigherThanToleranceRange(lane, this->straightPositionBefore) || isPositionInToleranceRange(lane, this->straightPositionBefore)) && 
			isPositionHigherThanToleranceRange(lane, position);
	}
}


tVoid DriveAlgorithm::calculateOptimalSteeringAngle(const Steer &steer, const tInt &difference, tInt &steerAngle)
{
	smootheCurve(difference, steerAngle);

	if (steer == STEER_LEFT)
	{
		steerAngle *= -1;
	}

	if (isDebugActive)
	{
		LOG_INFO(cString::Format("Optimized steer angle: %d", steerAngle));
	}
}

tVoid DriveAlgorithm::smootheCurve(const tInt &difference, tInt &steerAngle)
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

void DriveAlgorithm::setNewSteerAngle(tInt &steerAngle, const tInt &value)
{
	static const tInt MIN_STEER_ANGLE = -25;
	static const tInt MAX_STEER_ANGLE = 25;

	if (this->isDebugActive)
	{
		LOG_INFO(cString::Format("value: %d, steerangle: %d", value, steerAngle));
	}

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

bool DriveAlgorithm::isPositionInToleranceRange(const Lane &lane, const tInt &position) const
{	
	if (lane == LANE_RIGHT)
	{
		return position >= minRightLanePosition && position <= maxRightLanePosition;
	}
	else if (lane == LANE_LEFT)
	{
		return position >= this->minLeftLanePosition && position <= this->maxLeftLanePosition;
	}
	else
	{
		return position >= (this->crossroadStraightLanePosition - 3) && position <= (this->crossroadStraightLanePosition + 3);
	}
}

tBool DriveAlgorithm::isPositionLowerThanToleranceRange(const Lane &lane, const tInt &position) const
{	
	if (lane == LANE_RIGHT)
	{
		return position < this->minRightLanePosition;
	}
	else if (lane == LANE_LEFT)
	{
		return position < this->minLeftLanePosition;
	}
	else
	{
		return position < (this->crossroadStraightLanePosition - 3);
	}
}

tBool DriveAlgorithm::isPositionHigherThanToleranceRange(const Lane &lane, const tInt &position) const
{
	if (lane == LANE_RIGHT)
	{
		return position > this->maxRightLanePosition;
	}
	else if (lane == LANE_LEFT)
	{
		return position > this->maxLeftLanePosition;
	}
	else
	{
		return position > (this->crossroadStraightLanePosition + 3);
	}
}

void DriveAlgorithm::setIsDebugActive(const bool &isDebugActive)
{
	this->isDebugActive = isDebugActive;
}

void DriveAlgorithm::setSmoothCurveValue(const unsigned int smoothCurveValue) 
{
	this->smoothCurveValue = smoothCurveValue;
}

void DriveAlgorithm::setCrossroadStraightLanePosition(const int &lanePosition) 
{
	this->crossroadStraightLanePosition = lanePosition;
}

bool DriveAlgorithm::getCrossroadStraightLanePosition(const Mat &image, tInt &position) 
{
	static tInt startPositionOnX;
	startPositionOnX = 200;

	static tInt counter;
	counter = 0;

	position = 0;

	for (tInt height = 280; height < 300; height++)
	{
		for (tInt width = startPositionOnX; width < image.cols; width++)
		{
			if (foundWhitePixel(image, width, height))
			{
				position += width;
				counter++;
				break;
			}
		}
	}

	if (counter == 0)
	{
		return false;
	}
	
	position /= counter;

	return true;
}