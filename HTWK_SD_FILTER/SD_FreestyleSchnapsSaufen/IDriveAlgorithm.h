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

#ifndef _I_DRIVE_ALGORITHM_H_
#define _I_DRIVE_ALGORITHM_H_

class IDriveAlgorithm
{
protected:
	bool isDebugActive;
	unsigned int smoothCurveValue;

public:
	// Property set
	virtual void setMinRightLanePosition(const int &minRightLanePosition) = 0;
	virtual void setMaxRightLanePosition(const int &maxRightLanePosition) = 0;
	virtual void setMinLeftLanePosition(const int &minLeftLanePosition) = 0;
	virtual void setMaxLeftLanePosition(const int &maxLeftLanePosition) = 0;

	virtual void setCrossroadStraightLanePosition(const int &lanePosition) = 0;
	virtual bool getCrossroadStraightLanePosition(const Mat &image, tInt &position) = 0;

	virtual void setIsDebugActive(const bool &isDebugActive) = 0;
	virtual void setSmoothCurveValue(const unsigned int smoothCurveValue) = 0;

	virtual void prepareImage(cv::Mat &image) = 0;
	virtual bool getRightLanePosition(const Mat &image, tInt &xPosition) = 0;
	virtual bool getLeftLanePosition(const Mat &image, tInt &yPosition) = 0;
	virtual bool getLeftLanePositionFromX(const Mat &image, tInt &steeringAngle) = 0;
	virtual void calculateSteeringAngle(const Lane &lane, const int &position, int &steeringAngle) = 0;
};

#endif