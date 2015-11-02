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

#ifndef _DRIVE_ALGORITHM_H_
#define _DRIVE_ALGORITHM_H_

class DriveAlgorithm : public IDriveAlgorithm
{
private:
	bool isLeftLaneDetected;
	bool isRightLaneDetected;

	// Properties
	int minLeftLanePosition;
	int maxLeftLanePosition;
	int minRightLanePosition;
	int maxRightLanePosition;

	int crossroadStraightLanePosition;

	int xPositionBefore;
	int yPositionBefore;
	int straightPositionBefore;

public:
	DriveAlgorithm();
	~DriveAlgorithm();

	// Property get
	void setMinRightLanePosition(const int &minRightLanePosition) override;
	void setMaxRightLanePosition(const int &maxRightLanePosition) override;
	void setMinLeftLanePosition(const int &minLeftLanePosition) override;
	void setMaxLeftLanePosition(const int &maxLeftLanePosition) override;
	
	void setIsDebugActive(const bool &isDebugActive) override;
	void setSmoothCurveValue(const unsigned int smoothCurveValue) override;

	void setCrossroadStraightLanePosition(const int &lanePosition) override;
	bool getCrossroadStraightLanePosition(const Mat &image, tInt &position) override;

	bool getRightLanePosition(const Mat &image, tInt &xPosition) override;
	bool getLeftLanePosition(const Mat &image, tInt &yPosition) override;
	bool getLeftLanePositionFromX(const Mat &image, tInt &steeringAngle) override;
	void calculateSteeringAngle(const Lane &lane, const int &position, int &steeringAngle) override;
	void prepareImage(cv::Mat &image) override;

private:
	bool foundWhitePixel(const Mat &image, const tInt &width, const tInt &height) const;

	bool hasNotToSteer(const Lane &lane, const tInt &position) const;
	bool hasToSteerLeft(const Lane &lane, const tInt &position) const;
	bool hasToSteerRight(const Lane &lane, const tInt &position) const;

	void calculateOptimalSteeringAngle(const Steer &steer, const tInt &difference, tInt &steerAngle);
	void smootheCurve(const tInt &difference, tInt &steerAngle);
	void setNewSteerAngle(tInt &steerAngle, const tInt &value);
	
	bool isPositionInToleranceRange(const Lane &lane, const tInt &position) const;
	bool isPositionLowerThanToleranceRange(const Lane &lane, const tInt &position) const;
	bool isPositionHigherThanToleranceRange(const Lane &lane, const tInt &position) const;
};

#endif