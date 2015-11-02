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

#ifndef _PI_REGLER_H_
#define _PI_REGLER_H_

class PI_Regler : public IDriveAlgorithm
{
private:
	bool isLeftLaneDetected;
	bool isRightLaneDetected;

	// Properties
	int minLeftLanePosition;
	int maxLeftLanePosition;
	int minRightLanePosition;
	int maxRightLanePosition;

	int xPositionBefore;
	int yPositionBefore;

public:
	PI_Regler();
	~PI_Regler();

	// Property get
	void setMinRightLanePosition(const int &minRightLanePosition) override;
	void setMaxRightLanePosition(const int &maxRightLanePosition) override;
	void setMinLeftLanePosition(const int &minLeftLanePosition) override;
	void setMaxLeftLanePosition(const int &maxLeftLanePosition) override;
	void setIsDebugActive(const bool &isDebugActive) override;
	void setSmoothCurveValue(const unsigned int smoothCurveValue) override;

	bool getRightLanePosition(const Mat &image, tInt &xPosition) override;
	bool getLeftLanePosition(const Mat &image, tInt &yPosition) override;
	void calculateSteeringAngle(const Lane &lane, const int &position, int &steeringAngle) override;
	void prepareImage(cv::Mat &image) override;

private:
	bool foundWhitePixel(const Mat &image, const tInt &width, const tInt &height) const;

	bool hasNotToSteer(const Lane &lane, const tInt &position, tInt &steerAngle) const;
	bool hasToSteerLeft(const Lane &lane, const tInt &position, tInt &steerAngle);
	bool hasToSteerRight(const Lane &lane, const tInt &position, tInt &steerAngle);
	
	void getDifference(const tInt &positionBefore, const tInt &positionCurrent, tUInt difference) const;
	void smoothCurve(const tInt &difference, tInt &steerAngle);

	bool isPositionInToleranceRange(const Lane &lane, const tInt &position) const;
	bool isPositionLowerThanToleranceRange(const Lane &lane, const tInt &position) const;
	bool isPositionHigherThanToleranceRange(const Lane &lane, const tInt &position) const;

	void getSteerAngle(const Lane &lane, const tInt &position, tInt &value) const;
	void setSteerAngle(const tInt &value, tInt &steerAngle) const;
	
};

#endif