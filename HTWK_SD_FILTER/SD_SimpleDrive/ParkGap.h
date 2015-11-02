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

#ifndef _PARK_GAP_H_
#define _PARK_GAP_H_

class ParkGap : public IParkGap
{

public:
	ParkGap();
	~ParkGap();

	//tInt step;
	//enum steps {searchGap, driveBackToGap, stopDriveBackToGap, driveStraightInGap, stopleftMove, driveBack, driveStraight, stopDriveStraight, stopDriveBack, cpDriveStraight, cpDriveStraightFin, cpLeftMove, cpLeftMoveFin, cpRightMove, cpRightMoveFin, cpDriveStraightLast, cpDriveStraightLastFin, cpExit, ppDriveRightMove, ppDriveLeftMove, ppDriveStraightMove, ppExit, pullOutLeftCrossPart0, pullOutLeftCrossPart1, pullOutLeftCrossPart2, pullOutRightCrossPart0, pullOutRightCrossPart1, pullOutRightCrossPart2, pullOutLeftParallel, pullOutRightParallel0, pullOutRightParallel1, pullOutRightParallel2, pullOutRightParallel3,pullOutExit};

	// Property get
	//bool isParkingGapFound();
	bool searchParkingGap(const tInt &irValue,const tInt &distance);
	void setIsDebugActive(const bool &isDebugActive);
	void setDecision(const Decision &currentDecision);

private:

	bool isDebugActive;
	tInt distance;
	tInt gapStart;
	tInt lastDist;
	tBool isFirstCarFound;
	tBool isFirstCarComplete;
	tInt measuredParkSpace;
	bool foundGap;
	Decision currentDecision;
	
	/*bool foundWhitePixel(const Mat &image, const tInt &width, const tInt &height) const;

	bool hasNotToSteer(const Lane &lane, const tInt &position) const;
	bool hasToSteerLeft(const Lane &lane, const tInt &position) const;
	bool hasToSteerRight(const Lane &lane, const tInt &position) const;

	void calculateOptimalSteeringAngle(const Steer &steer, const tInt &difference, tInt &steerAngle);
	void smootheCurve(const tInt &difference, tInt &steerAngle);
	void setNewSteerAngle(tInt &steerAngle, const tInt &value);
	
	bool isPositionInToleranceRange(const Lane &lane, const tInt &position) const;
	bool isPositionLowerThanToleranceRange(const Lane &lane, const tInt &position) const;
	bool isPositionHigherThanToleranceRange(const Lane &lane, const tInt &position) const;*/
};

#endif