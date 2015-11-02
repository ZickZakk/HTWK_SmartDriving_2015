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

DecisionEvaluator::DecisionEvaluator()
{
	this->isDebugActive = false;
}

DecisionEvaluator::~DecisionEvaluator()
{

}
	
void DecisionEvaluator::getDecisionFromValue(const tUInt16 &decisionValue, Decision &currentDecision, Decision &prevDecision)
{
	switch(decisionValue)
	{
	case DECISION_DRIVE:
		prevDecision = currentDecision;
		currentDecision = DECISION_DRIVE;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision DRIVE");
		}
		break;

	case DECISION_STOP:
		prevDecision = currentDecision;
		currentDecision = DECISION_STOP;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision STOP");
		}
		break;

	case DECISION_PARALLEL_PARKING:
		prevDecision = currentDecision;
		currentDecision = DECISION_PARALLEL_PARKING;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision PARALLEL_PARKING");
		}
		break;
	
	case DECISION_CROSS_PARKING:
		prevDecision = currentDecision;
		currentDecision = DECISION_CROSS_PARKING;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision CROSS_PARKING");
		}
		break;

	case DECISION_PULL_OUT_LEFT:
		prevDecision = currentDecision;
		currentDecision = DECISION_PULL_OUT_LEFT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision PULL_OUT_LEFT");
		}
		break;

	case DECISION_PULL_OUT_RIGHT:
		prevDecision = currentDecision;
		currentDecision = DECISION_PULL_OUT_RIGHT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision PULL_OUT_RIGHT");
		}
		break;

	case DECISION_DETECT_CROSSROADS_AND_TURN_LEFT:
		prevDecision = currentDecision;
		currentDecision = DECISION_DETECT_CROSSROADS_AND_TURN_LEFT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision DETECT_CROSSROADS_AND_TURN_LEFT");
		}
		break;

	case DECISION_DETECT_CROSSROADS_AND_TURN_RIGHT:
		prevDecision = currentDecision;
		currentDecision = DECISION_DETECT_CROSSROADS_AND_TURN_RIGHT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision DETECT_CROSSROADS_AND_TURN_RIGHT");
		}
		break;

	case DECISION_DETECT_CROSSROADS_AND_DRIVE_STRAIGHT:
		prevDecision = currentDecision;
		currentDecision = DECISION_DETECT_CROSSROADS_AND_DRIVE_STRAIGHT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision DETECT_CROSSROADS_AND_DRIVE_STRAIGHT");
		}
		break;

	case DECISION_DETECT_CROSSROADS_STOP_AND_DRIVE_STRAIGHT:
		prevDecision = currentDecision;
		currentDecision = DECISION_DETECT_CROSSROADS_STOP_AND_DRIVE_STRAIGHT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision DECISION_DETECT_CROSSROADS_STOP_AND_DRIVE_STRAIGHT");
		}
		break;

	case DECISION_DETECT_CROSSROADS_STOP_AND_TURN_LEFT:
		prevDecision = currentDecision;
		currentDecision = DECISION_DETECT_CROSSROADS_STOP_AND_TURN_LEFT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision DECISION_DETECT_CROSSROADS_STOP_AND_TURN_LEFT");
		}
		break;

	case DECISION_DETECT_CROSSROADS_STOP_AND_TURN_RIGHT:
		prevDecision = currentDecision;
		currentDecision = DECISION_DETECT_CROSSROADS_STOP_AND_TURN_RIGHT;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision DECISION_DETECT_CROSSROADS_STOP_AND_TURN_RIGHT");
		}
		break;


	default:
		prevDecision = currentDecision;
		currentDecision = DECISION_STOP;
		
		if (isDebugActive)
		{
			LOG_INFO("Got Decision STOP");
		}
		break;
	}
}

bool DecisionEvaluator::isDecisionParking(const Decision &decision) const
{
	return decision == DECISION_CROSS_PARKING || decision == DECISION_PARALLEL_PARKING;
}

bool DecisionEvaluator::isDecisionPullOut(const Decision &decision) const
{
	return decision == DECISION_PULL_OUT_LEFT || decision == DECISION_PULL_OUT_RIGHT;
}

bool DecisionEvaluator::isDecisionDetectCrossroadAndStop(const Decision &decision) const
{
	return decision == DECISION_DETECT_CROSSROADS_STOP_AND_DRIVE_STRAIGHT || decision == DECISION_DETECT_CROSSROADS_STOP_AND_TURN_LEFT || decision == DECISION_DETECT_CROSSROADS_STOP_AND_TURN_RIGHT;
}

bool DecisionEvaluator::isDecisionDetectCrossroadAndStopTurnLeft(const Decision &decision) const
{
	return decision == DECISION_DETECT_CROSSROADS_STOP_AND_TURN_LEFT;
}

bool DecisionEvaluator::isDecisionDetectCrossroadAndStopTurnRight(const Decision &decision) const
{
	return decision == DECISION_DETECT_CROSSROADS_STOP_AND_TURN_RIGHT;
}

bool DecisionEvaluator::isDecisionDetectCrossroadAndDriveStraight(const Decision &decision) const
{
	return decision == DECISION_DETECT_CROSSROADS_AND_DRIVE_STRAIGHT;
}

bool DecisionEvaluator::isDecisionDetectCrossroadAndTurnLeft(const Decision &decision) const
{
	return decision == DECISION_DETECT_CROSSROADS_AND_TURN_LEFT;
}

bool DecisionEvaluator::isDecisionDetectCrossroadAndTurnRight(const Decision &decision) const
{
	return decision == DECISION_DETECT_CROSSROADS_AND_TURN_RIGHT;
}

void DecisionEvaluator::setIsDebugActive(const bool &isDebugActive)
{
	this->isDebugActive = isDebugActive;
}