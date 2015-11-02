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

/**
 * @filename
 * @copyright (c) Copyright 2014 SD-Team HTWK
 * @author dhecht
 * @details
 */
#include "stdafx.h"
#include "SimpleDrive.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, SimpleDrive);

SimpleDrive::SimpleDrive(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
	this->xPositionBefore = 560;
	this->yPositionBefore = 310;
	this->previousSteerAngle = 0;
	this->steerAngle = 0;
	this->isDriveActive = false;
	this->isTurnStraightActive = false;
	this->isTurnStraightWaitActive = false;
	this->isDriveStraightActive = false;
	this->initDriveStraight = true;

	// stop line
	this->standsOnStopLine = false;
	this->isDriveToStopLineActivated = false;
	this->neededDistance = 50000;
	this->isStopLineDetected = false;

	this->photoShot = false;
	this->isEmergencyBrakeActive = false;
	this->lastWheelRotation = 0;
	this->wheelRotationCounter = 0;

	// depthImage
	this->searchLeftAccess = false;
	this->isLeftAccesFound = false;

	// brake
	this->isHardBrake = true;

	// turn right
	this->isTurnRightActive = false;
	this->isStep1Active = false;
	this->isStep2Active = false;

	this->ticksToFinishStep1 = 0;
	this->ticksToFinishStep2 = 0;

	// turn left
	this->isTurnLeftActive = false;
	this->ticksToStopAtCrossroad = 0;

	this->isTurnLeftStep1Active = false;
	this->isTurnLeftStep2Active = false;
	this->isTurnLeftStep3Active = false;
	this->isTurnLeftStep4Active = false;
	this->isTurnLeftTimeSet = false;

	// RPM
	this->initRpm = true;
	this->currentTime = -1;
	this->previousTime = -1;
		
	// interfaces
	this->crossroadDetector.reset(new CrossRoadDetector());
	this->driveAlgorithm.reset(new DriveAlgorithm());
	this->obstacleDetection.reset(new Vdar());
	this->decisionEvaluator.reset(new DecisionEvaluator());
	this->emergencySystem.reset(new EmergencySystem());
	this->parkGap.reset(new ParkGap());

	loadReferenceBackground();

	// park gap
	this->searchParkGap = false;
	this->isParkGapFound = false;

	// properties
	initProperties();

	time1 = -1;
}

tVoid SimpleDrive::loadReferenceBackground(void)
{
	this->obstacleDetection->setReferenceBackground(imread("/home/odroid/Desktop/REPOS/AADC/main/HTWK_SD_FILTER/SD_SimpleDrive/depth.png"));
}

tResult SimpleDrive::initProperties()
{
	// smooth curve properties
	this->smoothCurveValue = 1;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE, this->smoothCurveValue));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_MINIMUM, 1));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_MAXIMUM, 255));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_ISCHANGEABLE, tTrue));
	this->driveAlgorithm->setSmoothCurveValue(this->smoothCurveValue);

	// lane assist properties
	MIN_RIGHT_LANE_POSITION = 540;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MIN_RIGHT_LANE_POSITION, MIN_RIGHT_LANE_POSITION));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MIN_RIGHT_LANE_POSITION NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MIN_RIGHT_LANE_POSITION NSSUBPROP_MAXIMUM, 639));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MIN_RIGHT_LANE_POSITION NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MIN_RIGHT_LANE_POSITION NSSUBPROP_ISCHANGEABLE, tTrue));
	this->driveAlgorithm->setMinRightLanePosition(MIN_RIGHT_LANE_POSITION);

	MAX_RIGHT_LANE_POSITION = 545;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MAX_RIGHT_LANE_POSITION, MAX_RIGHT_LANE_POSITION));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MAX_RIGHT_LANE_POSITION NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MAX_RIGHT_LANE_POSITION NSSUBPROP_MAXIMUM, 639));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MAX_RIGHT_LANE_POSITION NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MAX_RIGHT_LANE_POSITION NSSUBPROP_ISCHANGEABLE, tTrue));
	this->driveAlgorithm->setMaxRightLanePosition(MAX_RIGHT_LANE_POSITION);

	MIN_LEFT_LANE_POSITION = 305;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MIN_LEFT_LANE_POSITION, MIN_LEFT_LANE_POSITION));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MIN_LEFT_LANE_POSITION NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MIN_LEFT_LANE_POSITION NSSUBPROP_MAXIMUM, 479));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MIN_LEFT_LANE_POSITION NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MIN_LEFT_LANE_POSITION NSSUBPROP_ISCHANGEABLE, tTrue));
	this->driveAlgorithm->setMinLeftLanePosition(MIN_LEFT_LANE_POSITION);

	MAX_LEFT_LANE_POSITION = 310;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MAX_LEFT_LANE_POSITION, MAX_LEFT_LANE_POSITION));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MAX_LEFT_LANE_POSITION NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_MAX_LEFT_LANE_POSITION NSSUBPROP_MAXIMUM, 479));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MAX_LEFT_LANE_POSITION NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_MAX_LEFT_LANE_POSITION NSSUBPROP_ISCHANGEABLE, tTrue));
	this->driveAlgorithm->setMaxLeftLanePosition(MAX_LEFT_LANE_POSITION);

	// debug property
	this->isDebugActive = false;
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_DRIVE_ALGORITHM, this->isDebugActive));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_DRIVE_ALGORITHM NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_DRIVE_ALGORITHM NSSUBPROP_ISCHANGEABLE, tTrue));
	this->driveAlgorithm->setIsDebugActive(this->isDebugActive);

	// drive, break speed properties
	this->driveSpeed = 30.0;
	RETURN_IF_FAILED(SetPropertyFloat(PROP_NAME_DRIVE_SPEED, static_cast<tFloat64>(this->driveSpeed)));
	RETURN_IF_FAILED(SetPropertyFloat(PROP_NAME_DRIVE_SPEED NSSUBPROP_MINIMUM, 0.0));
    RETURN_IF_FAILED(SetPropertyFloat(PROP_NAME_DRIVE_SPEED NSSUBPROP_MAXIMUM, 100.0));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DRIVE_SPEED NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DRIVE_SPEED NSSUBPROP_ISCHANGEABLE, tTrue));

	this->breakSpeed = -15.0;
	RETURN_IF_FAILED(SetPropertyFloat(PROP_NAME_BREAK_SPEED, static_cast<tFloat64>(this->breakSpeed)));
	RETURN_IF_FAILED(SetPropertyFloat(PROP_NAME_BREAK_SPEED NSSUBPROP_MINIMUM, -20.0));
    RETURN_IF_FAILED(SetPropertyFloat(PROP_NAME_BREAK_SPEED NSSUBPROP_MAXIMUM, 0.0));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_BREAK_SPEED NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_BREAK_SPEED NSSUBPROP_ISCHANGEABLE, tTrue)); 

	this->ticksToStopLine = 5;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE, this->ticksToStopLine));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_MAXIMUM, 20));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_ISCHANGEABLE, tTrue)); 

	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_EMERGENCY_SYSTEM, false));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_EMERGENCY_SYSTEM NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_EMERGENCY_SYSTEM NSSUBPROP_ISCHANGEABLE, tTrue));
	this->emergencySystem->setIsDebugActive(false);

	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_IR_FRONT_CENTER, 8));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_IR_FRONT_CENTER NSSUBPROP_MINIMUM, 8));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_IR_FRONT_CENTER NSSUBPROP_MAXIMUM, 30));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_IR_FRONT_CENTER NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_IR_FRONT_CENTER NSSUBPROP_ISCHANGEABLE, tTrue)); 
	this->emergencySystem->setIrDistanceFront(8);

	// turn right
	this->turnRightTicksStraight = 5;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_STRAIGHT, this->turnRightTicksStraight));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_STRAIGHT NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_STRAIGHT NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_RIGHT_TICKS_STRAIGHT NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_RIGHT_TICKS_STRAIGHT NSSUBPROP_ISCHANGEABLE, tTrue));

	this->turnRightSteeringAngle = 2;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_STEERING_ANGLE, this->turnRightSteeringAngle));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_STEERING_ANGLE NSSUBPROP_MINIMUM, -30));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_STEERING_ANGLE NSSUBPROP_MAXIMUM, 30));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_RIGHT_STEERING_ANGLE NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_RIGHT_STEERING_ANGLE NSSUBPROP_ISCHANGEABLE, tTrue));
 
	this->turnRightTicksRight = 20;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_RIGHT, this->turnRightTicksRight));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_RIGHT NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_RIGHT NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_RIGHT_TICKS_RIGHT NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_RIGHT_TICKS_RIGHT NSSUBPROP_ISCHANGEABLE, tTrue));

	// turn left
	this->turnLeftMeasuringError = 20;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_ERROR, this->turnLeftMeasuringError));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_ISCHANGEABLE, tTrue));
	
	this->turnLeftWait = 5000000;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_ERROR, this->turnLeftMeasuringError));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_ERROR NSSUBPROP_ISCHANGEABLE, tTrue));

	this->turnLeftWait = 5000000;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_WAIT, this->turnLeftWait));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_WAIT NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_WAIT NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_WAIT NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_WAIT NSSUBPROP_ISCHANGEABLE, tTrue));

	this->turnLeftTicksStraight = 2;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_STRAIGHT, this->turnLeftTicksStraight));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_STRAIGHT NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_STRAIGHT NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_TICKS_STRAIGHT NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_TICKS_STRAIGHT NSSUBPROP_ISCHANGEABLE, tTrue));

	this->turnLeftSteeringAngle = -25;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_STEERING_ANGLE, this->turnLeftSteeringAngle));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_STEERING_ANGLE NSSUBPROP_MINIMUM, -30));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_STEERING_ANGLE NSSUBPROP_MAXIMUM, 30));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_STEERING_ANGLE NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_STEERING_ANGLE NSSUBPROP_ISCHANGEABLE, tTrue));

	this->turnLeftTicksLeft = 20;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_LEFT, this->turnLeftTicksLeft));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_LEFT NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_LEFT NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_TICKS_LEFT NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_TICKS_LEFT NSSUBPROP_ISCHANGEABLE, tTrue));

	this->turnLeftStopTicksStraight = 5;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_STOP_TICKS_STRAIGHT, this->turnLeftStopTicksStraight));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_STOP_TICKS_STRAIGHT NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_TURN_LEFT_STOP_TICKS_STRAIGHT NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_STOP_TICKS_STRAIGHT NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_TURN_LEFT_STOP_TICKS_STRAIGHT NSSUBPROP_ISCHANGEABLE, tTrue));

	this->turnStraightTicks = 280;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION, this->turnStraightTicks));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION NSSUBPROP_MINIMUM, 200));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION NSSUBPROP_MAXIMUM, 639));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION NSSUBPROP_ISCHANGEABLE, tTrue));
		
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_PARK_GAP, false));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_PARK_GAP NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_DEBUG_PARK_GAP NSSUBPROP_ISCHANGEABLE, tTrue));
	this->parkGap->setIsDebugActive(false);

	this->crossDriveStraightDistance = 30;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCE, this->crossDriveStraightDistance));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCE NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCE NSSUBPROP_MAXIMUM, 100));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTDISTANCE NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue));

	this->parallelDriveStraightDistance = 9500000;
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP, this->parallelDriveStraightDistance));
	RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP NSSUBPROP_MINIMUM, 0));
    RETURN_IF_FAILED(SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP NSSUBPROP_MAXIMUM, 1000000000));
    RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP NSSUBPROP_REQUIRED, tTrue));
	RETURN_IF_FAILED(SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP NSSUBPROP_ISCHANGEABLE, tTrue));

	RETURN_NOERROR;
}

SimpleDrive::~SimpleDrive(void)
{
}

tResult SimpleDrive::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	if (eStage == StageFirst)
	{	
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

		cObjectPtr<IMediaType> wheelTickMediaType;
		cObjectPtr<IMediaType> rpmMediaType;
		cObjectPtr<IMediaType> irMediaType;
		cObjectPtr<IMediaType> irRightMediaType;
		cObjectPtr<IMediaType> steeringAngleMediaType;
		cObjectPtr<IMediaType> accelerationMediaType;

		cObjectPtr<IMediaType> maneuverFinishedMediaType;
		cObjectPtr<IMediaType> decisionMediaType;
		cObjectPtr<IMediaType> lightMediaType;

		RETURN_IF_FAILED(initMediaType("tSignalValue", wheelTickMediaType, this->coderDescriptionWheelTicks));
		RETURN_IF_FAILED(initMediaType("tSignalValue", rpmMediaType, this->coderDescriptionRpm));
		RETURN_IF_FAILED(initMediaType("tSignalValue", irMediaType, this->coderDescriptionIr));
		RETURN_IF_FAILED(initMediaType("tSignalValue", irRightMediaType, this->coderDescriptionIrRight));
		RETURN_IF_FAILED(initMediaType("tSignalValue", steeringAngleMediaType, this->coderDescriptionSteeringAngle));
		RETURN_IF_FAILED(initMediaType("tSignalValue", accelerationMediaType, this->coderDescriptionAcceleration));
		
		RETURN_IF_FAILED(initMediaType("tSteeringAngleData", maneuverFinishedMediaType, this->coderDescriptionManeuverFinished));
		RETURN_IF_FAILED(initMediaType("tSteeringAngleData", decisionMediaType, this->coderDescriptionDecision));
		RETURN_IF_FAILED(initMediaType("tSteeringAngleData", lightMediaType, this->coderDescriptionLight));
		
		// input pins
		RETURN_IF_FAILED(createVideoInputPin("rgbVideo", this->ipmVideoInput));
		RETURN_IF_FAILED(createVideoInputPin("depthVideo", this->depthImage));
		RETURN_IF_FAILED(createInputPin("mergedWheelRotation", this->mergedWheelRotationPin, wheelTickMediaType));
		RETURN_IF_FAILED(createInputPin("RPM", this->rpmPin, rpmMediaType));
		RETURN_IF_FAILED(createInputPin("decision_in", this->decisionPin, decisionMediaType));
		RETURN_IF_FAILED(createInputPin("irFrontCenter", this->irFrontCenterPin, irMediaType));
		RETURN_IF_FAILED(createInputPin("irRight", this->irRightPin, irRightMediaType));

		// output pins
		RETURN_IF_FAILED(createOutputPin("steerAngleOut", this->steerAngleOutput, steeringAngleMediaType));
		RETURN_IF_FAILED(createOutputPin("acceleration", this->accelerationPin, accelerationMediaType));
		RETURN_IF_FAILED(createOutputPin("Maneuver_Finished", this->maneuverFinishedPin, maneuverFinishedMediaType));
		RETURN_IF_FAILED(createOutputPin("decision_out", this->decisionOutputPin, decisionMediaType));
		RETURN_IF_FAILED(createOutputPin("light", this->lightPin, lightMediaType));
	}
	else if (eStage == StageGraphReady)
	{
		cObjectPtr<IMediaSerializer> serializer;
		RETURN_IF_FAILED(this->coderDescriptionManeuverFinished->GetMediaSampleSerializer(&serializer));
		this->ddlSizeUI16 = serializer->GetDeserializedSize();

		RETURN_IF_FAILED(this->coderDescriptionAcceleration->GetMediaSampleSerializer(&serializer));
		this->ddlSizeF32 = serializer->GetDeserializedSize();
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
	tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
    RETURN_IF_POINTER_NULL(descriptionSignalValue);        

	mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
	RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

	RETURN_NOERROR;
}

tResult SimpleDrive::createVideoInputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult SimpleDrive::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult SimpleDrive::createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult SimpleDrive::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult SimpleDrive::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult SimpleDrive::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult SimpleDrive::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);

	if (eventCore == IPinEventSink::PE_MediaSampleReceived)
	{
		if (source == &this->depthImage)
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(processDepthImage(mediaSample), "cant read out depth image");
		}
		else if (source == &this->irRightPin)
		{
			if (this->searchParkGap)
			{
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(processSearchParkGap(), "cant process search park gap");
			}
		}
		else if (source == &this->ipmVideoInput)
		{
			if (this->isFirstFrame)
			{
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(initVideoStream(), "Cant init video stream");
				this->isFirstFrame = false;
			}
			else 
			{
				if (this->emergencySystem->isObstacleFrontCenter(this->irFrontValue) && this->isDriveActive)
				{
					// LOG_WARNING("Obstacle detected");
					RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitStop(this->breakSpeed), "Cant stop the car");
					this->isEmergencyBrakeActive = true;
				}
				else 
				{
					if (this->isDriveActive && this->isEmergencyBrakeActive)
					{
						RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitDrive(this->driveSpeed), "Cant drive");
						this->isEmergencyBrakeActive = false;
					}

					RETURN_IF_FAILED_AND_LOG_ERROR_STR(processImage(mediaSample), "Error in image processing");
				}
			}
		}
		else if (source == &this->mergedWheelRotationPin)
		{
			processWheelTicks(mediaSample);
		}
		else if (source == &this->decisionPin)
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getDecision(mediaSample, this->currentDecision, this->prevDecision, this->ticksToCrossroad), "Cant get Decision from RoadSignDetection");
			
			if (this->prevDecision == DECISION_PULL_OUT_LEFT || this->prevDecision == DECISION_PULL_OUT_RIGHT)
			{
				this->steerAngle = 0;
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitSteeringAngle(this->steerAngle), "Cant init steer angle");
			}

			if (this->decisionEvaluator->isDecisionParking(this->currentDecision))
			{
				// leite den Befehl weiter an den MakroPlayer
				// bzw. suche nach einer Parklücke und leite dann den Befehl weiter
				/*this->isDriveActive = false;
				LOG_WARNING("Stop because macro player");
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitStop(this->breakSpeed), "cant stop the car");
				
				LOG_INFO("Sending parking decision to macro_player");
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitDecision(this->currentDecision), "cant send decision to macro player");*/

				this->searchParkGap = true;
				this->parkGap->setDecision(this->currentDecision);
			}
			else if (this->decisionEvaluator->isDecisionPullOut(this->currentDecision))
			{
				this->isDriveActive = false;
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitStop(this->breakSpeed), "cant stop the car");
				
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitDecision(this->currentDecision), "cant send decision to macro player");
			}
			else if (this->currentDecision == DECISION_STOP)
			{
				this->isDriveActive = false;
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitStop(this->breakSpeed), "cant stop the car");
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitDecision(DECISION_STOP), "cant send stop to macro_player");
			}
			else if (this->currentDecision == DECISION_DRIVE)
			{
				this->isDriveActive = true;
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitDrive(this->driveSpeed), "cant start the car");
				
				// init steering angle
				this->steerAngle = 0;
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitSteeringAngle(this->steerAngle), "Cant init steer angle");
			}
		}
		else if (source == &this->irFrontCenterPin)
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getIrValue(mediaSample, this->irFrontValue), "Cant read out ir front center value");
		}
		else if (source == &this->rpmPin)
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getRpm(mediaSample, this->currentRPM), "Cant get RPM");
		}
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::initVideoStream(void)
{
	//Read media type
	cObjectPtr<IMediaType> type;
	RETURN_IF_FAILED(this->ipmVideoInput.GetMediaType(&type));

	cObjectPtr<IMediaTypeVideo> typeVideo;
	RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)(&typeVideo)));

	const tBitmapFormat *format = typeVideo->GetFormat();
	RETURN_IF_POINTER_NULL(format);

	//Set media type
	setBitmapFormat(format);

	RETURN_NOERROR;
}

tResult SimpleDrive::processSearchParkGap()
{
	if (!this->isParkGapFound)
	{
		if (this->parkGap->searchParkingGap(this->irFrontValue, this->wheelRotation))
		{
			this->isParkGapFound = true;

			this->distanceToFinishCrossParking = this->crossDriveStraightDistance + (this->wheelRotation * 4);
			this->timeToFinishParallelParking = cHighResTimer::GetTime() + this->parallelDriveStraightDistance;
		}
	}
	else
	{
		if (this->currentDecision == DECISION_CROSS_PARKING)
		{
			static tInt tmpDistance;
			tmpDistance = this->distanceToFinishCrossParking - (this->wheelRotation * 4);

			if (tmpDistance <= 0)
			{
				this->searchParkGap = false;
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitStop(this->breakSpeed), "cant stop the car");

				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitDecision(this->currentDecision), "cant send decision to macro player");

				this->isDriveActive = false;
				this->isParkGapFound = false;
			}
		}
		else if (this->currentDecision == DECISION_PARALLEL_PARKING)
		{
			if (cHighResTimer::GetTime() >= this->timeToFinishParallelParking)
			{
				this->searchParkGap = false;
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitStop(this->breakSpeed), "cant stop the car");

				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitDecision(this->currentDecision), "cant send decision to macro player");

				this->isDriveActive	= false;
				this->isParkGapFound = false;
			}
		}
	}
	

	RETURN_NOERROR;
}

tVoid SimpleDrive::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInputInfo.nPaletteSize = format->nPaletteSize;
	this->videoInputInfo.nPixelFormat = format->nPixelFormat;
	this->videoInputInfo.nHeight = format->nHeight;
	this->videoInputInfo.nWidth = format->nWidth;
	this->videoInputInfo.nSize = format->nSize;
}

tResult SimpleDrive::processImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (isDriveActive && IS_OK(mediaSample->Lock(&buffer)))
	{
		//Receive the image
		Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*) buffer);
		Mat result = image.clone();
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaSample->Unlock(buffer), "Cant unlock image");

		this->driveAlgorithm->prepareImage(result);

		// Schritt 1
		// x-Pixel der rechten Fahrspur bestimmen
		static tInt xPosition;
		static tInt yPosition;

		static tBool isRightLaneDetected;
		static tBool isLeftLaneDetected;

		isRightLaneDetected = this->driveAlgorithm->getRightLanePosition(result, xPosition);

		isLeftLaneDetected = this->driveAlgorithm->getLeftLanePosition(result, yPosition);

		if (!this->isTurnLeftActive && this->decisionEvaluator->isDecisionDetectCrossroadAndTurnLeft(this->currentDecision))
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitLight(LIGHT_TURN_LEFT), "cant send turn left");
			this->currentTurn = TURN_LEFT;
			this->isTurnLeftActive = true;
			this->isTurnLeftStep1Active = true;
			this->initRpm = true;

			this->ticksToStopAtCrossroad = this->wheelRotation + static_cast<tInt>(this->ticksToCrossroad) - this->turnLeftMeasuringError;
		}
		else if (this->decisionEvaluator->isDecisionDetectCrossroadAndTurnRight(this->currentDecision))
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitLight(LIGHT_TURN_RIGHT), "cant send turn right");

			if (!isRightLaneDetected)
			{
				this->isDriveActive = false;
				this->isTurnRightActive = true;
				this->isStep1Active = true;
				this->ticksToFinishStep1 = this->wheelRotation + this->turnRightTicksStraight;
				this->initRpm = true;
			}
		}
		// Geradeaus fahren
		else if (!this->isTurnStraightActive && this->decisionEvaluator->isDecisionDetectCrossroadAndDriveStraight(this->currentDecision))
		{
			// 2 Varianten
			if (isRightLaneDetected)
			{
				if (this->initDriveStraight)
				{
					this->ticksToStopAtCrossroad = this->wheelRotation + static_cast<tInt>(this->ticksToCrossroad) - this->turnLeftMeasuringError;
					this->initDriveStraight = false;
				}
				else
				{
					RETURN_IF_FAILED_AND_LOG_ERROR_STR(driveStraight(), "failed to drive straight");
				}
			}
			else
			{
				if (!this->isTurnStraightActive)
				{
					this->isTurnStraightActive = true;
					this->turnStraightTicksToFinish = 10 + this->wheelRotation;
				}
			}
			// 1 fahren bis rechts gasse kommt und dann 20 ticks oder
			// 2 fahre so weit bis das Schild passiert is
		}

		/****** Steering ******/
		if (!this->isTurnStraightActive)
		{
			if (!isRightLaneDetected)
			{
				if (isLeftLaneDetected)
				{
					this->driveAlgorithm->calculateSteeringAngle(LANE_LEFT, yPosition, this->steerAngle);
					RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitSteeringAngle(this->steerAngle), "Cant steer :( OMG!!!");
				}
				else
				{
					this->driveAlgorithm->getLeftLanePositionFromX(image, this->steerAngle);
					RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitSteeringAngle(this->steerAngle), "cant transmit steering angle");
				}
			}
			else
			{
				// LOG_INFO(cString::Format("x-Position: %d", xPosition));
				// Lenkwinkel berechnen	
				this->driveAlgorithm->calculateSteeringAngle(LANE_RIGHT, xPosition, this->steerAngle);

				// Schritt 3 
				// Lenkwinkel senden
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitSteeringAngle(this->steerAngle), "Cant steer :( OMG!!!");
			}
		}

		// Schritt 2 
		if (this->isTurnStraightActive || this->isTurnStraightWaitActive)
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(driveStraightOverCrossroad(result), "Failed to drive over the crossroad");
		}
		else if (this->decisionEvaluator->isDecisionDetectCrossroadAndStop(this->currentDecision))
		{
			if (this->decisionEvaluator->isDecisionDetectCrossroadAndStopTurnLeft(this->currentDecision))
			{
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitLight(LIGHT_TURN_LEFT), "cant send turn left");
			}
			else if (this->decisionEvaluator->isDecisionDetectCrossroadAndStopTurnRight(this->currentDecision))
			{
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitLight(LIGHT_TURN_RIGHT), "cant send turn left");
			}

			if (!this->standsOnStopLine)
			{
				if (!this->isStopLineDetected)
				{
					if (this->crossroadDetector->searchStopLine(result))
					{
						this->neededDistance = this->wheelRotation + this->ticksToStopLine;
						this->isDriveToStopLineActivated = true;
						this->isStopLineDetected = true;
					}
				}
			}
			else
			{
				// turn left
				if (this->decisionEvaluator->isDecisionDetectCrossroadAndStopTurnLeft(this->currentDecision))
				{
					this->isTurnLeftActive = true;
					this->isTurnLeftStep2Active = true;
					this->currentTurn = TURN_LEFT_STOP;
					this->initRpm = true;
				}
				// turn right
				else if (this->decisionEvaluator->isDecisionDetectCrossroadAndStopTurnRight(this->currentDecision))
				{
					this->isDriveActive = false;
					this->isTurnRightActive = true;
					this->initRpm = true;
					this->isTurnRightWaitActive = true;
				}
				else
				{
					// drive straight!
					// wir stehen an der Haltelinie
					this->isTurnStraightWaitActive = true;
				}
			}
		}
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::processDepthImage(IMediaSample *mediaSample)
{
	const tVoid *buffer;

	if (this->searchLeftAccess && IS_OK(mediaSample->Lock(&buffer)))
	{
		//Receive the image
		Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_16U, (char*) buffer);
		Mat depthImage = image.clone();
		RETURN_IF_FAILED(mediaSample->Unlock(buffer));

		if (this->obstacleDetection->isObstacleInAreaInDepthImg(Rect(-100, 150, 30, -20), depthImage))
		{
			RETURN_IF_FAILED(transmitStop(this->breakSpeed));
			this->searchLeftAccess = false;
		}
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::processWheelTicks(IMediaSample *mediaSample)
{
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(getWheelTicks(mediaSample, this->wheelRotation), "Cant get distance");

	if (isDriveToStopLineActivated)
	{
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(driveToStopLine(), "drive to stop line failed");
	}
	else if (this->isTurnRightActive)
	{
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(turnRight(), "turn right failed");
	}
	else if (this->isTurnLeftActive)
	{
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(turnLeft(), "cant turn left!");
	}
	else if (this->searchParkGap)
	{
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(processSearchParkGap(), "cant process search park gap");
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::processDecisions(IMediaSample *mediaSample)
{
	RETURN_NOERROR;
}

tResult SimpleDrive::processRPM(void)
{
	static tInt counter;
	static tFloat32 driveInc;

	if (this->initRpm)
	{
		counter = 0;
		driveInc = 0.0f;
		this->initRpm = false;
	}
	else
	{
		counter++;

		if (counter >= 50)
		{
			this->driveSpeed += driveInc;
			RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			counter = 0;
			driveInc += 1.5f;
		}
	}

	RETURN_NOERROR;
}

tVoid SimpleDrive::validateSteeringAngle(tInt &steeringAngle)
{
	const static tInt MIN_STEER_ANGLE = -30;
	const static tInt MAX_STEER_ANGLE = 30;

	steeringAngle = (steeringAngle < MIN_STEER_ANGLE) ? MIN_STEER_ANGLE : steeringAngle;
	steeringAngle = (steeringAngle > MAX_STEER_ANGLE) ? MAX_STEER_ANGLE : steeringAngle;
}

tResult SimpleDrive::driveToStopLine()
{
	static tInt tmpDistance;
	tmpDistance = this->neededDistance - this->wheelRotation;
	
	if (tmpDistance <= 0)
	{
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitStop(this->breakSpeed), "Cant stop the car");
		this->isDriveToStopLineActivated = false;
		this->standsOnStopLine = true;
		this->isStopLineDetected = false;
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::turnRight()
{
	if (this->isTurnRightWaitActive)
	{
		// wait
		if (!this->isTurnLeftTimeSet)
		{
			this->turnLeftTime = cHighResTimer::GetTime() + this->turnLeftWait;
			
			RETURN_IF_FAILED(transmitStop(this->breakSpeed));
			this->isTurnLeftTimeSet = true;
		}
		else
		{
			if ((this->turnLeftTime - cHighResTimer::GetTime()) <= 0)
			{
				// time is over
				this->isTurnLeftTimeSet = false;
				this->isTurnRightWaitActive = false;
				this->isStep1Active = true;

				this->turnStraightTicksToFinish = this->turnStraightTicks + this->wheelRotation;
				
				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			}
		}
	}
	else if (this->isStep1Active)
	{
		static tInt tmpDistance1;
		tmpDistance1 = this->ticksToFinishStep1 - this->wheelRotation;

		if (this->currentRPM >= 0 && this->currentRPM <= 35)
		{
			RETURN_IF_FAILED(processRPM());
		}
		else if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
		{
			this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
			RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
		}

		if (tmpDistance1 <= 0)
		{
			if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
			{
				this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			}

			this->isStep1Active = false;
			this->isStep2Active = true;

			this->ticksToFinishStep2 = this->wheelRotation + this->turnRightTicksRight;
			this->steerAngle = this->turnRightSteeringAngle;

			RETURN_IF_FAILED(transmitSteeringAngle(this->steerAngle));
		}
	}
	else if (this->isStep2Active)
	{
		static tInt tmpDistance3;
		tmpDistance3 = this->ticksToFinishStep2 - this->wheelRotation;

		if (this->currentRPM >= 0 && this->currentRPM <= 35)
		{
			RETURN_IF_FAILED(processRPM());
		}
		else if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
		{
			this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
			RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
		}

		if (tmpDistance3 <= 0)
		{
			if (this->driveSpeed > GetPropertyFloat(PROP_NAME_DRIVE_SPEED))
			{
				this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			}

			this->isStep2Active = false;
			this->isTurnRightActive = false;
			
			this->steerAngle = 0;
			RETURN_IF_FAILED(transmitSteeringAngle(this->steerAngle));
			RETURN_IF_FAILED(transmitLight(LIGHT_TURN_DISABLED));
			RETURN_IF_FAILED(transmitManeuverIsFinished());

			this->isDriveActive = true;
		}
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::turnLeft()
{
	static tInt tmpTicks;
	tmpTicks = this->ticksToStopAtCrossroad - this->wheelRotation;

	if (this->isTurnLeftStep1Active)
	{
		if (tmpTicks <= 0)
		{
			this->isDriveActive = false;
			this->searchLeftAccess = false;

			RETURN_IF_FAILED(transmitStop(this->breakSpeed));

			this->isTurnLeftStep1Active = false;
			this->isTurnLeftStep2Active = true;
		}
	}
	else if (this->isTurnLeftStep2Active)
	{
		this->isDriveActive = false;

		// wait
		if (!this->isTurnLeftTimeSet)
		{
			this->turnLeftTime = cHighResTimer::GetTime() + this->turnLeftWait;
			
			RETURN_IF_FAILED(transmitStop(this->breakSpeed));
			this->isTurnLeftTimeSet = true;
		}
		else
		{
			if ((this->turnLeftTime - cHighResTimer::GetTime()) <= 0)
			{
				// time is over
				this->isTurnLeftTimeSet = false;
				this->isTurnLeftStep2Active = false;
				this->isTurnLeftStep3Active = true;

				if (this->currentTurn == TURN_LEFT)
				{
					this->turnLeftTicksToFinishDriveStraight = this->turnLeftTicksStraight + this->wheelRotation;
				}
				else if (this->currentTurn == TURN_LEFT_STOP)
				{
					this->turnLeftTicksToFinishDriveStraight = this->turnLeftStopTicksStraight + this->wheelRotation;
				}

				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			}
		}
	}
	else if (this->isTurnLeftStep3Active)
	{
		if (this->currentRPM >= 0 && this->currentRPM <= 35)
		{
			RETURN_IF_FAILED(processRPM());
		}
		else if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
		{
			this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
			RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
		}

		static tInt tmpDistance;
		tmpDistance = this->turnLeftTicksToFinishDriveStraight - this->wheelRotation;

		if (tmpDistance <= 0)
		{
			if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
			{
				this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			}

			this->isTurnLeftStep3Active = false;
			this->isTurnLeftStep4Active = true;

			this->turnLeftTicksToFinishDriveLeft = this->wheelRotation + this->turnLeftTicksLeft;
			this->steerAngle = this->turnLeftSteeringAngle;

			RETURN_IF_FAILED(transmitSteeringAngle(this->steerAngle));
		}
	}
	else if (this->isTurnLeftStep4Active)
	{
		static tInt tmpDistance;
		tmpDistance = this->turnLeftTicksToFinishDriveLeft - this->wheelRotation;

		if (this->currentRPM >= 0 && this->currentRPM <= 35)
		{
			RETURN_IF_FAILED(processRPM());
		}
		else if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
		{
			this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
			RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
		}

		if (tmpDistance <= 0)
		{
			this->isTurnLeftStep4Active = false;
			this->isTurnLeftActive = false;
			
			if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
			{
				this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			}

			this->steerAngle = 0;
			RETURN_IF_FAILED(transmitSteeringAngle(this->steerAngle));
			RETURN_IF_FAILED(transmitLight(LIGHT_TURN_DISABLED));
			RETURN_IF_FAILED(transmitLight(LIGHT_TURN_DISABLED));
			RETURN_IF_FAILED(transmitManeuverIsFinished());

			this->isDriveActive = true;
		}
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::driveStraightOverCrossroad(const Mat &image)
{

	if (this->isTurnStraightWaitActive)
	{
		// wait
		if (!this->isTurnLeftTimeSet)
		{
			this->turnLeftTime = cHighResTimer::GetTime() + this->turnLeftWait;

			RETURN_IF_FAILED(transmitStop(this->breakSpeed));
			this->isTurnLeftTimeSet = true;
		}
		else
		{
			if ((this->turnLeftTime - cHighResTimer::GetTime()) <= 0)
			{
				this->turnStraightTicksToFinish = this->turnStraightTicks + this->wheelRotation;

				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));

				this->isTurnStraightActive = true;
				this->isTurnStraightWaitActive = false;
				this->isTurnLeftTimeSet = false;
			}
		}
	}
	else if (this->isTurnStraightActive)
	{
		static tInt tmpDistance;
		tmpDistance = this->turnStraightTicksToFinish - this->wheelRotation;
		
		static tInt position;
		this->driveAlgorithm->getCrossroadStraightLanePosition(image, position);
		this->driveAlgorithm->calculateSteeringAngle(LANE_CROSSROAD, position, this->steerAngle);

		RETURN_IF_FAILED(transmitSteeringAngle(this->steerAngle));

		if (this->currentRPM >= 0 && this->currentRPM <= 35)
		{
			RETURN_IF_FAILED(processRPM());
		}
		else if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
		{
			this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
			RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
		}
		
		if (tmpDistance <= 0)
		{
			if (this->driveSpeed > static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED)))
			{
				this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
				RETURN_IF_FAILED(transmitDrive(this->driveSpeed));
			}

			this->isTurnStraightActive = false;

			RETURN_IF_FAILED(transmitManeuverIsFinished());
		}
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::driveStraight(void)
{
	static tInt tmpDistance;
	tmpDistance = this->ticksToStopAtCrossroad - this->wheelRotation;
	
	if (tmpDistance <= 0)
	{
		RETURN_IF_FAILED(transmitManeuverIsFinished());

		this->initDriveStraight = true;
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::PropertyChanged(const char *name)
{
	// drive, break speed properties
	this->driveSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_DRIVE_SPEED));
	this->breakSpeed = static_cast<tFloat32>(GetPropertyFloat(PROP_NAME_BREAK_SPEED));
	this->ticksToStopLine = GetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE);

	// smooth curve properties
	this->smoothCurveValue = GetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE);
	this->driveAlgorithm->setSmoothCurveValue(this->smoothCurveValue);
	
	// lane assist properties
	MIN_RIGHT_LANE_POSITION = GetPropertyInt(PROP_NAME_MIN_RIGHT_LANE_POSITION);
	this->driveAlgorithm->setMinRightLanePosition(GetPropertyInt(PROP_NAME_MIN_RIGHT_LANE_POSITION));

	MAX_RIGHT_LANE_POSITION = GetPropertyInt(PROP_NAME_MAX_RIGHT_LANE_POSITION);
	this->driveAlgorithm->setMaxRightLanePosition(GetPropertyInt(PROP_NAME_MAX_RIGHT_LANE_POSITION));

	MIN_LEFT_LANE_POSITION = GetPropertyInt(PROP_NAME_MIN_LEFT_LANE_POSITION);
	this->driveAlgorithm->setMinLeftLanePosition(GetPropertyInt(PROP_NAME_MIN_LEFT_LANE_POSITION));
	
	MAX_LEFT_LANE_POSITION = GetPropertyInt(PROP_NAME_MAX_LEFT_LANE_POSITION);
	this->driveAlgorithm->setMaxLeftLanePosition(GetPropertyInt(PROP_NAME_MAX_LEFT_LANE_POSITION));
	
	// debug propertiy
	this->isDebugActive = GetPropertyBool(PROP_NAME_DEBUG_DRIVE_ALGORITHM);
	this->driveAlgorithm->setIsDebugActive(GetPropertyBool(PROP_NAME_DEBUG_DRIVE_ALGORITHM));

	this->emergencySystem->setIsDebugActive(GetPropertyBool(PROP_NAME_DEBUG_EMERGENCY_SYSTEM));
	this->emergencySystem->setIrDistanceFront(GetPropertyInt(PROP_NAME_IR_FRONT_CENTER));

	// turn right
	this->turnRightTicksStraight = GetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_STRAIGHT);
	this->turnRightTicksRight = GetPropertyInt(PROP_NAME_TURN_RIGHT_TICKS_RIGHT);
	this->turnRightSteeringAngle = GetPropertyInt(PROP_NAME_TURN_RIGHT_STEERING_ANGLE);

	// turn left
	this->turnLeftMeasuringError = GetPropertyInt(PROP_NAME_TURN_LEFT_ERROR);
	this->turnLeftWait = GetPropertyInt(PROP_NAME_TURN_LEFT_WAIT);
	this->turnLeftTicksStraight = GetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_STRAIGHT);
	this->turnLeftSteeringAngle = GetPropertyInt(PROP_NAME_TURN_LEFT_STEERING_ANGLE);
	this->turnLeftTicksLeft = GetPropertyInt(PROP_NAME_TURN_LEFT_TICKS_LEFT);
	this->turnLeftStopTicksStraight = GetPropertyInt(PROP_NAME_TURN_LEFT_STOP_TICKS_STRAIGHT);
	this->turnStraightTicks = GetPropertyInt(PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION);
	this->driveAlgorithm->setCrossroadStraightLanePosition(GetPropertyInt(PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION));

	this->parkGap->setIsDebugActive(GetPropertyBool(PROP_NAME_DEBUG_PARK_GAP));

	this->parallelDriveStraightDistance = GetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP);
	this->crossDriveStraightDistance = GetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCE);

	RETURN_NOERROR;
}

// get values
tResult SimpleDrive::getSteeringAngle(IMediaSample *mediaSample, tInt &steeringAngle)
{
	static tFloat32 tmpSteeringAngle;

	RETURN_IF_FAILED(getF32Value(mediaSample, this->coderDescriptionSteeringAngle, tmpSteeringAngle));
	steeringAngle = static_cast<tInt>(tmpSteeringAngle );

	RETURN_NOERROR;
}

tResult SimpleDrive::getIrValue(IMediaSample *mediaSample, tInt &irValue)
{
	static tFloat32 tmpIrValue;

	RETURN_IF_FAILED(getF32Value(mediaSample, this->coderDescriptionIr, tmpIrValue));
	irValue = static_cast<tInt>(tmpIrValue);

	RETURN_NOERROR;
}

tResult SimpleDrive::getWheelTicks(IMediaSample *mediaSample, tInt &wheelTicks)
{
	static tFloat32 tmpWheelTicks;

	RETURN_IF_FAILED(getF32Value(mediaSample, this->coderDescriptionWheelTicks, tmpWheelTicks));
	wheelTicks = static_cast<tInt>(tmpWheelTicks);

	RETURN_NOERROR;
}

tResult SimpleDrive::getRpm(IMediaSample *mediaSample, tInt &rpm)
{
	static tFloat32 tmpRpm;

	RETURN_IF_FAILED(getF32Value(mediaSample, this->coderDescriptionRpm, tmpRpm));
	rpm = static_cast<tInt>(tmpRpm);

	RETURN_NOERROR;
}

tResult SimpleDrive::getF32Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tFloat32 &value)
{
	static tFloat32 tmpValue;
	static tTimeStamp timeStamp;

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Lock(mediaSample, &coder), "Get32 Failed to lock f32");
            
	coder->Get("f32Value", (tVoid*) &tmpValue);
	coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
	value = tmpValue;

	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Get32 Failed to unlock f32");

	RETURN_NOERROR;
}
	
tResult SimpleDrive::getDecision(IMediaSample *mediaSample, Decision &currentDecision, Decision &previousDecision, tUInt32 &ticksToCrossroad)
{
	static tUInt16 tmpDecisionValue;

	RETURN_IF_FAILED(getUI16Value(mediaSample, this->coderDescriptionDecision, tmpDecisionValue, ticksToCrossroad));
	this->decisionEvaluator->getDecisionFromValue(tmpDecisionValue, currentDecision, previousDecision);

	RETURN_NOERROR;
}

tResult SimpleDrive::getUI16Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tUInt16 &value, tUInt32 &ticksToCrossroad)
{
	static tUInt16 tmpValue;

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Lock(mediaSample, &coder), "Get UI16 failed to unlock");
	
	coder->Get("ui16Angle", (tVoid*) &tmpValue);
	coder->Get("ui32ArduinoTimestamp", (tVoid*) &ticksToCrossroad);
	value = tmpValue;

	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Get UI16 failed to unlock");

	RETURN_NOERROR;
}

// transmit values
tResult SimpleDrive::transmitSteeringAngle(tInt &steeringAngle)
{
	validateSteeringAngle(steeringAngle);

	if (this->previousSteerAngle != this->steerAngle)
	{
		RETURN_IF_FAILED(transmitF32Value(this->steerAngleOutput, this->coderDescriptionSteeringAngle, static_cast<tFloat32>(steeringAngle)));
	}

	this->previousSteerAngle = this->steerAngle;

	RETURN_NOERROR;
}

tResult SimpleDrive::transmitDrive(const tFloat32 &driveSpeed)
{
	RETURN_IF_FAILED(transmitLight(LIGHT_HEAD));
	RETURN_IF_FAILED(transmitLight(LIGHT_BREAK_DISABLED));

	if (this->prevDecision == DECISION_STOP)
	{
		RETURN_IF_FAILED(transmitLight(LIGHT_TURN_DISABLED));
	}

	RETURN_IF_FAILED(transmitF32Value(this->accelerationPin, this->coderDescriptionAcceleration, driveSpeed));
	this->isHardBrake = true;

	RETURN_NOERROR;
}

tResult SimpleDrive::transmitStop(const tFloat32 &stopSpeed)
{
	RETURN_IF_FAILED(transmitLight(LIGHT_BREAK));

	if (this->isHardBrake)
	{
		RETURN_IF_FAILED(transmitF32Value(this->accelerationPin, this->coderDescriptionAcceleration, stopSpeed));
		this->isHardBrake = false;
	}
	else
	{
		RETURN_IF_FAILED(transmitF32Value(this->accelerationPin, this->coderDescriptionAcceleration, 0.0f));
	}

	RETURN_NOERROR;
}

tResult SimpleDrive::transmitF32Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tFloat32 value)
{
	cObjectPtr<IMediaSample> mediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeF32));
       
    // write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->WriteLock(mediaSample, &coder), "Set F32 Failed to lock f32");	
		
	static tTimeStamp now;
	now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

	coder->Set("f32Value", (tVoid*) &value);
    coder->Set("ui32ArduinoTimestamp", (tVoid*) &now);
	
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Set F32 Failed to lock f32");
    
    // transmit media sample over output pin
    RETURN_IF_FAILED(mediaSample->SetTime(now));
    RETURN_IF_FAILED(pin.Transmit(mediaSample));
	
	RETURN_NOERROR;
}

tResult SimpleDrive::transmitManeuverIsFinished(void)
{
	static tUInt16 maneuverFinished;
	maneuverFinished = MANEUVER_FINISHED;
	this->standsOnStopLine = false;
	RETURN_IF_FAILED(transmitUI16Value(this->maneuverFinishedPin, this->coderDescriptionManeuverFinished, maneuverFinished));

	RETURN_NOERROR;
}

tResult SimpleDrive::transmitDecision(const Decision &decision)
{
	static tUInt16 decisionValue;
	decisionValue = decision;

	RETURN_IF_FAILED(transmitUI16Value(this->decisionOutputPin, this->coderDescriptionDecision, decisionValue));

	RETURN_NOERROR;
}

tResult SimpleDrive::transmitLight(const Light &light)
{
	static tUInt16 lightValue;
	lightValue = light;

	RETURN_IF_FAILED(transmitUI16Value(this->lightPin, this->coderDescriptionLight, lightValue));

	RETURN_NOERROR;
}

tResult SimpleDrive::transmitUI16Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tUInt16 value)
{
	cObjectPtr<IMediaSample> mediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeUI16));

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->WriteLock(mediaSample, &coder), "Set UI16 failed to unlock");
	
	static tTimeStamp now;
	now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

	coder->Set("ui16Angle", (tVoid*) &value);
	coder->Set("ui32ArduinoTimestamp", (tVoid*) &now);
	
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Set UI16 failed to unlock");

	RETURN_IF_FAILED(mediaSample->SetTime(now));
	RETURN_IF_FAILED(pin.Transmit(mediaSample));

	RETURN_NOERROR;
}