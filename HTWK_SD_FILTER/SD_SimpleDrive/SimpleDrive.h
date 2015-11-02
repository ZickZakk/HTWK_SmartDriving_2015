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

#ifndef _SimpleDrive_H_
#define _SimpleDrive_H_

#ifdef _DEBUG
#define FILTER_NAME "SD SimpleDrive Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.SimpleDrive.debug"
#else
#define FILTER_NAME "SD Engine"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.SimpleDrive.release"
#endif

#define PROP_NAME_DRIVE_SPEED "1_DriveSpeed"
#define PROP_NAME_BREAK_SPEED "2_BreakSpeed"
#define PROP_NAME_TICKS_TO_STOP_LINE "3_TicksToStopLine"

#define PROP_NAME_DEBUG_DRIVE_ALGORITHM "4_DebugDriveAlgorithm"
#define PROP_NAME_MIN_RIGHT_LANE_POSITION "5_MinRightLanePosition"
#define PROP_NAME_MAX_RIGHT_LANE_POSITION "6_MaxRightLanePosition"
#define PROP_NAME_MIN_LEFT_LANE_POSITION "7_MinLeftLanePosition"
#define PROP_NAME_MAX_LEFT_LANE_POSITION "8_MaxLeftLanePosition"
#define PROP_NAME_SMOOTH_CURVE_VALUE "9_SmoothCurveValue"

#define PROP_NAME_DEBUG_DECISION_EVALUATOR "10_DebugDecisionEvaluator"
#define PROP_NAME_DEBUG_EMERGENCY_SYSTEM "11_DebugEmergencySystem"
#define PROP_NAME_IR_FRONT_CENTER "12_IrFrontCenterValue"

#define PROP_NAME_TURN_RIGHT_TICKS_STRAIGHT "99_0_TurnRightTicksStraight"
#define PROP_NAME_TURN_RIGHT_STEERING_ANGLE "99_1_TurnRightSteeringAngle"
#define PROP_NAME_TURN_RIGHT_TICKS_RIGHT "99_2_TurnRightTicksRight"
#define PROP_NAME_TURN_LEFT_ERROR "99_3_TurnLeftError"
#define PROP_NAME_TURN_LEFT_WAIT "99_4_TurnLeftWait"
#define PROP_NAME_TURN_LEFT_TICKS_STRAIGHT "99_5_TurnLeftTicksStraight"
#define PROP_NAME_TURN_LEFT_STEERING_ANGLE "99_6_TurnLeftSteeringAnle"
#define PROP_NAME_TURN_LEFT_TICKS_LEFT "99_7_TurnLeftTicksLeft"
#define PROP_NAME_TURN_LEFT_STOP_TICKS_STRAIGHT "99_8_TurnLeftStopTicksStraight"

#define PROP_NAME_CROSSROAD_STRAIGHT_LANE_POSITION "98_0_CrossroadStraightLanePosition"
#define PROP_NAME_DEBUG_PARK_GAP "98_1_DebugSearchParkGap"
#define PROP_NAME_CPDRIVESTRAIGHTDISTANCE "6_Cross_DriveStraightDistance"
#define PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP "7_Parallel_driveStraightBeforeGoInGapDistance"

class SimpleDrive : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// lane assist properties
	tInt MIN_RIGHT_LANE_POSITION;
	tInt MAX_RIGHT_LANE_POSITION;

	tInt MIN_LEFT_LANE_POSITION;
	tInt MAX_LEFT_LANE_POSITION;

	// drive, break speed properties
	tFloat32 driveSpeed;
	tFloat32 breakSpeed;
	tInt ticksToStopLine;

	// smooth curve property
	tUInt smoothCurveValue;

	// debug property
	tBool isDebugActive;

	// Coder description for input pins
	cObjectPtr<IMediaTypeDescription> coderDescriptionWheelTicks;
	cObjectPtr<IMediaTypeDescription> coderDescriptionRpm;
	cObjectPtr<IMediaTypeDescription> coderDescriptionIr;
	cObjectPtr<IMediaTypeDescription> coderDescriptionIrRight;
	cObjectPtr<IMediaTypeDescription> coderDescriptionSteeringAngle;
	cObjectPtr<IMediaTypeDescription> coderDescriptionAcceleration;

	cObjectPtr<IMediaTypeDescription> coderDescriptionManeuverFinished;
	cObjectPtr<IMediaTypeDescription> coderDescriptionDecision;
	cObjectPtr<IMediaTypeDescription> coderDescriptionLight;
	
	cObjectPtr<IMediaDescriptionManager> descriptionManager;
	tInt ddlSizeUI16;
	tInt ddlSizeF32;

	// video pins
	cVideoPin ipmVideoInput;
	cVideoPin depthImage;
	
	// Lenkung
	cOutputPin steerAngleOutput;
		
	// Geschwindigkeit
	cInputPin mergedWheelRotationPin;
	cInputPin rpmPin;
	cOutputPin accelerationPin;

	// video stuff
	tBool isFirstFrame;
	tBitmapFormat videoInputInfo;

	// light
	cOutputPin lightPin;

	// Maneuver
	cInputPin decisionPin;
	cOutputPin maneuverFinishedPin;
	cOutputPin decisionOutputPin;

	// sensors
	cInputPin irFrontCenterPin;
	cInputPin irRightPin;

	// variablen
	tInt xPositionBefore;
	tInt yPositionBefore;
	tInt steerAngle;
	tInt previousSteerAngle;

	// stop line
	tInt neededDistance;
	tBool isDriveToStopLineActivated;
	bool isStopLineDetected;
	tBool standsOnStopLine;

	tBool isDriveActive;
	bool photoShot;
	bool isEmergencyBrakeActive;
	int irFrontValue;

	// depthImage
	tBool searchLeftAccess;
	tBool isLeftAccesFound;

	// park gap
	bool searchParkGap;
	bool isParkGapFound;
	tInt distanceToFinishCrossParking;
	tInt crossDriveStraightDistance;
	tInt parallelDriveStraightDistance;
	tTimeStamp timeToFinishParallelParking;

	// brake
	tBool isHardBrake;
	tTimeStamp time1;

	// turn
	Turn currentTurn;
	tBool isDriveStraightActive;
	tBool isTurnStraightWaitActive;
	tBool isTurnStraightActive;
	tBool initDriveStraight;
	tInt turnStraightTicksToFinish;
	tInt turnStraightTicks;
	tInt turnStraightLanePosition;

	// turn right
	tBool isTurnRightActive;
	tBool isTurnRightWaitActive;
	tBool isStep1Active;
	tBool isStep2Active;

	tInt ticksToFinishStep1;
	tInt ticksToFinishStep2;

	tInt turnRightSteeringAngle;
	tInt turnRightTicksStraight;
	tInt turnRightTicksRight;

	// turn left
	tBool isTurnLeftActive;
	tBool isTurnLeftStep1Active;
	tBool isTurnLeftStep2Active;
	tBool isTurnLeftStep3Active;
	tBool isTurnLeftStep4Active;
	tBool isTurnLeftTimeSet;
	tTimeStamp turnLeftTime;

	tInt turnLeftMeasuringError;
	tInt turnLeftWait;
	tInt turnLeftTicksStraight;
	tInt turnLeftSteeringAngle;
	tInt turnLeftTicksLeft;
	tInt ticksToStopAtCrossroad;
	tInt turnLeftTicksToFinishDriveStraight;
	tInt turnLeftTicksToFinishDriveLeft;

	tInt turnLeftStopTicksStraight;

	// wheel rotation
	tInt wheelRotation;
	tInt lastWheelRotation;
	tInt wheelRotationCounter;

	// RPM
	tInt currentRPM;
	tBool initRpm;
	tTimeStamp currentTime;
	tTimeStamp previousTime;
	tInt currentWheelTicks;
	tInt previousWheelTicks;

	// Decisions
	Decision currentDecision;
	Decision prevDecision;
	tUInt32 ticksToCrossroad;

	// interfaces
	unique_ptr<ICrossRoadDetector> crossroadDetector;
	unique_ptr<IDecisionEvaluator> decisionEvaluator;
	unique_ptr<IObstacleDetection> obstacleDetection;
	unique_ptr<IEmergencySystem> emergencySystem;
	unique_ptr<IDriveAlgorithm> driveAlgorithm;
	unique_ptr<IParkGap> parkGap;

public:
	SimpleDrive(const tChar *__info);
	virtual ~SimpleDrive(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	tResult PropertyChanged(const char *name);

private:
	tVoid loadReferenceBackground(void);
	tResult initProperties();
	tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);

	tResult createVideoInputPin(const tChar*, cVideoPin&);
	tResult createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	tResult createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	
	tResult processDepthImage(IMediaSample *mediaSample);
	tResult initVideoStream(void);
	tVoid setBitmapFormat(const tBitmapFormat*);

	tResult processImage(IMediaSample*);
	
	tResult processWheelTicks(IMediaSample *mediaSample);
	tResult processDecisions(IMediaSample *mediaSample);
	tResult processRPM(void);

	tVoid validateSteeringAngle(tInt &steeringAngle);

	tResult driveToStopLine(void);
	tResult turnRight(void);
	tResult turnLeft(void);
	tResult driveStraightOverCrossroad(const Mat &image);
	tResult driveStraight(void);
	tResult processSearchParkGap(void);

	// get values
	tResult getSteeringAngle(IMediaSample *mediaSample, tInt &steeringAngle);
	tResult getIrValue(IMediaSample *mediaSample, tInt &irValue);
	tResult getWheelTicks(IMediaSample *mediaSample, tInt &wheelTicks);
	tResult getRpm(IMediaSample *mediaSample, tInt &rpm);
	tResult getF32Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tFloat32 &value);
	
	tResult getDecision(IMediaSample *mediaSample, Decision &currentDecision, Decision &previousDecision, tUInt32 &ticksToCrossroad);
	tResult getUI16Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tUInt16 &value, tUInt32 &ticksToCrossroad);

	// transmit values
	tResult transmitSteeringAngle(tInt &steeringAngle);
	tResult transmitDrive(const tFloat32 &driveSpeed);
	tResult transmitStop(const tFloat32 &stopSpeed);
	tResult transmitF32Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tFloat32 value);

	tResult transmitManeuverIsFinished(void);
	tResult transmitDecision(const Decision &decision);
	tResult transmitLight(const Light &light);
	tResult transmitUI16Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tUInt16 value);
};

#endif