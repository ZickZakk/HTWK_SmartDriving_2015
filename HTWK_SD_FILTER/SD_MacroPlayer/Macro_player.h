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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
// #include "std_functions.h"

#ifndef _SD_Macro_player_FILTER_HEADER_
#define _SD_Macro_player_FILTER_HEADER_

#define _USE_MATH_DEFINES

#ifdef _DEBUG
#define FILTER_NAME "SD MacroPlayer Debug"
#define OID_ADTF_SD_Macro_Player "adtf.smartdriving.macroPlayer.debug"
#else
#define FILTER_NAME "SD MacroPlayer"
#define OID_ADTF_SD_Macro_Player "adtf.smartdriving.macroPlayer"
#endif

//Property definitions
#define PROP_NAME_DEBUG "0_Debug"
#define PROP_NAME_MEASURE_DISTANCE_FOR_CROSS_PARKING "2_toMeasureDistanceForCrossParking"
#define PROP_NAME_MEASURE_DISTANCE_FOR_PARALLEL_PARKING "3_toMeasureDistanceForParallelParking"
#define PROP_NAME_IR_SHORT_RANGE_THRESHOLD "4_Threshold of the ir short sensor"
#define PROP_NAME_CONFIDENCERANGE "5_confidenceRange"

//Properties for cross-parking
#define PROP_NAME_CPDRIVESTRAIGHTDISTANCE "6_Cross_DriveStraightDistance"
#define PROP_NAME_CPDRIVESTRAIGHTDISTANCESTEER "6_Cross_DriveStraightDistanceSteer"
#define PROP_NAME_CPDRIVESTRAIGHTDISTANCEDRIVE "6_Cross_DriveStraightDistanceDrive"

#define PROP_NAME_CPDRIVELEFTDISTANCE "6_Cross_DriveLeftDistance"
#define PROP_NAME_CPDRIVELEFTDISTANCESTEER "6_Cross_DriveLeftDistanceSteer"
#define PROP_NAME_CPDRIVELEFTDISTANCEDRIVE "6_Cross_DriveLeftDistanceDrive"

#define PROP_NAME_CPDRIVERIGHTDISTANCE "6_Cross_DriveRightDistance"
#define PROP_NAME_CPDRIVERIGHTDISTANCESTEER "6_Cross_DriveRightDistanceSteer"
#define PROP_NAME_CPDRIVERIGHTDISTANCEDRIVE "6_Cross_DriveRightDistanceDrive"

#define PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCE "6_Cross_DriveStraightInGapDistance"
#define PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCESTEER "6_Cross_DriveStraightInGapDistanceSteer"
#define PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCEDRIVE "6_Cross_DriveStraightInGapDistanceDrive"

//Properties for parallel-parking
#define PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP "7_Parallel_driveStraightBeforeGoInGapDistance"
#define PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPSTEER "7_Parallel_driveStraightBeforeGoInGapDistanceSteer"
#define PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPDRIVE "7_Parallel_driveStraightBeforeGoInGapDistanceDrive"

#define PROP_NAME_PPDRIVELEFTINTOGAPDISTANCE "7_Parallel_driveLeftIntoGapDistance"
#define PROP_NAME_PPDRIVELEFTINTOGAPDISTANCESTEER "7_Parallel_driveLeftIntoGapDistanceSteer"
#define PROP_NAME_PPDRIVELEFTINTOGAPDISTANCEDRIVE "7_Parallel_driveLeftIntoGapDistanceDrive"

#define PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCE "7_Parallel_driveRightIntoGapDistance"
#define PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCESTEER "7_Parallel_driveRightIntoGapDistanceSteer"
#define PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCEDRIVE "7_Parallel_driveRightIntoGapDistanceDrive"

#define PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCE "7_Parallel_driveStraightInGapDistance"
#define PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCESTEER "7_Parallel_driveStraightInGapDistanceSteer"
#define PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCEDRIVE "7_Parallel_driveStraightInGapDistanceDrive"

//Properties for pull-out-left cross-parking
#define PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCE "6_Pull-out-left_cross_driveStraigtExitGapDistance"
#define PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCESTEER "6_Pull-out-left_cross_driveStraigtExitGapDistanceSteer"
#define PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE "6_Pull-out-left_cross_driveStraigtExitGapDistanceDrive"

#define PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCE "6_Pull-out-left_cross_driveLeftExitGapDistance"
#define PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCESTEER "6_Pull-out-left_cross_driveLeftExitGapDistanceSteer"
#define PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCEDRIVE "6_Pull-out-left_cross_driveLeftExitGapDistanceDrive"

//Properties for pull-out-right cross-parking
#define PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCE "6_Pull-out-right_cross_driveStraigtExitGapDistance"
#define PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCESTEER "6_Pull-out-right_cross_driveStraigtExitGapDistanceSteer"
#define PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE "6_Pull-out-right_cross_driveStraigtExitGapDistanceDrive"

#define PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCE "6_Pull-out-right_cross_driveRightExitGapDistance"
#define PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCESTEER "6_Pull-out-right_cross_driveRightExitGapDistanceSteer"
#define PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCEDRIVE "6_Pull-out-right_cross_driveRightExitGapDistanceDrive"

//Properties for pull-out-right parallel-parking
#define PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCE "7_Pull-out-right_parallel_driveStraigtInGapDistance"
#define PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCESTEER "7_Pull-out-right_parallel_driveStraigtInGapDistanceSteer"
#define PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCEDRIVE "7_Pull-out-right_parallel_driveStraigtInGapDistanceDrive"

#define PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCE "7_Pull-out-right_parallel_driveLeftExitGapDistance"
#define PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCESTEER "7_Pull-out-right_parallel_driveLeftExitGapDistanceSteer"
#define PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCEDRIVE "7_Pull-out-right_parallel_driveLeftExitGapDistanceDrive"

#define PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCE "7_Pull-out-right_parallel_driveRightExitGapDistance"
#define PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCESTEER "7_Pull-out-right_parallel_driveRightExitGapDistanceSteer"
#define PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCEDRIVE "7_Pull-out-right_parallel_driveRightExitGapDistanceDrive"



class cMacro_player : public adtf::cFilter
{
	ADTF_FILTER(OID_ADTF_SD_Macro_Player, FILTER_NAME, adtf::OBJCAT_DataFilter);

    private:
		/**
		 * @author dhecht
		 * stuff for decision pins
		 */
		// Variablen
		cObjectPtr<IMediaTypeDescription> coderDescriptionManeuver;
		cObjectPtr<IMediaDescriptionManager> descriptionManager;
		//cObjectPtr<IMediaDescriptionManager> descriptionManager;
		
		cInputPin decisionPin;
		cOutputPin maneuverFinishedPin;
		cOutputPin lightPin;

		Decision currentDecision;
		Decision prevDecision;
		bool isDebugActive;
		tInt ddlSizeManeuver;
		tInt ddlSizeTSignal;

		// Methoden
		tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);
		tResult getDecisionValue(IMediaSample *mediaSample, tUInt16 &decisionValue, tTimeStamp &timeStamp);
		tVoid getDecision(const tUInt16 &decisionValue, Decision &currentDecision, Decision &prevDecision);
		
		// method for sending MANEUVER_FINISHED if a macro is finished
		tResult sendManeuver(cOutputPin &pin, const Maneuver &manuever, const tTimeStamp &timeStamp);
		tResult transmitLight(cOutputPin &pin, const Light &light);
		

		// private methods
		tVoid initVariabels();
		tVoid initProperties();
		tInt getDistanceFromWheelRotation(tInt &rotation);
		tResult drive(IMediaSample *pMediaSample);
		tResult drive(IMediaSample *pMediaSample, const tInt &speed);
		tBool isSpeedValid(const tInt &speed) const;
		tResult isFrontFree();
		tResult stopDrive(IMediaSample *pMediaSample);
		tResult crossParkingScript(IMediaSample *pMediaSample);
		tResult parallelParkingScript(IMediaSample *pMediaSample);
		tResult pullOutLeftCrossParkingScript(IMediaSample *pMediaSample);
		tResult pullOutRightCrossParkingScript(IMediaSample *pMediaSample);
		tResult pullOutRightParallelParkingScript(IMediaSample *pMediaSample);
		tResult runHazzardLights(tInt enduration);
		

		tResult steer(IMediaSample *pMediaSample, const tInt &steerAngle);
		tBool isSteerAngleValid(const tInt &steerAngle) const;

		tVoid signal();
		tInt measure();

		tResult getValue(IMediaSample *mediaSample, tFloat32 &value, tTimeStamp &timeStamp);

		// Pin - methods
		tResult createInputPin(cInputPin &pin, const char *pinName, cObjectPtr<IMediaType> typeSignal);
		tResult createOutputPin(cOutputPin &pin, const char *pinName, cObjectPtr<IMediaType> typeSignal);

		//functions
		tResult sendNewValue(cOutputPin * outpin, const tFloat32 &value, const tTimeStamp &timeStamp);
		tResult sendNewValueBool(cOutputPin * outpin, const tBool &value, const tTimeStamp &timeStamp);

		//variables
		tTimeStamp m_nLastMSTime;
		tFloat32 untilclick;
		tFloat32 startclick;
		tFloat32 currentclick;
		tFloat32 currentclick_glob;
		tFloat32 servo;
		tFloat32 for_or_backward;
		std::vector<std::string> drivelist;
		tFloat32 L;
		tFloat32 R;
		tBool isDriveActive;
		//gap specific var
		tInt distance;
		tInt gapStart;
		tInt lastDist;
		tBool initStep;
		tBool isFirstCarFound;
		tBool isFirstCarComplete;
		tBool isMaybeGapEnd;
		tBool foundGap;
		tBool initPullOut;
		tBool decideParkingPosition;
		tInt measuredParkSpace;
		tInt irFront;
		tInt step;
		enum steps {searchGap, driveBackToGap, stopDriveBackToGap, driveStraightInGap, stopleftMove, driveBack, driveStraight, stopDriveStraight, stopDriveBack, cpDriveStraight, cpDriveStraightFin, cpLeftMove, cpLeftMoveFin, cpRightMove, cpRightMoveFin, cpDriveStraightLast, cpDriveStraightLastFin, cpExit, ppDriveRightMove, ppDriveLeftMove, ppDriveStraightMove, ppExit, pullOutLeftCrossPart0, pullOutLeftCrossPart1, pullOutLeftCrossPart2, pullOutRightCrossPart0, pullOutRightCrossPart1, pullOutRightCrossPart2, pullOutLeftParallel, pullOutRightParallel0, pullOutRightParallel1, pullOutRightParallel2, pullOutRightParallel3,doHaazzardLight,pullOutExit};
		tInt backmove;
		//Properties
		tUInt8 thresholdIRShortRange;
		tInt8 confidenceRange;
    protected:
		//input pins
		cInputPin m_pin_input_wheelRotation;
		cInputPin m_ir_RR; // Rear Right
		cInputPin m_pin_input_decision;
		cInputPin m_ir_Front_Center;
		
		// output pin for signal data
		cOutputPin  m_pin_output_acceleration; 
		cOutputPin	m_pin_output_steeringAngle;
		cOutputPin  m_pin_output_finish;
		
		// cInputPin m_oIWheelL; // input pin for signal data
		// cInputPin m_oIWheelR; // input pin for signal data

		// cInputPin m_oIServo; // input pin for signal data

		//Coder description for input pins
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutput;
		cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalOutput;


    public: // construction
        cMacro_player(const tChar* __info);
        virtual ~cMacro_player();

        tResult Init(tInitStage eStage, __exception);
		tResult Start(__exception);
		tResult Stop(__exception);
		tResult Shutdown(tInitStage eStage, __exception);
		tResult OnPinEvent( IPin *pSource, tInt nEventCore, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
		tResult PropertyChanged(const char*);
};

//*************************************************************************************************
#endif // _SD_Macro_player_FILTER_HEADER_
