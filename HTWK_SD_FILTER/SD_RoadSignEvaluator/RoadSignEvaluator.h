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
#ifndef _RoadSignEvaluator_H_
#define _RoadSignEvaluator_H_

#ifdef _DEBUG
#define FILTER_NAME "SD RoadSignEvaluator Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.RoadSignEvaluator.debug"
#else
#define FILTER_NAME "SD RoadSignEvaluator"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.RoadSignEvaluator.release"
#endif

#define PROP_NAME_DEBUG "0_Debug"
#define PROP_NAME_MIN_MARKER_AREA "1_Min marker area"
#define PROP_NAME_MAX_MARKER_AREA "2_Max marker area"
#define PROP_NAME_MIN_FRAMES_FOR_VALID_MARKER "3_Min frame for valid markers"
#define PROP_NAME_MARKER_DELAY "4_Delay time for markers"

class RoadSignEvaluator : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// constants
	tUInt8 NUMBER_OF_MARKERS;
	tTimeStamp MAX_DELAY_TIME;
	tUInt MIN_FRAMES_FOR_VALID_MARKER;
	tUInt16 MIN_MARKER_AREA;
	tUInt16 MAX_MARKER_AREA;

	// pins
	cInputPin roadSign;
	cInputPin currentManeuverPin;
		
	cOutputPin decisionPin;

	// stuff
	cObjectPtr<IMediaTypeDescription> coderDescriptionMarker;
	cObjectPtr<IMediaTypeDescription> coderDescriptionManeuver;
	cObjectPtr<IMediaDescriptionManager> descriptionManager;
	tInt ddlSize;

	// Maneuver stuff
	Maneuver currentManeuver;
	tBool isManeuverSet;

	MarkerObject markerList[11];
	bool isDebugActive;

public:
	RoadSignEvaluator(const tChar *__info);
	virtual ~RoadSignEvaluator(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	tResult PropertyChanged(const char *propertyName);
	
private:
	tResult initProperties();
	tVoid initMarkerList();
	
	tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);
	tResult createInputPin(const tChar *pinName, cInputPin &pin);
	tResult createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	tResult createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	
	tResult readOutRoadSign(IMediaSample *mediaSample, tInt8 &markerId, tFloat32 &markerArea) const;
	tResult initCoderDescription(IPin *sourcePin);
	tBool isMarkerAreaValid(const tFloat32 &area) const;
	tVoid getListOfValidMarkers(vector<ValidMarker> &markers, const tTimeStamp &now);
	tBool isTimeSpanOk(const tTimeStamp &oldTime, const tTimeStamp &currentTime) const;
	tVoid printListOfValidMarkers(vector<ValidMarker> &markers);
	
	tBool isNoSignDetected(const vector<ValidMarker> &markers) const;
	tBool isParkingSignDetectedAndParallelParking(const vector<ValidMarker> &markers, const Maneuver &maneuver) const;
	tBool isParkingSignDetectedAndCrossParking(const vector<ValidMarker> &markers, const Maneuver &maneuver) const;
	tBool isParkingSignDetected(const vector<ValidMarker> &markers) const;
	tBool isCrossroadsWithPrioritySignDetecetd(const vector<ValidMarker> &markers, tUInt32 &ticksToCrossroad) const;
	tBool isGiveWaySignDetected(const vector<ValidMarker> &markers) const;
	tBool isStopSignDetected(const vector<ValidMarker> &markers) const;
	tBool isCrossroadsSignDetected(const vector<ValidMarker> &markers) const;
	tBool isDrivingDirectionStraighSignDetected(const vector<ValidMarker> &markers) const;

	tBool isManeuverLeft(const Maneuver &maneuver) const;
	tBool isManeuverRight(const Maneuver &maneuver) const;
	tBool isManeuverStraight(const Maneuver &maneuver) const;

	tResult sendDecision(const Decision &decision);
	tResult sendDecision(const Decision &decision, const tUInt32 &ticksToCrossroad);

	tResult getManeuver(IMediaSample *mediaSample, tUInt16 &maneuverValue, tTimeStamp &timeStamp);
	tVoid getManeuver(const tUInt16 &maneuverId, Maneuver &maneuver);
	tVoid resetProperties();

	tVoid calculateDistanceFromMarkerSize(const tFloat32 &markerArea, tUInt32 &distance) const;
	tVoid round(const tFloat32 &value, tUInt32 &result) const;
};

#endif