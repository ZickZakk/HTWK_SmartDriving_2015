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
#include "RoadSignEvaluator.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, RoadSignEvaluator);

RoadSignEvaluator::RoadSignEvaluator(const char *__info) : cFilter(__info)
{
	NUMBER_OF_MARKERS = 11;
	this->isManeuverSet = false;
	this->isDebugActive = false;

	initProperties();
	initMarkerList();
}

tResult RoadSignEvaluator::initProperties()
{
	SetPropertyBool(PROP_NAME_DEBUG, this->isDebugActive);
    SetPropertyBool(PROP_NAME_DEBUG NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_DEBUG NSSUBPROP_ISCHANGEABLE, tTrue);

	MIN_MARKER_AREA = 600;
	SetPropertyInt(PROP_NAME_MIN_MARKER_AREA, MIN_MARKER_AREA);
    SetPropertyInt(PROP_NAME_MIN_MARKER_AREA NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_MIN_MARKER_AREA NSSUBPROP_MAXIMUM, 250000);
	SetPropertyBool(PROP_NAME_MIN_MARKER_AREA NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_MIN_MARKER_AREA NSSUBPROP_ISCHANGEABLE, tTrue);

    MAX_MARKER_AREA = 7000;
	SetPropertyInt(PROP_NAME_MAX_MARKER_AREA, MAX_MARKER_AREA);
    SetPropertyInt(PROP_NAME_MAX_MARKER_AREA NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_MAX_MARKER_AREA NSSUBPROP_MAXIMUM, 250000);
	SetPropertyBool(PROP_NAME_MAX_MARKER_AREA NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_MAX_MARKER_AREA NSSUBPROP_ISCHANGEABLE, tTrue);

	MIN_FRAMES_FOR_VALID_MARKER = 10;
	SetPropertyInt(PROP_NAME_MIN_FRAMES_FOR_VALID_MARKER, MIN_FRAMES_FOR_VALID_MARKER);
    SetPropertyInt(PROP_NAME_MIN_FRAMES_FOR_VALID_MARKER NSSUBPROP_MINIMUM, 1);
    SetPropertyInt(PROP_NAME_MIN_FRAMES_FOR_VALID_MARKER NSSUBPROP_MAXIMUM, 2500);
	SetPropertyBool(PROP_NAME_MIN_FRAMES_FOR_VALID_MARKER NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_MIN_FRAMES_FOR_VALID_MARKER NSSUBPROP_ISCHANGEABLE, tTrue);

	// 1 second = 1.000.000 microseconds
	MAX_DELAY_TIME = 1000000;
	SetPropertyInt(PROP_NAME_MARKER_DELAY, MAX_DELAY_TIME);
    SetPropertyInt(PROP_NAME_MARKER_DELAY NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_MARKER_DELAY NSSUBPROP_MAXIMUM, 10000000);
	SetPropertyBool(PROP_NAME_MARKER_DELAY NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_MARKER_DELAY NSSUBPROP_ISCHANGEABLE, tTrue);

	RETURN_NOERROR;
}

tVoid RoadSignEvaluator::initMarkerList()
{
	for (tUInt8 i = 0; i < NUMBER_OF_MARKERS; i++)
	{
		this->markerList[i].count = 0;
		this->markerList[i].lastTimeStamp = 0;
		this->markerList[i].markerArea = 0;
	}
}

RoadSignEvaluator::~RoadSignEvaluator(void)
{}

tResult RoadSignEvaluator::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	if (eStage == StageFirst)
	{
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

		cObjectPtr<IMediaType> maneuverMediaType;
		cObjectPtr<IMediaType> markerMediaType;
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(initMediaType("tSteeringAngleData", maneuverMediaType, this->coderDescriptionManeuver), "cant init media type for maneuver");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(initMediaType("tRoadSign", markerMediaType, this->coderDescriptionMarker), "cant init media type for road signs");

		// input pins
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createInputPin("roadSign", this->roadSign, markerMediaType), "Cant create input pin for road sign");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createInputPin("Current_Maneuver", this->currentManeuverPin, maneuverMediaType), "cant create input pin for maneuver");

		// output pins
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin("decision", this->decisionPin, maneuverMediaType), "Cant create output pin for decisions");
	}
	else if (eStage == StageGraphReady)
	{
		cObjectPtr<IMediaSerializer> serializer;
		RETURN_IF_FAILED(this->coderDescriptionManeuver->GetMediaSampleSerializer(&serializer));
		this->ddlSize = serializer->GetDeserializedSize();
	}
	
	RETURN_NOERROR;
}

tResult RoadSignEvaluator::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
	tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
    RETURN_IF_POINTER_NULL(descriptionSignalValue);        

	mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
	RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::createInputPin(const tChar *pinName, cInputPin &pin)
{
	RETURN_IF_FAILED(pin.Create(pinName, new cMediaType(0, 0, 0, "tRoadSign") , static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult RoadSignEvaluator::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);
	
	if (this->coderDescriptionMarker == NULL)
	{
		LOG_INFO("Set coder description");
		RETURN_IF_FAILED(initCoderDescription(source));
	}

	if (eventCore == IPinEventSink::PE_MediaSampleReceived && this->coderDescriptionMarker != NULL)
	{
		if (source == &this->roadSign)
		{
			static tInt8 roadSignId;
			static tFloat32 markerArea;
			
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(readOutRoadSign(mediaSample, roadSignId, markerArea), "Cant read out marker");

			if (isDebugActive)
			{
				LOG_INFO(cString::Format("Recieved id:   %d", roadSignId));
				LOG_INFO(cString::Format("Recieved area: %f", markerArea));
			}

			if (isMarkerAreaValid(markerArea))
			{
				if (isDebugActive)
				{
					LOG_INFO(cString::Format("Recieved v_id:   %d", roadSignId));
					LOG_INFO(cString::Format("Recieved v_area: %f", markerArea));
				}

				static tTimeStamp now;
				now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

				this->markerList[roadSignId % 11].count++;
				this->markerList[roadSignId % 11].lastTimeStamp = now;
				this->markerList[roadSignId % 11].markerArea += markerArea;

				static vector<ValidMarker> validMarker;
				getListOfValidMarkers(validMarker, now);
							
				if (this->isManeuverSet)
				{
					static tUInt32 distanceToCrossroad;
					distanceToCrossroad = 0;

					// wenn Befehl Parken und Park-Schild erkannt dann sende einparken
					if (isParkingSignDetectedAndCrossParking(validMarker, this->currentManeuver))
					{
						LOG_INFO("Send decision CROSS_PARKING to DriveAlgorithm!");
						RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_CROSS_PARKING), "Cant send decision to DriveAlgorithm");
						this->isManeuverSet = false;
					}
					else if (isParkingSignDetectedAndParallelParking(validMarker, this->currentManeuver))
					{
						LOG_INFO("Send decision PARALLEL_PARKING to DriveAlgorithm!");
						RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_PARALLEL_PARKING), "Cant send decision to DriveAlgorithm");
						this->isManeuverSet = false;
					}
					else if (isCrossroadsWithPrioritySignDetecetd(validMarker, distanceToCrossroad))
					{
						if (isManeuverLeft(this->currentManeuver))
						{
							// Es kann Geradeausverkehr geben, deswegen vorher anhalten
							LOG_INFO("Send decision DETECT_CROSSROADS_AND_TURN_LEFT to DriveAlgorithm!");
							RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DETECT_CROSSROADS_AND_TURN_LEFT, distanceToCrossroad), "Cant send decision to DriveAlgorithm");
							this->isManeuverSet = false;
						}
						else if (isManeuverRight(this->currentManeuver))
						{
							// Wir haben vorfahrt also volles Mett aufs Brett!
							LOG_INFO("Send decision DETECT_CROSSROADS_AND_TURN_RIGHT to DriveAlgorithm!");
							RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DETECT_CROSSROADS_AND_TURN_RIGHT), "Cant send decision to DriveAlgorithm");
							this->isManeuverSet = false;
						}
						else if(isManeuverStraight(this->currentManeuver))
						{
							// Wir haben vorfahrt, also voll durhbrettern
							LOG_INFO("Send decision DETECT_CROSSROADS_AND_DRIVE_STRAIGHT to DriveAlgorithm!");
							RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DETECT_CROSSROADS_AND_DRIVE_STRAIGHT, distanceToCrossroad), "Cant send decision to DriveAlgorithm");
							this->isManeuverSet = false;
						}
					}
					// Gleiche Entscheidungen bei Stop-, Vorfahrts- und Kreuzungsschild
					// (vorlaufig, spater mussen andere Entscheidungen getroffen werden)
					else if (isStopSignDetected(validMarker) || isCrossroadsSignDetected(validMarker) || isGiveWaySignDetected(validMarker))
					{
						// immer vor der Kruzug anhalten!
						if (isManeuverLeft(this->currentManeuver))
						{
							LOG_INFO(cString::Format("Send decision DETECT_CROSSROADS_STOP_AND_TURN_LEFT to DriveAlgorithm! %d", DECISION_DETECT_CROSSROADS_STOP_AND_TURN_LEFT));
							RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DETECT_CROSSROADS_STOP_AND_TURN_LEFT), "Cant send decision to DriveAlgorithm");
							this->isManeuverSet = false;
						}
						else if (isManeuverRight(this->currentManeuver))
						{
							LOG_INFO("Send decision DETECT_CROSSROADS_STOP_AND_TURN_RIGHT to DriveAlgorithm!");
							RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DETECT_CROSSROADS_STOP_AND_TURN_RIGHT), "Cant send decision to DriveAlgorithm");
							this->isManeuverSet = false;
						}
						else if (isManeuverStraight(this->currentManeuver))
						{
							LOG_INFO("Send decision DETECT_CROSSROADS_STOP_AND_DRIVE_STRAIGHT to DriveAlgorithm!");
							RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DETECT_CROSSROADS_STOP_AND_DRIVE_STRAIGHT), "Cant send decision to DriveAlgorithm");
							this->isManeuverSet = false;
						}
					}
					else if (isDrivingDirectionStraighSignDetected(validMarker))
					{
						LOG_INFO("Send decision DETECT_CROSSROADS_AND_DRIVE_STRAIGHT to DriveAlgorithm!");
						RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DETECT_CROSSROADS_AND_DRIVE_STRAIGHT), "Cant send decision to DriveAlgorithm");
						this->isManeuverSet = false;
					}
				}
			}
		}
		else if (source == &this->currentManeuverPin)
		{
			// reset marker list
			initMarkerList();

			// lese den Zustand des aktuellen Befehls aus
			static tUInt16 maneuverValue;
			static tTimeStamp timeStamp;

			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getManeuver(mediaSample, maneuverValue, timeStamp), "Cant get Maneuver from ManeuverPin!");

			getManeuver(maneuverValue, this->currentManeuver);
			LOG_INFO(cString::Format("Got manuever: %d", this->currentManeuver));
			this->isManeuverSet = true;

			if (this->currentManeuver == MANEUVER_STOP)
			{
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_STOP), "Cant send STOP to DriveAlgorithm");
				this->isManeuverSet = false;
			}
			else if (this->currentManeuver == MANEUVER_PULL_OUT_LEFT)
			{
				LOG_INFO("Send decision PULL_OUT_LEFT to DriveAlgorithm!");
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_PULL_OUT_LEFT), "Cant send PULL_OUT_LEFT to DriveAlgorithm");
			}
			else if (this->currentManeuver == MANEUVER_PULL_OUT_RIGHT)
			{
				LOG_INFO("Send decision PULL_OUT_RIGHT to DriveAlgorithm!");
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_PULL_OUT_RIGHT), "Cant send PULL_OUT_RIGHT to DriveAlgorithm");
			}
			else if (this->currentManeuver == MANEUVER_STRAIGHT || this->currentManeuver == MANEUVER_LEFT || this->currentManeuver == MANEUVER_RIGHT || this->currentManeuver == MANEUVER_PARALLEL_PARKING || this->currentManeuver == MANEUVER_CROSS_PARKING )
			{
				LOG_INFO("Send decision DRIVE to DriveAlgorithm!");
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(DECISION_DRIVE), "Cant send DRIVE to DriveAlgorithm");
				this->isManeuverSet = true;
			}
		}
	}

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::readOutRoadSign(IMediaSample *mediaSample, tInt8 &markerId, tFloat32 &markerArea) const
{
	cObjectPtr<IMediaCoder> coderInput;
	RETURN_IF_FAILED(this->coderDescriptionMarker->Lock(mediaSample, &coderInput));
	coderInput->Get("i8Identifier", (tVoid*) &markerId);
	coderInput->Get("fl32Imagesize", (tVoid*) &markerArea);
	RETURN_IF_FAILED(this->coderDescriptionMarker->Unlock(coderInput));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::initCoderDescription(IPin *sourcePin)
{
	cObjectPtr<IMediaType> mediaType;
	RETURN_IF_FAILED(sourcePin->GetMediaType(&mediaType));

	if (mediaType != NULL)
	{
		RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &this->coderDescriptionMarker));
	}

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::getManeuver(IMediaSample *mediaSample, tUInt16 &maneuverValue, tTimeStamp &timeStamp)
{
	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionManeuver->Lock(mediaSample, &coder));
	
	coder->Get("ui16Angle", (tVoid*) &maneuverValue);
	coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
	
	RETURN_IF_FAILED(this->coderDescriptionManeuver->Unlock(coder));

	RETURN_NOERROR;
}

tBool RoadSignEvaluator::isMarkerAreaValid(const tFloat32 &area) const
{
	return area >= MIN_MARKER_AREA && area <= MAX_MARKER_AREA;
}

tVoid RoadSignEvaluator::getListOfValidMarkers(vector<ValidMarker> &validMarker, const tTimeStamp &now)
{
	validMarker.clear();

	for (tUInt8 i = 0; i < NUMBER_OF_MARKERS; i++)
	{
		if (isTimeSpanOk(this->markerList[i].lastTimeStamp, now))
		{
			if (this->markerList[i].count >= MIN_FRAMES_FOR_VALID_MARKER)
			{
				static ValidMarker marker;
				marker.markerId = i;
				marker.avgMarkerArea = (this->markerList[i].markerArea / static_cast<tFloat32>(this->markerList[i].count));

				validMarker.push_back(marker);
			}
		}
		else
		{
			this->markerList[i].count = 0;
		}
	}
}

tBool RoadSignEvaluator::isTimeSpanOk(const tTimeStamp &oldTime, const tTimeStamp &currentTime) const
{
	return (currentTime - oldTime) <= MAX_DELAY_TIME;
}

tVoid RoadSignEvaluator::printListOfValidMarkers(vector<ValidMarker> &markers)
{
	for (auto marker = markers.begin(); marker != markers.end(); ++marker)
	{
		LOG_INFO(cString::Format("Valid marker: %d", marker->markerId));
	}
}

tBool RoadSignEvaluator::isNoSignDetected(const vector<ValidMarker> &markers) const
{
	return markers.size() == 0;
}

tBool RoadSignEvaluator::isParkingSignDetectedAndParallelParking(const vector<ValidMarker> &markers, const Maneuver &maneuver) const
{
	return isParkingSignDetected(markers) && maneuver == MANEUVER_PARALLEL_PARKING;
}

tBool RoadSignEvaluator::isParkingSignDetectedAndCrossParking(const vector<ValidMarker> &markers, const Maneuver &maneuver) const
{
	return isParkingSignDetected(markers) && maneuver == MANEUVER_CROSS_PARKING;
}

tBool RoadSignEvaluator::isParkingSignDetected(const vector<ValidMarker> &markers) const
{
	for (auto marker = markers.begin(); marker != markers.end(); ++marker)
	{
		if (marker->markerId == SIGN_PARKING)
		{
			LOG_INFO("Detecetd Parking sign");
			return true;
		}
	}

	return false;
}

tBool RoadSignEvaluator::isCrossroadsWithPrioritySignDetecetd(const vector<ValidMarker> &markers, tUInt32 &ticksToCrossroad) const
{
	for (auto marker = markers.begin(); marker != markers.end(); ++marker)
	{
		if (marker->markerId == SIGN_CROSSROADS_WITH_PRIORITY)
		{
			LOG_INFO("Detected crossroad with priority sign");
			calculateDistanceFromMarkerSize(marker->avgMarkerArea, ticksToCrossroad);

			return true;
		}
	}

	return false;
}

tBool RoadSignEvaluator::isGiveWaySignDetected(const vector<ValidMarker> &markers) const
{
	for (auto marker = markers.begin(); marker != markers.end(); ++marker)
	{
		if (marker->markerId == SIGN_GIVE_WAY)
		{
			LOG_INFO("Detected give way sign");
			return true;
		}
	}

	return false;
}

tBool RoadSignEvaluator::isStopSignDetected(const vector<ValidMarker> &markers) const
{
	for (auto marker = markers.begin(); marker != markers.end(); ++marker)
	{
		if (marker->markerId == SIGN_STOP)
		{
			LOG_INFO("Detected stop sign");
			return true;
		}
	}

	return false;
}

tBool RoadSignEvaluator::isCrossroadsSignDetected(const vector<ValidMarker> &markers) const
{
	for (auto marker = markers.begin(); marker != markers.end(); ++marker)
	{
		if (marker->markerId == SIGN_CROSSROADS)
		{
			LOG_INFO("Detected crossroads sign");
			return true;
		}
	}

	return false;
}

tBool RoadSignEvaluator::isDrivingDirectionStraighSignDetected(const vector<ValidMarker> &markers) const
{
	for (auto marker = markers.begin(); marker != markers.end(); ++marker)
	{
		if (marker->markerId == SIGN_DRIVING_DIRECTION_STRAIGHT)
		{
			LOG_INFO("Detected driving direction straight sign");
			return true;
		}
	}

	return false;
}

tBool RoadSignEvaluator::isManeuverLeft(const Maneuver &maneuver) const
{
	return maneuver == MANEUVER_LEFT;
}

tBool RoadSignEvaluator::isManeuverRight(const Maneuver &maneuver) const
{
	return maneuver == MANEUVER_RIGHT;
}

tBool RoadSignEvaluator::isManeuverStraight(const Maneuver &maneuver) const
{
	return maneuver == MANEUVER_STRAIGHT;
}

tResult RoadSignEvaluator::sendDecision(const Decision &decision)
{
	RETURN_IF_FAILED(sendDecision(decision, 0));

	RETURN_NOERROR;
}

tResult RoadSignEvaluator::sendDecision(const Decision &decision, const tUInt32 &ticksToCrossroad)
{
	LOG_INFO(cString::Format("Distance to crossroad %u", ticksToCrossroad));

	cObjectPtr<IMediaSample> mediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));

	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSize));

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionManeuver->WriteLock(mediaSample, &coder));
	
	static tUInt16 decisionValue;
	decisionValue = static_cast<tUInt16>(decision);

	LOG_INFO(cString::Format("Decision value %d", decisionValue));

	coder->Set("ui16Angle", (tVoid*) &decisionValue);
	coder->Set("ui32ArduinoTimestamp", (tVoid*) &ticksToCrossroad);

	RETURN_IF_FAILED(this->coderDescriptionManeuver->Unlock(coder));

	static tTimeStamp now;
	now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

	RETURN_IF_FAILED(mediaSample->SetTime(now));
	RETURN_IF_FAILED(this->decisionPin.Transmit(mediaSample));

	RETURN_NOERROR;
}

tVoid RoadSignEvaluator::getManeuver(const tUInt16 &maneuverId, Maneuver &maneuver)
{
	switch (maneuverId)
	{
	case MANEUVER_STOP:
		maneuver = MANEUVER_STOP;
		LOG_INFO("Got Maneuver STOP");
		break;

	case MANEUVER_LEFT:
		maneuver = MANEUVER_LEFT;
		LOG_INFO("Got Maneuver LEFT");
		break;

	case MANEUVER_RIGHT:
		maneuver = MANEUVER_RIGHT;
		LOG_INFO("Got Maneuver RIGHT");
		break;

	case MANEUVER_STRAIGHT:
		maneuver = MANEUVER_STRAIGHT;
		LOG_INFO("Got Maneuver STRAIGHT");
		break;

	case MANEUVER_PARALLEL_PARKING:
		maneuver = MANEUVER_PARALLEL_PARKING;
		LOG_INFO("Got Maneuver PARALLEL_PARKING");
		break;

	case MANEUVER_CROSS_PARKING:
		maneuver = MANEUVER_CROSS_PARKING;
		LOG_INFO("Got Maneuver CROSS_PARKING;");
		break;

	case MANEUVER_PULL_OUT_LEFT:
		maneuver = MANEUVER_PULL_OUT_LEFT;
		LOG_INFO("Got Maneuver PULL_OUT_LEFT");
		break;

	case MANEUVER_PULL_OUT_RIGHT:
		maneuver = MANEUVER_PULL_OUT_RIGHT;
		LOG_INFO("Got Maneuver OUT_RIGHT");
		break;

	default:
		maneuver = MANEUVER_DEFAULT;
		LOG_INFO("Got Maneuver DEFAULT");
		break;
	}
}

tResult RoadSignEvaluator::PropertyChanged(const char *propertyName)
{
	resetProperties();

	RETURN_NOERROR;
}

tVoid RoadSignEvaluator::resetProperties()
{
	this->isDebugActive = GetPropertyBool(PROP_NAME_DEBUG);
	MAX_DELAY_TIME = GetPropertyInt(PROP_NAME_MARKER_DELAY);
	MIN_FRAMES_FOR_VALID_MARKER = GetPropertyInt(PROP_NAME_MIN_FRAMES_FOR_VALID_MARKER);
	MIN_MARKER_AREA = GetPropertyInt(PROP_NAME_MIN_MARKER_AREA);
	MAX_MARKER_AREA = GetPropertyInt(PROP_NAME_MAX_MARKER_AREA);
}

tVoid RoadSignEvaluator::calculateDistanceFromMarkerSize(const tFloat32 &markerArea, tUInt32 &distance) const
{
	static const tFloat32 m = -0.1125f;
	static const tFloat32 n = 180.0f;
	
	round((m * markerArea + n) / 4, distance);
}

tVoid RoadSignEvaluator::round(const tFloat32 &value, tUInt32 &result) const
{
	result = static_cast<tUInt32>(floor(value + 0.5));
}