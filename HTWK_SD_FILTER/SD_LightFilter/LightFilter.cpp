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

// Create filter
ADTF_FILTER_PLUGIN(FILTER_NAME, OID_ADTF_SD_Light_Filter, LightFilter);


LightFilter::LightFilter(const tChar* __info) : adtf::cFilter(__info)
{
	this->headLight = false;
	this->breakLight = false;
	this->turnRightLight = false;
	this->turnLeftLight = false;
	this->reverseLight = false;
	this->hazardLight = false;
}

LightFilter::~LightFilter()
{
}

tResult LightFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));
		
		cObjectPtr<IMediaType> lightInputMediaType;
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(initMediaType("tSteeringAngleData", lightInputMediaType, this->coderDescriptionInput), "Cant init media type for light enum");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createInputPin(this->lightInputPin, "lightInput", lightInputMediaType), "Cant create light input pin");

		cObjectPtr<IMediaType> lightOutputMediaType;
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(initMediaType("tBoolSignalValue", lightOutputMediaType, this->coderDescriptionOutput), "Cant init media type for light output");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin(this->headLightPin, "headLight", lightOutputMediaType), "Cant create head light pin");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin(this->breakLightPin, "breakLight", lightOutputMediaType), "Cant create break light pin");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin(this->turnRightPin, "turnRightLight", lightOutputMediaType), "Cant create turn right light pin");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin(this->turnLeftPin, "turnLeftLight", lightOutputMediaType), "Cant create turn left light pin");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin(this->reverseLightPin, "reverseLight", lightOutputMediaType), "Cant create reverse light pin");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin(this->hazardLightPin, "hazardLight", lightOutputMediaType), "Cant create hazard light");
    }
    else if (eStage == StageGraphReady)
    {
		cObjectPtr<IMediaSerializer> serializer;
		RETURN_IF_FAILED(this->coderDescriptionOutput->GetMediaSampleSerializer(&serializer));
		this->ddlSize = serializer->GetDeserializedSize();
    }

    RETURN_NOERROR;
}

tResult LightFilter::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
	tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
    RETURN_IF_POINTER_NULL(descriptionSignalValue);        

	mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
	RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

	RETURN_NOERROR;
}

tResult LightFilter::createInputPin(cInputPin &pin, const char *pinName, cObjectPtr<IMediaType> &mediaType)
{
	RETURN_IF_FAILED(pin.Create(pinName, mediaType, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}
tResult LightFilter::createOutputPin(cOutputPin &pin, const char *pinName, cObjectPtr<IMediaType> &mediaType)
{
	RETURN_IF_FAILED(pin.Create(pinName, mediaType));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult LightFilter::Start(__exception) 
{
	RETURN_IF_FAILED(cFilter::Start( __exception_ptr));
	
	RETURN_NOERROR;
}

tResult LightFilter::Stop(__exception) 
{
	RETURN_IF_FAILED(cFilter::Stop( __exception_ptr));

	RETURN_NOERROR;
}

tResult LightFilter::Shutdown(tInitStage eStage, __exception) 
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult LightFilter::OnPinEvent(IPin *source, tInt eventCore, tInt nParam1, tInt nParam2, IMediaSample *mediaSample) 
{
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);
	
	if (eventCore == IPinEventSink::PE_MediaSampleReceived)
	{
		if (source == &this->lightInputPin)
		{
			static Light lightValue;

			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getInputValue(mediaSample, lightValue), "Cant get light value");
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitLightValue(lightValue), "cant transmit light");
		}
	}

	RETURN_NOERROR;
}

tResult LightFilter::getInputValue(IMediaSample *mediaSample, Light &lightValue)
{
	static tTimeStamp timeStamp;
	static tInt16 value;

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionInput->Lock(mediaSample, &coder));
	
	coder->Get("ui16Angle", (tVoid*) &value);
	coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
	
	RETURN_IF_FAILED(this->coderDescriptionInput->Unlock(coder));

	getLightFromIntValue(value, lightValue);

	RETURN_NOERROR;
}

tVoid LightFilter::getLightFromIntValue(const tInt16 &value, Light &lightValue)
{
	switch (value)
	{
	case LIGHT_HEAD:
		lightValue = LIGHT_HEAD;
		this->headLight = true;
		break;

	case LIGHT_BREAK:
		lightValue = LIGHT_BREAK;
		this->breakLight = true;
		break;

	case LIGHT_REVERSE:
		lightValue = LIGHT_REVERSE;
		this->reverseLight = true;
		break;

	case LIGHT_TURN_RIGHT:
		lightValue = LIGHT_TURN_RIGHT;
		this->turnRightLight = true;
		break;

	case LIGHT_TURN_LEFT:
		lightValue = LIGHT_TURN_LEFT;
		this->turnLeftLight = true;
		break;

	case LIGHT_HAZARD:
		lightValue = LIGHT_HAZARD;
		this->hazardLight = true;
		break;

	case LIGHT_BREAK_DISABLED:
		lightValue = LIGHT_BREAK_DISABLED;
		this->breakLight = false;
		break;

	case LIGHT_TURN_DISABLED:
		lightValue = LIGHT_TURN_DISABLED;
		this->turnRightLight = false;
		this->turnLeftLight = false;
		break;

	case LIGHT_REVERSE_DISABLED:
		lightValue = LIGHT_REVERSE_DISABLED;
		this->reverseLight = false;
		break;

	case LIGHT_HAZARD_DISABLED:
		lightValue = LIGHT_HAZARD_DISABLED;
		this->hazardLight = false;
		break;

	default:
		lightValue = LIGHT_HEAD;
		this->headLight = true;
		break;
	}
}

tResult LightFilter::transmitLightValue(const Light &light)
{
	switch (light)
	{
	case LIGHT_HEAD:
		RETURN_IF_FAILED(transmit(this->headLightPin, this->headLight));
		break;

	case LIGHT_BREAK_DISABLED:
	case LIGHT_BREAK:
		RETURN_IF_FAILED(transmit(this->breakLightPin, this->breakLight));
		break;

	case LIGHT_REVERSE:
	case LIGHT_REVERSE_DISABLED:
		RETURN_IF_FAILED(transmit(this->reverseLightPin, this->reverseLight));
		break;


	case LIGHT_TURN_RIGHT:
	case LIGHT_TURN_LEFT:
	case LIGHT_TURN_DISABLED:
		RETURN_IF_FAILED(transmit(this->turnLeftPin, this->turnLeftLight));
		RETURN_IF_FAILED(transmit(this->turnRightPin, this->turnRightLight));
		break;
	
	case LIGHT_HAZARD:
	case LIGHT_HAZARD_DISABLED:
		RETURN_IF_FAILED(transmit(this->hazardLightPin, this->hazardLight));
		break;

	default:
		RETURN_IF_FAILED(transmit(this->headLightPin, this->headLight));
		break;
	}

	RETURN_NOERROR;
}

tResult LightFilter::transmit(cOutputPin &pin, const tBool &value)
{
	cObjectPtr<IMediaSample> mediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSize));

	//write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionOutput->WriteLock(mediaSample, &coder));	

	static tTimeStamp now;
	now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

	coder->Set("bValue", (tVoid*) &value);
	coder->Set("ui32ArduinoTimestamp", (tVoid*) &now);
	RETURN_IF_FAILED(this->coderDescriptionOutput->Unlock(coder));

	//transmit media sample over output pin
	RETURN_IF_FAILED(mediaSample->SetTime(now));
	RETURN_IF_FAILED(pin.Transmit(mediaSample));

	RETURN_NOERROR;
}