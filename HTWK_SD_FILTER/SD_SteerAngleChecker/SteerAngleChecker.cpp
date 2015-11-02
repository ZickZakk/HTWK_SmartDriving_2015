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
#include "SteerAngleChecker.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, SteerAngleChecker);

SteerAngleChecker::SteerAngleChecker(const char *__info) : cFilter(__info)
{
}

SteerAngleChecker::~SteerAngleChecker(void)
{}

tResult SteerAngleChecker::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	if (eStage == StageFirst)
	{
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

		cObjectPtr<IMediaType> steerAngleInputMediaType;
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(initMediaType("tSignalValue", steerAngleInputMediaType, this->steerAngleMediaDescription), "cant init mediatype");
		RETURN_IF_FAILED_AND_LOG_ERROR_STR(createInputPin("steerAngle", this->steerAngleInput, steerAngleInputMediaType), "cant create input pin");
	}
	
	RETURN_NOERROR;
}

tResult SteerAngleChecker::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult SteerAngleChecker::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult SteerAngleChecker::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult SteerAngleChecker::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);

	LOG_INFO("Got input");

	if (eventCore == IPinEventSink::PE_MediaSampleReceived)
	{
		if (source == &this->steerAngleInput)
		{
			static tInt steerAngleInt;
			// static tUInt16 steerAngleInt16;
			LOG_INFO("Got steering angle");
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getValue(mediaSample, steerAngleInt), "Cant get int value");
			// RETURN_IF_FAILED_AND_LOG_ERROR_STR(getValue(mediaSample, steerAngleInt16), "Cant get int16 value");

			LOG_INFO(cString::Format("Steer angle int  : %d", steerAngleInt));
			// LOG_INFO(cString::Format("Steer angle int16: %d", steerAngleInt16));
		}
	}

	RETURN_NOERROR;
}

tResult SteerAngleChecker::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
	tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
    RETURN_IF_POINTER_NULL(descriptionSignalValue);        

	mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
	RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

	RETURN_NOERROR;
}

tResult SteerAngleChecker::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult SteerAngleChecker::getValue(IMediaSample *mediaSample, tInt &distance) const
{
	static tTimeStamp timeStamp;
	static tFloat32 steerAngle;

	cObjectPtr<IMediaCoder> coderInput;
	RETURN_IF_FAILED(this->steerAngleMediaDescription->Lock(mediaSample, &coderInput));
            
	//get values from media sample        
	RETURN_IF_FAILED(coderInput->Get("f32Value", (tVoid*) &steerAngle));
	RETURN_IF_FAILED(coderInput->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp));
	RETURN_IF_FAILED(this->steerAngleMediaDescription->Unlock(coderInput));
	
	LOG_INFO(cString::Format("f32value: %f", steerAngle));

	distance = static_cast<tInt>(steerAngle);

	RETURN_NOERROR;
}

tResult SteerAngleChecker::getValue(IMediaSample *mediaSample, tUInt16 &value) const
{
	static tTimeStamp timeStamp;

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->steerAngleMediaDescription->Lock(mediaSample, &coder));
	
	RETURN_IF_FAILED(coder->Get("ui16Angle", (tVoid*) &value));
	RETURN_IF_FAILED(coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp));
	
	RETURN_IF_FAILED(this->steerAngleMediaDescription->Unlock(coder));

	RETURN_NOERROR;
}