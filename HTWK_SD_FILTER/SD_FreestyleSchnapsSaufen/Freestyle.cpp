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

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, Test);

Test::Test(const char *__info) : cFilter(__info)
{
	this->isFirstFrame = true;
	this->isStopLineFound = false;
	this->isConnectedToServer = false;
	this->crossroadDetector.reset(new CrossRoadDetector());
	this->driveAlgorithm.reset(new DriveAlgorithm());
	this->tcpClient.reset(new Client());
	this->stopOnStopLine = false;

	this->tcpServer.reset(new Server(60000));

	this->ip = "192.168.1.253";
	SetPropertyStr(PROP_NAME_IP, this->ip.c_str());
    SetPropertyBool(PROP_NAME_IP NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_IP NSSUBPROP_ISCHANGEABLE, tTrue);

	this->numberOfStopLines = 20;
	SetPropertyInt(PROP_NAME_STOP_LINES, this->numberOfStopLines);
	SetPropertyInt(PROP_NAME_STOP_LINES NSSUBPROP_MIN, 1);
	SetPropertyInt(PROP_NAME_STOP_LINES NSSUBPROP_MAX, 1000);
	SetPropertyBool(PROP_NAME_STOP_LINES NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_STOP_LINES NSSUBPROP_ISCHANGEABLE, tTrue);

	this->delay = 100000;
	SetPropertyInt(PROP_NAME_DELAY, this->delay);
	SetPropertyInt(PROP_NAME_DELAY NSSUBPROP_MIN, 1);
	SetPropertyInt(PROP_NAME_DELAY NSSUBPROP_MAX, 1000);
	SetPropertyBool(PROP_NAME_DELAY NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_DELAY NSSUBPROP_ISCHANGEABLE, tTrue);

	this->driveSpeed = 30;
	SetPropertyInt(PROP_NAME_DRIVE_SPEED, this->driveSpeed);
	SetPropertyInt(PROP_NAME_DRIVE_SPEED NSSUBPROP_MIN, 1);
	SetPropertyInt(PROP_NAME_DRIVE_SPEED NSSUBPROP_MAX, 1000);
	SetPropertyBool(PROP_NAME_DRIVE_SPEED NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_DRIVE_SPEED NSSUBPROP_ISCHANGEABLE, tTrue);

	this->smoothCurveValue = 7;
	SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE, this->smoothCurveValue);
	SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_MIN, 1);
	SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_MAX, 1000);
	SetPropertyBool(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_ISCHANGEABLE, tTrue);

	this->ticksToStopLine = 10;
	SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE, this->ticksToStopLine);
	SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_MIN, 1);
	SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_MAX, 1000);
	SetPropertyBool(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_ISCHANGEABLE, tTrue);
}

Test::~Test()
{

}

tResult Test::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{	

		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

		cObjectPtr<IMediaType> typeSignalManeuver;
		cObjectPtr<IMediaType> typeSignalSteeringAngle;
		cObjectPtr<IMediaType> typeSignalAcceleration;
		cObjectPtr<IMediaType> typeSignalWheelTicks;

		RETURN_IF_FAILED(initMediaType("tSteeringAngleData", typeSignalManeuver, this->coderDescriptionManeuver));
		RETURN_IF_FAILED(initMediaType("tSignalValue", typeSignalSteeringAngle, this->coderDescriptionSteeringAngle));
		RETURN_IF_FAILED(initMediaType("tSignalValue", typeSignalAcceleration, this->coderDescriptionAcceleration));
		RETURN_IF_FAILED(initMediaType("tSignalValue", typeSignalWheelTicks, this->coderDescriptionWheelTicks));

		// input pins
		RETURN_IF_FAILED(createVideoInputPin("rgbVideo", this->xtionPin));
		RETURN_IF_FAILED(createInputPin("maneuver", this->maneuverPin, typeSignalManeuver));
		RETURN_IF_FAILED(createInputPin("wheelTicks", this->wheelTicksPin, typeSignalWheelTicks));

		// output pins
		RETURN_IF_FAILED(createOutputPin("steeringAngle", this->steeringAnglePin, typeSignalSteeringAngle));
		RETURN_IF_FAILED(createOutputPin("acceleration", this->accelerationPin, typeSignalAcceleration));
	}
	else if (eStage == StageGraphReady)
	{
		cObjectPtr<IMediaSerializer> serializer;
		RETURN_IF_FAILED(this->coderDescriptionAcceleration->GetMediaSampleSerializer(&serializer));
		this->ddlSizeUI16 = serializer->GetDeserializedSize();

		std::thread test(&Test::accept, this);
	}

	RETURN_NOERROR;
}

tResult Test::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
	tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
    RETURN_IF_POINTER_NULL(descriptionSignalValue);        

	mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
	RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

	RETURN_NOERROR;
}

tResult Test::createVideoInputPin(const tChar *pinName, cVideoPin &pin)
{
	pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
	RETURN_IF_FAILED(RegisterPin(&pin));
	
	RETURN_NOERROR;
}

tResult Test::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult Test::createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult Test::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult Test::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult Test::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult Test::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);

	if (eventCore == IPinEventSink::PE_MediaSampleReceived)
	{
		if (source == &this->maneuverPin)
		{
			static tUInt16 tmpManeuver;
			getManeuver(mediaSample, maneuverPin, this->coderDescriptionManeuver, tmpManeuver);

			if (tmpManeuver == MANEUVER_STRAIGHT)
			{
				this->isDriveActive = true;

				RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitAcceleration(this->driveSpeed), "Cant transmit drive");
			}
		}
		else if (source == &this->wheelTicksPin)
		{
			if (!this->isConnectedToServer)
			{
				 if (!tcpClient->connectToServer(this->ip, 5555))
				 {
					 LOG_INFO(cString::Format("Cant connect to server with ip: %s:5555", this->ip.c_str()));

					 RETURN_NOERROR;
				 }
				 else
				 {
					 this->isConnectedToServer = true;
					 LOG_INFO("Could connect to server :)");
				 }
			}
			
			static tFloat32 tmpWheelTicks;
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getWheelTicks(mediaSample, tmpWheelTicks), "cant get wheel ticks");

			this->currentWheelTicks = static_cast<int>(tmpWheelTicks);

			if (this->isStopLineFound)
			{
				driveToStopLine();
			}
		}
		else if(source == &this->xtionPin)
		{
			if (this->isFirstFrame)
			{
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(initVideoStream(), "Cant init video stream");
				this->isFirstFrame = false;
			}
			else 
			{
				const tVoid *buffer;

				if (this->isDriveActive && IS_OK(mediaSample->Lock(&buffer)))
				{
					//Receive the image
					Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*) buffer);
					Mat result = image.clone();
					RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaSample->Unlock(buffer), "Cant unlock image");

					this->driveAlgorithm->prepareImage(result);

					if (!this->isStopLineFound)
					{
						this->isStopLineFound = this->crossroadDetector-> searchStopLine(result);
						this->ticksToDrive = this->ticksToStopLine + this->currentWheelTicks;
					}
				}
			}
		}
	}
	
	RETURN_NOERROR;
}

void Test::accept(void)
{
	tcpServer->startServer();
}

tResult Test::initVideoStream(void)
{
	//Read media type
	cObjectPtr<IMediaType> type;
	RETURN_IF_FAILED(this->xtionPin.GetMediaType(&type));

	cObjectPtr<IMediaTypeVideo> typeVideo;
	RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)(&typeVideo)));

	const tBitmapFormat *format = typeVideo->GetFormat();
	RETURN_IF_POINTER_NULL(format);

	//Set media type
	setBitmapFormat(format);

	RETURN_NOERROR;
}

tVoid Test::setBitmapFormat(const tBitmapFormat *format)
{
	this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
	this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
	this->videoInputInfo.nPaletteSize = format->nPaletteSize;
	this->videoInputInfo.nPixelFormat = format->nPixelFormat;
	this->videoInputInfo.nHeight = format->nHeight;
	this->videoInputInfo.nWidth = format->nWidth;
	this->videoInputInfo.nSize = format->nSize;
}

tResult Test::driveToStopLine(void)
{
	static tInt tmpDistance;
	tmpDistance = this->ticksToDrive - this->currentWheelTicks;
	
	if (tmpDistance <= 0)
	{
		if (this->stopOnStopLine)
		{
			RETURN_IF_FAILED(transmitAcceleration(-5.0f));

			// sende zum anderen Auto, dass ich warte
		}
		else
		{
			// sende zum anderen Auto
			this->numberOfStopLines--;

			// sende...
			LOG_INFO(cString::Format("StopLines: %d", this->numberOfStopLines));

			this->isStopLineFound = false;
		}
	}

	RETURN_NOERROR;
}

tResult Test::transmitSteeringAngle(const tFloat32 value)
{
	RETURN_IF_FAILED(transmitF32Value(this->steeringAnglePin, this->coderDescriptionSteeringAngle, value));
	RETURN_NOERROR;
}

tResult Test::transmitAcceleration(const tFloat32 value)
{
	RETURN_IF_FAILED(transmitF32Value(this->accelerationPin, this->coderDescriptionAcceleration, value));
	RETURN_NOERROR;
}

tResult Test::transmitStop(const tFloat32 value)
{
	RETURN_IF_FAILED(transmitF32Value(this->accelerationPin, this->coderDescriptionAcceleration, value));
	RETURN_NOERROR;
}

tResult Test::transmitF32Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tFloat32 value)
{
	cObjectPtr<IMediaSample> mediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeUI16));
       
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

tResult Test::getWheelTicks(IMediaSample *mediaSample, tFloat32 &value)
{
	RETURN_IF_FAILED(getF32Value(mediaSample, this->coderDescriptionWheelTicks, value));
	RETURN_NOERROR;
}

tResult Test::getF32Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tFloat32 &value)
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

tResult Test::getManeuver(IMediaSample *mediaSample, cInputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, tUInt16 &value)
{
	static tUInt16 tmpValue;
	static tUInt32 timeStamp;

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Lock(mediaSample, &coder), "Get UI16 failed to unlock");
	
	coder->Get("ui16Angle", (tVoid*) &tmpValue);
	coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
	value = tmpValue;

	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Get UI16 failed to unlock");

	RETURN_NOERROR;
}

tResult Test::PropertyChanged(const char *name)
{
	this->ip = GetPropertyStr(PROP_NAME_IP);
	this->numberOfStopLines = GetPropertyInt(PROP_NAME_STOP_LINES);
	this->delay = GetPropertyInt(PROP_NAME_DELAY);
	this->driveSpeed = GetPropertyInt(PROP_NAME_DRIVE_SPEED);
	this->smoothCurveValue = GetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE);
	this->ticksToStopLine = GetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE);

	RETURN_NOERROR;
}