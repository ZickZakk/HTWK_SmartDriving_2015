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
#include "SmartCurve.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, SmartCurve);

SmartCurve::SmartCurve(const char *__info) : cFilter(__info)
{
    this->isDebugActive = false;
    this->rotationLeft = 0;
    this->rotationRight = 0;
    this->wheelRotation.reset(new WheelRotation());
    this->leftMeasuringError = 0;
    this->rightMeasuringError = 0;
    this->maxAmountOfMeasuringErrors = 5;
    this->toleranceMeasuringError = 15;

    SetPropertyInt(PROPERTY_MAX_AMOUNT_MEASURING_ERROR, this->maxAmountOfMeasuringErrors);
    SetPropertyBool(PROPERTY_MAX_AMOUNT_MEASURING_ERROR NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(PROPERTY_TOLERANCE_OF_MEASURING_ERROR, this->toleranceMeasuringError);
    SetPropertyBool(PROPERTY_TOLERANCE_OF_MEASURING_ERROR NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyBool(PROP_NAME_DEBUG_SMARTCURVE, this->isDebugActive);
    SetPropertyBool(PROP_NAME_DEBUG_SMARTCURVE NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool(PROP_NAME_DEBUG_SMARTCURVE NSSUBPROP_ISCHANGEABLE, tTrue);
    
	setIsDebugActive(this->isDebugActive);
}


SmartCurve::~SmartCurve(void)
{}

tResult SmartCurve::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaType> typeSignalInput;
        cObjectPtr<IMediaType> typeSignalOutput;

        RETURN_IF_FAILED(initInputPins(__exception_ptr, typeSignalInput));
        RETURN_IF_FAILED(initOutputPins(typeSignalOutput));

        // input pins
        RETURN_IF_FAILED(createInputPin("wheelRotationLeft", this->wheelRotationLeft, typeSignalInput));
        RETURN_IF_FAILED(createInputPin("wheelRotationright", this->wheelRotationRight, typeSignalInput));

        // output pins
        RETURN_IF_FAILED(createOutputPin("mergedRotation", this->mergedRotation, typeSignalOutput));
    }
    else if (eStage == StageGraphReady)
    {
        cObjectPtr<IMediaSerializer> serializer;
        RETURN_IF_FAILED(this->coderDescriptionSignalInput->GetMediaSampleSerializer(&serializer));
        this->ddlSizeSteeringAngle = serializer->GetDeserializedSize();
    }

    RETURN_NOERROR;
}

tResult SmartCurve::initInputPins(IException** __exception_ptr, cObjectPtr<IMediaType> &typeSignal)
{
    std::string signalValueName = "tSignalValue";

    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

    tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(signalValueName.c_str());
    RETURN_IF_POINTER_NULL(descriptionSignalValue);

    typeSignal = new cMediaType(0, 0, 0, signalValueName.c_str(), descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(typeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &this->coderDescriptionSignalInput));

    RETURN_NOERROR;
}

tResult SmartCurve::initOutputPins(cObjectPtr<IMediaType> &typeSignal)
{
    std::string signalValueName = "tSignalValue";

    tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(signalValueName.c_str());
    RETURN_IF_POINTER_NULL(descriptionSignalValue);

    typeSignal = new cMediaType(0, 0, 0, signalValueName.c_str(), descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(typeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &this->coderDescriptionSignalOutput));

    RETURN_NOERROR;
}

tResult SmartCurve::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
    RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&pin));

    RETURN_NOERROR;
}

tResult SmartCurve::createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
    RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
    RETURN_IF_FAILED(RegisterPin(&pin));

    RETURN_NOERROR;
}

tResult SmartCurve::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    RETURN_NOERROR;
}

tResult SmartCurve::Stop(__exception)
{
    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult SmartCurve::Shutdown(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

    RETURN_NOERROR;
}

tResult SmartCurve::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
    RETURN_IF_POINTER_NULL(source);
    RETURN_IF_POINTER_NULL(mediaSample);

    if (eventCore == IPinEventSink::PE_MediaSampleReceived)
    {
        if (source == &this->wheelRotationLeft || source == &this->wheelRotationRight)
        {
            static tInt currentTicks;

            if (source == &this->wheelRotationLeft)
            {
                static tFloat32 wheelRotationLeft;
                RETURN_IF_FAILED_AND_LOG_ERROR_STR(getRotation(mediaSample, wheelRotationLeft), "Cant get distance");

                tInt tempRotationLeft = this->rotationLeft;
                if(this->isDebugActive){
                    //LOG_INFO(cString::Format("Old Rotation Left: %d", tempRotationLeft));
                    //LOG_INFO(cString::Format("New Rotation Left: %d", this->rotationLeft));
                }
                this->rotationLeft = static_cast<tInt>(wheelRotationLeft);
                if(tempRotationLeft < rotationLeft-toleranceMeasuringError && leftMeasuringError < maxAmountOfMeasuringErrors){
                    leftMeasuringError++;
                    if(this->isDebugActive){
                        //LOG_INFO(cString::Format("Amount of Measuring Error: %d", this->leftMeasuringError));
                    }
                    this->wheelRotation->mergeWheelRotation(this->rotationRight, this->rotationLeft, currentTicks);
                }
                else if(tempRotationLeft >= rotationLeft-toleranceMeasuringError){
                    this->wheelRotation->mergeWheelRotation(this->rotationRight, this->rotationLeft, currentTicks);
                    this->leftMeasuringError = 0;
                    if(this->isDebugActive){
                        //LOG_WARNING("Everything is okay, NO error!");
                    }
                }
                else if(leftMeasuringError >= maxAmountOfMeasuringErrors){
                    this->wheelRotation->mergeWheelRotation(this->rotationRight, this->rotationRight, currentTicks);
                    if(this->isDebugActive){
                       // LOG_WARNING("Max Amount of Errors reached!!!!");
                    }
                }
                if(this->isDebugActive){
                    LOG_INFO(cString::Format("Left, Right, Current Ticks merged: %d %d %d", this->rotationLeft, this->rotationRight, currentTicks));
                }
            }
            else if (source == &this->wheelRotationRight)
            {
                static tFloat32 wheelRotationRight;
                RETURN_IF_FAILED_AND_LOG_ERROR_STR(getRotation(mediaSample, wheelRotationRight), "Cant get distance");

                tInt tempRotationRight = this->rotationRight;
                if(this->isDebugActive){
                    //LOG_INFO(cString::Format("Old Rotation Right: %d", tempRotationRight));
                    //LOG_INFO(cString::Format("New Rotation Right: %d", this->rotationRight));
                }
                this->rotationRight = static_cast<tInt>(wheelRotationRight);
                if(tempRotationRight < rotationRight-toleranceMeasuringError && rightMeasuringError < maxAmountOfMeasuringErrors){
                    rightMeasuringError++;
                    if(this->isDebugActive){
                      //  LOG_INFO(cString::Format("Amount of Measuring Error: %d", this->rightMeasuringError));
                    }
                    this->wheelRotation->mergeWheelRotation(this->rotationRight, this->rotationLeft, currentTicks);
                }
                else if(tempRotationRight >= rotationRight-toleranceMeasuringError){
                    this->wheelRotation->mergeWheelRotation(this->rotationRight, this->rotationLeft, currentTicks);
                    this->rightMeasuringError=0;
                    if(this->isDebugActive){
                       // LOG_WARNING("Everything is okay, NO error!");
                    }
                }
                else if(rightMeasuringError >= maxAmountOfMeasuringErrors){
                    this->wheelRotation->mergeWheelRotation(this->rotationLeft, this->rotationLeft, currentTicks);
                    if(this->isDebugActive){
                        //LOG_WARNING("Max Amount of Errors reached!!!!");
                    }
                }
                if(this->isDebugActive){
                    LOG_INFO(cString::Format("Left, Right, Current Ticks merged: %d %d %d", this->rotationLeft, this->rotationRight, currentTicks));
                }
            }

            RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitValue(this->mergedRotation, static_cast<tInt>(currentTicks)), "cant transmit merged wheel ticks");
        }
    }

    RETURN_NOERROR;
}

tResult SmartCurve::PropertyChanged(const char *name)
{
	this->maxAmountOfMeasuringErrors = GetPropertyInt(PROPERTY_MAX_AMOUNT_MEASURING_ERROR);
	this->toleranceMeasuringError = GetPropertyInt(PROPERTY_TOLERANCE_OF_MEASURING_ERROR);
	setIsDebugActive(this->isDebugActive = GetPropertyInt(PROP_NAME_DEBUG_SMARTCURVE));

	RETURN_NOERROR;
}

tResult SmartCurve::getRotation(IMediaSample *mediaSample, tFloat32 &rotation) const
{
    cObjectPtr<IMediaCoder> coderInput;
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Lock(mediaSample, &coderInput));

    //get values from media sample
    static tUInt32 timeStamp;

    coderInput->Get("f32Value", (tVoid*) &rotation);
    coderInput->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);

    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Unlock(coderInput));

    RETURN_NOERROR;
}


tResult SmartCurve::setIsDebugActive(const bool &isDebugActive)
{
    this->isDebugActive = isDebugActive;

	RETURN_NOERROR;
}

tResult SmartCurve::transmitValue(cOutputPin &pin, const tFloat32 &value)
{
    tTimeStamp timeStamp = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();

    // create new media sample
    cObjectPtr<IMediaSample> mediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));

    // allocate memory with the size given by the descriptor
    RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeSteeringAngle));

    // write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> coder;
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->WriteLock(mediaSample, &coder));
    coder->Set("f32Value", (tVoid*) &value);
    coder->Set("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Unlock(coder));

    // transmit media sample over output pin
    RETURN_IF_FAILED(mediaSample->SetTime(timeStamp));
    RETURN_IF_FAILED(pin.Transmit(mediaSample));
    
    RETURN_NOERROR;
}
