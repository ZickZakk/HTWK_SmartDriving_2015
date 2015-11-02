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
#include "Emergency.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_ADTF_SD_EMERGENCY_SYSTEM, Emergency);

Emergency::Emergency(const char *__info) : cFilter(__info)
{
    this->m_nLastMSTime = 0;
    this->m_nEmergencyDistanceFront = 8;
    this->m_nEmergencyDistanceRear = 8;
    this->m_nEmergencyDistanceFrontUss = 8;
    this->m_bEmergencybreak = true;

    this->steerAngle = 0;
    this->initSteerAngle = true;

    SetPropertyInt(PROPERTY_EMERGENCY_DISTANCE_FRONT, m_nEmergencyDistanceFront);
    SetPropertyBool(PROPERTY_EMERGENCY_DISTANCE_FRONT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROPERTY_EMERGENCY_DISTANCE_REAR, m_nEmergencyDistanceRear);
    SetPropertyBool(PROPERTY_EMERGENCY_DISTANCE_REAR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROPERTY_EMERGENCY_DISTANCE_FRONT_USS, m_nEmergencyDistanceFrontUss);
    SetPropertyBool(PROPERTY_EMERGENCY_DISTANCE_FRONT_USS NSSUBPROP_ISCHANGEABLE, tTrue);

    this->m_size=0;
}

Emergency::~Emergency(void)
{}

tResult Emergency::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaType> typeSignal;
        cObjectPtr<IMediaType> typeSignalOut;

        RETURN_IF_FAILED(initInputPins(__exception_ptr, typeSignal));
        RETURN_IF_FAILED(initOutputPins(typeSignalOut));

        RETURN_IF_FAILED(typeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &this->coderDescriptionSignalInput));

        RETURN_IF_FAILED_AND_LOG_ERROR_STR(createInputPin("front_ir_fusion", this->m_oIFrontIRF, typeSignal), "Failed creating IR Fusion!");
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(createOutputPin("decision", this->decisionPin, typeSignalOut), "Failed creating decision output!");
    }

    else if (eStage == StageNormal)
    {
        this->m_nEmergencyDistanceFront = GetPropertyInt(PROPERTY_EMERGENCY_DISTANCE_FRONT);
        this->m_nEmergencyDistanceRear = GetPropertyInt(PROPERTY_EMERGENCY_DISTANCE_REAR);
        this->m_nEmergencyDistanceFrontUss = GetPropertyInt(PROPERTY_EMERGENCY_DISTANCE_FRONT_USS);
    }
    else if (eStage == StageGraphReady)
    {
        cObjectPtr<IMediaSerializer> serializer;
        RETURN_IF_FAILED(this->coderDescriptionSignalOutput->GetMediaSampleSerializer(&serializer));
        m_size = serializer->GetDeserializedSize();
    }

    RETURN_NOERROR;
}
tResult Emergency::initInputPins(IException** __exception_ptr, cObjectPtr<IMediaType> &typeSignal)
{
    std::string signalValueName = "tSignalValue";

    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

    tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(signalValueName.c_str());
    RETURN_IF_POINTER_NULL(descriptionSignalValue);

    typeSignal = new cMediaType(0, 0, 0, signalValueName.c_str(), descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(typeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &this->coderDescriptionSignalInput));

    RETURN_NOERROR;
}
tResult Emergency::initOutputPins(cObjectPtr<IMediaType> &typeSignalOut)
{
    std::string signalValueName = "tSteeringAngleData";

    tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(signalValueName.c_str());
    RETURN_IF_POINTER_NULL(descriptionSignalValue);

    typeSignalOut = new cMediaType(0, 0, 0, signalValueName.c_str(), descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(typeSignalOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &this->coderDescriptionSignalOutput));

    RETURN_NOERROR;
}
tResult Emergency::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
    RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&pin));

    RETURN_NOERROR;
}
tResult Emergency::createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
    RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
    RETURN_IF_FAILED(RegisterPin(&pin));

    RETURN_NOERROR;
}
tResult Emergency::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    RETURN_NOERROR;
}
tResult Emergency::Stop(__exception)
{
    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}
tResult Emergency::Shutdown(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

    RETURN_NOERROR;
}
tResult Emergency::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
    RETURN_IF_POINTER_NULL(source);
    RETURN_IF_POINTER_NULL(mediaSample);

    if (eventCore == IPinEventSink::PE_MediaSampleReceived)
    {
        if (source == &this->m_oIFrontIRF) //&& (steerAngle< 5  && steerAngle > -5) && !initSteerAngle
        {
            static tFloat32 signalValue = 0;
           // this->m_nLastMSTime = pMediaSample->GetTime();

            RETURN_IF_FAILED_AND_LOG_ERROR_STR(getValueFloat(mediaSample, signalValue), "Failed getting signalValue");

            this->m_nEmergencyDistanceFront = GetPropertyInt(PROPERTY_EMERGENCY_DISTANCE_FRONT);
            this->m_bEmergencybreak = (signalValue <= this->m_nEmergencyDistanceFront);

            LOG_INFO(cString::Format("IRF  _____  value: %d, steerangle: %d", signalValue, steerAngle));
            if (this->m_bEmergencybreak)
            {
                RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(EMERGENCY_STOP), "Cant send decision to DriveAlgorithm");
                //break_and_wink(pMediaSample->GetTime(), true);
                this->initSteerAngle = true;
            }
            else{
                RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDecision(EMERGENCY_START), "Cant send decision to DriveAlgorithm");
            }
        }
    }

    RETURN_NOERROR;
}
tResult Emergency::getValueFloat(IMediaSample *mediaSample, tFloat32 &value) const
{
    cObjectPtr<IMediaCoder> coderInput;
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Lock(mediaSample, &coderInput));

    //get values from media sample
    RETURN_IF_FAILED(coderInput->Get("f32Value", (tVoid*) &value));
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Unlock(coderInput));

    RETURN_NOERROR;
}

tResult Emergency::getValue(IMediaSample *mediaSample, tInt &value) const
{
    cObjectPtr<IMediaCoder> coderInput;
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Lock(mediaSample, &coderInput));

    //get values from media sample
    RETURN_IF_FAILED(coderInput->Get("f32Value", (tVoid*) &value));
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Unlock(coderInput));

    RETURN_NOERROR;
}


/*
tResult Emergency::transmitValue(cOutputPin &pin, const tFloat32 &value)
{
    tUInt32 timeStamp = (tUInt32) (_clock ? _clock->GetStreamTime()/1000 : adtf_util::cHighResTimer::GetTime()/1000);

    // create new media sample
    cObjectPtr<IMediaSample> mediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));

    // allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> serializer;
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->GetMediaSampleSerializer(&serializer));
    tInt size = serializer->GetDeserializedSize();
    RETURN_IF_FAILED(mediaSample->AllocBuffer(size));

    // write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> coder;
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->WriteLock(mediaSample, &coder));
    RETURN_IF_FAILED(coder->Set("f32Value", (tVoid*) &value));
    RETURN_IF_FAILED(coder->Set("ui32ArduinoTimestamp", (tVoid*) &timeStamp));
    RETURN_IF_FAILED(this->coderDescriptionSignalInput->Unlock(coder));

    // transmit media sample over output pin
    RETURN_IF_FAILED(mediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(pin.Transmit(mediaSample));
    RETURN_NOERROR;
}
*/
tResult Emergency::sendDecision(const Emergency_Decision &decision)
{
    tTimeStamp timeStamp = (_clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime());

    cObjectPtr<IMediaSample> mediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));

    RETURN_IF_FAILED(mediaSample->AllocBuffer(m_size));

    cObjectPtr<IMediaCoder> coder;
    RETURN_IF_FAILED(this->coderDescriptionSignalOutput->WriteLock(mediaSample, &coder));

    RETURN_IF_FAILED(coder->Set("ui16Angle", (tVoid*) &decision));
    RETURN_IF_FAILED(coder->Set("ui32ArduinoTimestamp", (tVoid*) &timeStamp));

    RETURN_IF_FAILED(this->coderDescriptionSignalOutput->Unlock(coder));

    RETURN_IF_FAILED(mediaSample->SetTime(timeStamp));
    RETURN_IF_FAILED(this->decisionPin.Transmit(mediaSample));


    RETURN_NOERROR;
}
