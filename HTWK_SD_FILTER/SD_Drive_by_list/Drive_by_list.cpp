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
 * @author akluge
 * @details
 */
#include <list>
#include "stdafx.h"
#include "Drive_by_list.h"

/// Create filter for Driving actions by list commands
ADTF_FILTER_PLUGIN("SD Drive_by_list", OID_ADTF_SD_Drive_by_list, cDrive_by_list);


cDrive_by_list::cDrive_by_list(const tChar* __info) : adtf::cFilter(__info)
{
}

cDrive_by_list::~cDrive_by_list()
{
}

tResult cDrive_by_list::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
	
	    // get a media type for the output pin
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);        
        cObjectPtr<IMediaType> pTypeSignal = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
        
        // create and register the output pin
      	RETURN_IF_FAILED(m_pin_output_acceleration.Create("Acceleration_out", pTypeSignal, static_cast<IPinEventSink*> (this)));
	    RETURN_IF_FAILED(RegisterPin(&m_pin_output_acceleration));

		RETURN_IF_FAILED(m_pin_output_steeringAngle.Create("steeringAngle_out", pTypeSignal, static_cast<IPinEventSink*> (this)));
	    RETURN_IF_FAILED(RegisterPin(&m_pin_output_steeringAngle));

        RETURN_IF_FAILED(pTypeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput));
		//Create the input pins
		RETURN_IF_FAILED(m_pin_input_wheelRotation.Create("WheelRotation_in", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_pin_input_wheelRotation));

		RETURN_IF_FAILED(m_pin_input_init.Create("init", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_pin_input_init));

		// build CommandList
		addListElement(20,1,100);
		addListElement(20,20,50);
		addListElement(30,1,100);
		addListElement(20,-20,100);
		addListElement(40,20,50);
		addListElement(20,1,100);
		addListElement(0,0,0);

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
		m_nInitWaitCounter = 0;
		m_nWaitValue = 300;
		m_nFirstMove = true;
		distance = 0;
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cDrive_by_list::Start(__exception) {
	RETURN_IF_FAILED(cFilter::Start( __exception_ptr));

	RETURN_NOERROR;
}

tResult cDrive_by_list::Stop(__exception) {
	RETURN_IF_FAILED(cFilter::Stop( __exception_ptr));

	RETURN_NOERROR;
}

tResult cDrive_by_list::Shutdown(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageGraphReady)
    {
    }

	RETURN_NOERROR;
}

tResult cDrive_by_list::OnPinEvent( IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		tFloat32 signalValue = 0;
        tUInt32 timeStamp = 0;
		if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL) {
            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));
            
            //get values from media sample        
            pCoderInput->Get("f32Value", (tVoid*)&signalValue);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignalInput->Unlock(pCoderInput);                 
        } else
			RETURN_ERROR(ERR_FAILED);
	
		if (pSource == &m_pin_input_wheelRotation) {
			if(!m_nFirstMove){
			tInt rotation = tInt(signalValue);
			if((rotation - distance) >= curCmdElement.rotation-3){ // don't miss the next step
				LOG_INFO(cString::Format("New CMD %d Rotations",rotation));
				distance += curCmdElement.rotation;
				getNextCmd();
			}
			LOG_INFO(cString::Format("Recive %d Rotations -> Send ACC %f",rotation,curCmdElement.acc));
			sendNewValue(&m_pin_output_acceleration,curCmdElement.acc,pMediaSample->GetTime());
			LOG_INFO(cString::Format("Recive %d Rotations -> Send Angle %f",rotation,curCmdElement.angle));
			sendNewValue(&m_pin_output_steeringAngle,curCmdElement.angle,pMediaSample->GetTime());
			}
		}

		if( m_nInitWaitCounter <=m_nWaitValue){
			m_nInitWaitCounter++;
			if(m_nInitWaitCounter ==100){
				sendNewValue(&m_pin_output_acceleration, 0, pMediaSample->GetTime());
			} // dummy init debug steps
			if(m_nInitWaitCounter ==120){
				sendNewValue(&m_pin_output_steeringAngle, 5, pMediaSample->GetTime());
			}
			if(m_nInitWaitCounter ==140){
				sendNewValue(&m_pin_output_steeringAngle, 10, pMediaSample->GetTime());
			}
			if(m_nInitWaitCounter ==160){
				sendNewValue(&m_pin_output_steeringAngle, 15, pMediaSample->GetTime());
			}
			if(m_nInitWaitCounter ==180){
				sendNewValue(&m_pin_output_steeringAngle, 5, pMediaSample->GetTime());
			}
			LOG_INFO(cString::Format("Waiting steps: %d",m_nWaitValue-m_nInitWaitCounter));

			if(m_nFirstMove && m_nInitWaitCounter == m_nWaitValue){
					m_nFirstMove = false;
					getNextCmd();
					LOG_INFO(cString::Format("Init-Pin -> Send Acc %f",curCmdElement.acc));
					sendNewValue(&m_pin_output_acceleration, curCmdElement.acc, pMediaSample->GetTime());
					sendNewValue(&m_pin_output_steeringAngle, curCmdElement.angle, pMediaSample->GetTime());
				}
		}

		/*
		if (pSource == &m_pin_input_init) {
				if(m_nFirstMove){
					m_nFirstMove = false;
					getNextCmd();
					LOG_INFO(cString::Format("Init-Pin -> Send Acc %f",curCmdElement.acc));
					sendNewValue(&m_pin_output_acceleration, curCmdElement.acc, pMediaSample->GetTime());
					sendNewValue(&m_pin_output_steeringAngle, curCmdElement.angle, pMediaSample->GetTime());
				}
		}
		*/
	}
	RETURN_NOERROR;
}

tResult cDrive_by_list::getNextCmd(){
	if(!list.empty()){
		curCmdElement = list.front();
		LOG_INFO(cString::Format("CurElement %f -- %f -- %d ",curCmdElement.acc,curCmdElement.angle,curCmdElement.rotation));
		list.pop_front();
	}
	RETURN_NOERROR;	
}


void cDrive_by_list::addListElement(tFloat32 accel, tFloat32 angle, tInt rota){
	curCmdElement.acc = accel;
	curCmdElement.angle = angle;
	curCmdElement.rotation = rota;
	list.push_back(curCmdElement);
}

tResult cDrive_by_list::sendNewValue(cOutputPin * outpin, tFloat32 flValue, tTimeStamp timeStamp) {
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalInput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoder;
	RETURN_IF_FAILED(m_pCoderDescSignalInput->WriteLock(pMediaSample, &pCoder));	
		
	pCoder->Set("f32Value", (tVoid*)&flValue);	
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    m_pCoderDescSignalInput->Unlock(pCoder);
    
    //transmit media sample over output pin
    pMediaSample->SetTime(timeStamp);
    outpin->Transmit(pMediaSample);
	RETURN_NOERROR;
}
