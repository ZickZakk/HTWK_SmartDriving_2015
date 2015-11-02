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
 * @author epeterse
 * @details
 */

#include "stdafx.h"
#include "Macro_recorder.h"

#include <iostream>
#include <fstream>


/// Create filter
ADTF_FILTER_PLUGIN("SD macro recorder", OID_ADTF_SD_Macro_recorder, cMacro_recorder);


cMacro_recorder::cMacro_recorder(const tChar* __info) : adtf::cFilter(__info)
{
	m_nLastMSTime = 0;
}

cMacro_recorder::~cMacro_recorder()
{
}

tResult cMacro_recorder::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		cObjectPtr<IMediaType> pTypeStruct;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);

		// Media type definition
        cObjectPtr<IMediaType> pTypeSignal = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(AllocMediaType((tVoid**)&pTypeStruct, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED, NULL, NULL));

        // Create the output pins
        //RETURN_IF_FAILED(m_oOAcceleration.Create("acceleration_out", pTypeSignal, NULL));
        //RETURN_IF_FAILED(RegisterPin(&m_oOAcceleration));

		// Media type definition
        //cObjectPtr<IMediaType> pTypeIn = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput)); 

		//Create the input pins		
		RETURN_IF_FAILED(m_oIWheelL.Create("wheel_left", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIWheelL));

		RETURN_IF_FAILED(m_oIWheelR.Create("wheel_right", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIWheelR));

		
		RETURN_IF_FAILED(m_oIServo.Create("servo", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIServo));
    }
    else if (eStage == StageNormal)
    {
		R = -1.0;
		L = -1.0;
		S = -1.0;
    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

tResult cMacro_recorder::Start(__exception) {
	RETURN_IF_FAILED(cFilter::Start( __exception_ptr));
	m_nLastMSTime = 0;

	RETURN_NOERROR;
}

tResult cMacro_recorder::Stop(__exception) {
	RETURN_IF_FAILED(cFilter::Stop( __exception_ptr));

	RETURN_NOERROR;
}

tResult cMacro_recorder::Shutdown(tInitStage eStage, __exception) {
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

tResult cMacro_recorder::sendNewValue(cOutputPin * outpin, tFloat32 value, tTimeStamp timeStamp) {
   	tFloat32 flValue= value;
							
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

void cMacro_recorder::writeToFile(std::string source, tFloat32 R, tFloat32 L, tFloat32 S) {
  std::ofstream myfile;
  myfile.open ("/home/odroid/Desktop/Macro_record.txt", ios::out | ios::app);
  myfile << source << " " << R << " " << L << " " << S << "\n";
  LOG_INFO(cString::Format("R %f L %f S %f", R, L, S));
  myfile.close();
}

tResult cMacro_recorder::OnPinEvent( IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		tFloat32 signalValue = 0;
		tUInt32 timeStamp = 0;
		m_nLastMSTime = _clock->GetStreamTime();

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
	
		if (pSource == &m_oIWheelL) { 
			L = signalValue;
		} else if (pSource == &m_oIWheelR) {
			R = signalValue;
		} else if (pSource == &m_oIServo) {
			if (R != -1 && L != -1) {
				writeToFile("S", R, L, signalValue);
			}
		}
	}
	RETURN_NOERROR;
}
