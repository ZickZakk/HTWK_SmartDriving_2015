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
#include "TSignal_converter.h"


/// Create filter
ADTF_FILTER_PLUGIN(FILTER_NAME, OID_ADTF_SD_TSIGNAL_CONVERTER, cTSignal_converter);

cTSignal_converter::cTSignal_converter(const tChar* __info) : adtf::cFilter(__info)
{
	this->m_nLastMSTime = 0;
}

cTSignal_converter::~cTSignal_converter()
{
}

tResult cTSignal_converter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);

		// Media type definition
		cObjectPtr<IMediaType> pTypeSignalFloat32 = new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_FLOAT32);
		cObjectPtr<IMediaType> pTypeSignalInt32 = new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED);
		cObjectPtr<IMediaType> pTypeSignalInt = new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED);
		cObjectPtr<IMediaType> pTypeSignalUInt32 = new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT32);

		// Create the output pins
		RETURN_IF_FAILED(this->m_oOFloat32.Create("tfloat32_out", pTypeSignalFloat32, NULL));
		RETURN_IF_FAILED(RegisterPin(&this->m_oOFloat32));

		RETURN_IF_FAILED(this->m_oOInteger32.Create("tuint32_out", pTypeSignalInt32, NULL));
		RETURN_IF_FAILED(RegisterPin(&this->m_oOInteger32));

		RETURN_IF_FAILED(this->m_oOInteger8.Create("int_out", pTypeSignalInt, NULL));
		RETURN_IF_FAILED(RegisterPin(&this->m_oOInteger8));

		RETURN_IF_FAILED(this->m_oOTimestamp_as_UInt32.Create("timestamp_as_uint32_out", pTypeSignalUInt32, NULL));
		RETURN_IF_FAILED(RegisterPin(&this->m_oOTimestamp_as_UInt32));

		// Media type definition
		cObjectPtr<IMediaType> pTypeSignal = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&this->m_pCoderDescSignalInput)); 

		//Create the input pins
		RETURN_IF_FAILED(this->m_oITSignal.Create("tsignal_in", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&this->m_oITSignal));
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageGraphReady)
	{
	}

	RETURN_NOERROR;
}

tResult cTSignal_converter::Start(__exception) {
	RETURN_IF_FAILED(cFilter::Start( __exception_ptr));
	this->m_nLastMSTime = 0;

	RETURN_NOERROR;
}

tResult cTSignal_converter::Stop(__exception) {
	RETURN_IF_FAILED(cFilter::Stop( __exception_ptr));

	RETURN_NOERROR;
}

tResult cTSignal_converter::Shutdown(tInitStage eStage, __exception) {
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

tResult cTSignal_converter::OnPinEvent( IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		tFloat32 signalValue = 0;
		tUInt32 timeStamp = 0;
		this->m_nLastMSTime = pMediaSample->GetTime();

		if (this->m_pCoderDescSignalInput != NULL)
		{
			// read-out the incoming Media Sample
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(this->m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

			//get values from media sample        
			pCoderInput->Get("f32Value", (tVoid*)&signalValue);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			this->m_pCoderDescSignalInput->Unlock(pCoderInput);                 
		}                
		else
		{
			RETURN_ERROR(ERR_FAILED);
		}

		if (pSource == &this->m_oITSignal) {
			/** INTEGER 8 **/
			//create new sample
			cObjectPtr<IMediaSample> pSampleRealInt;
			AllocMediaSample((tVoid**)&pSampleRealInt);
			
			//fill up sample
			int iValue = int(signalValue);
			pSampleRealInt->Update(timeStamp, &iValue, sizeof(int), 0);
			
			//transmit sample
			this->m_oOInteger8.Transmit(pSampleRealInt);

			/** INTEGER 32 **/
			//create new sample
			cObjectPtr<IMediaSample> pSampleInt;
			AllocMediaSample((tVoid**)&pSampleInt);
			
			//fill up sample
			tInt32 i32Value = tInt32(signalValue);
			pSampleInt->Update(timeStamp, &i32Value, sizeof(tInt32), 0);
			
			//transmit sample
			this->m_oOInteger32.Transmit(pSampleInt);

			/** FLOAT 32 **/
			//create new sample
			cObjectPtr<IMediaSample> pSampleFloat;
			AllocMediaSample((tVoid**)&pSampleFloat);
			
			//fill up sample
			pSampleFloat->Update(timeStamp, &signalValue, sizeof(tFloat32), 0);
			
			//transmit sample
			this->m_oOInteger32.Transmit(pSampleFloat);

			/** Timestamp 32 **/
			//create new sample
			cObjectPtr<IMediaSample> pSampleTimestamp;
			AllocMediaSample((tVoid**)&pSampleTimestamp);
			
			//fill up sample
			pSampleTimestamp->Update(timeStamp, &timeStamp, sizeof(tUInt32), 0);
			
			//transmit sample
			this->m_oOTimestamp_as_UInt32.Transmit(pSampleTimestamp);
		}
	}
	RETURN_NOERROR;
}
