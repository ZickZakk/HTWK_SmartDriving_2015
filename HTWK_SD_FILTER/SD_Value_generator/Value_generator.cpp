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
 * @author epetersen
 * @details
 */
#include "stdafx.h"
#include "Value_generator.h"

/// Create filter for random number generator
ADTF_FILTER_PLUGIN("SD Value generator", OID_ADTF_SD_Value_generator, cValue_generator);

#define PROPERTY_VALUE "value"
#define PROPERTY_GENERATERATE "CycleTime"
#define PROPERTY_TRANSMIT "transmit"

cValue_generator::cValue_generator(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{
    SetPropertyFloat(PROPERTY_VALUE, 0);	
    SetPropertyBool(PROPERTY_VALUE NSSUBPROP_ISCHANGEABLE, tTrue); 
    SetPropertyFloat(PROPERTY_GENERATERATE, 1);
	SetPropertyBool(PROPERTY_GENERATERATE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyBool(PROPERTY_TRANSMIT, true);
	SetPropertyBool(PROPERTY_TRANSMIT NSSUBPROP_ISCHANGEABLE, tTrue);

}

cValue_generator::~cValue_generator()
{
}

tResult cValue_generator::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
    RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr));
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
	
	    // get a media type for the output pin
        tChar const * strDescSignalValueOutput = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutput);        
        cObjectPtr<IMediaType> pTypeSignalValueOutput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
        
        // create and register the output pin
      	RETURN_IF_FAILED(m_pin_output_value.Create("value_out", pTypeSignalValueOutput, static_cast<IPinEventSink*> (this)));
	    RETURN_IF_FAILED(RegisterPin(&m_pin_output_value));

        RETURN_IF_FAILED(pTypeSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
		m_nGenerateRate = (tUInt32) (GetPropertyFloat( PROPERTY_GENERATERATE ) * 1000000);
        if(m_nGenerateRate <= 0)
        {
            m_nGenerateRate = 5 * 1000000;
        }
        
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
		
        RETURN_IF_FAILED(SetInterval(m_nGenerateRate));
    }

    RETURN_NOERROR;
}

tResult cValue_generator::Cycle(__exception)
{
	m_nValue = (tFloat32)GetPropertyFloat( PROPERTY_VALUE );
	if(GetPropertyBool(PROPERTY_TRANSMIT)){
		slotValue(m_nValue);
	}
    RETURN_NOERROR;
}

tResult cValue_generator::slotValue(tFloat32 val)
{
	tFloat32 flValue = val;
	tUInt32 timeStamp = (tUInt32) (_clock ? _clock->GetStreamTime()/1000 : adtf_util::cHighResTimer::GetTime()/1000);
							
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescSignalOutput->WriteLock(pMediaSample, &pCoder));
		
    pCoder->Set("f32Value", (tVoid*)&(flValue));	
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp); 
    m_pCoderDescSignalOutput->Unlock(pCoder);
    
    //transmit media sample over output pin
    m_pin_output_value.Transmit(pMediaSample);
    RETURN_NOERROR;
}