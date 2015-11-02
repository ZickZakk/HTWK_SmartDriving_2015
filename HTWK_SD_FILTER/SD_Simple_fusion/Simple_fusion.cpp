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
 * @author epetersen & lkollman
 * @details
 */
#include "stdafx.h"
#include "Simple_fusion.h"
#include <math.h>
#include <vector>

/// Create filter for random number generator
ADTF_FILTER_PLUGIN("SD simple fusion", OID_ADTF_SD_SIMPLE_FUSION, cSimpleFusion);

// GetRandomInt returns a random integer value between min and max. (nMin <= n <= nMax)

cSimpleFusion::cSimpleFusion(const tChar* __info) : adtf::cFilter(__info)
{
	m_nLastMSTime = 0;
	ir_short_iterations = 1;	
	ir_long_iterations = 1;
	sigma_iterations = 0;
	median = 0;
	initProperties();


}

cSimpleFusion::~cSimpleFusion()
{
}

tVoid cSimpleFusion::initProperties()
{
	// 
	stateOfLongeRangeInputPin = 1;	
	SetPropertyInt(PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE, stateOfLongeRangeInputPin);
	SetPropertyStr(PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE NSSUBPROP_VALUELISTNOEDIT, "1@1|2@0");
	SetPropertyStr(PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE NSSUBPROP_DESCRIPTION, "Choose 1 for active/ 0 for inactive");
	SetPropertyBool(PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	// 
	stateOfShortRangeInputPin1 = 1;	
	SetPropertyInt(PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE, stateOfShortRangeInputPin1);
	SetPropertyStr(PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE NSSUBPROP_VALUELISTNOEDIT, "1@1|2@0");
	SetPropertyStr(PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE NSSUBPROP_DESCRIPTION, "Choose 1 for active/ 0 for inactive");
	SetPropertyBool(PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	// 
	stateOfShortRangeInputPin2 = 1;	
	SetPropertyInt(PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE, stateOfShortRangeInputPin2);
	SetPropertyStr(PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE NSSUBPROP_VALUELISTNOEDIT, "1@1|2@0");
	SetPropertyStr(PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE NSSUBPROP_DESCRIPTION, "Choose 1 for active/ 0 for inactive");
	SetPropertyBool(PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	// 
	stateOfUSLeftInputPin = 1;	
	SetPropertyInt(PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE, stateOfUSLeftInputPin);
	SetPropertyStr(PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE NSSUBPROP_VALUELISTNOEDIT, "1@1|2@0");
	SetPropertyStr(PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE NSSUBPROP_DESCRIPTION, "Choose 1 for active/ 0 for inactive");
	SetPropertyBool(PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE NSSUBPROP_ISCHANGEABLE, tTrue);
	// 
	stateOfUSRightInputPin = 1;	
	SetPropertyInt(PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE, stateOfUSRightInputPin);
	SetPropertyStr(PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE NSSUBPROP_VALUELISTNOEDIT, "1@1|2@0");
	SetPropertyStr(PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE NSSUBPROP_DESCRIPTION, "Choose 1 for active/ 0 for inactive");
	SetPropertyBool(PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	// Schwellwerte IR-Longrange
	thresholdIRLongRange = 20;
	SetPropertyInt(PROP_NAME_IR_LONG_RANGE_THRESHOLD, thresholdIRLongRange);
    SetPropertyInt(PROP_NAME_IR_LONG_RANGE_THRESHOLD NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_IR_LONG_RANGE_THRESHOLD NSSUBPROP_MAXIMUM, 255);
    SetPropertyBool(PROP_NAME_IR_LONG_RANGE_THRESHOLD NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_IR_LONG_RANGE_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);

	// Schwellwerte IR-Shortrange
	thresholdIRShortRange = 8;
	SetPropertyInt(PROP_NAME_IR_SHORT_RANGE_THRESHOLD, thresholdIRShortRange);
    SetPropertyInt(PROP_NAME_IR_SHORT_RANGE_THRESHOLD NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_IR_SHORT_RANGE_THRESHOLD NSSUBPROP_MAXIMUM, 255);
    SetPropertyBool(PROP_NAME_IR_SHORT_RANGE_THRESHOLD NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_IR_SHORT_RANGE_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);

	// Schwellwerte Ultaschal sensor(US)
	thresholdUS = 20;
	SetPropertyInt(PROP_NAME_US_THRESHOLD, thresholdUS);
    SetPropertyInt(PROP_NAME_US_THRESHOLD NSSUBPROP_MINIMUM, 0);
    SetPropertyInt(PROP_NAME_US_THRESHOLD NSSUBPROP_MAXIMUM, 255);
    SetPropertyBool(PROP_NAME_US_THRESHOLD NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_US_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);

	// Median for using by the median-filter
	lengthOfMedianVector = 7;
	SetPropertyInt(PROP_NAME_MEDIAN, lengthOfMedianVector);
    SetPropertyInt(PROP_NAME_MEDIAN NSSUBPROP_MINIMUM, 1);
    SetPropertyInt(PROP_NAME_MEDIAN NSSUBPROP_MAXIMUM, 20);
    SetPropertyBool(PROP_NAME_MEDIAN NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_MEDIAN NSSUBPROP_ISCHANGEABLE, tTrue);
}

tResult cSimpleFusion::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
		cObjectPtr<IMediaDescriptionManager> pDescManager;

		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,NULL));
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);   

		pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput)); 

		createOutputPin();
		createInputPinIRLongeRange("ir_long_in");
		createInputPinIRShortRange1("ir_short_in1");
		createInputPinIRShortRange2("ir_short_in2");
		createInputPinUSLeft("us_left_in");
		createInputPinUSRight("us_right_in");
    }
    else if (eStage == StageNormal)
    {
		// In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
		
		stateOfLongeRangeInputPin = GetPropertyInt(PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE);
		stateOfShortRangeInputPin1 = GetPropertyInt(PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE);
		stateOfShortRangeInputPin2 = GetPropertyInt(PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE);
		stateOfUSRightInputPin = GetPropertyInt(PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE);
		stateOfUSLeftInputPin = GetPropertyInt(PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE);
		lengthOfMedianVector = GetPropertyInt(PROP_NAME_MEDIAN);
		thresholdIRLongRange = GetPropertyInt(PROP_NAME_IR_LONG_RANGE_THRESHOLD);
		thresholdIRShortRange = GetPropertyInt(PROP_NAME_IR_SHORT_RANGE_THRESHOLD);
		thresholdUS = GetPropertyInt(PROP_NAME_US_THRESHOLD);
		calculatedMedian();
    }

    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

tResult cSimpleFusion::calculatedMedian()
{
	// median ist ungerade
	if(lengthOfMedianVector%2 == 1)
	{
		median = (lengthOfMedianVector+1)/2;
	}
	else
	{
		median = lengthOfMedianVector/2;
	}
	RETURN_NOERROR;
}


tResult cSimpleFusion::createOutputPin()
{
    RETURN_IF_FAILED(m_ir_fusion_out.Create("ir_fusion_out", pTypeSignalValue, NULL));
    RETURN_IF_FAILED(RegisterPin(&m_ir_fusion_out));

	RETURN_NOERROR;
}

tResult cSimpleFusion::createInputPinIRLongeRange(const tChar *pinNameShort)
{
	RETURN_IF_FAILED(m_ir_long_in.Create(pinNameShort, pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_ir_long_in));

	RETURN_NOERROR;
}

tResult cSimpleFusion::createInputPinIRShortRange1(const tChar *pinName)
{
	RETURN_IF_FAILED(m_ir_short_in1.Create(pinName, pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_ir_short_in1));

	RETURN_NOERROR;
}

tResult cSimpleFusion::createInputPinIRShortRange2(const tChar *pinName)
{
	RETURN_IF_FAILED(m_ir_short_in2.Create(pinName, pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_ir_short_in2));

	RETURN_NOERROR;
}

tResult cSimpleFusion::createInputPinUSLeft(const tChar *pinName)
{
	RETURN_IF_FAILED(m_us_left_in.Create(pinName, pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_us_left_in));

	RETURN_NOERROR;
}

tResult cSimpleFusion::createInputPinUSRight(const tChar *pinName)
{
	RETURN_IF_FAILED(m_us_right_in.Create(pinName, pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_us_right_in));

	RETURN_NOERROR;
}

tResult cSimpleFusion::Start(__exception) 
{
	RETURN_IF_FAILED(cFilter::Start( __exception_ptr));
	m_nLastMSTime = 0;

	RETURN_NOERROR;
}

tResult cSimpleFusion::Stop(__exception) 
{
	RETURN_IF_FAILED(cFilter::Stop( __exception_ptr));

	RETURN_NOERROR;
}

tResult cSimpleFusion::Shutdown(tInitStage eStage, __exception) 
{
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

tFloat32 last_ir_long_input = 0;
tFloat32 last_ir_short_input = 0;
tFloat32 last_us_left_input = 0;
tFloat32 last_us_right_input = 0;
tBool   last_ir_long_oor = false;
tBool   last_ir_short_oor = false;
tBool   last_us_left_oor = false;
tBool   last_us_right_oor = false;
tFloat32 current_value = 0;
tFloat32 genauigkeitsanpassung;
tFloat32 sigma_Value = 0;
tFloat32 ir_long_tempValue = 0;
tFloat32 ir_short_tempValue = 0;
vector<tFloat32> ir_long_vector; 
vector<tFloat32> ir_short_vector; 
vector<tFloat32> us_vector;
vector<tFloat32> sigma_vector;


tResult cSimpleFusion::sendNewValue(tFloat32 flValue) 
{
   	//tFloat32 flValue= value;
	tUInt32 timeStamp = 0;
							
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalInput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoder;
    m_pCoderDescSignalInput->WriteLock(pMediaSample, &pCoder);	
		
	pCoder->Set("f32Value", (tVoid*)&(flValue));	
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    m_pCoderDescSignalInput->Unlock(pCoder);
    
    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_ir_fusion_out.Transmit(pMediaSample);

	// bitte noch drin lassen->brauch ich für die XML-Kalibrierung des Calibration Filter Extended
	//LOG_INFO(cString::Format("%f",flValue));

	RETURN_NOERROR;
}

tResult cSimpleFusion::OnPinEvent( IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) 
{
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		tFloat32 signalValue = 0;
        tUInt32 timeStamp = 0;
		m_nLastMSTime = pMediaSample->GetTime();

		if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL)
        {
            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));
            
            //get values from media sample and sets it on signalValue / timeStamp       
            pCoderInput->Get("f32Value", (tVoid*)&signalValue);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignalInput->Unlock(pCoderInput);                 
        }                
        else
			RETURN_ERROR(ERR_FAILED);
	

		// do stuff for the IR-longrange sensor
		if (pSource == &m_ir_long_in)
		{
			//Do not iterate if the input pin is deactivated but connected
			if(stateOfLongeRangeInputPin != 1) 
			{
				RETURN_NOERROR;
			}

			//Schwellwerte für ir long range
			if (signalValue < thresholdIRLongRange) 
			{
				last_ir_long_input = signalValue;
				sigma_vector.push_back((tFloat32)20.0); // kürzesten Wert ausgeben zw. 20
			}
			// Ausreißer
			else if(abs(int(last_ir_long_input - tUInt32(signalValue))) > 30)
			{
				//Ausreißer  Statistisch auswerten ob es wirklich ein Ausreißer ist
				last_ir_long_input = signalValue;
				last_ir_long_oor = true;
				//sigma_vector.push_back();
				RETURN_NOERROR;
			}
			else
			{
				last_ir_long_input = signalValue;
				// add every iteration the signalValue into the vector
				sigma_vector.push_back(signalValue);
			}
		} 

		//Do not iterate if the input pin is deactivated but connected
		else if(pSource == &m_ir_short_in1 && stateOfShortRangeInputPin1 != 1)
		{
			RETURN_NOERROR;
		}
		//Do not iterate if the input pin is deactivated but connected
		else if(pSource == &m_ir_short_in2 && stateOfShortRangeInputPin2 != 1)
		{
			RETURN_NOERROR;
		}
		// do stuff for the IR-shortrange sensor
		else if (pSource == &m_ir_short_in1 || pSource == &m_ir_short_in2) 
		{
			// Schwellwert von short range einhalten
			if (signalValue < thresholdIRShortRange)
			{
				last_ir_short_input = signalValue;
				last_ir_short_oor = true;

				sigma_vector.push_back((tFloat32)8.0);// kürzesten Wert ausgeben 8
			}
			// Ausreißer
			else if(abs(int(last_ir_short_input - tUInt32(signalValue))) > 30)
			{
				//Ausreißer...todo: Statistisch auswerten ob es wirklich ein Ausreißer ist
				last_ir_short_input = signalValue;
				last_ir_short_oor = true;

				RETURN_NOERROR;
			}
			else
			{
				last_ir_short_input = signalValue;
				sigma_vector.push_back(signalValue);
			}

		}

		//Do not iterate if the input pin is deactivated but connected
		else if(pSource == &m_us_left_in && stateOfUSLeftInputPin != 1)
		{
			RETURN_NOERROR;
		}
		//Do not iterate if the input pin is deactivated but connected
		else if(pSource == &m_us_right_in && stateOfUSRightInputPin != 1)
		{
			RETURN_NOERROR;
		}
		// do stuff for the Ultrasonic sensors(left||right)
		else if(pSource == &m_us_left_in || pSource == &m_us_right_in)
		{
			//Setion für Schwellwerte für US
			if(signalValue < thresholdUS && pSource == &m_us_left_in )
			{
				last_us_left_input = signalValue;
				sigma_vector.push_back((tFloat32)thresholdUS); // kürzesten Wert ausgeben
				last_us_left_oor = true;
			}
			else if(signalValue < thresholdUS && pSource == &m_us_right_in)
			{
				last_us_right_input = signalValue;
				sigma_vector.push_back((tFloat32)thresholdUS); // kürzesten Wert ausgeben
				last_us_right_oor = true;
			}
			else
			{
				sigma_vector.push_back(signalValue); 
			}
		}

		//decrement everytime a signalValue comes in
		sigma_iterations++;

		//send the median value from the sigma_vector to the output
		if(sigma_iterations == lengthOfMedianVector && sigma_vector.capacity() != 0)
		{
			sort(sigma_vector.begin(),sigma_vector.end());

			sigma_Value = sigma_vector.at(median-1);
			//LOG_INFO(cString::Format("%f",sigma_Value));
			sendNewValue(sigma_Value);

			sigma_vector.clear();
			sigma_iterations = 0;
		}
	} 
	else 
	{
		sendNewValue(0);
	}
	RETURN_NOERROR;
}

// todo
// Executed when a property is changed
tResult cSimpleFusion::PropertyChanged(const char *propertyName)
{
	if(cString::Compare(propertyName, PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE))
	{
		stateOfLongeRangeInputPin = GetPropertyInt(PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE);

		#ifdef _DEBUG
		LOG_INFO("changed state of ir longe range input pin");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE))
	{
		stateOfShortRangeInputPin1 = GetPropertyInt(PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE);

		#ifdef _DEBUG
		LOG_INFO("changed state of ir short range input pin 1");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE))
	{
		stateOfShortRangeInputPin2 = GetPropertyInt(PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE);

		#ifdef _DEBUG
		LOG_INFO("changed state of ir short range input pin 2");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE))
	{
		stateOfUSLeftInputPin = GetPropertyInt(PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE);
		#ifdef _DEBUG
		LOG_INFO("changed state of us left input pin");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE))
	{
		stateOfUSRightInputPin = GetPropertyInt(PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE);
		#ifdef _DEBUG
		LOG_INFO("changed state of us right input pin");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_IR_LONG_RANGE_THRESHOLD))
	{		
		thresholdIRLongRange = GetPropertyInt(PROP_NAME_IR_LONG_RANGE_THRESHOLD);

		#ifdef _DEBUG
		LOG_INFO("changed longe range threshold");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_IR_SHORT_RANGE_THRESHOLD))
	{
		thresholdIRShortRange = GetPropertyInt(PROP_NAME_IR_SHORT_RANGE_THRESHOLD);

		#ifdef _DEBUG
		LOG_INFO("changed short range threshold");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_US_THRESHOLD))
	{
		thresholdUS = GetPropertyInt(PROP_NAME_US_THRESHOLD);

		#ifdef _DEBUG
		LOG_INFO("changed US threshold");
		#endif
	}
	else if(cString::Compare(propertyName, PROP_NAME_MEDIAN))
	{
		lengthOfMedianVector = GetPropertyInt(PROP_NAME_MEDIAN);

		#ifdef _DEBUG
		LOG_INFO("change median");
		#endif
	}

	RETURN_NOERROR;
}