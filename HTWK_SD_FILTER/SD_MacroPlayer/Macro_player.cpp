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
#include "Macro_player.h"

#include <iostream>
#include <fstream>

/// Create filter
ADTF_FILTER_PLUGIN(FILTER_NAME, OID_ADTF_SD_Macro_Player, cMacro_player);


cMacro_player::cMacro_player(const tChar* __info) : adtf::cFilter(__info)
{
	this->isDebugActive = false;
	initVariabels();
	initProperties();
}

cMacro_player::~cMacro_player()
{
}

tVoid cMacro_player::initVariabels()
{
	this->m_nLastMSTime = 0;
	this->isDriveActive = false;
	this->distance = 0;
	this->lastDist = 0;
	this->step = 0;
	this->measuredParkSpace = 0;
	this->isFirstCarComplete = false;
	this->isFirstCarFound = false;
	this->isMaybeGapEnd = false;
	this->foundGap = false;
	this->irFront = 8;
	this->initPullOut = false;
	this->decideParkingPosition = false;
	this->initStep = false;
}

tVoid cMacro_player::initProperties()
{
	SetPropertyInt(PROP_NAME_MEASURE_DISTANCE_FOR_CROSS_PARKING, 25);
	SetPropertyBool(PROP_NAME_MEASURE_DISTANCE_FOR_CROSS_PARKING NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_MEASURE_DISTANCE_FOR_PARALLEL_PARKING, 31);
	SetPropertyBool(PROP_NAME_MEASURE_DISTANCE_FOR_PARALLEL_PARKING NSSUBPROP_ISCHANGEABLE, tTrue); 

	// Schwellwerte IR-Shortrange
	thresholdIRShortRange = 8;
	SetPropertyInt(PROP_NAME_IR_SHORT_RANGE_THRESHOLD, thresholdIRShortRange);
	SetPropertyBool(PROP_NAME_IR_SHORT_RANGE_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);

	confidenceRange = 35;
	SetPropertyInt(PROP_NAME_CONFIDENCERANGE, confidenceRange);
	SetPropertyBool(PROP_NAME_CONFIDENCERANGE NSSUBPROP_ISCHANGEABLE, tTrue);

	//Properties for cross-parking
	SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCE, 0);
	SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCESTEER, 0);
	SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCEDRIVE, 29);
	SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_CPDRIVELEFTDISTANCE, 40);
	SetPropertyBool(PROP_NAME_CPDRIVELEFTDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_CPDRIVELEFTDISTANCESTEER, -29);
	SetPropertyBool(PROP_NAME_CPDRIVELEFTDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_CPDRIVELEFTDISTANCEDRIVE, 29);
	SetPropertyBool(PROP_NAME_CPDRIVELEFTDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_CPDRIVERIGHTDISTANCE, 40);
	SetPropertyBool(PROP_NAME_CPDRIVERIGHTDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_CPDRIVERIGHTDISTANCESTEER, 29);
	SetPropertyBool(PROP_NAME_CPDRIVERIGHTDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_CPDRIVERIGHTDISTANCEDRIVE, -22);
	SetPropertyBool(PROP_NAME_CPDRIVERIGHTDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCE, 24);
	SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCESTEER, 0);
	SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCEDRIVE, -22);
	SetPropertyBool(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	//Properties for parallel-parking
	SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP, 9500000);
	SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPSTEER, 0);
	SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPSTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPDRIVE, 27);
	SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCE, 3200000);
	SetPropertyBool(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCESTEER, -30);
	SetPropertyBool(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCEDRIVE, -21);
	SetPropertyBool(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCE, 3600000);
	SetPropertyBool(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCESTEER, 30);
	SetPropertyBool(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCEDRIVE, -21);
	SetPropertyBool(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCE, 8);
	SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCESTEER, 0);
	SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCEDRIVE, 23);
	SetPropertyBool(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	//Properties for pull-out-left cross-parking
	SetPropertyInt(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCE, 22);
	SetPropertyBool(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCESTEER, 0);
	SetPropertyBool(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE, 25);
	SetPropertyBool(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCE, 94);
	SetPropertyBool(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCESTEER, -29);
	SetPropertyBool(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCEDRIVE, 27);
	SetPropertyBool(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	//Properties for pull-out-right cross-parking
	SetPropertyInt(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCE, 8);
	SetPropertyBool(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCESTEER, 0);
	SetPropertyBool(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE, 25);
	SetPropertyBool(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCE, 61);
	SetPropertyBool(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCESTEER, 30);
	SetPropertyBool(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCEDRIVE, 28);
	SetPropertyBool(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	//Properties for pull-out-right parallel-parking
	SetPropertyInt(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCE, 11);
	SetPropertyBool(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCESTEER, 0);
	SetPropertyBool(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue); 
	SetPropertyInt(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCEDRIVE, -22);
	SetPropertyBool(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyInt(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCE, 41);
	SetPropertyBool(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCESTEER, -30);
	SetPropertyBool(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCEDRIVE, 27);
	SetPropertyBool(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCE, 33);
	SetPropertyBool(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCESTEER, 30);
	SetPropertyBool(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCESTEER NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCEDRIVE, 27);
	SetPropertyBool(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCEDRIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	this->isDebugActive = false;
	SetPropertyBool(PROP_NAME_DEBUG, this->isDebugActive);
    SetPropertyBool(PROP_NAME_DEBUG NSSUBPROP_REQUIRED, tTrue);
	SetPropertyBool(PROP_NAME_DEBUG NSSUBPROP_ISCHANGEABLE, tTrue);
}

tResult cMacro_player::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		// Input Pins
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

		tChar const *strDescSignalValue = this->descriptionManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);   
		cObjectPtr<IMediaType> pTypeSignal = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
		RETURN_IF_FAILED(pTypeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput));

		// create input pins
		RETURN_IF_FAILED(createInputPin(this->m_pin_input_wheelRotation, "WheelRotation_in", pTypeSignal));
		RETURN_IF_FAILED(createInputPin(this->m_ir_RR, "IR_RR", pTypeSignal));
		RETURN_IF_FAILED(createInputPin(this->m_ir_Front_Center, "IR_FRONT_CENTER", pTypeSignal));

		// Output Pins
		tChar const * strDescBoolSignalValue = this->descriptionManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBoolSignalValue);        
        cObjectPtr<IMediaType> pTypeBoolSignalValueOutput = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
		RETURN_IF_FAILED(pTypeBoolSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignalOutput)); 

		tChar const *strDescSignalValueOutput = this->descriptionManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValueOutput);        

		cObjectPtr<IMediaType> pTypeSignalOutput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
		RETURN_IF_FAILED(pTypeSignalOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));

		// create and register the output pin
		RETURN_IF_FAILED(createOutputPin(this->m_pin_output_acceleration, "Acceleration_out", pTypeSignalOutput));
		RETURN_IF_FAILED(createOutputPin(this->m_pin_output_steeringAngle, "SteeringAngle_out", pTypeSignalOutput));
		
		// @author dhecht
		cObjectPtr<IMediaType> maneuverMediaType;
		RETURN_IF_FAILED(initMediaType("tSteeringAngleData", maneuverMediaType, this->coderDescriptionManeuver));

		RETURN_IF_FAILED(createInputPin(this->decisionPin, "decision", maneuverMediaType));
		RETURN_IF_FAILED(createOutputPin(this->maneuverFinishedPin, "maneuverFinished", maneuverMediaType));
		RETURN_IF_FAILED(createOutputPin(this->lightPin, "light", maneuverMediaType));

	}
	else if (eStage == StageGraphReady)
	{
		cObjectPtr<IMediaSerializer> serializer;
		RETURN_IF_FAILED(this->coderDescriptionManeuver->GetMediaSampleSerializer(&serializer));
		this->ddlSizeManeuver = serializer->GetDeserializedSize();

		RETURN_IF_FAILED(this->m_pCoderDescSignalOutput->GetMediaSampleSerializer(&serializer));
		this->ddlSizeTSignal = serializer->GetDeserializedSize();
	}

	RETURN_NOERROR;
}

tResult cMacro_player::createInputPin(cInputPin &pin, const char *pinName, cObjectPtr<IMediaType> typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}
tResult cMacro_player::createOutputPin(cOutputPin &pin, const char *pinName, cObjectPtr<IMediaType> typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult cMacro_player::Start(__exception) {
	RETURN_IF_FAILED(cFilter::Start( __exception_ptr));
	m_nLastMSTime = 0;

	RETURN_NOERROR;
}

tResult cMacro_player::Stop(__exception) {
	RETURN_IF_FAILED(cFilter::Stop( __exception_ptr));

	RETURN_NOERROR;
}

tResult cMacro_player::Shutdown(tInitStage eStage, __exception) {
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

tResult cMacro_player::sendNewValue(cOutputPin * outpin, const tFloat32 &value, const tTimeStamp &timeStamp) 
{
	tFloat32 flValue= value;

	//create new media sample
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(this->ddlSizeTSignal));

	//write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> pCoder;
	RETURN_IF_FAILED(m_pCoderDescSignalInput->WriteLock(pMediaSample, &pCoder));	

	pCoder->Set("f32Value", (tVoid*)&flValue);
	pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	RETURN_IF_FAILED(m_pCoderDescSignalInput->Unlock(pCoder));

	//transmit media sample over output pin
	RETURN_IF_FAILED(pMediaSample->SetTime(timeStamp));
	RETURN_IF_FAILED(outpin->Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult cMacro_player::sendNewValueBool(cOutputPin *outpin, const tBool &value, const tTimeStamp &timeStamp)
{
	tBool bValue = value;
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(this->ddlSizeTSignal));

	//write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> pCoder;
	RETURN_IF_FAILED(m_pCoderDescSignalInput->WriteLock(pMediaSample, &pCoder));	

	pCoder->Set("tBool", (tVoid*)&bValue);
	pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	RETURN_IF_FAILED(m_pCoderDescSignalInput->Unlock(pCoder));

	//transmit media sample over output pin
	RETURN_IF_FAILED(pMediaSample->SetTime(timeStamp));
	RETURN_IF_FAILED(outpin->Transmit(pMediaSample));

	if (this->isDebugActive)
	{
		cString::Format("Sending boolfor light stuff %d", bValue);
	}

	RETURN_NOERROR;
}

tResult cMacro_player::OnPinEvent( IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) 
{
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// Wenn ein Kommando einparken oder ausparken kommt
		if (pSource == &this->decisionPin)
		{
			if (this->isDebugActive)
			{
				LOG_INFO("Getting an decision");
			}

			// lese den Zustand des aktuellen Befehls aus
			static tUInt16 decisionValue;
			static tTimeStamp timeStamp;

			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getDecisionValue(pMediaSample, decisionValue, timeStamp), "Cant get decision!");

			getDecision(decisionValue, this->currentDecision, this->prevDecision);

			if (this->isDebugActive)
			{
				LOG_INFO(cString::Format("Got decision %d", this->currentDecision));
			}
		}

		tFloat32 signalValue = 0;
		tTimeStamp timeStamp = 0;
		m_nLastMSTime = _clock->GetStreamTime();

		if (this->currentDecision == DECISION_STOP)
		{
			if (this->isDebugActive)
			{
				LOG_INFO("Get DECISION_STOP!");
			}

			initVariabels();
		}

		if (pSource == &this->m_ir_RR || pSource == &this->m_pin_input_wheelRotation || pSource == &this->m_ir_Front_Center)
		{
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getValue(pMediaSample, signalValue, timeStamp), "cant get value!");
		}
		else 
		{
			RETURN_ERROR(ERR_FAILED);
		}

		if (pSource == &m_pin_input_wheelRotation)
		{
			tInt rotation = tInt(signalValue);
			this->distance = getDistanceFromWheelRotation(rotation);

			if((this->currentDecision == DECISION_CROSS_PARKING || this->currentDecision == DECISION_PARALLEL_PARKING) && !this->isDriveActive)
			{
				this->isDriveActive = true;
				steer(pMediaSample, 0);
				RETURN_IF_FAILED_AND_LOG_ERROR_STR(drive(pMediaSample), "Cant start the car!");

				drive(pMediaSample, 27);
			}
		}

		if(pSource == &m_ir_Front_Center)
		{
			this->irFront = static_cast<tUInt>(signalValue);
		}

		//###############################################################################################
		// Start Parking(cross/parallel) section 
		//###############################################################################################

		//Macro
		if(this->currentDecision == DECISION_CROSS_PARKING)
		{
			if(!this->initStep)
			{
				this->step = cpLeftMove;
				this->initStep = true;
			}
			
			crossParkingScript(pMediaSample);
			
		}//end if( this->currentDecision == DECISION_CROSS_PARKING)
		
		if(this->currentDecision == DECISION_PARALLEL_PARKING)
		{	
			if(!this->initStep)
			{
				this->step = stopDriveBackToGap;
				this->initStep = true;
			}
			parallelParkingScript(pMediaSample);
		}//end if(this->currentDecision == DECISION_PARALLEL_PARKING)

		//###############################################################################################
		// End Parking(cross/parallel) section 
		//###############################################################################################


		//###############################################################################################
		// Start Pull-out section 
		//###############################################################################################

		/*if((!this->decideParkingPosition && this->currentDecision == DECISION_PULL_OUT_RIGHT && this->prevDecision != DECISION_CROSS_PARKING)
			|| (!this->decideParkingPosition && this->currentDecision == DECISION_PULL_OUT_RIGHT && this->prevDecision != DECISION_PARALLEL_PARKING))*/

		if((!this->decideParkingPosition && this->currentDecision == DECISION_PULL_OUT_RIGHT) || (!this->decideParkingPosition && this->currentDecision == DECISION_PULL_OUT_LEFT))
		{
			// this->prevDecision was not DECISION_CROSS_PARKING or was not DECISION_PARALLEL_PARKING
			// go here for one time do decide how we park
			isFrontFree();
			this->decideParkingPosition = true;
		}

		if((this->currentDecision == DECISION_PULL_OUT_LEFT && this->prevDecision == DECISION_CROSS_PARKING))
		{
			pullOutLeftCrossParkingScript(pMediaSample);
		}//end if(this->currentDecision == DECISION_PULL_OUT_LEFT && this->prevDecision == DECISION_CROSS_PARKING)
		
		if((this->currentDecision == DECISION_PULL_OUT_RIGHT && this->prevDecision == DECISION_CROSS_PARKING))
		{
			pullOutRightCrossParkingScript(pMediaSample);
		}//end if(this->currentDecision == DECISION_PULL_OUT_RIGHT && this->prevDecision == DECISION_CROSS_PARKING)
		
		if((this->currentDecision == DECISION_PULL_OUT_RIGHT && this->prevDecision == DECISION_PARALLEL_PARKING))
		{
			pullOutRightParallelParkingScript(pMediaSample);
		}//end if(this->currentDecision == DECISION_PULL_OUT_LEFT && this->prevDecision == DECISION_PARALLEL_PARKING)

		//###############################################################################################
		// End Pull-out section 
		//###############################################################################################

	} //end if(nEventCode == IPinEventSink::PE_MediaSampleReceived)

	RETURN_NOERROR;
}

tResult cMacro_player::getValue(IMediaSample *mediaSample, tFloat32 &value, tTimeStamp &timeStamp)
{
	// read-out the incoming Media Sample
	cObjectPtr<IMediaCoder> pCoderInput;
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(m_pCoderDescSignalInput->Lock(mediaSample, &pCoderInput), "cant lock");

	//get values from media sample        
	pCoderInput->Get("f32Value", (tVoid*)&value);
	pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(m_pCoderDescSignalInput->Unlock(pCoderInput), "cant unlock");

	RETURN_NOERROR;
}

tResult cMacro_player::stopDrive(IMediaSample *pMediaSample)
{
	tInt fireStopValue = 2;

	transmitLight(lightPin,LIGHT_BREAK);

	for(int i = 0;i<fireStopValue;i++)
	{
		RETURN_IF_FAILED(sendNewValue(&m_pin_output_acceleration, 0.0f, pMediaSample->GetTime()));
	}
	
	RETURN_IF_FAILED(sendNewValue(&m_pin_output_steeringAngle, 0.0f, pMediaSample->GetTime()));

	RETURN_NOERROR;
}

tResult cMacro_player::runHazzardLights(tInt enduration)
{
	static tTimeStamp now;
	static tTimeStamp fin;

	transmitLight(lightPin,LIGHT_HAZARD);

	now = cHighResTimer::GetTime();
	fin = now + enduration;

	//while(cHighResTimer::GetTime() < fin)
	//{
	//	//just do nothing till fin-time is reached
	//}
	for(int i = 0;cHighResTimer::GetTime() < fin;i++)
	{
		//just do nothing till fin-time is reached
	}

	transmitLight(lightPin,LIGHT_HAZARD_DISABLED);

	RETURN_NOERROR;
}

tResult cMacro_player::crossParkingScript(IMediaSample *pMediaSample)
{
	//cpDriveStraight->cpDriveStraightFin-> cpLeftMove->cpLeftMoveFin-> cpRightMove->cpRightMoveFin-> cpDriveStraight->cpDriveStraightFin-> sendManeuver()

	//tInt cpDriveStraightDistance = GetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCE); //1.
	//tInt cpDriveStraightDistanceSteer = GetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCESTEER); //1.
	//tInt cpDriveStraightDistanceDrive = GetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTDISTANCEDRIVE); //1.

	tInt cpDriveLeftDistance = GetPropertyInt(PROP_NAME_CPDRIVELEFTDISTANCE);	//2.
	tInt cpDriveLeftDistanceSteer = GetPropertyInt(PROP_NAME_CPDRIVELEFTDISTANCESTEER);	//2.
	tInt cpDriveLeftDistanceDrive = GetPropertyInt(PROP_NAME_CPDRIVELEFTDISTANCEDRIVE);	//2.

	tInt cpDriveRightDistance = GetPropertyInt(PROP_NAME_CPDRIVERIGHTDISTANCE);	//3.
	tInt cpDriveRightDistanceSteer = GetPropertyInt(PROP_NAME_CPDRIVERIGHTDISTANCESTEER);	//3.
	tInt cpDriveRightDistanceDrive = GetPropertyInt(PROP_NAME_CPDRIVERIGHTDISTANCEDRIVE);	//3.

	tInt cpDriveStraightInGapDistance = GetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCE);	//4.
	tInt cpDriveStraightInGapDistanceSteer = GetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCESTEER);	//4.
	tInt cpDriveStraightInGapDistanceDrive = GetPropertyInt(PROP_NAME_CPDRIVESTRAIGHTINGAPDISTANCEDRIVE);	//4.

	static tTimeStamp now;
	static tTimeStamp fin;

	if (this->isDebugActive)
	{
		LOG_INFO("Cross parking is active");
	}

	/*if (this->step == cpDriveStraight)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("dist  %d ", this->distance));
		}
		this->step = cpDriveStraightFin;

		if (this->isDebugActive)
		{
			LOG_INFO("STEP 1->2");
			LOG_INFO(" ");
		}
		this->lastDist = this->distance;

		steer(pMediaSample,cpDriveStraightDistanceSteer);
		drive(pMediaSample, cpDriveStraightDistanceDrive);
	}
	if (this->step == cpDriveStraightFin)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("cpDriveStraightFin: dist:  %d, lastDist+x: %d, last: %d", this->distance , (this->lastDist + cpDriveStraightDistance), this->lastDist));
		}

		if(this->distance >= (this->lastDist + cpDriveStraightDistance))
		{
			if (this->isDebugActive)
			{
				LOG_INFO("Finished straight driving...");
			}
			this->step = cpLeftMove;

			if (this->isDebugActive)
			{
				LOG_INFO("STEP 2->3");
			}
		}
	}*/
	if(this->step == cpLeftMove)
	{
		if (this->isDebugActive)
		{
			LOG_INFO("Start left move");
		}
		this->step = cpLeftMoveFin;
		if (this->isDebugActive)
		{
			LOG_INFO("STEP 3->4");
		}
		//transmitLight(lightPin,LIGHT_TURN_LEFT);
		steer(pMediaSample, cpDriveLeftDistanceSteer);
		drive(pMediaSample, cpDriveLeftDistanceDrive);

		this->lastDist = this->distance;
	}
	if(this->step == cpLeftMoveFin)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("cpLeftMoveFin: dist:  %d, lastDist+x: %d, last: %d", this->distance , (this->lastDist + cpDriveLeftDistance), this->lastDist));
		}

		if(this->distance >= (this->lastDist + cpDriveLeftDistance))
		{
			//transmitLight(lightPin,LIGHT_TURN_DISABLED);

			this->step = cpRightMove;
			if (this->isDebugActive)
			{
				LOG_INFO("Finished left move...");
				LOG_INFO("STEP 4->5");
			}

			this->lastDist = this->distance;
		}
	}

	if(this->step == cpRightMove)
	{	
		this->step = cpRightMoveFin;
		if (this->isDebugActive)
		{
			LOG_INFO("Start drive right Back...");
			LOG_INFO("STEP 5->6");
		}
		//transmitLight(lightPin,LIGHT_TURN_RIGHT);

		steer(pMediaSample, cpDriveRightDistanceSteer);
		drive(pMediaSample, cpDriveRightDistanceDrive);

		this->lastDist = this->distance;
	}
	if (this->step == cpRightMoveFin)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("cpRightMoveFin: dist:  %d, lastDist+x: %d, last: %d", this->distance , (this->lastDist + cpDriveRightDistance), this->lastDist));
		}
		if (this->distance >= this->lastDist + cpDriveRightDistance)
		{
			//transmitLight(lightPin,LIGHT_TURN_DISABLED);
			this->step = cpDriveStraightLast;
			if (this->isDebugActive)
			{
				LOG_INFO("Finished drive right Back");
				LOG_INFO("STEP 6->7");
			}
			this->lastDist = this->distance;
		}
	}
	if(this->step == cpDriveStraightLast)
	{	
		this->step = cpDriveStraightLastFin;
		if (this->isDebugActive)
		{
			LOG_INFO("Start drive straight back into gap...");
			LOG_INFO("STEP 7->8");
		}
		steer(pMediaSample, cpDriveStraightInGapDistanceSteer);
		drive(pMediaSample, cpDriveStraightInGapDistanceDrive);

		this->lastDist = this->distance;
	}
	if (this->step == cpDriveStraightLastFin)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("cpDriveStraightFin: dist:  %d, lastDist+x: %d, last: %d", this->distance , (this->lastDist + cpDriveStraightInGapDistance), this->lastDist));
		}
		if (this->distance >= this->lastDist + cpDriveStraightInGapDistance)
		{
			stopDrive(pMediaSample);
			this->step = doHaazzardLight;
			
			transmitLight(lightPin,LIGHT_HAZARD);
			if (this->isDebugActive)
			{
				LOG_INFO("STOP, finished cross parking!!");
			}
			now = cHighResTimer::GetTime();
			fin = now + 5000000;
			//runHazzardLights(5000000);
		}
	}
	if(this->step == doHaazzardLight)
	{
		if(cHighResTimer::GetTime() >= fin)
		{
			this->step = cpExit;
			transmitLight(lightPin,LIGHT_HAZARD_DISABLED);
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendManeuver(maneuverFinishedPin, MANEUVER_FINISHED,pMediaSample->GetTime()), "Cant send maneuver finished");
		}
	}
	RETURN_NOERROR;
}

tResult cMacro_player::parallelParkingScript(IMediaSample *pMediaSample)
{
	//driveBackToGap-> stopDriveBackToGap-> ppDriveRightMove-> ppDriveLeftMove-> ppDriveStraightMove-> sendManeuver()
	//tInt driveStraightBeforeGoInGap = GetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAP);
	//tInt driveStraightBeforeGoInGapSteer = GetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPSTEER);
	//tInt driveStraightBeforeGoInGapDrive = GetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTBEFORGOINGAPDRIVE);

	tInt steerLeftIntoGapDistance = GetPropertyInt(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCE);
	tInt steerLeftIntoGapDistanceSteer = GetPropertyInt(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCESTEER);
	tInt steerLeftIntoGapDistanceDrive = GetPropertyInt(PROP_NAME_PPDRIVELEFTINTOGAPDISTANCEDRIVE);

	tInt steerRightIntoGapDistance = GetPropertyInt(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCE);
	tInt steerRightIntoGapDistanceSteer = GetPropertyInt(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCESTEER);
	tInt steerRightIntoGapDistanceDrive = GetPropertyInt(PROP_NAME_PPDRIVERIGHTINTOGAPDISTANCEDRIVE);

	tInt driveStraightInGapDistance = GetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCE);
	tInt driveStraightInGapDistanceSteer = GetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCESTEER);
	tInt driveStraightInGapDistanceDrive = GetPropertyInt(PROP_NAME_PPDRIVESTRAIGHTINGAPDISTANCEDRIVE);

	static tTimeStamp now;
	static tTimeStamp fin;

	if (this->isDebugActive)
	{
		//LOG_INFO("Parallel parking is active");
	}
	if (this->step == driveBackToGap)
	{
		//transmitLight(lightPin,LIGHT_REVERSE);
		
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("dist  %d", this->distance));
		}

		this->step = stopDriveBackToGap;

		if (this->isDebugActive)
		{
			LOG_INFO("STEP 1->2");
		}
		this->lastDist = this->distance;

		//steer(pMediaSample, driveStraightBeforeGoInGapSteer);
		//drive(pMediaSample, driveStraightBeforeGoInGapDrive);

		//now = cHighResTimer::GetTime();
		//fin = now + driveStraightBeforeGoInGap;
	}
	if (this->step == stopDriveBackToGap)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("stopDriveBackToGap dist:  %d, last: %d", this->distance , this->lastDist));
		}
		/*if(cHighResTimer::GetTime() >= fin)
		{*/
			//stopDrive(pMediaSample);

			if (this->isDebugActive)
			{
				LOG_INFO("...Reached gapEnd");
			}

			//this->step = stopDriveBackToGap; //zum Schrittweise testen
			this->step = ppDriveRightMove;

			if (this->isDebugActive)
			{
				LOG_INFO("STEP 2->3");
				LOG_INFO("Start steering right into gap...");
			}

			transmitLight(lightPin,LIGHT_TURN_RIGHT);
			transmitLight(lightPin,LIGHT_REVERSE);

			steer(pMediaSample, steerRightIntoGapDistanceSteer);
			drive(pMediaSample, steerRightIntoGapDistanceDrive);
			this->lastDist = this->distance;

			now = cHighResTimer::GetTime();
			fin = now + steerRightIntoGapDistance;
		//}
	}
	if (this->step == ppDriveRightMove)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("ppDriveRightMove distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + steerRightIntoGapDistance), this->lastDist));
		}
		//if(this->distance >= this->lastDist + steerRightIntoGapDistance)
		if(cHighResTimer::GetTime() >= fin)
		{
			transmitLight(lightPin,LIGHT_TURN_DISABLED);
			this->step = ppDriveLeftMove;//zum Schrittweise testen
			//this->step = ppDriveRightMove;
			if (this->isDebugActive)
			{
				LOG_INFO("STEP 3->4");
				LOG_INFO("Start steering left into gap...");
			}
			transmitLight(lightPin,LIGHT_TURN_LEFT);
			//stopDrive(pMediaSample);
			//stopDrive(pMediaSample);
			steer(pMediaSample, steerLeftIntoGapDistanceSteer);
			drive(pMediaSample, steerLeftIntoGapDistanceDrive);
			this->lastDist = this->distance;

			now = cHighResTimer::GetTime();
			fin = now + steerLeftIntoGapDistance;
		}
	}
	if (this->step == ppDriveLeftMove)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("ppDriveLeftMove distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + steerLeftIntoGapDistance), this->lastDist));
		}
		//if(this->distance >= this->lastDist + steerLeftIntoGapDistance)
		if(cHighResTimer::GetTime() >= fin)
		{
			transmitLight(lightPin,LIGHT_TURN_DISABLED);
			transmitLight(lightPin,LIGHT_REVERSE_DISABLED);
			//stopDrive(pMediaSample);

			this->step = ppDriveStraightMove; // for testing purposes here
			//this->step = ppDriveStraightMove;
			if (this->isDebugActive)
			{
				LOG_INFO("STEP 4->5");
				LOG_INFO("Start drive straight in the gap");
			}
			this->lastDist = this->distance;
			//stopDrive(pMediaSample);
			steer(pMediaSample, driveStraightInGapDistanceSteer);
			drive(pMediaSample, driveStraightInGapDistanceDrive);

			now = cHighResTimer::GetTime();
			fin = now + driveStraightInGapDistance;
		}
	}//end if step==4
	if (this->step == ppDriveStraightMove)
	{	
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("ppDriveStraightMove distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + driveStraightInGapDistance), this->lastDist));
		}
		if(cHighResTimer::GetTime() >= fin)
		{
			//this->step = ppExit;
			this->step = doHaazzardLight;
			//stopDrive(pMediaSample);
			transmitLight(lightPin,LIGHT_HAZARD);
			if(this->isDebugActive)
			{
				LOG_INFO("STOP...hopefully reached final parking position");
			}
			now = cHighResTimer::GetTime();
			fin = now + 5000000;
			//runHazzardLights(5000000);
		}
	}
	if(this->step == doHaazzardLight)
	{
		if(cHighResTimer::GetTime() >= fin)
		{
			this->step = ppExit;
			transmitLight(lightPin,LIGHT_HAZARD_DISABLED);
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendManeuver(maneuverFinishedPin, MANEUVER_FINISHED,pMediaSample->GetTime()), "Cant send maneuver finished");
		}
	}
	RETURN_NOERROR;
}

tResult cMacro_player::pullOutLeftCrossParkingScript(IMediaSample *pMediaSample)
{
	tInt driveStraigtExitGapDistance = GetPropertyInt(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCE);
	tInt driveStraigtExitGapDistanceSteer = GetPropertyInt(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCESTEER);
	tInt driveStraigtExitGapDistanceDrive = GetPropertyInt(PROP_NAME_POLCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE);

	tInt driveLeftExitGapDistance = GetPropertyInt(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCE);
	tInt driveLeftExitGapDistanceSteer = GetPropertyInt(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCESTEER);
	tInt driveLeftExitGapDistanceDrive = GetPropertyInt(PROP_NAME_POLCPDRIVELEFTEXITGAPDISTANCEDRIVE);

	if(!this->initPullOut)
	{
		this->step = pullOutLeftCrossPart0;
		this->initPullOut = true;
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("Init the pull-out"));
		}
	}

	if (this->isDebugActive)
	{
		LOG_INFO(cString::Format("current Step in pull-out left cross == %d", this->step));
	}

	if(this->step == pullOutLeftCrossPart0)
	{
		if (this->isDebugActive)
		{
			LOG_INFO("...start leaving gap straight");
		}
		
		this->lastDist = this->distance;

		this->step = pullOutLeftCrossPart1;

		steer(pMediaSample, driveStraigtExitGapDistanceSteer);
		drive(pMediaSample, driveStraigtExitGapDistanceDrive);
	}
	if(this->step == pullOutLeftCrossPart1)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("pullOutLeftCrossPart1 distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + driveStraigtExitGapDistance), this->lastDist));
		}
		
		if(this->distance >= this->lastDist + driveStraigtExitGapDistance)
		{	
			if(this->isDebugActive)
			{
				LOG_INFO("...start leaving gap to the left");
			}

			this->lastDist = this->distance;

			this->step = pullOutLeftCrossPart2;
			transmitLight(lightPin,LIGHT_TURN_LEFT);
					
			steer(pMediaSample, driveLeftExitGapDistanceSteer);
			drive(pMediaSample, driveLeftExitGapDistanceDrive);
		}
	}		
	if(this->step == pullOutLeftCrossPart2)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("pullOutLeftCrossPart2 distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + driveLeftExitGapDistance), this->lastDist));
		}
		if(this->distance >= this->lastDist + driveLeftExitGapDistance)
		{	
			if (this->isDebugActive)
			{
				LOG_INFO("left the gap to the left side!");
			}
					
			transmitLight(lightPin,LIGHT_TURN_DISABLED);
				
			this->step = pullOutExit;
			stopDrive(pMediaSample);
			if (this->isDebugActive)
			{
				LOG_INFO("sending maneuver finished");
			}
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendManeuver(maneuverFinishedPin,MANEUVER_FINISHED,pMediaSample->GetTime()), "Cant send maneuver finished");
		}
	}//endifs pullOutLeftCross
	RETURN_NOERROR;
}

tResult cMacro_player::pullOutRightCrossParkingScript(IMediaSample *pMediaSample)
{
	tInt driveStraigtExitGapDistance = GetPropertyInt(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCE);
	tInt driveStraigtExitGapDistanceSteer = GetPropertyInt(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCESTEER);
	tInt driveStraigtExitGapDistanceDrive = GetPropertyInt(PROP_NAME_PORCPDRIVESTRAIGHTEXITGAPDISTANCEDRIVE);

	tInt driveRightExitGapDistance = GetPropertyInt(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCE);
	tInt driveRightExitGapDistanceSteer = GetPropertyInt(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCESTEER);
	tInt driveRightExitGapDistanceDrive = GetPropertyInt(PROP_NAME_PORCPDRIVERIGHTEXITGAPDISTANCEDRIVE);

	if(!this->initPullOut)
	{
		this->step = pullOutRightCrossPart0;
		this->initPullOut = true;
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("Init the pull-out"));
		}
	}

	if (this->isDebugActive)
	{
		LOG_INFO(cString::Format("current Step in pull-out right cross == %d", this->step));
	}

	if(this->step == pullOutRightCrossPart0)
	{
		if (this->isDebugActive)
		{
			LOG_INFO("...start leaving gap straight");
		}
		this->lastDist = this->distance;

		this->step = pullOutRightCrossPart1;
		steer(pMediaSample, driveStraigtExitGapDistanceSteer);
		drive(pMediaSample, driveStraigtExitGapDistanceDrive);
	}

	if(this->step == pullOutRightCrossPart1)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("pullOutRightCrossPart1 distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + driveStraigtExitGapDistance), this->lastDist));
		}

		if(this->distance >= this->lastDist + driveStraigtExitGapDistance)
		{
			if (this->isDebugActive)
			{
				LOG_INFO("...start leaving gap to the right");
			}

			this->step = pullOutRightCrossPart2;

			this->lastDist = this->distance;

			transmitLight(lightPin,LIGHT_TURN_RIGHT);

			steer(pMediaSample, driveRightExitGapDistanceSteer);
			drive(pMediaSample, driveRightExitGapDistanceDrive);
		}
	}
	if(this->step == pullOutRightCrossPart2)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("pullOutRightCrossPart2 distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + driveRightExitGapDistance), this->lastDist));
		}

		if(this->distance >= this->lastDist + driveRightExitGapDistance)
		{	
			transmitLight(lightPin,LIGHT_TURN_DISABLED);
			if (this->isDebugActive)
			{
				LOG_INFO("left the gap to the right side!");
			}

			this->step = pullOutExit;
			stopDrive(pMediaSample);

			if (this->isDebugActive)
			{
				LOG_INFO("sending maneuver finished");
			}
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendManeuver(maneuverFinishedPin,MANEUVER_FINISHED,pMediaSample->GetTime()), "Cant send maneuver finished");
		}
	}//endifs pullOutRightCross

	RETURN_NOERROR;
}

tResult cMacro_player::pullOutRightParallelParkingScript(IMediaSample *pMediaSample)
{
	tInt driveStraigtInGapDistance = GetPropertyInt(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCE);
	tInt driveStraigtInGapDistanceSteer = GetPropertyInt(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCESTEER);
	tInt driveStraigtInGapDistanceDrive = GetPropertyInt(PROP_NAME_PORPPDRIVESTRAIGHTINGAPDISTANCEDRIVE);

	tInt steerLeftExitGapDistance = GetPropertyInt(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCE);
	tInt steerLeftExitGapDistanceSteer = GetPropertyInt(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCESTEER);
	tInt steerLeftExitGapDistanceDrive = GetPropertyInt(PROP_NAME_PORPPDRIVELEFTEXITGAPDISTANCEDRIVE);

	tInt steerRightExitGapDistance = GetPropertyInt(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCE);
	tInt steerRightExitGapDistanceSteer = GetPropertyInt(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCESTEER);
	tInt steerRightExitGapDistanceDrive = GetPropertyInt(PROP_NAME_PORPPDRIVERIGHTEXITGAPDISTANCEDRIVE);
	
	static tTimeStamp now;
	static tTimeStamp fin;

	if(!this->initPullOut)
	{
		this->step = pullOutRightParallel0;
		this->initPullOut = true;

		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("Init the pull-out"));
		}
	}

	if (this->isDebugActive)
	{
		LOG_INFO(cString::Format("current Step in pull-out parallel == %d", this->step));
	}

	if(this->step == pullOutRightParallel0)
	{
		if (this->isDebugActive)
		{
			LOG_INFO("Start leaving gap straight back...");
		}

		this->lastDist = this->distance;

		this->step = pullOutRightParallel1;

		transmitLight(lightPin,LIGHT_REVERSE);

		steer(pMediaSample, driveStraigtInGapDistanceSteer);
		drive(pMediaSample, driveStraigtInGapDistanceDrive);

		now = cHighResTimer::GetTime();
		fin = now + driveStraigtInGapDistance;
	}
	if(this->step == pullOutRightParallel1)
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("pullOutRightParallel1 distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + driveStraigtInGapDistance), this->lastDist));
		}

		if(cHighResTimer::GetTime() >= fin)
		{
			transmitLight(lightPin,LIGHT_REVERSE_DISABLED);

			//stopDrive(pMediaSample);

			if (this->isDebugActive)
			{
				LOG_INFO("Start leaving gap straight left");
			}
			this->step = pullOutRightParallel2;
			this->lastDist = this->distance;

			transmitLight(lightPin,LIGHT_TURN_LEFT);
			steer(pMediaSample, steerLeftExitGapDistanceSteer);
			drive(pMediaSample, steerLeftExitGapDistanceDrive);

			now = cHighResTimer::GetTime();
			fin = now + steerLeftExitGapDistance;
		}
	}
	if(this->step == pullOutRightParallel2) 
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("pullOutRightParallel2 distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + steerLeftExitGapDistance), this->lastDist));
		}
		if(cHighResTimer::GetTime() >= fin)
		{
			if (this->isDebugActive)
			{
				LOG_INFO("Start leaving gap straight right");
			}

			this->step = pullOutRightParallel3;
			this->lastDist = this->distance;

			transmitLight(lightPin,LIGHT_TURN_DISABLED);
			transmitLight(lightPin,LIGHT_TURN_RIGHT);

			steer(pMediaSample, steerRightExitGapDistanceSteer);
			drive(pMediaSample, steerRightExitGapDistanceDrive);

			now = cHighResTimer::GetTime();
			fin = now + steerRightExitGapDistance;
		}
	}
	if(this->step == pullOutRightParallel3) 
	{
		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("pullOutRightParallel3 distance: %d, lastDist+x: %d, LastDist %d", this->distance, (this->lastDist + steerRightExitGapDistance), this->lastDist));
		}
		if(cHighResTimer::GetTime() >= fin)
		{
			if (this->isDebugActive)
			{
				LOG_INFO("...hopefully left the gap to the right");
			}
			transmitLight(lightPin,LIGHT_TURN_DISABLED);

			this->step = pullOutExit;
			stopDrive(pMediaSample);

			if (this->isDebugActive)
			{
				LOG_INFO("sending maneuver finished");
			}
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendManeuver(maneuverFinishedPin,MANEUVER_FINISHED,pMediaSample->GetTime()), "Cant send maneuver finished");
		}
	}
	RETURN_NOERROR;
}

tResult cMacro_player::PropertyChanged(const char *propertyName)
{
	this->isDebugActive = GetPropertyBool(PROP_NAME_DEBUG);
	RETURN_NOERROR;
}

tInt cMacro_player::getDistanceFromWheelRotation(tInt &rotation)
{
	const static tInt CM_PER_ROTATION = 4;

	return rotation * CM_PER_ROTATION;
}

tResult cMacro_player::drive(IMediaSample *pMediaSample)
{
	transmitLight(lightPin,LIGHT_BREAK_DISABLED);
	RETURN_IF_FAILED(sendNewValue(&m_pin_output_acceleration, 30.0f, pMediaSample->GetTime()));

	RETURN_NOERROR;
}

tResult cMacro_player::drive(IMediaSample *pMediaSample, const tInt &speed)
{
	tInt fireSendNewValue = 3;

	if (isSpeedValid(speed))
	{
		//firing fireSendNewValue-times the sendNewValue()-method
		for(int i = 0; i<fireSendNewValue; i++)
		{
			RETURN_IF_FAILED(sendNewValue(&m_pin_output_acceleration, static_cast<tFloat32>(speed), pMediaSample->GetTime()));
		}
	}
	else
	{
		drive(pMediaSample);
	}

	if(this->isDebugActive)
	{
		LOG_INFO(cString::Format("drive: %d", speed));
	}

	RETURN_NOERROR;
}

tBool cMacro_player::isSpeedValid(const tInt &speed) const
{
	const static tInt MIN_SPEED = -100;
	const static tInt MAX_SPEED = 100;

	return speed >= MIN_SPEED && speed <= MAX_SPEED;
}

tResult cMacro_player::steer(IMediaSample *pMediaSample, const tInt &steerAngle)
{
	//transmitLight(lightPin,LIGHT_TURN_DISABLED);

	if (isSteerAngleValid(steerAngle))
	{
		tInt fireSteerAngle = 3;

		if (this->isDebugActive)
		{
			LOG_INFO(cString::Format("steer: %d", steerAngle));
		}

		for(int i = 0; i < fireSteerAngle; i++)
		{
			RETURN_IF_FAILED(sendNewValue(&m_pin_output_steeringAngle, static_cast<tFloat32>(steerAngle), pMediaSample->GetTime()));
		}

		/*if(steerAngle > 0)
		{
			transmitLight(lightPin,LIGHT_TURN_RIGHT);
		}
		else if(steerAngle < 0) 
		{
			transmitLight(lightPin,LIGHT_TURN_LEFT);
		}*/
	}
	RETURN_NOERROR;
}

tBool cMacro_player::isSteerAngleValid(const tInt &steerAngle) const
{
	const static tInt MIN_STEER_ANGLE = -30;
	const static tInt MAX_STEER_ANGLE = 30;

	return steerAngle >= MIN_STEER_ANGLE && steerAngle <= MAX_STEER_ANGLE;
}

tResult cMacro_player::isFrontFree()
{
	//check via front center shortrange sensor if there is enough space(for the decision if we standing parallel or cross in the parking gap)
	if(this->irFront > 45)
	{
		this->prevDecision = DECISION_CROSS_PARKING;
		//this->step = cpLeftMove;
	}
	else
	{
		this->prevDecision = DECISION_PARALLEL_PARKING;
		//this->step == stopDriveBackToGap;
	}

	if (this->isDebugActive)
	{
		LOG_INFO(cString::Format("this->irFront: %d", this->irFront));
		LOG_INFO(cString::Format("this->currentDecision: %d, this->prevDecision: %d", this->currentDecision, this->prevDecision));
	}
	RETURN_NOERROR;
}

/**
* @author dhecht
* implementation of decision pins
*/
tResult cMacro_player::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
	tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
	RETURN_IF_POINTER_NULL(descriptionSignalValue);        

	mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
	RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

	RETURN_NOERROR;
}

tResult cMacro_player::getDecisionValue(IMediaSample *mediaSample, tUInt16 &decisionValue, tTimeStamp &timeStamp)
{
	cObjectPtr<IMediaCoder> coderInput;
	RETURN_IF_FAILED(this->coderDescriptionManeuver->Lock(mediaSample, &coderInput));

	coderInput->Get("ui16Angle", (tVoid*) &decisionValue);
	coderInput->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);

	RETURN_IF_FAILED(this->coderDescriptionManeuver->Unlock(coderInput));

	RETURN_NOERROR;
}

tVoid cMacro_player::getDecision(const tUInt16 &decisionValue, Decision &currentDecision, Decision &prevDecision)
{
	switch(decisionValue)
	{
	case DECISION_STOP:
		prevDecision = currentDecision;
		currentDecision = DECISION_STOP;
		LOG_INFO("Got Decision STOP");
		break;

	case DECISION_PARALLEL_PARKING:
		prevDecision = currentDecision;
		currentDecision = DECISION_PARALLEL_PARKING;
		LOG_INFO("Got Decision PARALLEL_PARKING");
		break;

	case DECISION_CROSS_PARKING:
		prevDecision = currentDecision;
		currentDecision = DECISION_CROSS_PARKING;
		LOG_INFO("Got Decision CROSS_PARKING");
		break;

	case DECISION_PULL_OUT_LEFT:
		prevDecision = currentDecision;
		currentDecision = DECISION_PULL_OUT_LEFT;
		LOG_INFO("Got Decision PULL_OUT_LEFT");
		break;

	case DECISION_PULL_OUT_RIGHT:
		prevDecision = currentDecision;
		currentDecision = DECISION_PULL_OUT_RIGHT;
		LOG_INFO("Got Decision PULL_OUT_RIGHT");
		break;

	default:
		prevDecision = currentDecision;
		currentDecision = DECISION_STOP;
		LOG_INFO("Got Decision STOP");
		break;
	}
}

tResult cMacro_player::sendManeuver(cOutputPin &pin, const Maneuver &manuever, const tTimeStamp &timeStamp)
{
	// create new media sample
	cObjectPtr<IMediaSample> mediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeManeuver));

	// write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionManeuver->WriteLock(mediaSample, &coder));

	coder->Set("ui16Angle", (tVoid*) &manuever);
	coder->Set("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
	RETURN_IF_FAILED(this->coderDescriptionManeuver->Unlock(coder));

	// transmit media sample over output pin
	RETURN_IF_FAILED(mediaSample->SetTime(timeStamp));
	RETURN_IF_FAILED(pin.Transmit(mediaSample));
	
	RETURN_NOERROR;
}

tResult cMacro_player::transmitLight(cOutputPin &pin, const Light &light)
{
	if (!pin.IsConnected())
	{
		RETURN_NOERROR;
	}

	cObjectPtr<IMediaSample> mediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));

	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeManeuver));

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionManeuver->WriteLock(mediaSample, &coder));
	
	static tTimeStamp now;
	now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

	coder->Set("ui16Angle", (tVoid*) &light);
	coder->Set("ui32ArduinoTimestamp", (tVoid*) &now);
	
	RETURN_IF_FAILED(this->coderDescriptionManeuver->Unlock(coder));

	RETURN_IF_FAILED(mediaSample->SetTime(now));
	RETURN_IF_FAILED(pin.Transmit(mediaSample));

	RETURN_NOERROR;
}