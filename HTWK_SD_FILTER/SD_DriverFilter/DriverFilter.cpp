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
#include "DriverFilter.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID_NEW_LANE_DETECTION, DriverFilter);

DriverFilter::DriverFilter(const char *__info) : cFilter(__info)
{
	this->isFirstTimeRun = true;
	this->stopCounter = 0;
	initProperties();
}

tVoid DriverFilter::initProperties()
{
	SetPropertyStr(PROP_NAME, "");
    SetPropertyBool(PROP_NAME NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr(PROP_NAME NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
}

DriverFilter::~DriverFilter(void)
{}

tResult DriverFilter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	
	if (eStage == StageFirst)
	{
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &this->descriptionManager, __exception_ptr));

		cObjectPtr<IMediaType> juryMediaType;
		cObjectPtr<IMediaType> driverMediaType;
		cObjectPtr<IMediaType> maneuverMediaType;
		
		RETURN_IF_FAILED(initMediaType("tJuryStruct", juryMediaType, this->coderDescriptionJury));
		RETURN_IF_FAILED(initMediaType("tDriverStruct", driverMediaType, this->coderDescriptionDriver));
		RETURN_IF_FAILED(initMediaType("tSteeringAngleData", maneuverMediaType, this->coderDescriptionManeuver));

		// input pins
		RETURN_IF_FAILED(createInputPin("Jury_Struct", this->juryStatePin, juryMediaType));
		RETURN_IF_FAILED(createInputPin("Maneuver_Finished", this->maneuverFinishedPin, maneuverMediaType));

		// output pins
		RETURN_IF_FAILED(createOutputPin("Driver_Struct", this->driverStatePin, driverMediaType));
		RETURN_IF_FAILED(createOutputPin("Current_Maneuver", this->currentManeuverPin, maneuverMediaType));
	}
	else if (eStage == StageGraphReady)
	{
		RETURN_IF_FAILED(loadManeuverList());
		getHighestManeuverId(this->highestManeuverId);

		cObjectPtr<IMediaSerializer> serializer;
		RETURN_IF_FAILED(this->coderDescriptionManeuver->GetMediaSampleSerializer(&serializer));
		this->ddlSizeManeuver = serializer->GetDeserializedSize();

		RETURN_IF_FAILED(this->coderDescriptionDriver->GetMediaSampleSerializer(&serializer));
		this->ddlSizeDriver = serializer->GetDeserializedSize();
	}
	
	RETURN_NOERROR;
}

tResult DriverFilter::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
	tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
    RETURN_IF_POINTER_NULL(descriptionSignalValue);        

	mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
	RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

	RETURN_NOERROR;
}

tResult DriverFilter::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult DriverFilter::createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
	RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
	RETURN_IF_FAILED(RegisterPin(&pin));

	RETURN_NOERROR;
}

tResult DriverFilter::Start(__exception)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

	RETURN_NOERROR;
}

tResult DriverFilter::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

	RETURN_NOERROR;
}

tResult DriverFilter::Shutdown(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

	RETURN_NOERROR;
}

tResult DriverFilter::OnPinEvent(IPin *source, tInt eventCore, tInt param1, tInt param2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);

	if (eventCore == IPinEventSink::PE_MediaSampleReceived)
	{
		if (source == &this->juryStatePin)
		{
			static tInt8 actionId;
            static tInt16 maneuverId;
            
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getManeuver(mediaSample, actionId, maneuverId), "Cant get Maneuver from Jury");

			switch (actionId)
			{
			case STOP:
				LOG_INFO(cString::Format("Jury sends stop in %d", maneuverId));
				
				this->isFirstTimeRun = true;

				RETURN_IF_FAILED(sendError(maneuverId));
				
				if (this->stopCounter <= 3)
				{
					RETURN_IF_FAILED(sendManeuver(this->currentManeuverPin, MANEUVER_STOP, mediaSample->GetTime()));
					this->stopCounter++;
				}
				break;

			case READY:
				LOG_INFO(cString::Format("Jury sends ready in %d", maneuverId));
				
				RETURN_IF_FAILED(sendReady(maneuverId));
				this->stopCounter = 0;
				break;

			case RUN:
				LOG_INFO(cString::Format("Jury sends run in %d", maneuverId));

				if (this->isFirstTimeRun)
				{
					LOG_INFO("First time run!");
					this->isFirstTimeRun = false;
					this->stopCounter = 0;

					getManeuverFromId(maneuverId, this->currentManeuver);
					this->currentManeuverId = maneuverId;
					
					LOG_INFO("Sending Manuever");
					RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendManeuver(this->currentManeuverPin, this->currentManeuver, mediaSample->GetTime()), "Cant send maneuver");
				}
				
				RETURN_IF_FAILED(sendRunning(maneuverId));
				break;
			}
		}
		else if (source == &this->maneuverFinishedPin)
		{
			LOG_WARNING("Got maneuver finished state");
			this->stopCounter = 0;
			// lese den Zustand des aktuellen Befehls aus
			static tUInt16 maneuverValue;
			static tTimeStamp timeStamp;
			
			RETURN_IF_FAILED_AND_LOG_ERROR_STR(getManeuver(mediaSample, maneuverValue, timeStamp), "Cant get Maneuver from ManeuverFinishedPin");

			if (isManeuverFinished(maneuverValue))
			{
				// hole neuen befehl und sende ihn an das Juri modul und an den Befehlsauswerter
				if (getNextManeuver(this->currentManeuverId, this->currentManeuver))
				{
					LOG_WARNING(cString::Format("next maneuver id: %d", this->currentManeuverId));
					RETURN_IF_FAILED(sendRunning(this->currentManeuverId));
					RETURN_IF_FAILED(sendManeuver(this->currentManeuverPin, this->currentManeuver, mediaSample->GetTime()));
				}
				else
				{
					LOG_INFO(cString::Format("send complete to jury in %d", this->currentManeuverId));
					RETURN_IF_FAILED(sendComplete(this->currentManeuverId));
					RETURN_IF_FAILED(sendManeuver(this->currentManeuverPin, MANEUVER_STOP, mediaSample->GetTime()));
				}
			}
			else
			{
				RETURN_IF_FAILED(sendRunning(this->currentManeuverId));
			}
		}
	}

	RETURN_NOERROR;
}

tResult DriverFilter::getManeuver(IMediaSample *mediaSample, tInt8 &actionId, tInt16 &maneuverId)
{
	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionJury->Lock(mediaSample, &coder));
	
	coder->Get("i8ActionID", (tVoid*) &actionId);
	coder->Get("i16ManeuverEntry", (tVoid*) &maneuverId);
	
	RETURN_IF_FAILED(this->coderDescriptionJury->Unlock(coder));

	RETURN_NOERROR;
}

tResult DriverFilter::getManeuver(IMediaSample *mediaSample, tUInt16 &maneuverValue, tTimeStamp &timeStamp)
{
	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionManeuver->Lock(mediaSample, &coder));
	
	coder->Get("ui16Angle", (tVoid*) &maneuverValue);
	coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
	
	RETURN_IF_FAILED(this->coderDescriptionManeuver->Unlock(coder));

	RETURN_NOERROR;
}

tResult DriverFilter::sendError(const tInt16 &maneuverId)
{
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDriverState(ERROR, maneuverId), "Cant send ERROR");
	RETURN_NOERROR;
}

tResult DriverFilter::sendReady(const tInt16 &maneuverId)
{
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDriverState(READY, maneuverId), "Cant send READY");
	RETURN_NOERROR;
}

tResult DriverFilter::sendRunning(const tInt16 &maneuverId)
{
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDriverState(RUNNING, maneuverId), "Cant send RUNNING");
	RETURN_NOERROR;
}

tResult DriverFilter::sendComplete(const tInt16 &maneuverId)
{
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(sendDriverState(COMPLETE, maneuverId), "Cant send COMPLETE");
	RETURN_NOERROR;
}

tResult DriverFilter::sendDriverState(const DriverState &driverState, const tInt16 &maneuverId)
{
	cObjectPtr<IMediaSample> mediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeDriver));

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionDriver->WriteLock(mediaSample, &coder));
	
	coder->Set("i8StateID", (tVoid*) &driverState);
	coder->Set("i16ManeuverEntry", (tVoid*) &maneuverId);
	
	RETURN_IF_FAILED(this->coderDescriptionDriver->Unlock(coder));

	static tTimeStamp now;
	now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

	RETURN_IF_FAILED(mediaSample->SetTime(now));
	RETURN_IF_FAILED(this->driverStatePin.Transmit(mediaSample));

	RETURN_NOERROR;
}

tBool DriverFilter::isManeuverFinished(const tUInt16 &maneuverValue) const
{
	return maneuverValue == MANEUVER_FINISHED;
}

tVoid DriverFilter::getManeuverFromId(const tInt &maneuverId, Maneuver &maneuver)
{
	for (auto sector = this->sectorList.begin(); sector != this->sectorList.end(); ++sector)
	{
		for (auto action = sector->maneuverList.begin(); action != sector->maneuverList.end(); ++action)
		{
			if (maneuverId == action->id)
			{
				getManeuverFromString(action->action, maneuver);
				LOG_INFO(cString::Format("Maneuver from string: %s -> %d", action->action.GetPtr(), maneuver));
				return;
			}
		}
	}
}

tBool DriverFilter::getNextManeuver(tInt &maneuverId, Maneuver &maneuver)
{
	if (maneuverId == this->highestManeuverId)
	{
		LOG_WARNING("Maneuver was last maneuver");
		return false;
	}
	else
	{
		maneuverId++;

		for (auto sector = this->sectorList.begin(); sector != this->sectorList.end(); ++sector)
		{
			for (auto action = sector->maneuverList.begin(); action != sector->maneuverList.end(); ++action)
			{
				if (maneuverId == action->id)
				{
					getManeuverFromString(action->action, maneuver);
					LOG_INFO(cString::Format("Maneuver from string: %d -> %d", action->id, maneuver));
					return true;
				}
			}
		}
	}
	LOG_WARNING("Could not find next meneuver");
	return false;
}

tVoid DriverFilter::getManeuverFromString(const cString &name, Maneuver &maneuver)
{
	if (name.Compare("left") == 0)
	{
		maneuver = MANEUVER_LEFT;
	}
	else if (name.Compare("right") == 0)
	{
		maneuver = MANEUVER_RIGHT;
	}
	else if (name.Compare("straight") == 0)
	{
		maneuver = MANEUVER_STRAIGHT;
	}
	else if (name.Compare("parallel_parking") == 0)
	{
		maneuver = MANEUVER_PARALLEL_PARKING;
	}
	else if (name.Compare("cross_parking") == 0)
	{
		maneuver = MANEUVER_CROSS_PARKING;
	}
	else if (name.Compare("pull_out_right") == 0)
	{
		maneuver = MANEUVER_PULL_OUT_RIGHT;
	}
	else if (name.Compare("pull_out_left") == 0)
	{
		maneuver = MANEUVER_PULL_OUT_LEFT;
	}
	else
	{
		LOG_INFO(cString::Format("Unbekannter Befehl: %s", name.GetPtr()));
	}
}

tResult DriverFilter::sendManeuver(cOutputPin &pin, const Maneuver &maneuver)
{
	cObjectPtr<IMediaSample> mediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
	RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeManeuver));

	cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED(this->coderDescriptionDriver->WriteLock(mediaSample, &coder));
	
	static tTimeStamp now;
	now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

	coder->Set("ui16Angle", (tVoid*) &maneuver);
	coder->Set("ui32ArduinoTimestamp", (tVoid*) &now);
	
	RETURN_IF_FAILED(this->coderDescriptionDriver->Unlock(coder)); 

	RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaSample->SetTime(now), "cant set time to media sample");
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(pin.Transmit(mediaSample), "cant transmit media sample");

	RETURN_NOERROR;
}

tResult DriverFilter::sendManeuver(cOutputPin &pin, const Maneuver &manuever, const tTimeStamp &timeStamp)
{							
    // create new media sample
    cObjectPtr<IMediaSample> mediaSample;
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(AllocMediaSample((tVoid**) &mediaSample), "cant alloc");
	LOG_INFO(cString::Format("ddlSize: %d", this->ddlSizeManeuver));
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaSample->AllocBuffer(this->ddlSizeManeuver), "cant alloc buffer size");
       
    // write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> coder;
	RETURN_IF_FAILED_AND_LOG_ERROR_STR(this->coderDescriptionManeuver->WriteLock(mediaSample, &coder), "cant lock");
	
	coder->Set("ui16Angle", (tVoid*) &manuever);
    coder->Set("ui32ArduinoTimestamp", (tVoid*) &timeStamp);

	RETURN_IF_FAILED_AND_LOG_ERROR_STR(this->coderDescriptionManeuver->Unlock(coder), "cant unlock");
    
    // transmit media sample over output pin
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaSample->SetTime(timeStamp), "cant set time");
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(pin.Transmit(mediaSample), "cant transmit");

	RETURN_NOERROR;
}

tResult DriverFilter::loadManeuverList()
{

    this->maneuverListFile = GetPropertyStr(PROP_NAME);
    
    if (this->maneuverListFile.IsEmpty())
    {
        LOG_ERROR("DriverFilter: Maneuver file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }    
    
    ADTF_GET_CONFIG_FILENAME(maneuverListFile);
    
    this->maneuverListFile = this->maneuverListFile.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data
   
    if (cFileSystem::Exists(this->maneuverListFile))
    {
        cDOM oDOM;
        oDOM.Load(this->maneuverListFile);        
        cDOMElementRefList oSectorElems;
        cDOMElementRefList oManeuverElems;

        //read first Sector Elem
        if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
        {                
            //iterate through sectors
            for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
            {
                //if sector found
                tSector sector;
                sector.id = (*itSectorElem)->GetAttributeUInt32("id");
                
                if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
                {
                    //iterate through maneuvers
                    for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                    {
                        tAADC_Maneuver man;
                        man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                        man.action = (*itManeuverElem)->GetAttribute("action");
                        sector.maneuverList.push_back(man);
                    }
                }
    
                this->sectorList.push_back(sector);
            }
        }
        if (oSectorElems.size() > 0)
        {
            LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        }
        else
        {
            LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver File found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
   
    RETURN_NOERROR;
}

tVoid DriverFilter::getHighestManeuverId(tInt &highestId)
{
	tInt tmpHighestId = -1;

	for (auto sector = this->sectorList.begin(); sector != this->sectorList.end(); ++sector)
	{
		for (auto action = sector->maneuverList.begin(); action != sector->maneuverList.end(); ++action)
		{
			if (tmpHighestId < action->id)
			{
				tmpHighestId = action->id;
			}
		}
	}

	highestId = tmpHighestId;
}