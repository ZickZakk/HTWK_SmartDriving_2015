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

#ifndef _DriverFilter_H_
#define _DriverFilter_H_

#ifdef _DEBUG
#define FILTER_NAME "SD DriverModul Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.DriverFilter.debug"
#else
#define FILTER_NAME "SD DriverModul"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.DriverFilter.release"
#endif

#define PROP_NAME "ManeuverFile"

class DriverFilter : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter);

private:
	// Coder description for input pins
	cObjectPtr<IMediaTypeDescription> coderDescriptionJury;
	cObjectPtr<IMediaTypeDescription> coderDescriptionDriver;
	cObjectPtr<IMediaTypeDescription> coderDescriptionManeuver;
	cObjectPtr<IMediaDescriptionManager> descriptionManager;

	// input pins
	cInputPin juryStatePin;
	cInputPin maneuverFinishedPin;

	// output pins
	cOutputPin driverStatePin;
	cOutputPin currentManeuverPin;

	// maneuver
	cFilename maneuverListFile;
	std::vector<tSector> sectorList;
	
	Maneuver currentManeuver;
	tInt currentManeuverId;
	tInt ddlSizeManeuver;
	tInt ddlSizeDriver;

	// stuff 
	tBool isFirstTimeRun;
	tInt highestManeuverId;
	tInt stopCounter;

public:
	DriverFilter(const tChar *__info);
	virtual ~DriverFilter(void);

public:
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);

private:
	tVoid initProperties();

	tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);
	tResult createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	tResult createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	
	tResult getManeuver(IMediaSample *mediaSample, tInt8 &actionId, tInt16 &maneuverId);
	tResult getManeuver(IMediaSample *mediaSample, tUInt16 &maneuverValue, tTimeStamp &timeStamp);
	
	tResult loadManeuverList();
	tVoid getHighestManeuverId(tInt &highestId);

	tResult sendError(const tInt16 &maneuverId);
	tResult sendReady(const tInt16 &maneuverId);
	tResult sendRunning(const tInt16 &maneuverId);
	tResult sendComplete(const tInt16 &maneuverId);
	tResult sendDriverState(const DriverState &driverState, const tInt16 &maneuverId);
	
	tBool isManeuverFinished(const tUInt16 &maneuverValue) const;

	/**
	 * @return returns true if a following maneuver exists, false otherwise
	 */
	tBool getNextManeuver(tInt &maneuverId, Maneuver &maneuver);
	tVoid getManeuverFromId(const tInt &maneuverId, Maneuver &maneuver);

	tVoid getManeuverFromString(const cString &name, Maneuver &maneuver);

	tResult sendManeuver(cOutputPin &pin, const Maneuver &maneuver);
	tResult sendManeuver(cOutputPin &pin, const Maneuver &manuever, const tTimeStamp &timeStamp);
};

#endif