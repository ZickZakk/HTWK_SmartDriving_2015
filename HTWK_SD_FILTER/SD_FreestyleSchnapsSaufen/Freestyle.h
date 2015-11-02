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
#ifndef _FREESTYLE_H_
#define _FREESTYLE_H_

#ifdef _DEBUG
#define FILTER_NAME "SD Freestyle Debug"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.Freestyle.debug"
#else
#define FILTER_NAME "SD Freestyle"
#define OID_NEW_LANE_DETECTION "atdf.smartdriving.Freestyle.release"
#endif

#define PROP_NAME_IP "0_IP"
#define PROP_NAME_STOP_LINES "1_StopLines"
#define PROP_NAME_DELAY "2_Delay"
#define PROP_NAME_DRIVE_SPEED "3_DriveSpeed"
#define PROP_NAME_SMOOTH_CURVE_VALUE "4_SmoothCurveValue"
#define PROP_NAME_TICKS_TO_STOP_LINE "5_TicksToStopLine"

class Test : public cFilter
{
	ADTF_FILTER(OID_NEW_LANE_DETECTION, FILTER_NAME, OBJCAT_DataFilter)

private:
	cVideoPin xtionPin;
	cInputPin maneuverPin;
	cInputPin wheelTicksPin;
	cOutputPin accelerationPin;
	cOutputPin steeringAnglePin;

	cObjectPtr<IMediaTypeDescription> coderDescriptionManeuver;
	cObjectPtr<IMediaTypeDescription> coderDescriptionSteeringAngle;
	cObjectPtr<IMediaTypeDescription> coderDescriptionAcceleration;
	cObjectPtr<IMediaTypeDescription> coderDescriptionWheelTicks;
	
	cObjectPtr<IMediaDescriptionManager> descriptionManager;

	int ddlSizeUI16;

	tBitmapFormat videoInputInfo;
	bool isFirstFrame;
	bool isDriveActive;
	bool isStopLineFound;
	bool isConnectedToServer;
	
	std::string ip;
	int numberOfStopLines;
	int delay;
	int steeringAngle;
	int driveSpeed;
	int smoothCurveValue;
	int ticksToStopLine;
	int ticksToDrive;
	int currentWheelTicks;
	int stopOnStopLine;
	
	// interfaces
	unique_ptr<ICrossRoadDetector> crossroadDetector;
	unique_ptr<IDriveAlgorithm> driveAlgorithm;
	unique_ptr<Client> tcpClient;
	unique_ptr<Server> tcpServer;

public:
	Test(const char *__info);
	virtual ~Test();

	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	tResult OnPinEvent(IPin*, tInt, tInt, tInt, IMediaSample*);
	tResult PropertyChanged(const char *name);

private:
	tResult initProperties();
	tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);

	tResult createVideoInputPin(const tChar *pinName, cVideoPin &pin);
	tResult createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal);
	tResult createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal);

	tResult initVideoStream(void);
	tVoid setBitmapFormat(const tBitmapFormat *format);
	tResult driveToStopLine(void);
	void accept(void);

	tResult transmitSteeringAngle(const tFloat32 value);
	tResult transmitAcceleration(const tFloat32 value);
	tResult transmitStop(const tFloat32 value);
	tResult transmitF32Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tFloat32 value);

	tResult getWheelTicks(IMediaSample *mediaSample, tFloat32 &value);
	tResult getF32Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tFloat32 &value);

	tResult getManeuver(IMediaSample *mediaSample, cInputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, tUInt16 &value);
};

#endif