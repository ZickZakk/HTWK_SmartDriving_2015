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
 * @copyright (c) Copyright 2014 SD. All rights reserved
 * @author epeterse
 * @details
 */
#ifndef _SD_Emergency_system_FILTER_HEADER_
#define _SD_Emergency_system_FILTER_HEADER_

#ifdef _DEBUG
#define FILTER_NAME "SD Emergency System Debug"
#define OID_ADTF_SD_EMERGENCY_SYSTEM "adtf.sd.emergencySystem.debug"
#else
#define FILTER_NAME "SD Emergency System Release"
#define OID_ADTF_SD_EMERGENCY_SYSTEM "adtf.sd.emergencySystem.release"
#endif

//Property definitions
#define PROPERTY_EMERGENCY_DISTANCE_FRONT "Emergency distance front"
#define PROPERTY_EMERGENCY_DISTANCE_REAR "Emergency distance rear"
#define PROPERTY_EMERGENCY_DISTANCE_FRONT_USS "Emergency distance front uss"

class cEmergency_system : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_SD_EMERGENCY_SYSTEM , FILTER_NAME, adtf::OBJCAT_DataFilter);

private:
    tResult CreateRawCanTestData(const tTimeStamp& tmStreamTime);
    tResult sendNewValue(cOutputPin * outpin, tTimeStamp timeStamp,IMediaSample *pMediaSample);
    tResult sendNewBoolValue(cOutputPin *outpin, tBool value, tTimeStamp timeStamp);
    tResult break_and_wink(tTimeStamp timeStamp, tBool on_off);
    tResult sendDecision(const Emergency_Decision &decision);
    tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);

    tResult checkFrontUSS(IMediaSample *pMediaSample);
    tResult checkFrontIFR(IMediaSample *pMediaSample);
    tResult reactOnSensors(IPin *pSource, IMediaSample *pMediaSample);

    tResult createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal);
    tResult createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal);

    tResult getValue(IMediaSample *mediaSample, tInt &distance);
    tResult getValueFloat(IMediaSample *mediaSample, tFloat32 &signal);

    bool m_bEmergencybreak;
    tTimeStamp m_nLastMSTime;

    tInt steerAngle;
    tBool initSteerAngle;

    //Stuff for decision
    cObjectPtr<IMediaTypeDescription> coderDescriptionSignalInput;
    //cObjectPtr<IMediaTypeDescription> coderDescriptionSignalWinkInput;
    cObjectPtr<IMediaTypeDescription> coderDescriptionManeuver;
    cObjectPtr<IMediaDescriptionManager> descriptionManager;

protected:
    //pass through data output
    //cOutputPin m_oOAcceleration; // output pin for signal data

        cOutputPin m_oOBrakeLight; // output pin for signal data
        cOutputPin m_oOWinkerLeft; // output pin for signal data
        cOutputPin m_oOWinkerRight; // output pin for signal data

    //OutputDecision
    cOutputPin decisionPin;

    //emergency data
    cInputPin  m_oIFrontIRF; // input pin for signal data
    cInputPin  m_oIFrontUSSR; // input pin for signal data
    cInputPin  m_oIFrontUSSL; // input pin for signal data
    //cInputPin  m_oIRearIRF; // input pin for signal data
    //cInputPin  m_oIRearUSSR; // input pin for signal data
    //cInputPin  m_oIRearUSSL; // input pin for signal data

    //Lenkung
    cInputPin steerAngleInput;

    //pass through data input
/*
        cInputPin  m_oIBrakeLight; // input pin for signal data
        cInputPin  m_oIWinkerLeft; // input pin for signal data
        cInputPin  m_oIWinkerRight; // input pin for signal data
*/
    //Coder description for input pins
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalOutput;

    //Properties
    int m_nEmergencyDistanceFront;
    int m_nEmergencyDistanceRear;
    int m_nEmergencyDistanceFrontUss;

public: // construction
    cEmergency_system(const tChar* __info);
    virtual ~cEmergency_system();

    tResult Init(tInitStage eStage, __exception);
    tResult Start(__exception);
    tResult Stop(__exception);
    tResult Shutdown(tInitStage eStage, __exception);
    tResult OnPinEvent( IPin *pSource, tInt nEventCore, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

};

//*************************************************************************************************
#endif // _SD_Emergency_system_FILTER_HEADER_
