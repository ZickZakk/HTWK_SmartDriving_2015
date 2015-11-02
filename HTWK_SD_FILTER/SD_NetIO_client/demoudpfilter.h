/**
 *
 * ADTF Demo Filter.
 *    This is only for demo a Filter application.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: ABOEMI9 $
 * $Date: 2013-08-27 16:36:26 +0200 (Tue, 27 Aug 2013) $
 * $Revision: 40298 $
 *
 * @remarks
 *
 */
#ifndef _DEMO_EMPTY_FILTER_HEADER_
#define _DEMO_EMPTY_FILTER_HEADER_


#define OID_ADTF_DEMO_UDP  "adtf.example.demo_udp"
 #define PROP_NAME_IP "0_IP"
#define PROP_NAME_STOP_LINES "1_StopLines"
#define PROP_NAME_DELAY "2_Delay"
#define PROP_NAME_DRIVE_SPEED "3_DriveSpeed"
#define PROP_NAME_SMOOTH_CURVE_VALUE "4_SmoothCurveValue"
#define PROP_NAME_TICKS_TO_STOP_LINE "5_TicksToStopLine"

class cDemoFilter : public cFilter
{
    ADTF_FILTER(OID_ADTF_DEMO_UDP, "Demo UDP Filter", OBJCAT_BridgeDevice)

    private: //private members
        class cReceiveThread: public cKernelThread
        {
            protected:
                cDemoFilter*    m_pParent;

            public:
                cReceiveThread();
                tResult SetParent(cDemoFilter* pParent);

            protected: //overwrite ThreadFunc of cKernelThread
                tResult ThreadFunc();
        };

        static const tInt           MAX_PACKET_SIZE = 8129;

        cInputPin                   m_oInput;
        cOutputPin                  m_oOutput;
		cObjectPtr<IMediaTypeDescription>	m_pCoderDesc;
        cDatagramSocket             m_oSendSocket;
        cDatagramSocket             m_oRecvSocket;
        cString                     m_strDest;
        tInt                        m_nPort;
        cReceiveThread              m_oReceiveThread;
        tChar*                      m_pBuffer;

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
    unique_ptr<ICrossRoadDetector> crossroadDetector;
    unique_ptr<IDriveAlgorithm> driveAlgorithm;

    public: //common implementation
        cDemoFilter(const tChar* __info);
        virtual ~cDemoFilter();

    public: // overwrites cFilter //implements IPinEventSink
        tResult OnPinEvent(IPin* pSource,
                           tInt nEventCode,
                           tInt nParam1,
                           tInt nParam2,
                           IMediaSample* pMediaSample);

    public: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);

    public:
        tResult Receive();

    private: //private functions
        tResult ProcessInput(IMediaSample* pSample);
        tResult initProperties();
        tResult createVideoInputPin(const tChar *pinName, cVideoPin &pin);
    tResult createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal);
    tResult createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal);

    tResult initVideoStream(void);
    tVoid setBitmapFormat(const tBitmapFormat *format);
    tResult driveToStopLine(void);

    tResult transmitSteeringAngle(const tFloat32 value);
    tResult transmitAcceleration(const tFloat32 value);
    tResult transmitStop(const tFloat32 value);
    tResult transmitF32Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tFloat32 value);

    tResult getWheelTicks(IMediaSample *mediaSample, tFloat32 &value);
    tResult getF32Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tFloat32 &value);

    tResult getManeuver(IMediaSample *mediaSample, cInputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, tUInt16 &value);


};

#endif // _DEMO_EMPTY_FILTER_HEADER_
