/**
 *
 * ADTF Empty Filter Demo.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: ELAMIHA $
 * $Date: 2014-03-27 16:40:09 +0100 (Thu, 27 Mar 2014) $
 * $Revision: 45736 $
 *
 * @remarks             This example shows how to implement a common adtf 
 *                      filter for processing data.
 *
 */
#include "stdafx.h"
#include "./demoudpfilter.h"


ADTF_FILTER_PLUGIN("Demo UDP Filter Plugin", OID_ADTF_DEMO_UDP, cDemoFilter)

/**
 * Constructor.
 * @param pParent The parent filter
 */
cDemoFilter::cReceiveThread::cReceiveThread():
        cKernelThread(),
        m_pParent(NULL)
{
}

tResult cDemoFilter::cReceiveThread::SetParent(cDemoFilter* pParent)
{
    m_pParent = pParent;
    RETURN_NOERROR;
}

/**
 * The thread function which will be called in a loop.
 * @return Standard Result Code.
 */
tResult cDemoFilter::cReceiveThread::ThreadFunc()
{
    RETURN_IF_POINTER_NULL(m_pParent);
    return m_pParent->Receive();
}

/**
 *   Contructor. The cFilter contructor needs to be called !!
 *   The SetProperty in the constructor is necessary if somebody wants to deal with 
 *   the default values of the properties before init 
 *
 */
cDemoFilter::cDemoFilter(const tChar* __info) : cFilter(__info)
{
    SetPropertyStr("DestHostname", "localhost");
    SetPropertyInt("DestPort", 3333);
    SetPropertyInt("ReceivePort", 3333);

    this->ip = "192.168.1.253";
    SetPropertyStr(PROP_NAME_IP, this->ip.c_str());
    SetPropertyBool(PROP_NAME_IP NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool(PROP_NAME_IP NSSUBPROP_ISCHANGEABLE, tTrue);

    this->numberOfStopLines = 20;
    SetPropertyInt(PROP_NAME_STOP_LINES, this->numberOfStopLines);
    SetPropertyInt(PROP_NAME_STOP_LINES NSSUBPROP_MIN, 1);
    SetPropertyInt(PROP_NAME_STOP_LINES NSSUBPROP_MAX, 1000);
    SetPropertyBool(PROP_NAME_STOP_LINES NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool(PROP_NAME_STOP_LINES NSSUBPROP_ISCHANGEABLE, tTrue);

    this->delay = 100000;
    SetPropertyInt(PROP_NAME_DELAY, this->delay);
    SetPropertyInt(PROP_NAME_DELAY NSSUBPROP_MIN, 1);
    SetPropertyInt(PROP_NAME_DELAY NSSUBPROP_MAX, 1000);
    SetPropertyBool(PROP_NAME_DELAY NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool(PROP_NAME_DELAY NSSUBPROP_ISCHANGEABLE, tTrue);

    this->driveSpeed = 30;
    SetPropertyInt(PROP_NAME_DRIVE_SPEED, this->driveSpeed);
    SetPropertyInt(PROP_NAME_DRIVE_SPEED NSSUBPROP_MIN, 1);
    SetPropertyInt(PROP_NAME_DRIVE_SPEED NSSUBPROP_MAX, 1000);
    SetPropertyBool(PROP_NAME_DRIVE_SPEED NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool(PROP_NAME_DRIVE_SPEED NSSUBPROP_ISCHANGEABLE, tTrue);

    this->smoothCurveValue = 7;
    SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE, this->smoothCurveValue);
    SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_MIN, 1);
    SetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_MAX, 1000);
    SetPropertyBool(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool(PROP_NAME_SMOOTH_CURVE_VALUE NSSUBPROP_ISCHANGEABLE, tTrue);

    this->ticksToStopLine = 10;
    SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE, this->ticksToStopLine);
    SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_MIN, 1);
    SetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_MAX, 1000);
    SetPropertyBool(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool(PROP_NAME_TICKS_TO_STOP_LINE NSSUBPROP_ISCHANGEABLE, tTrue);
}

/**
 *  Destructor. A Filter implementation needs always to have a virtual Destructor
 *              because the IFilter extends an IObject
 *
 */
cDemoFilter::~cDemoFilter()
{
}

tResult cDemoFilter::initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription)
{
    tChar const *descriptionSignalValue = this->descriptionManager->GetMediaDescription(mediaTypeDescriptionName);
    RETURN_IF_POINTER_NULL(descriptionSignalValue);        

    mediaType = new cMediaType(0, 0, 0, mediaTypeDescriptionName, descriptionSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);  
    RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**) &coderDescription));

    RETURN_NOERROR;
}

tResult cDemoFilter::createVideoInputPin(const tChar *pinName, cVideoPin &pin)
{
    pin.Create(pinName, IPin::PD_Input, static_cast<IPinEventSink*>(this));
    RETURN_IF_FAILED(RegisterPin(&pin));
    
    RETURN_NOERROR;
}

tResult cDemoFilter::createInputPin(const char *pinName, cInputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
    RETURN_IF_FAILED(pin.Create(pinName, typeSignal, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&pin));

    RETURN_NOERROR;
}

tResult cDemoFilter::createOutputPin(const char *pinName, cOutputPin &pin, cObjectPtr<IMediaType> &typeSignal)
{
    RETURN_IF_FAILED(pin.Create(pinName, typeSignal));
    RETURN_IF_FAILED(RegisterPin(&pin));

    RETURN_NOERROR;
}

/**
 *   The Filter Init Function. 
 *    eInitStage ... StageFirst ... should be used for creating and registering Pins 
 *               ... StageNormal .. should be used for reading the properies and initalizing 
 *                                  everything before pin connections are made 
 *   see {@link IFilter#Init IFilter::Init}.
 *
 */
tResult cDemoFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
        /*
        * the MediaDescription for <struct name="tNestedStruct" .../> has to exist in a description file (e.g. in $ADTF_DIR\description\ or $ADTF_DIR\src\examples\src\description
        * before (!) you start adtf_devenv !! if not: the Filter-Plugin will not loaded because cPin.Create() and so ::Init() failes !
        */
        tChar const * strDesc = pDescManager->GetMediaDescription("tNestedStruct");
        RETURN_IF_POINTER_NULL(strDesc);
        cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tNestedStruct", strDesc,  IMediaDescription::MDF_DDL020000);

        // register the input pin with the type "tNestedStruct", which has to be defined in a Media Description File
        RETURN_IF_FAILED(m_oInput.Create("input", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInput));
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDesc));

        // register the output pin with the same media type as the input pin
        RETURN_IF_FAILED(m_oOutput.Create("output", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutput));
        //RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDesc));

        cObjectPtr<IMediaType> typeSignalManeuver;
        cObjectPtr<IMediaType> typeSignalSteeringAngle;
        cObjectPtr<IMediaType> typeSignalAcceleration;
        cObjectPtr<IMediaType> typeSignalWheelTicks;

        RETURN_IF_FAILED(initMediaType("tSteeringAngleData", typeSignalManeuver, this->coderDescriptionManeuver));
        RETURN_IF_FAILED(initMediaType("tSignalValue", typeSignalSteeringAngle, this->coderDescriptionSteeringAngle));
        RETURN_IF_FAILED(initMediaType("tSignalValue", typeSignalAcceleration, this->coderDescriptionAcceleration));
        RETURN_IF_FAILED(initMediaType("tSignalValue", typeSignalWheelTicks, this->coderDescriptionWheelTicks));

        // input pins
        RETURN_IF_FAILED(createVideoInputPin("rgbVideo", this->xtionPin));
        RETURN_IF_FAILED(createInputPin("maneuver", this->maneuverPin, typeSignalManeuver));
        RETURN_IF_FAILED(createInputPin("wheelTicks", this->wheelTicksPin, typeSignalWheelTicks));

        // output pins
        RETURN_IF_FAILED(createOutputPin("steeringAngle", this->steeringAnglePin, typeSignalSteeringAngle));
        RETURN_IF_FAILED(createOutputPin("acceleration", this->accelerationPin, typeSignalAcceleration));
    }
    else if (eStage == StageNormal)
    {
        cObjectPtr<IMediaSerializer> serializer;
        RETURN_IF_FAILED(this->coderDescriptionAcceleration->GetMediaSampleSerializer(&serializer));
        this->ddlSizeUI16 = serializer->GetDeserializedSize();

        std::thread test(&Test::accept, this);
        m_pBuffer = new tChar[MAX_PACKET_SIZE];
        m_strDest = GetPropertyStr("DestHostname");
        m_nPort = GetPropertyInt("DestPort");
        tInt nLocalPort = GetPropertyInt("ReceivePort");
        RETURN_IF_FAILED(m_oSendSocket.Open(0, 0));
        RETURN_IF_FAILED(m_oRecvSocket.Open(nLocalPort, 0));
        if (!m_oRecvSocket.SetTimeout(100000))
        {
            RETURN_ERROR(ERR_UNEXPECTED);
        }
        m_oReceiveThread.SetParent(this);
        m_oReceiveThread.Create();
    }
    RETURN_NOERROR;
}

/**
 *   The Filters Start Function. see {@link IFilter#Start IFilter::Start}.
 *
 */
tResult cDemoFilter::Start(__exception)
{
    m_oReceiveThread.Run();
    return cFilter::Start(__exception_ptr);
}

/**
 *   The Filters Stop Function. see {@link IFilter#Stop IFilter::Stop}.
 *
 */
tResult cDemoFilter::Stop(__exception)
{
    m_oReceiveThread.Suspend();
    return cFilter::Stop(__exception_ptr);
}

/**
 *   The Filters Shutdown Function. see {@link IFilter#Shutdown IFilter::Shutdown}.
 *
 */
tResult cDemoFilter::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageNormal)
    {
        m_oReceiveThread.Release();
        m_oSendSocket.Close();
        m_oRecvSocket.Close();

        delete [] m_pBuffer;
        m_pBuffer = NULL;
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}


/**
 *   The Filters Pin Event Implementation. see {@link IPinEventSink#OnPinEvent IPinEventSink::OnPinEvent}.
 *   Here the receiving Pin (cInputPin) will call the OnPinEvent.
 *
 */
tResult cDemoFilter::OnPinEvent(IPin* source,
                                tInt eventCode,
                                tInt param1,
                                tInt param2,
                                IMediaSample* mediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oInput)
        {
            ProcessInput(pMediaSample);
        }
        else
        {
            RETURN_ERROR(ERR_NOT_SUPPORTED);
        }
    }
    RETURN_NOERROR;

    RETURN_IF_POINTER_NULL(source);
    RETURN_IF_POINTER_NULL(mediaSample);

    if (eventCore == IPinEventSink::PE_MediaSampleReceived)
    {
        if (source == &this->maneuverPin)
        {
            static tUInt16 tmpManeuver;
            getManeuver(mediaSample, maneuverPin, this->coderDescriptionManeuver, tmpManeuver);

            if (tmpManeuver == MANEUVER_STRAIGHT)
            {
                this->isDriveActive = true;

                RETURN_IF_FAILED_AND_LOG_ERROR_STR(transmitAcceleration(this->driveSpeed), "Cant transmit drive");
            }
        }
        else if (source == &this->wheelTicksPin)
        {
            if (!this->isConnectedToServer)
            {
                 // prepare output stream (buffer)
                cStream oStream(cStream::OUTPUT_STREAM);
                pSerializable->Serialize(&oStream);
                oStream.Seek(0, IStream::SO_Begin);
                m_oSendSocket.Write(m_strDest.GetPtr(), m_nPort, oStream.GetCurrentPtr(), (tInt)oStream.GetCurrentSize());
            }
            
            static tFloat32 tmpWheelTicks;
            RETURN_IF_FAILED_AND_LOG_ERROR_STR(getWheelTicks(mediaSample, tmpWheelTicks), "cant get wheel ticks");

            this->currentWheelTicks = static_cast<int>(tmpWheelTicks);

            if (this->isStopLineFound)
            {
                driveToStopLine();
            }
        }
        else if(source == &this->xtionPin)
        {
            if (this->isFirstFrame)
            {
                RETURN_IF_FAILED_AND_LOG_ERROR_STR(initVideoStream(), "Cant init video stream");
                this->isFirstFrame = false;
            }
            else 
            {
                const tVoid *buffer;

                if (this->isDriveActive && IS_OK(mediaSample->Lock(&buffer)))
                {
                    //Receive the image
                    Mat image(Size(this->videoInputInfo.nWidth, this->videoInputInfo.nHeight), CV_8UC3, (char*) buffer);
                    Mat result = image.clone();
                    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaSample->Unlock(buffer), "Cant unlock image");

                    this->driveAlgorithm->prepareImage(result);

                    if (!this->isStopLineFound)
                    {
                        this->isStopLineFound = this->crossroadDetector-> searchStopLine(result);
                        this->ticksToDrive = this->ticksToStopLine + this->currentWheelTicks;
                    }
                }
            }
        }
    }
    
    RETURN_NOERROR;
}

/**
 * Processes on mediasample. The method only shows an example, how to deal with
 * a mediasample that is received, to get the specific data.
 *
 * @param   pSample   [in]  The received sample.
 *
 * @return  Standard result code.
 */
tResult cDemoFilter::ProcessInput(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDesc->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    if (pSample->GetSize() == nSize) 
    {
        cObjectPtr<ISerializable> pSerializable;
        if (IS_OK(pSample->GetInterface(IID_SERIALIZABLE, (tVoid**) &pSerializable)))
        {
             // prepare output stream (buffer)
            cStream oStream(cStream::OUTPUT_STREAM);
            pSerializable->Serialize(&oStream);
            oStream.Seek(0, IStream::SO_Begin);
            m_oSendSocket.Write(m_strDest.GetPtr(), m_nPort, oStream.GetCurrentPtr(), (tInt)oStream.GetCurrentSize());
        }
    }

    RETURN_NOERROR;
}

/**
 * This function gets called by the thread function.
 * \note Read must have a timeout, otherwise the thread would block forever.
 * @return Standard Result Code
 */
tResult cDemoFilter::Receive()
{
    tInt nBytesReceived;
    // one read removes one packet from the receive queue (no matter of the size specified)
    if (IS_OK(m_oRecvSocket.Read(m_pBuffer, MAX_PACKET_SIZE, &nBytesReceived)))
    {
        cObjectPtr<IMediaSample> pNewSample;
        if (IS_OK(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample)))
        {
            cObjectPtr<ISerializable> pSerializable;
            if (IS_OK(pNewSample->GetInterface(IID_SERIALIZABLE, (tVoid**) &pSerializable)))
            {
                cStream oStream(cStream::INPUT_STREAM);
                oStream.AttachReference(m_pBuffer, nBytesReceived);
                if (IS_OK(pSerializable->Deserialize(&oStream)))
                {
                    m_oOutput.Transmit(pNewSample);
                }
                oStream.DetachReference();
            }
        }
    }

    RETURN_NOERROR;
}

tResult cDemoFilter::initVideoStream(void)
{
    //Read media type
    cObjectPtr<IMediaType> type;
    RETURN_IF_FAILED(this->xtionPin.GetMediaType(&type));

    cObjectPtr<IMediaTypeVideo> typeVideo;
    RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)(&typeVideo)));

    const tBitmapFormat *format = typeVideo->GetFormat();
    RETURN_IF_POINTER_NULL(format);

    //Set media type
    setBitmapFormat(format);

    RETURN_NOERROR;
}

tVoid cDemoFilter::setBitmapFormat(const tBitmapFormat *format)
{
    this->videoInputInfo.nBitsPerPixel = format->nBitsPerPixel;
    this->videoInputInfo.nBytesPerLine = format->nBytesPerLine;
    this->videoInputInfo.nPaletteSize = format->nPaletteSize;
    this->videoInputInfo.nPixelFormat = format->nPixelFormat;
    this->videoInputInfo.nHeight = format->nHeight;
    this->videoInputInfo.nWidth = format->nWidth;
    this->videoInputInfo.nSize = format->nSize;
}

tResult cDemoFilter::driveToStopLine(void)
{
    static tInt tmpDistance;
    tmpDistance = this->ticksToDrive - this->currentWheelTicks;
    
    if (tmpDistance <= 0)
    {
        if (this->stopOnStopLine)
        {
            RETURN_IF_FAILED(transmitAcceleration(-5.0f));

            // sende zum anderen Auto, dass ich warte
        }
        else
        {
            // sende zum anderen Auto
            this->numberOfStopLines--;

            // sende...
            LOG_INFO(cString::Format("StopLines: %d", this->numberOfStopLines));

            this->isStopLineFound = false;
        }
    }

    RETURN_NOERROR;
}

tResult cDemoFilter::transmitSteeringAngle(const tFloat32 value)
{
    RETURN_IF_FAILED(transmitF32Value(this->steeringAnglePin, this->coderDescriptionSteeringAngle, value));
    RETURN_NOERROR;
}

tResult cDemoFilter::transmitAcceleration(const tFloat32 value)
{
    RETURN_IF_FAILED(transmitF32Value(this->accelerationPin, this->coderDescriptionAcceleration, value));
    RETURN_NOERROR;
}

tResult cDemoFilter::transmitStop(const tFloat32 value)
{
    RETURN_IF_FAILED(transmitF32Value(this->accelerationPin, this->coderDescriptionAcceleration, value));
    RETURN_NOERROR;
}

tResult cDemoFilter::transmitF32Value(cOutputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, const tFloat32 value)
{
    cObjectPtr<IMediaSample> mediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**) &mediaSample));
    RETURN_IF_FAILED(mediaSample->AllocBuffer(this->ddlSizeUI16));
       
    // write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> coder;
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->WriteLock(mediaSample, &coder), "Set F32 Failed to lock f32");    
        
    static tTimeStamp now;
    now = _clock ? _clock->GetStreamTime() : cHighResTimer::GetTime();

    coder->Set("f32Value", (tVoid*) &value);
    coder->Set("ui32ArduinoTimestamp", (tVoid*) &now);
    
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Set F32 Failed to lock f32");
    
    // transmit media sample over output pin
    RETURN_IF_FAILED(mediaSample->SetTime(now));
    RETURN_IF_FAILED(pin.Transmit(mediaSample));
    RETURN_NOERROR;
}

tResult cDemoFilter::getWheelTicks(IMediaSample *mediaSample, tFloat32 &value)
{
    RETURN_IF_FAILED(getF32Value(mediaSample, this->coderDescriptionWheelTicks, value));
    RETURN_NOERROR;
}

tResult cDemoFilter::getF32Value(IMediaSample *mediaSample, cObjectPtr<IMediaTypeDescription> &mediaType, tFloat32 &value)
{
    static tFloat32 tmpValue;
    static tTimeStamp timeStamp;

    cObjectPtr<IMediaCoder> coder;
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Lock(mediaSample, &coder), "Get32 Failed to lock f32");
            
    coder->Get("f32Value", (tVoid*) &tmpValue);
    coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
    value = tmpValue;

    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Get32 Failed to unlock f32");

    RETURN_NOERROR;
}

tResult cDemoFilter::getManeuver(IMediaSample *mediaSample, cInputPin &pin, cObjectPtr<IMediaTypeDescription> &mediaType, tUInt16 &value)
{
    static tUInt16 tmpValue;
    static tUInt32 timeStamp;

    cObjectPtr<IMediaCoder> coder;
    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Lock(mediaSample, &coder), "Get UI16 failed to unlock");
    
    coder->Get("ui16Angle", (tVoid*) &tmpValue);
    coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
    value = tmpValue;

    RETURN_IF_FAILED_AND_LOG_ERROR_STR(mediaType->Unlock(coder), "Get UI16 failed to unlock");

    RETURN_NOERROR;
}

tResult cDemoFilter::PropertyChanged(const char *name)
{
    this->ip = GetPropertyStr(PROP_NAME_IP);
    this->numberOfStopLines = GetPropertyInt(PROP_NAME_STOP_LINES);
    this->delay = GetPropertyInt(PROP_NAME_DELAY);
    this->driveSpeed = GetPropertyInt(PROP_NAME_DRIVE_SPEED);
    this->smoothCurveValue = GetPropertyInt(PROP_NAME_SMOOTH_CURVE_VALUE);
    this->ticksToStopLine = GetPropertyInt(PROP_NAME_TICKS_TO_STOP_LINE);

    RETURN_NOERROR;
}
