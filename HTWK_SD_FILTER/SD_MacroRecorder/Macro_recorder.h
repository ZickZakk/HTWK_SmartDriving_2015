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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#ifndef _SD_Macro_recorder_FILTER_HEADER_
#define _SD_Macro_recorder_FILTER_HEADER_

#define OID_ADTF_SD_Macro_recorder "adtf.sd.macroRecorder"
#define _USE_MATH_DEFINES


//Property definitions

class cMacro_recorder : public adtf::cFilter
{
	ADTF_FILTER(OID_ADTF_SD_Macro_recorder, "SD Macro Recorder", adtf::OBJCAT_DataFilter);

    private:
		//functions
		tResult sendNewValue(cOutputPin * outpin, tFloat32 value, tTimeStamp timeStamp);
		void writeToFile(std::string source, tFloat32 R, tFloat32 L, tFloat32 S);

		//variables
		tTimeStamp m_nLastMSTime;
		tFloat32 R;
		tFloat32 L;
		tFloat32 S;
		
		//Properties
		
    protected:
		//input pins
		cInputPin  m_oIWheelL; // input pin for signal data
		cInputPin  m_oIWheelR; // input pin for signal data

		cInputPin  m_oIServo; // input pin for signal data

		//Coder description for input pins
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;


    public: // construction
        cMacro_recorder(const tChar* __info);
        virtual ~cMacro_recorder();

        tResult Init(tInitStage eStage, __exception);
		tResult Start(__exception);
		tResult Stop(__exception);
		tResult Shutdown(tInitStage eStage, __exception);
		tResult OnPinEvent( IPin *pSource, tInt nEventCore, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
};

//*************************************************************************************************
#endif // _SD_Macro_recorder_FILTER_HEADER_
