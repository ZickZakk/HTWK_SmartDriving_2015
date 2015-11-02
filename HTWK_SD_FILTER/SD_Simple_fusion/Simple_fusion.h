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
#ifndef _SD_Simple_fusion_FILTER_HEADER_
#define _SD_Simple_fusion_FILTER_HEADER_

#define PROP_NAME_IR_LONGE_RANGE_INPUT_PIN_ACTIVE "__IR longe range input pin state"
#define PROP_NAME_IR_SHORT_RANGE_1_INPUT_PIN_ACTIVE "__IR short range 1 input pin state"
#define PROP_NAME_IR_SHORT_RANGE_2_INPUT_PIN_ACTIVE "__IR short range 2 input pin state"
#define PROP_NAME_US_LEFT_INPUT_PIN_ACTIVE "__Ultrasonic left input pin state"
#define PROP_NAME_US_RIGHT_INPUT_PIN_ACTIVE "__Ultrasonic right input pin state"

#define PROP_NAME_IR_LONG_RANGE_THRESHOLD "_Threshold of the ir longe range sensor"
#define PROP_NAME_IR_SHORT_RANGE_THRESHOLD "_Threshold of the ir short sensor"
#define PROP_NAME_US_THRESHOLD "_Threshold of the us sensor"

#define PROP_NAME_MEDIAN "Median used by the median-filter"

#define OID_ADTF_SD_SIMPLE_FUSION "adtf.sd.simpleFusion"

class cSimpleFusion : public cFilter
{
    ADTF_FILTER(OID_ADTF_SD_SIMPLE_FUSION , "SD Simple fusion", adtf::OBJCAT_DataFilter);
			
    private:
		tVoid initProperties();										// todo
        tResult CreateRawCanTestData(const tTimeStamp& tmStreamTime);
		tResult createOutputPin();
		tResult createInputPinIRLongeRange(const tChar*);									
		tResult createInputPinIRShortRange1(const tChar*);			
		tResult createInputPinIRShortRange2(const tChar*);
		tResult createInputPinUSLeft(const tChar*);
		tResult createInputPinUSRight(const tChar*);
		tResult calculatedMedian();
		tTimeStamp m_nLastMSTime;
		tUInt32 sigma_iterations;
		tUInt32 ir_long_iterations;
		tUInt32 ir_short_iterations;
		tResult sendNewValue(tFloat32 value);
		cInputPin m_ir_short_in1; // input pin for signal data
		cInputPin m_ir_short_in2; // input pin for signal data
		cInputPin m_ir_long_in; // input pin for signal data
		cInputPin m_us_left_in; // input pin for us left signal date
		cInputPin m_us_right_in; // input pin for us left signal date
		
		// properties
		tUInt8 stateOfLongeRangeInputPin;
		tUInt8 stateOfShortRangeInputPin1;
		tUInt8 stateOfShortRangeInputPin2;
		tUInt8 stateOfUSLeftInputPin;
		tUInt8 stateOfUSRightInputPin;
		tUInt8 thresholdIRLongRange;
		tUInt8 thresholdIRShortRange;
		tUInt8 thresholdUS;
		tUInt8 lengthOfMedianVector;
		tUInt8 median;
		cObjectPtr<IMediaType> pTypeSignalValue;

    protected:
        cOutputPin m_ir_fusion_out; // output pin for signal data
		
		/*! Coder Descriptor for the input pins*/
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;

    public: // construction
        cSimpleFusion(const tChar* __info);
        virtual ~cSimpleFusion();

        tResult Init(tInitStage eStage, __exception = NULL);
		tResult Start(__exception = NULL);
		tResult Stop(__exception = NULL);
		tResult Shutdown(tInitStage eStage, __exception = NULL);
		tResult OnPinEvent( IPin *pSource, tInt nEventCore, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
		tResult PropertyChanged(const char*);
};

//*************************************************************************************************
#endif // _SD_Simple_fusion_FILTER_HEADER_
