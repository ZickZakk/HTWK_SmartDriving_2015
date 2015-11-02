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

#ifndef _LIGHT_FILTER_H_
#define _LIGHT_FILTER_H_

#ifdef _DEBUG
#define FILTER_NAME "SD LightFilter Debug"
#define OID_ADTF_SD_Light_Filter "adtf.smartdriving.LightFilter.debug"
#else
#define FILTER_NAME "SD LightFilter"
#define OID_ADTF_SD_Light_Filter "adtf.smartdriving.LightFilter"
#endif

class LightFilter : public adtf::cFilter
{
	ADTF_FILTER(OID_ADTF_SD_Light_Filter, FILTER_NAME, adtf::OBJCAT_DataFilter);

    private:
		cObjectPtr<IMediaDescriptionManager> descriptionManager;
		cObjectPtr<IMediaTypeDescription> coderDescriptionInput;
		cObjectPtr<IMediaTypeDescription> coderDescriptionOutput;
		tInt ddlSize;

		// input pin
		cInputPin lightInputPin;
		
		// output pins
		cOutputPin headLightPin;
		cOutputPin breakLightPin;
		cOutputPin turnRightPin;
		cOutputPin turnLeftPin;
		cOutputPin reverseLightPin;
		cOutputPin hazardLightPin;

		// variables
		tBool headLight;
		tBool breakLight;
		tBool turnRightLight;
		tBool turnLeftLight;
		tBool reverseLight;
		tBool hazardLight;

    public: 
        LightFilter(const tChar* __info);
        virtual ~LightFilter();

        tResult Init(tInitStage eStage, __exception);
		tResult Start(__exception);
		tResult Stop(__exception);
		tResult Shutdown(tInitStage eStage, __exception);
		tResult OnPinEvent(IPin *source, tInt nEventCore, tInt nParam1, tInt nParam2, IMediaSample *mediaSample);

private:
	tResult initMediaType(const char *mediaTypeDescriptionName, cObjectPtr<IMediaType> &mediaType, cObjectPtr<IMediaTypeDescription> &coderDescription);
	tResult createInputPin(cInputPin &pin, const char *pinName, cObjectPtr<IMediaType> &mediaType);
	tResult createOutputPin(cOutputPin &pin, const char *pinName, cObjectPtr<IMediaType> &mediaType);

	tResult getInputValue(IMediaSample *mediaSample, Light &lightValue);
	tVoid getLightFromIntValue(const tInt16 &value, Light &lightValue);

	tResult transmitLightValue(const Light &light);
	tResult transmit(cOutputPin &pin, const tBool &value);
};

#endif
