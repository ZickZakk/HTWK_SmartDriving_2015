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
#ifndef _SD_Value_generator_FILTER_HEADER_
#define _SD_Value_generator_FILTER_HEADER_

#define OID_ADTF_SD_Value_generator "adtf.sd.Value_generator"

class cValue_generator : public adtf::cTimeTriggeredFilter
{
    ADTF_FILTER(OID_ADTF_SD_Value_generator , "SD Value Generator", adtf::OBJCAT_DataFilter);
	
	private:
	tUInt32		m_nGenerateRate;
	tFloat32	m_nValue;

	cObjectPtr<IMediaTypeDescription> coderDescriptionLight;
	cObjectPtr<IMediaDescriptionManager> descriptionManager;

	public:
	cValue_generator(const tChar* __info);
    virtual ~cValue_generator();

	tResult slotValue(tFloat32 val);

	protected:
	tResult Init(tInitStage eStage, __exception);

	tResult Cycle(__exception = NULL);

	cOutputPin	m_pin_output_value;

	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutput;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSteeringOutput;
};

//*************************************************************************************************
#endif // _SD_Value_generator_FILTER_HEADER_
