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
 * @author akluge
 * @details
 */
#ifndef _SD_Drive_by_list_FILTER_HEADER_
#define _SD_Drive_by_list_FILTER_HEADER_

#define OID_ADTF_SD_Drive_by_list "adtf.sd.Drive_by_list"

class cDrive_by_list : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_SD_Drive_by_list , "SD Drive_by_list", adtf::OBJCAT_DataFilter);

	private:
		
		tBool m_nFirstMove;

		struct tComandList
		{
			tFloat32 acc;
			tFloat32 angle;
			tInt rotation;
		}cmdElement;

		std::list<tComandList> list;
		tComandList curCmdElement;

		tFloat32 distance;
		
		tInt m_nInitWaitCounter;	//counter for initstep 
		tInt m_nWaitValue;			//WaitingValue for finishing Car Init >200+

	protected:    
		cInputPin  m_pin_input_wheelRotation; // input pin for signal data
		cInputPin  m_pin_input_init;

		cOutputPin  m_pin_output_acceleration; // output pin for signal data
		cOutputPin	m_pin_output_steeringAngle;

		//Coder description for input pins
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;
		
		tResult sendNewValue(cOutputPin * outpin, tFloat32 value, tTimeStamp timeStamp);
		tResult getNextCmd();
		void addListElement(tFloat32 accel, tFloat32 angle, tInt Rota);
	public:
		cDrive_by_list(const tChar* __info);
		virtual ~cDrive_by_list();

		tResult Init(tInitStage eStage, __exception);
		tResult Start(__exception);
		tResult Stop(__exception);
		tResult Shutdown(tInitStage eStage, __exception);
		tResult OnPinEvent( IPin *pSource, tInt nEventCore, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
};

//*************************************************************************************************
#endif 
