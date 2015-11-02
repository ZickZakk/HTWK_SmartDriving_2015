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

#include "World_object.h"
#include "ObstacleLocation.h"
#include "SmartDrivingUtilities.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#ifndef _SD_World_model_FILTER_HEADER_
#define _SD_World_model_FILTER_HEADER_

#ifdef _DEBUG
#define FILTER_NAME "SD World model Debug"
#define OID_ADTF_SD_WORLD_MODEL "adtf.sd.worldModel.debug"
#else
#define FILTER_NAME "SD World model"
#define OID_ADTF_SD_WORLD_MODEL "adtf.sd.worldModel.release"
#endif

#define _USE_MATH_DEFINES

#define PROPERTY_MOCKUP_PATH "worldModel_MockupPath"
#define PROPERTY_AUTOSAVE_PATH "worldModel_autosavePath"

#define PROPERTY_CAMERA_RANGE_TOP_MID "worldModel_Camera_rangeTopMid"
#define PROPERTY_CAMERA_HEIGHT_AS_RANGE "worldModel_Camera_HeightAsRange"
#define PROPERTY_CAMERA_WIDTH_AS_RANGE "worldModel_Camera_widthAsRange"
#define PROPERTY_CAMERA_WIDTH_POSITION "worldModel_Camera_widthPosition"


//Property definitions

class cWorld_model : public adtf::cFilter
{
	ADTF_FILTER(OID_ADTF_SD_WORLD_MODEL, FILTER_NAME, adtf::OBJCAT_DataFilter);

    private:
		//functions
        tResult CreateRawCanTestData(const tTimeStamp& tmStreamTime);
		tResult sendNewValue(cOutputPin * outpin, tFloat32 value, tTimeStamp timeStamp);
		void paintWorldModelToWindow(tUInt32 timestamp);
		void addVectorToMap(cWorld_object * sensor, std::vector<smartdriving::SD_Point> vec);
		void relocateCarPosition(cWorld_object * sensor, std::vector<smartdriving::SD_Point> vec);
		void addObjectToVector(cWorld_object * sensor, tFloat32 * signalValue, tUInt32 * timestamp);
		void drawSensor (cv::Mat * image, cWorld_object * sensor);
		void carMoveAClick(int sensor,tFloat32 * signalValue);
		// OLD PATH PLANNING ALGO
		/*void doPathPlaning(tUInt32 timestamp);
		int checkLineX(int x, int y, int direction);
		int checkLineY(int x, int y, int direction);
		cv::Point checkWalkX(int x, int y, int direction, int half_lane_width);
		cv::Point checkWalkY(int x, int y, int direction, int half_lane_width);*/
		float angleBetween(const cv::Point &v1, const cv::Point &v2);
		tResult doStamping(tUInt32 timestamp);


		//variables
		tTimeStamp m_nLastMSTime;
		tTimeStamp m_nLastPaintTime;
		cWorld_object * my_pos; // the position of the car inside the worldmodel
		cv::Mat			my_world;
		cv::Mat			image;
		
		std::vector<cWorld_object> staticObjects;
		std::vector<cWorld_object> dynamicObjects;
		std::vector<cv::Point> pathPoints;
		// TODO: implement gateways !! : D std::vector<> gateways;

		cWorld_object * my_front_ir;
		cWorld_object * my_front_usl;
		cWorld_object * my_front_usr;
		cWorld_object * my_rear_ir;
		cWorld_object * my_rear_usl;
		cWorld_object * my_rear_usr;
		cWorld_object * my_front_lir;
		cWorld_object * my_front_rir;
		cWorld_object * my_rear_lir;
		cWorld_object * my_rear_rir;
		cWorld_object * my_camera;

		tFloat32 old_wheelL;
		tFloat32 old_wheelR;
		tFloat32 old_wheels;

		//Properties
		tInt maxWidthMultipliedWith2;
		tInt maxHeightMultipliedWith2;
		tInt showroomx;
		tInt showroomy;
		tInt showroomsizex;
		tInt showroomsizey;
		tBool showroomactive;
		tInt object_outtime_in_seconds;
		tBool objects_should_die;
		string mockup_path;
		string autosave_path;
		
		tFloat32 line_detection_border;
		tInt32 line_detection_radius;
		tInt32 line_detection_max_route;
		tInt32 line_detection_stepsize;
		tInt32 line_detection_lane_width;
		tFloat32 camera_range_top_mid; // distance to top mid  
		tFloat32 camera_height_as_range; //distance in cm from top to bottom
		tFloat32 camera_width_as_range; //distance in cm from left to right
		tFloat32 camera_width_position; //the car position in relation to the camera picture (should be between 0 and 1)

		tBool showroomautofollow;

		tFloat32 wheel_click_length;

		tInitStage currentStage;

    protected:
		//output pins
        cOutputPin m_oOAcceleration; // output pin for signal data

		//input pins
		cInputPin m_oIObstacleInput; //Input from another Filter
		cInputPin m_oIIPMInput; //Input from another Filter

		cInputPin  m_oIFrontIRF; // input pin for signal data
		cInputPin  m_oIFrontUSL; // input pin for signal data
		cInputPin  m_oIFrontUSR; // input pin for signal data
		
		cInputPin  m_oIWheelL; // input pin for signal data
		cInputPin  m_oIWheelR; // input pin for signal data


		//Coder description for input pins
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;


    public: // construction
        cWorld_model(const tChar* __info);
        virtual ~cWorld_model();

        tResult Init(tInitStage eStage, __exception);
		tResult Start(__exception);
		tResult Stop(__exception);
		tResult Shutdown(tInitStage eStage, __exception);
		tResult OnPinEvent( IPin *pSource, tInt nEventCore, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
        
};

//*************************************************************************************************
#endif // _SD_World_model_FILTER_HEADER_
