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
 * @copyright (c) Copyright 2014 SD-Team HTWK
 * @author epeterse
 * @details
 */

#include "stdafx.h"
#include "World_model.h"
#include "World_object.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>


/// Create filter
ADTF_FILTER_PLUGIN(FILTER_NAME, OID_ADTF_SD_WORLD_MODEL, cWorld_model);


cWorld_model::cWorld_model(const tChar* __info) : adtf::cFilter(__info)
{
	m_nLastMSTime = 0;
    
	//Propertys auslesen!!! Fehlt noch ^.^ Also hier das Mockup
	maxWidthMultipliedWith2 = (15 /*Max Größe des Feldes*/ + 1 /*sicherheit*/) * 2 /*wir wissen nicht wo wir stehen*/ * 100; //in cm
	maxHeightMultipliedWith2 = (15 + 2) * 2 * 100; //in cm
	showroomx = 1449;
	showroomy = 1500;
	showroomsizex = 602;
	showroomsizey = 602;
	showroomactive = true;
	showroomautofollow = true;

	object_outtime_in_seconds = 5;
	objects_should_die = true; //after oot

	line_detection_border = 2.55 * 70;
	line_detection_radius = 100;
	line_detection_lane_width = 50;
	line_detection_max_route = 10;
	line_detection_stepsize = 20;
	camera_range_top_mid = -50.0; // distance to top mid  
	camera_height_as_range = 250.0; //distance in cm from top to bottom
	camera_width_as_range = 120.0; //distance in cm from left to right
	camera_width_position = 0.25;

	old_wheels = 0.0;
	old_wheelR = 0.0;
	old_wheelL = 0.0;

	//Initalisieren der Sensorwerte
	this->my_front_ir  = new cWorld_object( 0,     30,    20, 0);
	this->my_front_usl = new cWorld_object(-5.5,   30,    20, 0);
	this->my_front_usr = new cWorld_object( 5.5,   30,    20, 0);
	this->my_rear_ir   = new cWorld_object( 0,    -30,    5, 180);
	this->my_rear_usl  = new cWorld_object(-5.5,  -30,    10, 180);
	this->my_rear_usr  = new cWorld_object( 5.5,  -30,    10, 180);
	this->my_front_lir = new cWorld_object(-12.5,  29,    5, 270);
	this->my_front_rir = new cWorld_object( 12.5,  29,    5, 90);
	this->my_rear_lir  = new cWorld_object(-13.5, -11.5,  5, 270);
	this->my_rear_rir  = new cWorld_object( 13.5, -11.5,  5, 90);
	this->my_camera    = new cWorld_object( 0,     10.5,  1, 0);

	wheel_click_length = 3.65f; // verkürzter click gegen fehler mit angleichung (3.65)

	this->mockup_path = "";//"D:\\C\\nonToolkit\\UNI\\AADC\\main\\HTWK_SD_FILTER\\SD_World_model\\4m_gerade.bmp";
	this->autosave_path = "";

	currentStage = StageFirst;
	
	SetPropertyStr(PROPERTY_MOCKUP_PATH, this->mockup_path.c_str());
	SetPropertyStr(PROPERTY_AUTOSAVE_PATH, this->autosave_path.c_str());

	SetPropertyFloat(PROPERTY_CAMERA_RANGE_TOP_MID, this->camera_range_top_mid);
	SetPropertyFloat(PROPERTY_CAMERA_HEIGHT_AS_RANGE, this->camera_height_as_range);
	SetPropertyFloat(PROPERTY_CAMERA_WIDTH_AS_RANGE, this->camera_width_as_range);
	SetPropertyFloat(PROPERTY_CAMERA_WIDTH_POSITION, this->camera_width_position);
}

cWorld_model::~cWorld_model()
{
	delete this->my_front_ir;
	this->my_front_ir = NULL;

	delete this->my_front_usl;
	this->my_front_usl = NULL;

	delete this->my_front_usr;
	this->my_front_usr = NULL;

	delete this->my_rear_ir;
	this->my_rear_ir = NULL;

	delete this->my_rear_usl;
	this->my_rear_usl = NULL;

	delete this->my_rear_usr;
	this->my_rear_usr = NULL;

	delete this->my_front_lir;
	this->my_front_lir = NULL;

	delete this->my_front_rir;
	this->my_front_rir = NULL;

	delete this->my_rear_lir;
	this->my_rear_lir = NULL;

	delete this->my_rear_rir;
	this->my_rear_rir = NULL;

	delete this->my_camera;
	this->my_camera = NULL;
}

tResult cWorld_model::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		cObjectPtr<IMediaType> pTypeStruct;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);

		// Media type definition
        cObjectPtr<IMediaType> pTypeSignal = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(AllocMediaType((tVoid**)&pTypeStruct, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED, NULL, NULL));

        // Create the output pins
        RETURN_IF_FAILED(m_oOAcceleration.Create("acceleration_out", pTypeSignal, NULL));
        RETURN_IF_FAILED(RegisterPin(&m_oOAcceleration));

		// Media type definition
        //cObjectPtr<IMediaType> pTypeIn = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput)); 

		//Create the input pins
		//von anderen Filtern
		RETURN_IF_FAILED(m_oIObstacleInput.Create("obstacle_location_input", pTypeStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIObstacleInput));

		RETURN_IF_FAILED(m_oIIPMInput.Create("ipm_input", pTypeStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIIPMInput));

		//front
		RETURN_IF_FAILED(m_oIFrontIRF.Create("front_ir_fusion", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIFrontIRF));
		
		RETURN_IF_FAILED(m_oIFrontUSL.Create("front_usl", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIFrontUSL));
		
		RETURN_IF_FAILED(m_oIFrontUSR.Create("front_usr", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIFrontUSR));
		
		RETURN_IF_FAILED(m_oIWheelL.Create("wheel_left", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIWheelL));

		RETURN_IF_FAILED(m_oIWheelR.Create("wheel_right", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIWheelR));

		/*
		RETURN_IF_FAILED(m_oIFrontIRF.Create("front_uss_left", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIFrontIRF));
		
		RETURN_IF_FAILED(m_oIFrontIRF.Create("front_uss_right", pTypeSignal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIFrontIRF));*/

		this->mockup_path = string(GetPropertyStr(PROPERTY_MOCKUP_PATH));
		this->autosave_path = string(GetPropertyStr(PROPERTY_AUTOSAVE_PATH));

		this->camera_height_as_range = GetPropertyFloat(PROPERTY_CAMERA_HEIGHT_AS_RANGE);
		this->camera_width_as_range = GetPropertyFloat(PROPERTY_CAMERA_WIDTH_AS_RANGE);
		this->camera_range_top_mid = GetPropertyFloat(PROPERTY_CAMERA_RANGE_TOP_MID);
		this->camera_width_position = GetPropertyFloat(PROPERTY_CAMERA_WIDTH_POSITION);
		
    }
    else if (eStage == StageNormal)
    {
		image = cv::Mat::zeros( showroomsizex, showroomsizey, CV_8UC3 );
		//Auto Startposition
		this->my_pos = new cWorld_object(tFloat32(((maxWidthMultipliedWith2 / 2)) - 30),tFloat32(((maxWidthMultipliedWith2 / 2-1)) - 20));
		this->my_pos->setRotation(0);
		my_world = cv::Mat::zeros( maxWidthMultipliedWith2, maxHeightMultipliedWith2, CV_8UC3 );
		if (mockup_path.length() > 0) {
			this->my_world = cv::imread(mockup_path, 1);
		}
		staticObjects.clear();
		dynamicObjects.clear();
		old_wheelR = 0;
		old_wheelL = 0;
		currentStage = StageNormal;
    }
    else if (eStage == StageGraphReady)
    {
		//TESTFUNKTION FÜR DIE NUTZUNG VON OPEN CV
        cv::namedWindow(cv::String("World_model_window"), cv::WINDOW_NORMAL);
		cv::startWindowThread();
		currentStage = StageGraphReady;
    }

    RETURN_NOERROR;
}

tResult cWorld_model::Start(__exception) {
	RETURN_IF_FAILED(cFilter::Start( __exception_ptr));
	m_nLastMSTime = 0;

	RETURN_NOERROR;
}

tResult cWorld_model::Stop(__exception) {
	RETURN_IF_FAILED(cFilter::Stop( __exception_ptr));

	RETURN_NOERROR;
}

tResult cWorld_model::Shutdown(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
    }
    else if (eStage == StageNormal)
    {
		currentStage = StageFirst;
    }
    else if (eStage == StageGraphReady)
    {
		currentStage = StageNormal;
		cv::destroyWindow("World_model_window");
    }

	RETURN_NOERROR;
}

tResult cWorld_model::sendNewValue(cOutputPin * outpin, tFloat32 value, tTimeStamp timeStamp) {
   	tFloat32 flValue= value;
							
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalInput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescSignalInput->WriteLock(pMediaSample, &pCoder));	
		
	pCoder->Set("f32Value", (tVoid*)&flValue);	
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    m_pCoderDescSignalInput->Unlock(pCoder);
    
    //transmit media sample over output pin
    pMediaSample->SetTime(timeStamp);
    outpin->Transmit(pMediaSample);
	RETURN_NOERROR;
}

void cWorld_model::drawSensor (cv::Mat * image, cWorld_object * sensor) {
	tFloat32 myx = this->my_pos->getx();
	tFloat32 myy = this->my_pos->gety();
	tFloat32 sensor_x = this->my_pos->getx() - sensor->getx();
	tFloat32 sensor_y = this->my_pos->gety() + sensor->gety();
	tFloat32 myrotation = this->my_pos->getRotation();
	tFloat32 objx = myx + (cos(myrotation) * (myx - sensor_x)) - (sin(myrotation) * (myy - sensor_y));
	tFloat32 objy = myy + (sin(myrotation) * (myx - sensor_x)) + (cos(myrotation) * (myy - sensor_y));
	if (objx > showroomx + 2 && objx < showroomx + showroomsizex - 2) {
		if (objy > showroomy + 2 && objy < showroomy + showroomsizey - 2) {
			cv::Vec3b curcolor;
			curcolor.val[0] = 0;
			curcolor.val[1] = 255;
			curcolor.val[2] = 0;
			image->at<cv::Vec3b>(cv::Point((int)objx-showroomx, (int)objy-showroomy)) = curcolor;
		}
	}
}

void cWorld_model::paintWorldModelToWindow(tUInt32 timestamp) {
	if (showroomactive) {
		int myx = (int)this->my_pos->getx();
		int myy = (int)this->my_pos->gety();
		
		//replace showroom to new position
		if (showroomautofollow) {
			showroomx = myx - (showroomsizex/2);
			showroomy = myy - (showroomsizey/2);
		}

		//create showroom
		cv::Mat	image;
		image = cv::Mat::zeros( showroomsizex, showroomsizey, CV_8UC3 );
		for (int x = 0; x < showroomsizex; x++) {
			for (int y = 0; y < showroomsizey; y++) {
				image.at<cv::Vec3b>(cv::Point(x, y)) = my_world.at<cv::Vec3b>(cv::Point(showroomx + x, showroomy + y));
			}
		}
		
		//background text
		std::string txt = std::string(cString::Format("my_pos = %d/%d",(int)my_pos->getx(), (int)my_pos->gety()));
		cv::Point textOrg = cv::Point(5,15);
		cv::putText(image, txt, textOrg, cv::FONT_HERSHEY_PLAIN, /*fontscale*/ 0.8, cv::Scalar::all(125), /*thickness*/ 1, 8);
		
		txt = std::string(cString::Format("last_paint = %.2f",(tFloat32)m_nLastPaintTime / 1000000));
		textOrg = cv::Point(5,30);
		cv::putText(image, txt, textOrg, cv::FONT_HERSHEY_PLAIN, /*fontscale*/ 0.8, cv::Scalar::all(125), /*thickness*/ 1, 8);

		txt = std::string(cString::Format("static objects = %d",staticObjects.size()));
		textOrg = cv::Point(5,45);
		cv::putText(image, txt, textOrg, cv::FONT_HERSHEY_PLAIN, /*fontscale*/ 0.8, cv::Scalar::all(125), /*thickness*/ 1, 8);

		//draw car main cs (red)
		if (myx > showroomx + 2 && myx < showroomx + showroomsizex - 2) {
			if (myy > showroomy + 2 && myy < showroomy + showroomsizey - 2) {
				//draw a cs
				cv::line(image, cv::Point(myx-showroomx-2, myy-showroomy), cv::Point(myx-showroomx+2, myy-showroomy), cv::Scalar(0, 0, 255));  //crosshair horizontal
				cv::line(image, cv::Point(myx-showroomx, myy-showroomy-2), cv::Point(myx-showroomx, myy-showroomy+2), cv::Scalar(0, 0, 255));  //crosshair vertical
			}
			//car rotation
			tFloat32 arrow_length = 100.0;
			tFloat32 myrotation = this->my_pos->getRotation();
			tFloat32 rotx = arrow_length * sin(myrotation) * -1;
			tFloat32 roty = arrow_length * cos(myrotation);
			cv::line(image, cv::Point(myx-showroomx, myy-showroomy), cv::Point((int)((myx-showroomx) - rotx), (int)((myy-showroomy) - roty)), cv::Scalar(0,0,255)); //rotation line
		}
		
		//draw sensors as dots (green)
		drawSensor(&image, this->my_front_ir);
		drawSensor(&image, this->my_front_usl);
		drawSensor(&image, this->my_front_usr);
		drawSensor(&image, this->my_rear_ir);
		drawSensor(&image, this->my_rear_usl);
		drawSensor(&image, this->my_rear_usr);
		drawSensor(&image, this->my_front_lir);
		drawSensor(&image, this->my_front_rir);
		drawSensor(&image, this->my_rear_lir);
		drawSensor(&image, this->my_rear_rir);
		drawSensor(&image, this->my_camera);

		//draw static objects (light blue)
		for(std::vector<cWorld_object>::iterator it = staticObjects.begin(); it != staticObjects.end(); ++it) {
			int objx = (int)it->getx();
			int objy = (int)it->gety();
			if (objx > showroomx + 2 && objx < showroomx + showroomsizex - 2) {
				if (objy > showroomy + 2 && objy < showroomy + showroomsizey - 2) {
					//draw a circle with the diameter of the likelihood
					
					int intensity = 255 - (((timestamp - it->getLastTimestamp()) / 1000) * (255 / object_outtime_in_seconds)); // --> 25,5 every object life is about 10 seconds
					if (intensity > 0) {
						//LOG_INFO(cString::Format("Intensity is %d, timestamp %d", intensity, it->getLastTimestamp()));
						//show the item
						cv::circle(image, cv::Point(objx-showroomx, objy-showroomy), (int)it->getLH(), cv::Scalar(intensity, intensity, 0));
					}
				}
			}
		}

		//remove out of time (oot) objects
		if (objects_should_die) {
			tBool found_one;
			do {
				found_one = false;
				for(std::vector<cWorld_object>::iterator it = staticObjects.begin(); it != staticObjects.end(); ++it) {
					int intensity = 255 - (((timestamp - it->getLastTimestamp()) / 1000) * (255 / object_outtime_in_seconds)); // --> 25,5 every object life is about 10 seconds
					if (intensity <= 0) {
						//erase the item
						staticObjects.erase(it);
						found_one = true;
						break;
					}
				}
			} while (found_one);
		}

		//draw driving lane (old path planning)
		/*for(std::vector<cv::Point>::iterator it = pathPoints.begin(); it != pathPoints.end(); ++it) {
			int objx = it->x;
			int objy = it->y;
			if (objx > showroomx + 2 && objx < showroomx + showroomsizex - 2) {
				if (objy > showroomy + 2 && objy < showroomy + showroomsizey - 2) {
					//draw a circle with the diameter of the likelihood
					cv::circle(image, cv::Point(objx-showroomx, objy-showroomy), 0, cv::Scalar(0, 255, 255));
				}
			}
		}*/

		//update shown image
		cv::imshow(cv::String("World_model_window"), image);
	}
}


void cWorld_model::relocateCarPosition(cWorld_object * sensor, std::vector<smartdriving::SD_Point> vec) {
	tFloat32 myx = this->my_pos->getx();
	tFloat32 myy = this->my_pos->gety();
	tFloat32 myrotation = this->my_pos->getRotation();
	tFloat32 sensor_likelihood = sensor->getLH();
	tFloat32 sensor_y = myy + sensor->gety();
	tFloat32 sensor_x = myx - sensor->getx();
	tFloat32 sensor_rot = sensor->getRotation();
	int search_distance = 3;
	int search_angle = 25;
	int check_accuracy = 1;

	std::vector<smartdriving::SD_Point> pointvec;
	pointvec.clear();
	smartdriving::SD_Point p;
	p.x = 0;
	p.y = 0;
	pointvec.push_back(p);

	for (int angle = -search_angle; angle <= search_angle; angle += 5) {
		tFloat32 tsin = tFloat32(sin((myrotation + angle)*M_PI/180));
		tFloat32 tcos = tFloat32(cos((myrotation + angle)*M_PI/180) * -1);
		for (int i = 1; i < search_distance; i++) {
			int j = 0;
			p.x = (int)(i * tsin);
			p.y = (int)(i * tcos);
			for (j = 0; j < pointvec.size(); j++) {
				if (pointvec.at(j).x == p.x && pointvec.at(j).y == p.y) {
					break;
				}
			}
			if (j == pointvec.size()) {
				pointvec.push_back(p);
			}
		}
	}

	int score = 0;
	tFloat32 testx = myx;
	tFloat32 testy = myy;
	tFloat32 testrot = myrotation;
	for (int c = 0; c < pointvec.size(); c+=check_accuracy) {
		for (int ret = -search_angle; ret <= search_angle; ret += 5) { 
			/*for (int x = search_in_cube * -1; x <= search_in_cube; x++) {
			for (int y = search_in_cube * -1; y <= search_in_cube; y++) {*/
			int funcscore = 0;
			tFloat32 mytestx = myx + pointvec.at(c).x;
			tFloat32 mytesty = myy + pointvec.at(c).y;
			tFloat32 mytestrot = myrotation + ret;
			tFloat32 tcos = (tFloat32) cos(mytestrot*M_PI/180);
			tFloat32 tsin = (tFloat32) sin(mytestrot*M_PI/180);
			for (int i = 0; i < vec.size(); i+= 2) {
				if (vec.at(i).y > 240) {
					tFloat32 picy = ((480-(tFloat32)vec.at(i).y)/480*this->camera_height_as_range)*-1; 
					tFloat32 picx = (((640-(tFloat32)vec.at(i).x)/640*this->camera_width_as_range)*-1)+this->camera_width_as_range*this->camera_width_position;

					//sensor calculation + picture
					tFloat32 sensorx = mytestx + (tcos * (mytestx - sensor_x + picx)) - (tsin * (mytesty - sensor_y + picy));
					tFloat32 sensory = mytesty + (tsin * (mytestx - sensor_x + picx)) + (tcos * (mytesty - sensor_y + picy));

					//sensor rotation calculation (for top pixel)
					tFloat32 sigx = sensorx + (sensor_rot == 90 ? this->camera_range_top_mid : (sensor_rot == 270 ? 0-this->camera_range_top_mid : 0));
	  				tFloat32 sigy = sensory + (sensor_rot == 0 ? this->camera_range_top_mid : (sensor_rot == 180 ? 0-this->camera_range_top_mid : 0));

					tFloat32 objx = sensorx + (tcos * (sensorx - sigx)) - (tsin * (sensory - sigy));
					tFloat32 objy = sensory + (tsin * (sensorx - sigx)) + (tcos * (sensory - sigy));

					//LOG_INFO(cString::Format("Get Line Pixel at with x=%f, y=%f, sensorx=%f, sensory=%f", myx, myy, objx, objy/*vec.at(i).y*/));
					if (my_world.at<cv::Vec3b>(cv::Point((int)objx, (int)objy)).val[0] == 255) {
							funcscore ++;
					}
				}
			}
			if (funcscore > score) {
				score = funcscore;
				testx = mytestx;
				testy = mytesty;
				testrot = mytestrot;
				if (funcscore >= vec.size() - 1) {
					LOG_INFO("Break with ideal score");
					break;
				}
			}
		}
	}
	my_pos->setx(testx);
	my_pos->sety(testy);
	my_pos->setRotation(testrot);
}

void cWorld_model::addVectorToMap(cWorld_object * sensor, std::vector<smartdriving::SD_Point> vec) {
	tFloat32 myx = this->my_pos->getx();
	tFloat32 myy = this->my_pos->gety();
	tFloat32 myrotation = this->my_pos->getRotation();
	tFloat32 sensor_likelihood = sensor->getLH();
	tFloat32 sensor_y = myy + sensor->gety();
	tFloat32 sensor_x = myx - sensor->getx();
	tFloat32 sensor_rot = sensor->getRotation();

	for (int i = 0; i < vec.size(); i++) {
		if (vec.at(i).y > 180) {
			tFloat32 picy = ((480-(tFloat32)vec.at(i).y)/480*this->camera_height_as_range)*-1; 
			tFloat32 picx = (((640-(tFloat32)vec.at(i).x)/640*this->camera_width_as_range)*-1)+this->camera_width_as_range*this->camera_width_position;

			//sensor calculation + picture
			tFloat32 sensorx = myx + (cos(myrotation) * (myx - sensor_x + picx)) - (sin(myrotation) * (myy - sensor_y + picy));
			tFloat32 sensory = myy + (sin(myrotation) * (myx - sensor_x + picx)) + (cos(myrotation) * (myy - sensor_y + picy));

			//sensor rotation calculation (for top pixel)
	  		tFloat32 sigy = sensory + (sensor_rot == 0 ? this->camera_range_top_mid : (sensor_rot == 180 ? 0-this->camera_range_top_mid : 0));
			tFloat32 sigx = sensorx + (sensor_rot == 90 ? this->camera_range_top_mid : (sensor_rot == 270 ? 0-this->camera_range_top_mid : 0));

			tFloat32 objx = sensorx + (cos(myrotation) * (sensorx - sigx)) - (sin(myrotation) * (sensory - sigy));
			tFloat32 objy = sensory + (sin(myrotation) * (sensorx - sigx)) + (cos(myrotation) * (sensory - sigy));

			//LOG_INFO(cString::Format("Get Line Pixel at with x=%f, y=%f, sensorx=%f, sensory=%f", myx, myy, objx, objy/*vec.at(i).y*/));

			/*int val = my_world.at<cv::Vec3b>(cv::Point((int)objx, (int)objy)).val[0];
			val += 50;
			if (val > 255) */
			int val = 255;
			my_world.at<cv::Vec3b>(cv::Point((int)objx, (int)objy)).val[0] = val;
			my_world.at<cv::Vec3b>(cv::Point((int)objx, (int)objy)).val[1] = val;
			my_world.at<cv::Vec3b>(cv::Point((int)objx, (int)objy)).val[2] = val;
		}
	}
}

void cWorld_model::addObjectToVector(cWorld_object * sensor, tFloat32 * signalValue, tUInt32 * timestamp) {
	tFloat32 myx = this->my_pos->getx();
	tFloat32 myy = this->my_pos->gety();
	tFloat32 myrotation = this->my_pos->getRotation();
	tFloat32 sensor_likelihood = sensor->getLH();
	tFloat32 sensor_y = myy + sensor->gety();
	tFloat32 sensor_x = myx - sensor->getx();
	tFloat32 sensor_rot = sensor->getRotation();
			
	//x' = x1 + cosq * (x - x1) - sinq * (y - y1)
	//y' = y1 + sinq * (x - x1) + cosq * (y - y1) 
	
	//sensor berechnung
	tFloat32 sensorx = myx + (cos(myrotation) * (myx - sensor_x)) - (sin(myrotation) * (myy - sensor_y));
	tFloat32 sensory = myy + (sin(myrotation) * (myx - sensor_x)) + (cos(myrotation) * (myy - sensor_y));
						
	//sensor rotations Einrechnung
	tFloat32 sigy = sensory + (sensor_rot == 0 ? *signalValue : (sensor_rot == 180 ? 0-*signalValue : 0));
	tFloat32 sigx = sensorx + (sensor_rot == 90 ? *signalValue : (sensor_rot == 270 ? 0-*signalValue : 0));

	//object berechnung
	tFloat32 objx = sensorx + (cos(myrotation) * (sensorx - sigx)) - (sin(myrotation) * (sensory - sigy));
	tFloat32 objy = sensory + (sin(myrotation) * (sensorx - sigx)) + (cos(myrotation) * (sensory - sigy));
	
	//check if its already in the vector
	std::vector<cWorld_object>::iterator it; 
	tBool found = false;
	for(it = staticObjects.begin(); it != staticObjects.end(); ++it) {
		if (it->getx() - sensor_likelihood < objx && it->getx() + sensor_likelihood > objx) {
			if (it->gety() - sensor_likelihood < objy && it->gety() + sensor_likelihood > objy) {
				//LOG_INFO(cString::Format("Update Obj with old_x=%f, new_x=%f, old_y=%f, new_y=%f", it->getx(), objx, it->gety(), objy));
				found = true;
				break;
			}
		}
	}
	if (!found) {
		cWorld_object * obj = new cWorld_object(objx, objy, sensor_likelihood);
		obj->setLastTimestamp(*timestamp);
		staticObjects.push_back(*obj);
		//LOG_INFO(cString::Format("Add OBJ CS at x=%f, y=%f with Sensor CS at x=%f, y=%f relative to my position x=%f, y=%f", objx, objy,sensorx, sensory, myx, myy));
	} else {
		it->calculateWithKalman(objx, objy, sensor_likelihood, *timestamp);
	}
}

void cWorld_model::carMoveAClick(int sensor,tFloat32 * signalValue) {
	//sensor 0 == left, 1 == right
	//TODO: Rotation mit einbeziehen
	tFloat32 myx = this->my_pos->getx();
	tFloat32 myy = this->my_pos->gety();
	tFloat32 myrotation = this->my_pos->getRotation();

	if (sensor == 0) {
		tFloat32 verschub = abs(old_wheels - (old_wheelR + *signalValue) / 2) * wheel_click_length;
		
		tFloat32 verschubx = verschub * sin(myrotation) * -1;
		tFloat32 verschuby = verschub * cos(myrotation);

		my_pos->sety(my_pos->gety() - verschuby);
		my_pos->setx(my_pos->getx() - verschubx);

		old_wheelL = *signalValue;
		old_wheels = (old_wheelR + old_wheelL) / 2;
	} else {
		tFloat32 verschub = abs(old_wheels - (old_wheelL + *signalValue) / 2) * wheel_click_length;

		tFloat32 verschubx = verschub * sin(myrotation) * -1;
		tFloat32 verschuby = verschub * cos(myrotation);

		my_pos->sety(my_pos->gety() - verschuby);
		my_pos->setx(my_pos->getx() - verschubx);

		//my_pos->sety(my_pos->gety() - abs(old_wheels - (old_wheelL + *signalValue) / 2) * wheel_click_length);

		old_wheelR = *signalValue;
		old_wheels = (old_wheelL + old_wheelR) / 2;
	}
}

tResult cWorld_model::doStamping(tUInt32 timestamp) {
	RETURN_NOERROR;
}

tResult cWorld_model::OnPinEvent( IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
	if (currentStage != StageGraphReady) {
		RETURN_NOERROR;
	}
	
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pSource == &m_oIObstacleInput) {
			//LOG_INFO("Obstacle Location retrieved");
			tObstacleLocation * pMyStruct;
			pMediaSample->Lock((const tVoid**) &pMyStruct);
			tObstacleLocation tmpMyStruct;
			memcpy(&tmpMyStruct, pMyStruct, sizeof(tObstacleLocation));
			pMediaSample->Unlock((const tVoid*)pMyStruct);

			//LOG_INFO(cString::Format("Obstacle detection test: x=%d/%d/%d", pMyStruct->distance, pMyStruct->rightEnd, pMyStruct->leftEnd));
			tFloat32 distance = tmpMyStruct.distance;
			tUInt32 curTime = (tUInt32) _clock->GetStreamTime();
			addObjectToVector(this->my_camera, &distance, &curTime);

		} else if (pSource == &m_oIIPMInput) {
			//LOG_INFO("IPM Input retrieved");
			smartdriving::Lane * pMyStruct;
			pMediaSample->Lock((const tVoid**) &pMyStruct);
			smartdriving::Lane tmpMyStruct;
			for (int i = 0; i < pMyStruct->lane.size(); i++) {
				tmpMyStruct.lane.push_back(pMyStruct->lane.at(i));
			}
			pMediaSample->Unlock((const tVoid*)pMyStruct);
			
			relocateCarPosition(this->my_camera, tmpMyStruct.lane);
			addVectorToMap(this->my_camera, tmpMyStruct.lane);
			//LOG_INFO(cString::Format("Obstacle detection test: x=%d/%d/%d", pMyStruct->distance, pMyStruct->rightEnd, pMyStruct->leftEnd));
		} else {
			tFloat32 signalValue = 0;
			tUInt32 timeStamp = 0;
			m_nLastMSTime = _clock->GetStreamTime();

			if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL) {
				// read-out the incoming Media Sample
				cObjectPtr<IMediaCoder> pCoderInput;
				RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));
            
				//get values from media sample        
				pCoderInput->Get("f32Value", (tVoid*)&signalValue);
				pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
				m_pCoderDescSignalInput->Unlock(pCoderInput);                 
			} else
				RETURN_ERROR(ERR_FAILED);
	
			if (pSource == &m_oIFrontIRF) {
				addObjectToVector(this->my_front_ir, &signalValue, &timeStamp);
			} else if (pSource == &m_oIFrontUSL) {
				addObjectToVector(this->my_front_usl, &signalValue, &timeStamp);
			} else if (pSource == &m_oIFrontUSR) {
				addObjectToVector(this->my_front_usr, &signalValue, &timeStamp);
			} else if (pSource == &m_oIWheelL) {
				carMoveAClick(0, &signalValue);
			} else if (pSource == &m_oIWheelR) {
				carMoveAClick(1, &signalValue);
			}
		
			if ((tUInt32)m_nLastPaintTime < ((tUInt32) m_nLastMSTime) - 100000) { // draw only ever 0.1 seconds
				//LOG_INFO(cString::Format("Paint %d, %d", (tUInt32)m_nLastPaintTime, ((tUInt32) m_nLastMSTime) - 100000));
				m_nLastPaintTime = m_nLastMSTime;
				//doPathPlaning(timeStamp); wird in zukunft erstmal vom Stamping übernommen
				doStamping(timeStamp);
				paintWorldModelToWindow(timeStamp);
				if (this->autosave_path != "") {
					cv::imwrite(autosave_path, my_world);
				}
			}
		}
	}
	RETURN_NOERROR;
}
