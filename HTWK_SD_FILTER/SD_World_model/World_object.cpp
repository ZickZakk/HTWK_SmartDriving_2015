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
#include "World_object.h"

//SETTER
void cWorld_object::setx(tFloat32 new_x) {
	this->x = new_x;
}

void cWorld_object::sety(tFloat32 new_y) {
	this->y = new_y;
}

void cWorld_object::setLH(tFloat32 new_likelihood) {
	this->likelihood = new_likelihood;
}

void cWorld_object::setRotation(tFloat32 new_rotation_towards_WCS) {
	this->rotation_towards_WCS = new_rotation_towards_WCS;
}

void cWorld_object::setVelocity(tFloat32 new_velocity) {
	this->velocity = new_velocity;
}

void cWorld_object::setLastTimestamp(tUInt32 new_timestamp) {
	last_seen = new_timestamp;
}
	
//GETTER
tFloat32 cWorld_object::getx() {
	return x;
}

tFloat32 cWorld_object::gety() {
	return y;
}

tFloat32 cWorld_object::getLH() {
	return likelihood;
}

tFloat32 cWorld_object::getRotation() {
	return rotation_towards_WCS;
}

tFloat32 cWorld_object::getVelocity() {
	return velocity;
}

tUInt32 cWorld_object::getLastTimestamp() {
	return last_seen;
}

//CONSTURTORS & DESTRUCTORS
cWorld_object::cWorld_object(tFloat32 x, tFloat32 y, tFloat32 likelihood, tFloat32 rotation_towards_WCS, tFloat32 velocity) {
	this->setx(x);
	this->sety(y);
	this->setLH(likelihood);
	this->setRotation(rotation_towards_WCS);
	this->setVelocity(velocity);
	this->kalman_filter_uninitialized = true;
	this->last_seen = 0;
}

cWorld_object::~cWorld_object() {
}

void cWorld_object::initKalman(tFloat32 x, tFloat32 y, tFloat32 sensor_noise) {
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    kalman_filter.init(4, 2, 0);
	
    cv::Mat measurement = cv::Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(1, 0) = y;

    cv::Mat trans = cv::Mat_<float>::zeros(4,4);
    trans.at<float>(0,0) = 1;
    trans.at<float>(1,1) = 1;
    trans.at<float>(2,2) = 1;
    trans.at<float>(3,3) = 1;
    trans.at<float>(0,2) = 1;
    trans.at<float>(1,3) = 1;

    kalman_filter.transitionMatrix = trans;
	
	kalman_filter.statePre.at<float>(0) = x;
	kalman_filter.statePre.at<float>(1) = y;
	kalman_filter.statePre.at<float>(2) = 0;
	kalman_filter.statePre.at<float>(3) = 0;
	
	kalman_filter.statePost.at<float>(0) = x;
	kalman_filter.statePost.at<float>(1) = y;

	setIdentity(kalman_filter.measurementMatrix);
	setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(1e-4));
	setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(sensor_noise));
	setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(sensor_noise));
}

void cWorld_object::kalmanPredict() {
	kalman_filter.predict();
}

void cWorld_object::kalmanCorrect(tFloat32 x, tFloat32 y)
{
	cv::Mat measurement = cv::Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(1, 0) = y;
    kalman_filter.correct(measurement);
}


void cWorld_object::calculateWithKalman(tFloat32 x, tFloat32 y, tFloat32 sensor_noise, tUInt32 timestamp) {
	if (kalman_filter_uninitialized) {
		//LOG_INFO(cString::Format("Kalman initalize with x=%f, y=%f", x, y));
		initKalman(x,y,sensor_noise);
		kalman_filter_uninitialized = false;
		
	} else {
		setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(sensor_noise));
		kalmanPredict();
		kalmanCorrect(x,y);
		
		this->x = kalman_filter.statePost.at<float>(0);
		this->y = kalman_filter.statePost.at<float>(1);
		this->likelihood = (kalman_filter.errorCovPost.at<float>(0) + kalman_filter.errorCovPost.at<float>(1)) / 2;
		if (this->likelihood < 1) this->likelihood = 1;
		//LOG_INFO(cString::Format("Kalman new point with x=%f, y=%f, z=%f", this->x, this->y, this->likelihood));
	}
	last_seen = timestamp;
}
