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
#include <opencv2/video/tracking.hpp>

#ifndef _SD_World_object_FILTER_HEADER_
#define _SD_World_object_FILTER_HEADER_

class cWorld_object 
{
    private:
		void initKalman(tFloat32 x, tFloat32 y, tFloat32 sensor_noise);
		void kalmanPredict();
		void kalmanCorrect(tFloat32 x, tFloat32 y);
		

		tFloat32 x;
		tFloat32 y;
		tFloat32 likelihood;
		tFloat32 rotation_towards_WCS; //Rotation towards the world coordinate system
		tFloat32 velocity;
		tUInt32  last_seen;
		cv::KalmanFilter kalman_filter;
		tBool kalman_filter_uninitialized;

    protected:

	public: 
		// constructors
		cWorld_object(tFloat32 x, tFloat32 y, tFloat32 likelihood = -1, tFloat32 trotation_to_WCS = 0, tFloat32 velocity = 0);
        virtual ~cWorld_object();

		void setx(tFloat32 new_x);
		void sety(tFloat32 new_y);
		void setLH(tFloat32 likelihood);
		void setRotation(tFloat32 new_rotation);
		void setVelocity(tFloat32 new_velocity);
		void setLastTimestamp(tUInt32 new_timestamp);

		tFloat32 getx();
		tFloat32 gety();
		tFloat32 getLH();
		tFloat32 getRotation();
		tFloat32 getVelocity();
		tUInt32  getLastTimestamp();

		void calculateWithKalman(tFloat32 x, tFloat32 y, tFloat32 sensor_noise, tUInt32 timestamp);
};

#endif // _SD_World_object_FILTER_HEADER_