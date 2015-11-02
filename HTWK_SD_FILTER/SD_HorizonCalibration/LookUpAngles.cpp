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

#include "LookUpAngles.h"
#include "stdafx.h"
using namespace smartdriving;

bool LookUpAngles::initialized = false;
double LookUpAngles::DEG_TO_RAD = CV_PI / 180.0;
double LookUpAngles::sin[180] = { 0 };
double LookUpAngles::cos[180] = { 0 };

void LookUpAngles::init(void)
{
	for (int angle = 0; angle < 180; angle++)
	{
		sin[angle] = std::sin(angle * DEG_TO_RAD);
		cos[angle] = std::cos(angle * DEG_TO_RAD);
	}

	initialized = true;
}

double LookUpAngles::getSin(const int &angle)
{
	if (!initialized)
	{
		init();
	}

	// Eventuell noch prüfen, ob die Zahl zwischen 0..179 liegt
	return sin[angle];
}

double LookUpAngles::getCos(const int &angle)
{
	if (!initialized)
	{
		init();
	}

	// Eventuell noch prüfen, ob die Zahl zwischen 0..179 liegt
	return cos[angle];
}