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

#ifndef MANEUVERLIST_H
#define MANEUVERLIST_H

struct tAADC_Maneuver
{
	int id;
	cString action;
};

struct tSector
{
	int id;
	std::vector<tAADC_Maneuver> maneuverList;
};

enum DriverState
{
	ERROR = -1,
	READY = 0,
	RUNNING = 1,
	COMPLETE = 2
};

enum JuryState
{
	STOP = -1,
	// READY = 0, because its defined in DriverState with same ID
	RUN = 1
};

enum Maneuver
{
	MANEUVER_RUNNING,
	MANEUVER_FINISHED,
	MANEUVER_STOP,
	MANEUVER_LEFT,
	MANEUVER_RIGHT,
	MANEUVER_STRAIGHT,
	MANEUVER_PARALLEL_PARKING,
	MANEUVER_CROSS_PARKING,
	MANEUVER_PULL_OUT_LEFT,
	MANEUVER_PULL_OUT_RIGHT
};

#endif