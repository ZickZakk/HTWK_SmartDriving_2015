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

#include "stdafx.h"

ParkGap::ParkGap()
{
	this->isDebugActive = false;
	this->foundGap = false;
	this->isFirstCarFound = false;
	this->isFirstCarComplete = false;
	this->distance = 0;
	this->measuredParkSpace = 0;
}

ParkGap::~ParkGap()
{
}

void ParkGap::setDecision(const Decision &currentDecision)
{
	this->currentDecision = currentDecision;
}

bool ParkGap::searchParkingGap(const tInt &irValue,const tInt &distance)
{
	tInt confidenceRange = 35;

		tInt distanceToObject = irValue;
		this->distance = distance;

		if (this->isDebugActive)
		{
			//nichts gefunden
			if (distanceToObject > confidenceRange)
			{
				//do Nothing
				LOG_INFO(cString::Format("distanceToObject %d", distanceToObject));
				LOG_INFO("distanceToObject > this->confidenceRange");
			}
		}

		//auf Höhe des Autos
		if (distanceToObject < confidenceRange && !this->isFirstCarFound && !this->foundGap)
		{
			this->isFirstCarFound = true;
			this->isFirstCarComplete = false;

			if (this->isDebugActive)
			{	
				LOG_INFO("Höhe Auto");
				LOG_INFO(cString::Format("distanceToObject %d", distanceToObject));
				LOG_INFO("isFirstCarFound = true");
				LOG_INFO("isFirstCarComplete = false");
			}
		}
		//Auto abgeschlossen nun kommt die Lücke
		if (distanceToObject > confidenceRange && this->isFirstCarFound && !this->isFirstCarComplete && !this->foundGap)
		{
			if (this->isDebugActive)
			{
				LOG_INFO(cString::Format("distanceToObject %d", distanceToObject));
				LOG_INFO(cString::Format("Drove distance %d", this->distance));
			}

			this->isFirstCarComplete = true;
			this->foundGap = true;
			this->gapStart = this->distance;
			this->measuredParkSpace = this->distance - this->gapStart;

			if (this->isDebugActive)
			{
				LOG_INFO("GapStart");
				LOG_INFO(cString::Format("---11---  %d", this->distance));
			}
		}

		if(this->isFirstCarComplete) //&& this->foundGap)
		{
			//setze Lücken Größe wieder auf 0 da anderes Auto gefunden wurde
			//if(distanceToObject < confidenceRange )
			//{
				//this->foundAnotherCar = true;
				//this->gapStart = this->distance;
				this->measuredParkSpace = this->distance - this->gapStart;

				//if (this->isDebugActive)
				//{
					//LOG_INFO("Set measuredParkSpae to zero because found another vehicle");
					//LOG_INFO("distanceToObject < confidenceRange");
					//LOG_INFO(cString::Format("distanceToObject  %d", distanceToObject));
					//LOG_INFO(cString::Format("confidenceRange %d", confidenceRange));
				//}
			//}
			//messe nur wenn wirklich Lücke
			//if(distanceToObject >= confidenceRange)
			//{
				//this->measuredParkSpace = this->distance - this->gapStart;

				//if (this->isDebugActive)
				//{
					//LOG_INFO("Another GAP detected");
					//LOG_INFO("distanceToObject >= confidenceRange");
					//LOG_INFO(cString::Format("distanceToObject  %d", distanceToObject));
					//LOG_INFO(cString::Format("confidenceRange %d", confidenceRange));
					//LOG_INFO(cString::Format("measuredParkSpace %d", measuredParkSpace));
				//}
			//}

			if (this->isDebugActive)
			{
				LOG_INFO(cString::Format("<---IST---> %d", this->measuredParkSpace));
				//LOG_INFO(cString::Format("<---SOLL4Cross--->  %d", GetPropertyInt(PROP_NAME_MEASURE_DISTANCE_FOR_CROSS_PARKING)));
				//LOG_INFO(cString::Format("<---SOLL4Parallel--->  %d", GetPropertyInt(PROP_NAME_MEASURE_DISTANCE_FOR_PARALLEL_PARKING)));
			}
			// go in here for cross parking todo: add && statement coming from input pin
			if((this->measuredParkSpace >= 25) && this->currentDecision == DECISION_CROSS_PARKING)
			{
				if (this->isDebugActive)
				{
					LOG_INFO(cString::Format("great!...found gap for cross parking, size: %d", this->measuredParkSpace));
				}
				this->isFirstCarComplete = false;
				return true;

				if (this->isDebugActive)
				{
					LOG_INFO("STEP 0->1");
					LOG_INFO(" ");
				}
			}
			//go in here for parallel parking
			else if((this->measuredParkSpace >= 31) && this->currentDecision == DECISION_PARALLEL_PARKING)
			{
				if (this->isDebugActive)
				{
					LOG_INFO(cString::Format("great!...found gap for parallel parking, size: %d", this->measuredParkSpace));
				}
				this->isFirstCarComplete = false;
				return true;

				if (this->isDebugActive)
				{
					LOG_INFO("STEP 0->1");
					LOG_INFO(" ");
				}
			}
			else
			{
				if (this->isDebugActive)
				{
					LOG_INFO("Needed parking space not reached...");
				}
			}
		}//end if (distanceToObject <= confidenceRange && isGapEnd)

	return false;
}

void ParkGap::setIsDebugActive(const bool &isDebugActive)
{
	this->isDebugActive = isDebugActive;
}