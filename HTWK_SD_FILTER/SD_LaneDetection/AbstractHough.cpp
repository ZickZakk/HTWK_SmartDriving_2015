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
#include "AbstractHough.h"
using namespace smartdriving;

AbstractHough::AbstractHough(void)
{}

AbstractHough::~AbstractHough()
{
	this->houghSpace.release();
}

void  AbstractHough::initHough(void)
{
	this->imageWidth = this->image.cols;
	this->imageHeight = this->image.rows;

	this->imageCenterX = this->imageWidth / 2.0;
	this->imageCenterY = this->imageHeight / 2.0;
	
	this->houghSpaceHeightRelative = ((sqrt(2.0) * static_cast<double>(this->imageHeight > this->imageWidth ? this->imageHeight : this->imageWidth)) / 2.0);
	this->houghSpaceHeight = static_cast<int>(this->houghSpaceHeightRelative * 2.0); // -r -> +r
	this->houghSpaceWidth = 180;

	this->houghSpace.reset(new unsigned int[static_cast<unsigned int>(houghSpaceHeight * houghSpaceWidth)]());
}

double AbstractHough::getDistance(const int &angle, const int &column, const int &row) const
{
	return ((static_cast<double>(column) -this->imageCenterX) * LookUpAngles::getCos(angle)) + ((static_cast<double>(row) - this->imageCenterY) * LookUpAngles::getSin(angle));
}

vector<SD_Line> AbstractHough::getLines(const int &threshold)
{
	vector<SD_Line> lines;

	if (this->houghSpace == nullptr)
	{
		return lines;
	}

	for (int distance = 0; distance < houghSpaceHeight; distance++)
	{
		for (int angle = 0; angle < houghSpaceWidth; angle++)
		{
			if (isLocalMaximaFound(distance, angle, threshold))
			{
				static int x1, y1, x2, y2;

				if (angle >= 45 && angle <= 135)
				{
					//y = (r - x cos(t)) / sin(t)
					x1 = 0;
					// y1 = static_cast<int>(((double) (distance - (houghSpaceHeight / 2)) - ((x1 - (imageWidth / 2)) * cos(angle * DEG_TO_RAD))) / sin(angle * DEG_TO_RAD) + (imageHeight / 2));
					y1 = getYCoord(distance, angle, x1);
					x2 = imageWidth;
					// y2 = static_cast<int>(((double) (distance - (houghSpaceHeight / 2)) - ((x2 - (imageWidth / 2)) * cos(angle * DEG_TO_RAD))) / sin(angle * DEG_TO_RAD) + (imageHeight / 2));
					y2 = getYCoord(distance, angle, x2);
				}
				else
				{
					//x = (r - y sin(t)) / cos(t);
					y1 = 0;
					// x1 = ((double) (distance - (houghSpaceHeight / 2)) - ((y1 - (imageHeight / 2)) * sin(angle * DEG_TO_RAD))) / cos(angle * DEG_TO_RAD) + (imageWidth / 2);
					x1 = getXCoord(distance, angle, y1);
					y2 = imageHeight;
					// x2 = ((double) (distance - (houghSpaceHeight / 2)) - ((y2 - (imageHeight / 2)) * sin(angle * DEG_TO_RAD))) / cos(angle * DEG_TO_RAD) + (imageWidth / 2);
					x2 = getXCoord(distance, angle, y2);
				}

				// lines.push_back(pair<pair<int, int>, pair<int, int> >(pair<int, int>(x1, y1), pair<int, int>(x2, y2)));
				SD_Line point;
				point.x1 = x1;
				point.y1 = y1;
				point.x2 = x2;
				point.y2 = y2;
				lines.push_back(point);
			}
		}
	}

	return lines;
}

bool AbstractHough::isLocalMaximaFound(int &distance, int &angle, const int &threshold)
{
	static int index;
	index = getHoughSpaceIndex(distance, angle);
	// printf_s("DEBUG: %d\n", index);
	if (static_cast<int>(this->houghSpace[index]) >= threshold)
	{
		//Is this point a local maxima (9x9)
		static int localMaxima;
		localMaxima = this->houghSpace[index];

		/*
		Ausgehend von dem Maximum das an der Position (distance, angle) liegt, wird geprüft, ob es einen Wert gibt der
		in einem Bereich von 4 Pixeln um den Wert herum größer ist.

		Wird ein Wert gefunden der größer ist, dann schleife abbrechen und im idealfall Distanz und Winkel neusetzen.
		*/
		for (int y = -4; y <= 4; y++)
		{
			for (int x = -4; x <= 4; x++)
			{
				static pair<int, int> point;
				point.first = y;
				point.second = x;

				if (isPositionInImage(point, distance, angle))
				{
					index = getHoughSpaceIndex(distance + y, angle + x);

					if (static_cast<int>(this->houghSpace[index]) > localMaxima)
					{
						return false;
					}
				}
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}

const int AbstractHough::getHoughSpaceIndex(const int &distance, const int &angle) const
{
	return static_cast<int>((distance * houghSpaceWidth) + angle);
}

bool AbstractHough::isPositionInImage(const pair<int, int> &point, const int &distance, const int &angle) const
{
	return ((point.first + distance >= 0 && point.first + distance < this->houghSpaceHeight) && (point.second + angle >= 0 && point.second + angle < this->houghSpaceWidth));
}

int AbstractHough::getYCoord(const int &distance, const int &angle, const int &x)
{//(this->imageWidth / 2)
	return static_cast<int>((static_cast<double>(distance - (this->houghSpaceHeight / 2)) - ((x - this->imageCenterX) * LookUpAngles::getCos(angle))) / LookUpAngles::getSin(angle) + (this->imageHeight / 2));
}

int AbstractHough::getXCoord(const int &distance, const int &angle, const int &y)
{//(this->imageHeight / 2)
	return static_cast<int>((static_cast<double>(distance - (this->houghSpaceHeight / 2)) - ((y - this->imageCenterY) * LookUpAngles::getSin(angle))) / LookUpAngles::getCos(angle) + (this->imageWidth / 2));
}

void AbstractHough::setImage(const cv::Mat &image)
{
	this->image = image.clone();
	initHough();
}