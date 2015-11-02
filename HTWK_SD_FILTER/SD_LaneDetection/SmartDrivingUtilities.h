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

#ifndef _SMART_DRIVING_UTILITIES_H_
#define _SMART_DRIVING_UTILITIES_H_

namespace smartdriving
{
		typedef struct
		{
			int x1;
			int y1;
			int x2;
			int y2;
		} SD_Line;

		typedef struct
		{
			int x;
			int y;
		} SD_Point;

		/**
		*
		*
		*/
		inline double round(double value)
		{
			return floor(value + 0.5);
		}

		double getRise(const SD_Line &line)
		{
			double denominator = static_cast<double>(line.x2 - line.x1);

			// Senkrechte Linie braucht nicht betrachtet werden,
			// deshalb wird der Error-Code 0xFFFF geliefert
			// TODO: dem Error-Code einen sinvollen Namen geben
			if (denominator == 0.0)
			{
				return 0xFFFF;
			}

			return static_cast<double>(line.y2 - line.y1) / denominator;
		}

		/**
		* Berechnet den Schnittpunkt zweier Linien.
		* Exisitert zwischen den beiden Linien kein Schnittpunkt, so gibt die Funktion false zurück, sonst true
		*/
		inline bool getLineIntersection(const SD_Line &line1, const SD_Line &line2, SD_Point &result)
		{
			double m1 = getRise(line1);
			double m2 = getRise(line2);

			// wenn beide Geraden den gleichen asntieg haben, dann ist der Anstieg gleich und es existiert kein Schnittpunkt
			if (m1 == m2 || m1 == 0xFFFF || m2 == 0xFFFF)
			{
				return false;
			}

			result.x = static_cast<int>((m1 * line1.x1 - m2 * line2.x1 - line1.y1 + line2.y1) / (m1 - m2));

			// Die y-Koordinate könnte genau so gut mit Linie 2 berechnet werden!
			result.y = static_cast<int>(m1 * (result.x - line1.x1) + line1.y1);

			return true;
		}
}
#endif