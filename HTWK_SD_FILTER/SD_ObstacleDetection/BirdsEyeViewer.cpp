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
#include "BirdsEyeViewer.h"

BirdsEyeViewer::BirdsEyeViewer() : screenWidth(640), screenHeight(480)
{
	topSearchArea = screenHeight - (screenHeight / 2);
	bottomSearchArea = screenHeight-20;
	screenCenterX = screenWidth / 2;
	screenCenterY = screenHeight / 2;
}


BirdsEyeViewer::~BirdsEyeViewer()
{

}


cv::Mat& BirdsEyeViewer::process(const cv::Mat &imSrc)
{
	int nrows = imSrc.rows;

	imRes.create(imSrc.rows, imSrc.cols, CV_8U);
	imRes = Scalar(0,0,0);

	float worldX, worldZ, worldY;
	int worldXinCentimeter, worldZinCentimeter;

	for (int row = topSearchArea; row <= bottomSearchArea; row+=4)
	{
		const ushort *dataSrc = imSrc.ptr<ushort>(row);
		for (int col = 0; col < screenWidth; ++col)
		{
			coordinateConverter.depthToWorld(col, row, *dataSrc++, worldX, worldY, worldZ);

			worldXinCentimeter = worldX * -100.0f;
			worldZinCentimeter = worldZ * 100.0f;

			try
			{
				imRes.ptr<uchar>(nrows-worldZinCentimeter)[screenCenterX+worldXinCentimeter] = 0xff;
			}
			catch( cv::Exception& e )
			{
				const char* err_msg = e.what();
				throw err_msg;
			}
		}
	}

	return imRes;
}


