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
*
* HTWK SmartDriving - ObstacleDetection
*
* $Author: sfeig $
*
* @remarks
*
*/
#ifndef _SD_OBSTACLE_DETECTION_FILTER_H_
#define _SD_OBSTACLE_DETECTION_FILTER_H_

#define OID_ADTF_SD_OBSTACLE_DETECTION_FILTER "adtf.sd.obstacle_detection_filter"

//*************************************************************************************************
class cObstacleDetectionFilter : public adtf::cFilter
{
	ADTF_FILTER(OID_ADTF_SD_OBSTACLE_DETECTION_FILTER, "SD ObstacleDetection", adtf::OBJCAT_DataFilter);

protected:
	cVideoPin m_oVideoInputPin;
	cVideoPin m_oTestDepthImage;
	cOutputPin m_oGLCOutput;
	cOutputPin m_oObstacleLocation;

public:
	cObstacleDetectionFilter(const tChar* __info);
	virtual ~cObstacleDetectionFilter();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample);

	tResult ProcessInput(IMediaSample *pSample);

private:
	tBool isFirstFrame;

	cv::Mat referenceDepthImage;
	cv::Mat processingImage;
	cv::Mat birdsEyeView;

	std::vector<SurfaceDescription> surfaceDescriptions;
	std::vector<cv::Rect> boundRects;
	std::vector<ObstacleDescription> obstacleDescriptions;

	NoiseRemover denoiser;
	EdgeRemover edgeRemover;
	GroundRemover groundRemover;
	BirdsEyeViewer birdsEyeViewer;
	ConnectedComponentFinder connectedComponentFinder;
	ObstacleExtractor obstacleExtractor;
	SurfaceExtractor surfaceExtractor;
	CoordinateConverter coordinateConverter;

	Vdar vdar;

	tBitmapFormat m_sInputFormat;

	tResult loadProperties();
	tResult findObstacles(Mat &imDepth);

	tResult createAndTransmitObstacleLocation(std::vector<SurfaceDescription>& surfaceDescriptions);
	tResult createAndTransmitGCL(std::vector<SurfaceDescription>& surfaceDescriptions);
	
	tResult transmitObstacleLocation(tFloat x, tFloat y, tFloat z);

	tResult markAndTransmitDepthImage(cv::Mat& depthImage, std::vector<SurfaceDescription>& surfaceDescriptions);
	tResult transmitDepthImage(const cv::Mat& depthImage);
};

#endif // _SD_OBSTACLE_DETECTION_FILTER_H_
