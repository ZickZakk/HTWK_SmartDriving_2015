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
#include "stdafx.h"
#include "obstacledetection.h"

#include <cmath>
#include <sys/time.h>

/// Create filter shell
ADTF_FILTER_PLUGIN("SD ObstacleDetection", OID_ADTF_SD_OBSTACLE_DETECTION_FILTER, cObstacleDetectionFilter);


cObstacleDetectionFilter::cObstacleDetectionFilter(const tChar* __info):cFilter(__info)
{
	isFirstFrame = true;
	
	SetPropertyStr("Background Image",""); 
	SetPropertyBool("Background Image" NSSUBPROP_FILENAME, tTrue); 
	SetPropertyStr("Background Image" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "PNG Files (*.png)"); 	
}


cObstacleDetectionFilter::~cObstacleDetectionFilter() { }


tResult cObstacleDetectionFilter::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		// create and register cVideoPin with PinDirection set to input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("DepthImage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

		// create and register output pin COMMAND_GCL
		cObjectPtr<IMediaType> pCmdType = NULL;
		RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
		RETURN_IF_FAILED(m_oGLCOutput.Create("GCL_Output", pCmdType, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oGLCOutput));

		// create and register output pin STRUCT_STRUCTURED
		cObjectPtr<IMediaType> pTypeStruct;
		RETURN_IF_FAILED(AllocMediaType((tVoid**)&pTypeStruct, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED, NULL, NULL));
		RETURN_IF_FAILED(m_oObstacleLocation.Create("ObstacleLocation_Output", pTypeStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oObstacleLocation));

		// create and register cVideoPin with PinDirection set to output
		RETURN_IF_FAILED(m_oTestDepthImage.Create("TestDepthImage_Output", adtf::IPin::PD_Output, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oTestDepthImage));
	}
	else if (eStage == StageNormal)
	{
		tResult nResult = loadProperties(); 
		if (IS_FAILED(nResult))
		{
			THROW_ERROR_DESC(nResult,"Failed to load the background image file for the ObstacleDetection");		  
		}
	}
	else if (eStage == StageGraphReady)
	{
        cv::namedWindow(cv::String("ObstacleDetectionWindow"), cv::WINDOW_NORMAL);
		cv::startWindowThread();
	}

	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::loadProperties()
{
	std::string resolved_path = GetPropertyStr("Background Image");
	if (0 == resolved_path.length())
	{
		LOG_WARNING("ObstacleDection: Background Image File not found");
		RETURN_ERROR(ERR_INVALID_FILE);
	}
	cv::Mat backgroundImage = imread(resolved_path);
	groundRemover.setBackground(backgroundImage);
	vdar.setReferenceBackground(backgroundImage);

	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
		cv::destroyWindow("ObstacleDetectionWindow");
    }
    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}


tResult cObstacleDetectionFilter::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);
		// by comparing it to our member pin variable we can find out which pin received
		if (pSource == &m_oVideoInputPin)
		{
			if (isFirstFrame)
			{
				cObjectPtr<IMediaType> pType;
				RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

				cObjectPtr<IMediaTypeVideo> pTypeVideo;
				RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

				const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
				if (pFormat == NULL)
				{
					LOG_ERROR("ObjectDetection: No Bitmap information found on pin");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}
				m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
				m_sInputFormat.nWidth = pFormat->nWidth;
				m_sInputFormat.nHeight =  pFormat->nHeight;
				m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
				m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
				m_sInputFormat.nSize = pFormat->nSize;
				m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
				m_oTestDepthImage.SetFormat(&m_sInputFormat, NULL);
				isFirstFrame = false;
			}
			else
			{
				RETURN_IF_FAILED(ProcessInput(pMediaSample));
			}
		}
	}
	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::ProcessInput(IMediaSample *pSample)
{
	RETURN_IF_POINTER_NULL(pSample);

	cObjectPtr<IMediaSample> pNewDepthSample;

	const tVoid* l_pSrcBuffer;
	
	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{
		IplImage *img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth,
							m_sInputFormat.nHeight),
							IPL_DEPTH_16U, 1);
		img->imageData = (char*) l_pSrcBuffer;
		Mat image(cvarrToMat(img));
		cvReleaseImage(&img);
		pSample->Unlock(l_pSrcBuffer);

		if (2 != image.type())
		{
			LOG_ERROR(cString::Format("Source Image not 16bit (type = %d)", image.type()));
		}

		findObstacles(image);
	}

	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::findObstacles(Mat &imDepth)
{
		// /////////////////////////////////
		// Examplary use of Vdar class
		
		// search in area 1.0m to 1.7m forward and 1.0m to 1.2m left
		cv::Rect searchArea(cv::Point(-120,170),cv::Point(-60,100));

		if (vdar.isObstacleInAreaInDepthImg(searchArea, imDepth))
		{
			LOG_INFO("Vdar found a obstacle");
		}

		std::vector<cv::Rect> obstacles;
		vdar.getObstaclesInAreaInDepthImg(searchArea, imDepth, obstacles);
		for (int i=0; i<(int)obstacles.size(); i++)
		{
			LOG_INFO(cString::Format("obstacle -> (%d, %d) x (%d, %d)", obstacles[i].tl().x, obstacles[i].tl().y, obstacles[i].br().x, obstacles[i].br().y));
		}

		std::vector<cv::Point> positions;
		vdar.getPositionsOfObstaclesInAreaInDepthImg(searchArea, imDepth, positions);
		for (int i=0; i<(int)positions.size(); i++)
		{
			LOG_INFO(cString::Format("positions -> (%d, %d)", positions[i].x, positions[i].y));
		}

		std::vector<int> obstacleDistances;
		vdar.getDistancesOfObstaclesInAreaInDepthImg(searchArea, imDepth, obstacleDistances);
		for (int i=0; i<(int)obstacleDistances.size(); i++)
		{
			LOG_INFO(cString::Format("obstacleDistances -> %dcm", obstacleDistances[i]));
		}

		// /////////////////////////////////
		//  generate Debug Output
		processingImage = denoiser.process(imDepth);
		try
		{
			processingImage = groundRemover.process(processingImage);
		} catch (const char* msg) {
			LOG_INFO(msg);
			LOG_INFO(cString::Format("height image = %d, size of vertLineMedian = %d",
						imDepth.rows, groundRemover.getVerLineMedianSize()));
		}
		try
		{
			birdsEyeView = birdsEyeViewer.process(processingImage);
		} catch (const char* msg) {
			LOG_INFO(msg);
		}
		boundRects.clear();
		try
		{
			boundRects = connectedComponentFinder.process(birdsEyeView);
		} catch (const char* msg) {
			LOG_INFO(msg);
			LOG_INFO(cString::Format("ConnectedComponentFinder srcImage depth = %d", processingImage.depth()));
		}

		// draw the search area to the right position in birdseyeview of size 640x480
		rectangle(birdsEyeView, cv::Point((320+searchArea.tl().x), 480-searchArea.tl().y),
								cv::Point((320+searchArea.br().x), 480-searchArea.br().y),
							   	cv::Scalar(255), 2);
		cv::imshow(cv::String("ObstacleDetectionWindow"), birdsEyeView);

		RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::createAndTransmitObstacleLocation(std::vector<SurfaceDescription>& surfaceDescriptions)
{
	if (!m_oObstacleLocation.IsConnected())
	{
		RETURN_NOERROR;
	}
	for (vector<SurfaceDescription>::iterator srfcDsc = surfaceDescriptions.begin();
			srfcDsc != surfaceDescriptions.end(); ++srfcDsc)
	{
		float leftWorldX, leftWorldZ, leftWorldY;
		coordinateConverter.depthToWorld(srfcDsc->topLeft.x,
										 srfcDsc->bottomRight.y,
										 srfcDsc->minimalIntensityValue,
										 leftWorldX, leftWorldY, leftWorldZ);
		float rightWorldX, rightWorldZ, rightWorldY;
		coordinateConverter.depthToWorld(srfcDsc->bottomRight.x,
										 srfcDsc->bottomRight.y,
										 srfcDsc->minimalIntensityValue,
										 rightWorldX, rightWorldY, rightWorldZ);

		transmitObstacleLocation(leftWorldX, rightWorldX, rightWorldZ);
		LOG_INFO(cString::Format("ObstacleDetection: id = %d, distance = %f, width = %f",
													 srfcDsc->id, rightWorldZ, abs(rightWorldX-leftWorldX)));
	}
	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::transmitObstacleLocation(tFloat leftEnd, tFloat rightEnd, tFloat distance)
{
	cObjectPtr<IMediaSample> pSample;
	// this will create a "standard" media sample, see cFilter::AllocMediaSample for more details)
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));
	tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();

	tObstacleLocation sObstacleLocation;
	sObstacleLocation.leftEnd = -leftEnd * 100.0f;  // convert meter to centimeter
	sObstacleLocation.rightEnd = -rightEnd * 100.0f;  // convert meter to centimeter
	sObstacleLocation.distance = distance * 100.0f;  // convert meter to centimeter

	pSample->Update(tmStreamTime, &sObstacleLocation, sizeof(sObstacleLocation), 0);
	m_oObstacleLocation.Transmit(pSample);

	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::createAndTransmitGCL(std::vector<SurfaceDescription>& surfaceDescriptions)
{
	if (!m_oGLCOutput.IsConnected())
	{
		RETURN_NOERROR;
	}
	cObjectPtr<IMediaSample> pSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));
	RETURN_IF_FAILED(pSample->AllocBuffer(2*8192));

	pSample->SetTime(_clock->GetStreamTime());
	tUInt32* aGCLProc;
	RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));
	
	tUInt32* pc = aGCLProc;
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());
	for (vector<SurfaceDescription>::iterator srfcDsc = surfaceDescriptions.begin();
			srfcDsc != surfaceDescriptions.end(); ++srfcDsc)
	{
		int topLeftX = srfcDsc->topLeft.x;
		int topLeftY = srfcDsc->topLeft.y;
		int bottomRightX = srfcDsc->bottomRight.x;
		int bottomRightY = srfcDsc->bottomRight.y;
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, topLeftX, topLeftY, bottomRightX, bottomRightY);
	}
	cGCLWriter::StoreCommand(pc, GCL_CMD_END);
	pSample->Unlock(aGCLProc);
	RETURN_IF_FAILED(m_oGLCOutput.Transmit(pSample));
	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::markAndTransmitDepthImage(cv::Mat& depthImage,
		std::vector<SurfaceDescription>& surfaceDescriptions)
{
	for (vector<SurfaceDescription>::iterator srfcDsc = surfaceDescriptions.begin();
			srfcDsc != surfaceDescriptions.end(); ++srfcDsc)
	{
		int topLeftX = srfcDsc->topLeft.x;
		int topLeftY = srfcDsc->topLeft.y;
		int bottomRightX = srfcDsc->bottomRight.x;
		int bottomRightY = srfcDsc->bottomRight.y;

        char cad[100];
        sprintf(cad,"id=%d",srfcDsc->id);
        //determine the centroid
        Point cent((bottomRightX+topLeftX)/2, (bottomRightY+topLeftY)/2);
        putText(depthImage, cad, cent,FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(0, 0, 0), 2);
 		rectangle(depthImage, Point(topLeftX, topLeftY), Point(bottomRightX, bottomRightY), Scalar(0, 0, 0), 3);
	}
	transmitDepthImage(depthImage);
	RETURN_NOERROR;
}


tResult cObstacleDetectionFilter::transmitDepthImage(const cv::Mat& depthImage)
{
	if (!m_oTestDepthImage.IsConnected())
	{
		RETURN_NOERROR;
	}
	// transmit data in media sample over the output pin
	cObjectPtr<IMediaSample> pDepthSample;
	if (IS_OK(AllocMediaSample(&pDepthSample)))
	{
		tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime()
						 : adtf_util::cHighResTimer::GetTime();
		pDepthSample->Update(tmStreamTime, depthImage.data, m_sInputFormat.nSize, 0);

		RETURN_IF_FAILED(m_oTestDepthImage.Transmit(pDepthSample));
	}
	RETURN_NOERROR;
}


