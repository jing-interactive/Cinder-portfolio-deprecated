//wrapper for Microsoft Kinect SDK beta 2
//author: vinjn.z@gmail.com

/*
The Kinect sensor returns x, y, and z values in the following ranges:

Values of x can range from approximately ¨C2.2 to 2.2.
Values of y can range from approximately ¨C1.6 to 1.6.
Values of z can range from 0.0 to 4.0.
*/

#ifndef KINECT_DEVICE_H
#define KINECT_DEVICE_H

#include <windows.h>
#include <objbase.h>
#include <MMSystem.h>
#include "MSR_NuiApi.h"
#include <opencv2/opencv.hpp>
#include "../ofxThread.h"

#pragma comment(lib, "winmm.lib")

#define MSG_BOX(str) ::MessageBox(NULL, TEXT(str), TEXT("kinect app"), MB_OK | MB_ICONHAND)

static char* state_desc[3] = {"not_tracked","half_tracked","tracked"};

class KinectDevice : public ofxThread
{

public:
	//implement use if you need us!
	virtual void onDepthEvent(const cv::Mat& depth_u16){}
	virtual void onRgbEvent(const cv::Mat& rgb){}
	virtual void onSkeletonEvent(cv::Mat& frame, NUI_SKELETON_DATA * pSkel, int playerIdx ){}

public:
	static int getDeviceCount()
	{
		int count = 0;
		HRESULT hr = MSR_NUIGetDeviceCount(&count);
		return count;
	}

	INuiInstance*           m_pNuiInstance;
	int m_DeviceId;
	KinectDevice(int deviceId = 0)
	{
		iplColor = NULL;
		m_DeviceId = deviceId;
		m_fps = 0;
		m_pNuiInstance = NULL;
		m_hEvNuiProcessStop = NULL;
		m_hNextDepthFrameEvent = NULL;
		m_hNextVideoFrameEvent = NULL;
		m_hNextSkeletonEvent = NULL;
		m_pDepthStreamHandle = NULL;
		m_pVideoStreamHandle = NULL;
	}
	virtual ~KinectDevice()
	{
		release();
	}

	void threadedFunction();

	cv::Mat renderedSkeleton;//rendered
	IplImage* iplColor;//for rendering
	cv::Mat renderedDepth;//rendered
	cv::Mat rawDepth;//raw data

	HRESULT setup(bool isColor = false, bool isDepth = true, bool isSkeleton = true);
	void release();

	static std::string getStateString(int state )
	{
		return state_desc[state];
	}

	int getFps() const{return m_fps;}

protected:
	cv::Point3f   m_Points[NUI_SKELETON_POSITION_COUNT];
	cv::Scalar_<uchar> Nui_ShortToQuad_Depth( USHORT s );
private:
	void Nui_GotDepthAlert();
	void Nui_GotVideoAlert();
	void Nui_GotSkeletonAlert();

	int	m_fps;
	int           m_LastSkeletonFoundTime;
	bool          m_bScreenBlanked;
	int           m_FramesTotal;
	int           m_LastFPStime;
	int           m_LastFramesTotal;

	HANDLE        m_hEvNuiProcessStop;
	HANDLE        m_hNextDepthFrameEvent;
	HANDLE        m_hNextVideoFrameEvent;
	HANDLE        m_hNextSkeletonEvent;
	HANDLE        m_pDepthStreamHandle;
	HANDLE        m_pVideoStreamHandle;
};

void KinectDevice::Nui_GotDepthAlert( )
{
	const NUI_IMAGE_FRAME * pImageFrame = NULL;

	HRESULT hr = m_pNuiInstance->NuiImageStreamGetNextFrame(
		m_pDepthStreamHandle,
		0,
		&pImageFrame );

	if( FAILED( hr ) )
	{
		return;
	}

	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		BYTE * pixels = (BYTE*)renderedDepth.ptr();
		USHORT * pBufferRun = (USHORT*) pBuffer;
		USHORT * pRaw = rawDepth.ptr<ushort>();
		int i=0;
		for( int i =0;i<320*240;i++,pBufferRun++,pRaw++)
		{ 
			cv::Scalar_<uchar> quad = Nui_ShortToQuad_Depth( *pBufferRun );

			pixels[i*3+0] = quad.val[0];
			pixels[i*3+1] = quad.val[1];
			pixels[i*3+2] = quad.val[2];

			*pRaw = (*pBufferRun & 0xfff8) >> 3;
		}
		onDepthEvent(rawDepth);
	}
	else
	{
		OutputDebugString( L"Buffer length of received texture is bogus\r\n" );
	}

	m_pNuiInstance->NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );
}


void KinectDevice::Nui_GotVideoAlert( )
{
	const NUI_IMAGE_FRAME * pImageFrame = NULL;

	HRESULT hr = m_pNuiInstance->NuiImageStreamGetNextFrame(
		m_pVideoStreamHandle,
		0,
		&pImageFrame );
	if( FAILED( hr ) )
	{
		return;
	}

	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		cvSetData(iplColor, LockedRect.pBits, iplColor->widthStep);
		// 		BYTE * pixels = (BYTE*)color->imageData;
		// 		DWORD * pBufferRun = (DWORD*) LockedRect.pBits;
		// 		for( int i =0;i<640*480;i++,pBufferRun++)
		// 		{ 
		// 			pixels[i*4+0] = GetRValue(*pBufferRun);
		// 			pixels[i*4+1] = GetGValue(*pBufferRun);
		// 			pixels[i*4+2] = GetBValue(*pBufferRun);
		// 		}
		cv::Mat ref(iplColor);
		onRgbEvent(ref);
	}
	else
	{
		OutputDebugString( L"Buffer length of received texture is bogus\r\n" );
	}

	m_pNuiInstance->NuiImageStreamReleaseFrame( m_pVideoStreamHandle, pImageFrame );
}

void KinectDevice::Nui_GotSkeletonAlert( )
{
	NUI_SKELETON_FRAME SkeletonFrame;

	HRESULT hr = m_pNuiInstance->NuiSkeletonGetNextFrame( 0, &SkeletonFrame );

	bool bFoundSkeleton = false;
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
		{
			bFoundSkeleton = true;
		}
	}

	// no skeletons!
	//
	if(!bFoundSkeleton )
	{
		return;
	}

	// smooth out the skeleton data
	m_pNuiInstance->NuiTransformSmooth(&SkeletonFrame,NULL);

	// we found a skeleton, re-start the timer
	m_bScreenBlanked = false;
	m_LastSkeletonFoundTime = -1;

	// draw each skeleton color according to the slot within they are found.
	// 
	renderedSkeleton = cv::Scalar(0,0,0);
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
		{
			onSkeletonEvent(renderedSkeleton, &SkeletonFrame.SkeletonData[i], i ); 
		}
	}
}

cv::Scalar_<uchar> KinectDevice::Nui_ShortToQuad_Depth( ushort s )
{
	USHORT RealDepth = (s & 0xfff8) >> 3;
	USHORT Player = s & 7;

	// transform 13-bit depth information into an 8-bit intensity appropriate
	// for display (we disregard information in most significant bit)
	BYTE l = 255 - (BYTE)(256*RealDepth/0x0fff);

	int r,g,b;
	r = g = b = 0;

	switch( Player )
	{
	case 0:
		r = l / 2;
		g = l / 2;
		b = l / 2;
		break;
	case 1:
		r = l;
		break;
	case 2:
		g = l;
		break;
	case 3:
		r= l / 4;
		g= l;
		b= l;
		break;
	case 4:
		r = l;
		g = l;
		b = l / 4;
		break;
	case 5:
		r = l;
		g = l / 4;
		b = l;
		break;
	case 6:
		r = l / 2;
		g = l / 2;
		b = l;
		break;
	case 7:
		r = 255 - ( l / 2 );
		g = 255 - ( l / 2 );
		b = 255 - ( l / 2 );
	}

	return cv::Scalar_<uchar>(r,g,b);
}

void KinectDevice::threadedFunction()
{
	//	CSkeletalViewerApp *pthis=(CSkeletalViewerApp *) pParam;
	HANDLE                hEvents[4];
	int                    nEventIdx,t,dt;

	// Configure events to be listened on
	hEvents[0]=m_hEvNuiProcessStop;
	hEvents[1]=m_hNextDepthFrameEvent;
	hEvents[2]=m_hNextVideoFrameEvent;
	hEvents[3]=m_hNextSkeletonEvent;

	// Main thread loop
	while (true)
	{
		// Wait for an event to be signaled
		nEventIdx=WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),hEvents,FALSE,100);

		// If the stop event, stop looping and exit
		if(nEventIdx==0)
			break;            

		// Perform FPS processing
		t = timeGetTime( );
		if( m_LastFPStime == -1 )
		{
			m_LastFPStime = t;
			m_LastFramesTotal = m_FramesTotal;
		}
		dt = t - m_LastFPStime;
		if( dt > 1000 )
		{
			m_LastFPStime = t;
			m_fps = m_FramesTotal - m_LastFramesTotal;
			m_LastFramesTotal = m_FramesTotal;
			//printf("%.1f\n", FrameDelta);
			//	SetDlgItemInt( m_hWnd, IDC_FPS, FrameDelta,FALSE );
		}

		// Perform skeletal panel blanking
		if( m_LastSkeletonFoundTime == -1 )
			m_LastSkeletonFoundTime = t;
		dt = t - m_LastSkeletonFoundTime;
		if( dt > 250 )
		{
			if( !m_bScreenBlanked )
			{
				renderedSkeleton = cv::Scalar(0,0,0);
				m_bScreenBlanked = true;
			}
		}

		// Process signal events
		switch(nEventIdx)
		{
		case 1:
			Nui_GotDepthAlert();
			m_FramesTotal++;
			break;

		case 2:
			Nui_GotVideoAlert();
			break;

		case 3:
			Nui_GotSkeletonAlert( );
			break;
		}
	}
}

HRESULT KinectDevice::setup(bool isColor, bool isDepth, bool isSkeleton)
{
	int flag = 0;
	if (isDepth) flag |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
	if (isColor) flag |= NUI_INITIALIZE_FLAG_USES_COLOR;
	if (isSkeleton) flag |= NUI_INITIALIZE_FLAG_USES_SKELETON;
	HRESULT hr = 0;

	hr = MSR_NuiCreateInstanceByIndex(m_DeviceId, &m_pNuiInstance);

	if (FAILED(hr))
	{
		MSG_BOX("Kinect is not detected.");
		return hr;
	}

	hr = m_pNuiInstance->NuiInitialize(flag);
	if( FAILED( hr ) )
	{
		MSG_BOX("Kinect is not detected.");
		return hr;
	}

	renderedSkeleton.create(cv::Size(640,480), CV_8UC3);
	iplColor = cvCreateImageHeader(cvSize(640,480), 8, 4);
	renderedDepth.create(cv::Size(320,240), CV_8UC3);
	rawDepth.create(cv::Size(320,240), CV_16UC1);

	m_hNextVideoFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_hNextSkeletonEvent = CreateEvent( NULL, TRUE, FALSE, NULL );

	if (isColor)
	{
		hr = m_pNuiInstance->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
			NUI_IMAGE_RESOLUTION_640x480,
			0,
			2,
			m_hNextVideoFrameEvent,
			&m_pVideoStreamHandle );
		if( FAILED( hr ) )
		{
			MSG_BOX("failed to NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR)");
			return hr;
		}
	}

	if (isDepth)
	{
		hr = m_pNuiInstance->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
			NUI_IMAGE_RESOLUTION_320x240,
			0,
			2,
			m_hNextDepthFrameEvent,
			&m_pDepthStreamHandle );
		if( FAILED( hr ) )
		{
			MSG_BOX("failed to NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX)");
			return hr;
		}
	}

	if (isSkeleton)
	{
		hr = m_pNuiInstance->NuiSkeletonTrackingEnable( m_hNextSkeletonEvent, 0 );
		if( FAILED( hr ) )
		{
			MSG_BOX("failed to NuiSkeletonTrackingEnable()");
			return hr;
		}
	}

	// Start the Nui processing thread
	m_hEvNuiProcessStop = CreateEvent(NULL,FALSE,FALSE,NULL);

	startThread();

	return hr;
}

void KinectDevice::release()
{
	// Stop the Nui processing thread
	if(m_hEvNuiProcessStop!=NULL)
	{
		// Signal the thread
		SetEvent(m_hEvNuiProcessStop);

		// Wait for thread to stop
		if(myThread!=NULL)
		{
			WaitForSingleObject(myThread,INFINITE);
		}
		CloseHandle(m_hEvNuiProcessStop);
	}

	if (iplColor)
		cvReleaseImageHeader(&iplColor);

	if (m_pNuiInstance)
		m_pNuiInstance->NuiShutdown();

	MSR_NuiDestroyInstance(m_pNuiInstance);

	if( m_hNextSkeletonEvent && ( m_hNextSkeletonEvent != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( m_hNextSkeletonEvent );
		m_hNextSkeletonEvent = NULL;
	}
	if( m_hNextDepthFrameEvent && ( m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( m_hNextDepthFrameEvent );
		m_hNextDepthFrameEvent = NULL;
	}
	if( m_hNextVideoFrameEvent && ( m_hNextVideoFrameEvent != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( m_hNextVideoFrameEvent );
		m_hNextVideoFrameEvent = NULL;
	}
}


#endif