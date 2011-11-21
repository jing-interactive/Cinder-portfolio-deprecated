//a multi-threaded device wrapper for Microsoft Kinect SDK beta 2
//author: vinjn.z@gmail.com
//http://www.weibo.com/vinjnmelanie

/*
The Kinect sensor returns x, y, and z values in the following ranges:

Values of x can range from approximately ¨C2.2 to 2.2.
Values of y can range from approximately ¨C1.6 to 1.6.
Values of z can range from 0.0 to 4.0.
*/

#ifndef KINECT_DEVICE_H
#define KINECT_DEVICE_H

#pragma warning( disable: 4244 )

#include <windows.h>
#include <objbase.h>
#include <MMSystem.h>
#include "MSR_NuiApi.h"
#include <opencv2/opencv.hpp>
#include "../ofxThread.h"
#include "../SimpleMutex.h"

#pragma comment(lib, "winmm.lib")

#define MSG_BOX(str) ::MessageBox(NULL, TEXT(str), TEXT("kinect app"), MB_OK | MB_ICONHAND)

static char* state_desc[3] = {"not_tracked","half_tracked","tracked"};

static CvScalar default_colors[] =
{
	{{169, 176, 155}},
	{{169, 176, 155}}, 
	{{168, 230, 29}},
	{{200, 0,   0}},
	{{79,  84,  33}}, 
	{{84,  33,  42}},
	{{255, 126, 0}},
	{{215,  86, 0}},
	{{33,  79,  84}}, 
	{{33,  33,  84}},
	{{77,  109, 243}}, 
	{{37,   69, 243}},
	{{77,  109, 243}},
	{{69,  33,  84}},
	{{229, 170, 122}}, 
	{{255, 126, 0}},
	{{181, 165, 213}}, 
	{{71, 222,  76}},
	{{245, 228, 156}}, 
	{{77,  109, 243}}
};

const int sizeOfColors = sizeof(default_colors)/sizeof(CvScalar);
static CvScalar vDefaultColor(int idx){ return default_colors[idx%sizeOfColors];}

class KinectDevice;

struct DepthThread : public ofxThread
{
	DepthThread(KinectDevice& ref):ref_(ref),ofxThread("DepthThread"){}
	void threadedFunction();
	KinectDevice& ref_;
};

struct RgbThread : public ofxThread
{
	RgbThread(KinectDevice& ref):ref_(ref),ofxThread("RgbThread"){}
	void threadedFunction();
	KinectDevice& ref_;
};

struct SkeletonThread : public ofxThread
{
	SkeletonThread(KinectDevice& ref):ref_(ref),ofxThread("SkeletonThread"){}
	void threadedFunction();
	KinectDevice& ref_;
};

class KinectDevice
{
public:
	//implement use if you need us!
	virtual void onDepthEvent(const cv::Mat& depth_u16){}
	virtual void onRgbEvent(const cv::Mat& rgb){}
	virtual void onSkeletonEvent(cv::Mat& frame, const cv::Point3f* skel_points, int playerIdx ){}

public:
	void setDeviceAngle(int angle)
	{
		if (m_pNuiInstance)
		{
			if (angle < NUI_CAMERA_ELEVATION_MINIMUM)
				angle = NUI_CAMERA_ELEVATION_MINIMUM;
			else
				if (angle > NUI_CAMERA_ELEVATION_MAXIMUM)
					angle = NUI_CAMERA_ELEVATION_MAXIMUM;
			m_pNuiInstance->NuiCameraElevationSetAngle(angle);
		}
	}

	int getDeviceAngle()
	{
		LONG angle = 0;
		if (m_pNuiInstance)
		{
			m_pNuiInstance->NuiCameraElevationGetAngle(&angle);
		}
		return angle;
	}

	static int getDeviceCount()
	{
		int count = 0;
		HRESULT hr = MSR_NUIGetDeviceCount(&count);
		return count;
	}

	INuiInstance* m_pNuiInstance;
	int m_DeviceId;
	KinectDevice(int deviceId = 0)
	{
		iplColor = NULL;
		m_DeviceId = deviceId;
		m_fps = 0;
		m_pNuiInstance = NULL;
		evt_devicequit = NULL;
		evt_nextDepth = NULL;
		evt_nextRgb = NULL;
		evt_nextSkeleton = NULL;
		m_pDepthStreamHandle = NULL;
		m_pVideoStreamHandle = NULL;
	}
	virtual ~KinectDevice()
	{
		release();
	}

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
	cv::Ptr<ofxThread> threads_[3];
	cv::Scalar_<uchar> Nui_ShortToQuad_Depth( USHORT s );
	void Nui_DrawSkeletonSegment(cv::Mat& frame, NUI_SKELETON_DATA * pSkel, int numJoints, ... );
	cv::Point3f   m_Points[NUI_SKELETON_POSITION_COUNT];

private:
	void Nui_GotDepthAlert();
	void Nui_GotRgbAlert();
	void Nui_GotSkeletonAlert();

	int	m_fps;
	int	m_LastSkeletonFoundTime;
	bool m_bScreenBlanked;
	int	m_FramesTotal;
	int	m_LastFPStime;
	int	m_LastFramesTotal;

	HANDLE evt_devicequit;
	HANDLE evt_nextDepth;
	HANDLE evt_nextRgb;
	HANDLE evt_nextSkeleton;
	HANDLE m_pDepthStreamHandle;
	HANDLE m_pVideoStreamHandle;

	friend struct DepthThread;
	friend struct RgbThread;
	friend struct SkeletonThread;
};

void KinectDevice::Nui_GotDepthAlert( )
{
	// Perform FPS processing
	int t = timeGetTime( );
	if( m_LastFPStime == -1 )
	{
		m_LastFPStime = t;
		m_LastFramesTotal = m_FramesTotal;
	}
	int dt = t - m_LastFPStime;
	if( dt > 1000 )
	{
		m_LastFPStime = t;
		m_fps = m_FramesTotal - m_LastFramesTotal;
		m_LastFramesTotal = m_FramesTotal;
		//printf("%.1f\n", FrameDelta);
		//	SetDlgItemInt( m_hWnd, IDC_FPS, FrameDelta,FALSE );
	}
	m_FramesTotal++;

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

		char buf[10];
		sprintf(buf,"fps: %d", m_fps);
		cv::putText(renderedDepth, buf, cv::Point(20,20), 0, 0.6, CV_RGB(255,255,255));

		onDepthEvent(rawDepth);
	}
	else
	{
		OutputDebugString( L"Buffer length of received texture is bogus\r\n" );
	}

	m_pNuiInstance->NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );
}


void KinectDevice::Nui_GotRgbAlert( )
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
	// Perform skeletal panel blanking
	int t = timeGetTime( );

	if( m_LastSkeletonFoundTime == -1 )
		m_LastSkeletonFoundTime = t;
	int dt = t - m_LastSkeletonFoundTime;
	if( dt > 250 )
	{
		if( !m_bScreenBlanked )
		{
			renderedSkeleton = cv::Scalar(0,0,0);
			m_bScreenBlanked = true;
		}
	}

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
	char buf[100];
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
		{
			NUI_SKELETON_DATA* pSkel = &SkeletonFrame.SkeletonData[i];
			for (int k = 0; k < NUI_SKELETON_POSITION_COUNT; k++)
			{
				float fx=0,fy=0;
				USHORT uDepth;
				NuiTransformSkeletonToDepthImageF( pSkel->SkeletonPositions[k], &fx, &fy, &uDepth);

				//for 2d rendering
				m_Points[k].x = fx;
				m_Points[k].y = fy;
				m_Points[k].z = pSkel->SkeletonPositions[k].z;

				cv::Point pos(m_Points[k].x*640+10, m_Points[k].y*480);
				cv::circle(renderedSkeleton, pos, 5, vDefaultColor(k),3, -1);
				sprintf(buf, "%.1f m", m_Points[k].z);
				cv::putText(renderedSkeleton, buf, cv::Point(pos.x, pos.y), 0, 0.5, CV_RGB(255,255,255));
			}

			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,4,NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_HEAD);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);

			onSkeletonEvent(renderedSkeleton, m_Points, i ); 
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

	evt_nextRgb = CreateEvent( NULL, TRUE, FALSE, NULL );
	evt_nextDepth = CreateEvent( NULL, TRUE, FALSE, NULL );
	evt_nextSkeleton = CreateEvent( NULL, TRUE, FALSE, NULL );

	if (isColor)
	{
		hr = m_pNuiInstance->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
			NUI_IMAGE_RESOLUTION_640x480,
			0,
			2,
			evt_nextRgb,
			&m_pVideoStreamHandle );
		if( FAILED( hr ) )
		{
			MSG_BOX("failed to NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR)");
			return hr;
		}
		threads_[0] = new DepthThread(*this);
		threads_[0]->startThread();
	}

	if (isDepth)
	{
		hr = m_pNuiInstance->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
			NUI_IMAGE_RESOLUTION_320x240,
			0,
			2,
			evt_nextDepth,
			&m_pDepthStreamHandle );
		if( FAILED( hr ) )
		{
			MSG_BOX("failed to NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX)");
			return hr;
		}
		threads_[1] = new DepthThread(*this);
		threads_[1]->startThread();
	}

	if (isSkeleton)
	{
		hr = m_pNuiInstance->NuiSkeletonTrackingEnable( evt_nextSkeleton, 0 );
		if( FAILED( hr ) )
		{
			MSG_BOX("failed to NuiSkeletonTrackingEnable()");
			return hr;
		}
		threads_[2] = new SkeletonThread(*this);
		threads_[2]->startThread();
	}

	// Start the Nui processing thread
	evt_devicequit = CreateEvent(NULL,FALSE,FALSE,NULL);

	m_FramesTotal = 0;

	return hr;
}

void KinectDevice::release()
{
	// Stop the Nui processing thread
	if(evt_devicequit!=NULL)
	{
		// Signal the thread
		SetEvent(evt_devicequit);

		for (int i=0;i<3;i++)
		{
			// Wait for thread to stop
			if(threads_[i]!=NULL)
			{
				//	WaitForSingleObject(threads_[i]->th,INFINITE);
			}
		}

		CloseHandle(evt_devicequit);
	}

	if (iplColor)
		cvReleaseImageHeader(&iplColor);

	if (m_pNuiInstance)
		m_pNuiInstance->NuiShutdown();

	MSR_NuiDestroyInstance(m_pNuiInstance);

	if( evt_nextSkeleton && ( evt_nextSkeleton != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( evt_nextSkeleton );
		evt_nextSkeleton = NULL;
	}
	if( evt_nextDepth && ( evt_nextDepth != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( evt_nextDepth );
		evt_nextDepth = NULL;
	}
	if( evt_nextRgb && ( evt_nextRgb != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( evt_nextRgb );
		evt_nextRgb = NULL;
	}
}


void KinectDevice::Nui_DrawSkeletonSegment(cv::Mat& frame, NUI_SKELETON_DATA * pSkel, int numJoints, ... )
{
	va_list vl;
	va_start(vl,numJoints);
	cv::Point prev;

	for (int i = 0; i < numJoints; i++)
	{
		NUI_SKELETON_POSITION_INDEX jointIndex = va_arg(vl,NUI_SKELETON_POSITION_INDEX);

		cv::Point curr(m_Points[jointIndex].x*640, m_Points[jointIndex].y*480);
		if (pSkel->eSkeletonPositionTrackingState[jointIndex] != NUI_SKELETON_POSITION_NOT_TRACKED)
		{
			if (i > 0)
				cv::line(renderedSkeleton, prev, curr, vDefaultColor(i), 4);
			prev = curr;
		}
		else if (i == 0)
			prev = curr;
  	}
	va_end(vl);
}

void DepthThread::threadedFunction()
{
	while (isThreadRunning())
	{
		::WaitForSingleObject(ref_.evt_nextDepth, INFINITE);
		ref_.Nui_GotDepthAlert();
	}
}

void RgbThread::threadedFunction()
{
	while (isThreadRunning())
	{
		::WaitForSingleObject(ref_.evt_nextRgb, INFINITE);
		ref_.Nui_GotRgbAlert();
	}
}

void SkeletonThread::threadedFunction()
{
	while (isThreadRunning())
	{
		::WaitForSingleObject(ref_.evt_nextSkeleton, INFINITE);
		ref_.Nui_GotSkeletonAlert();
	}
}

#endif