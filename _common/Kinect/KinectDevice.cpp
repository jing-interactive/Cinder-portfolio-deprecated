#include "KinectDevice.h"
#include "../MiniThread.h"

#pragma comment(lib, "winmm.lib")

#define OPENCV_VERSION CVAUX_STR(CV_MAJOR_VERSION)""CVAUX_STR(CV_MINOR_VERSION)""CVAUX_STR(CV_SUBMINOR_VERSION)
#ifdef _DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION".lib")
#else	//_DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION".lib")
#endif	//_DEBUG

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(fps) \
	do \
{ \
	static unsigned count = 0;\
	static DWORD last = timeGetTime();\
	DWORD now = timeGetTime(); \
	++count; \
	if (now - last >= 1000) \
	{ \
	fps = count; \
	count = 0; \
	last = now; \
	} \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
	do \
{ \
}while(false)
#endif 

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

const DWORD kWaitMillSec = 30; 
struct DepthThread : public MiniThread
{
	DepthThread(KinectDevice& ref):ref_(ref),MiniThread("DepthThread"){}
	void threadedFunction()
	{
		while (isThreadRunning())
		{
			if (::WaitForSingleObject(ref_.evt_nextDepth, kWaitMillSec) == WAIT_OBJECT_0)
                ref_.Nui_GotDepthAlert();
		}
	}
	KinectDevice& ref_;
};

struct RgbThread : public MiniThread
{
	RgbThread(KinectDevice& ref):ref_(ref),MiniThread("RgbThread"){}
	void threadedFunction()
	{
		while (isThreadRunning())
		{
			if (::WaitForSingleObject(ref_.evt_nextRgb, kWaitMillSec) == WAIT_OBJECT_0)
			    ref_.Nui_GotRgbAlert();
		}
	}
	KinectDevice& ref_;
};

struct SkeletonThread : public MiniThread
{
	SkeletonThread(KinectDevice& ref):ref_(ref),MiniThread("SkeletonThread"){}
	void threadedFunction()
	{
		while (isThreadRunning())
		{
			if (::WaitForSingleObject(ref_.evt_nextSkeleton, kWaitMillSec) == WAIT_OBJECT_0)
                ref_.Nui_GotSkeletonAlert();
		}
	}
	KinectDevice& ref_;
};

//angle ~ [0, 54]
void KinectDevice::setDeviceAngle(int angle)
{
	angle += NUI_CAMERA_ELEVATION_MINIMUM;
	if (m_pNuiSensor)
	{
		if (angle < NUI_CAMERA_ELEVATION_MINIMUM)
			angle = NUI_CAMERA_ELEVATION_MINIMUM;
		else
			if (angle > NUI_CAMERA_ELEVATION_MAXIMUM)
				angle = NUI_CAMERA_ELEVATION_MAXIMUM;
		m_pNuiSensor->NuiCameraElevationSetAngle(angle);
	}
}

//return [0, 54]
int KinectDevice::getDeviceAngle()
{
	LONG angle = 0;
	if (m_pNuiSensor)
	{
		m_pNuiSensor->NuiCameraElevationGetAngle(&angle);
	}
	return angle-NUI_CAMERA_ELEVATION_MINIMUM;
}

int KinectDevice::getDeviceCount()
{
	int count = 0;
	HRESULT hr = NuiGetSensorCount(&count);
	return count;
}

KinectDevice::KinectDevice(int deviceId/* = 0*/)
{
	iplColor = NULL;
	m_DeviceId = deviceId;
	m_pNuiSensor = NULL;

	evt_nextDepth = NULL;
	evt_nextRgb = NULL;
	evt_nextSkeleton = NULL;
	m_pDepthStreamHandle = NULL;
	m_pVideoStreamHandle = NULL;

	fps_depth = 0;
	fps_rgb = 0;
	fps_skeleton = 0;

	::ZeroMemory(m_PrevPoints, sizeof(m_PrevPoints));

	smooth_param.fSmoothing = 0.5f;
	smooth_param.fCorrection = 0.5f;
	smooth_param.fPrediction = 0.5f;
	smooth_param.fJitterRadius = 0.1f;
	smooth_param.fMaxDeviationRadius = 0.08f;
	//		registerDeviceCallback();
}

KinectDevice::~KinectDevice()
{
	release();
}

bool KinectDevice::setup(bool isColor/* = false*/, bool isDepth/* = true*/, bool isSkeleton/* = true*/, KinectDelegate* delegate/* = NULL*/)
{
	printf("\nKinServer is trying to launch Kinect #%d with\n", m_DeviceId);
	if (isColor)
		printf(" * RGB frame\n");
	if (isDepth)
		printf(" * depth frame\n");
	if (isSkeleton)
		printf(" * body tracking\n\n");

	_delegate = delegate;
	if (delegate == NULL)
		printf("\n [Caution] No delegate is assigned to current KinectDevice.\n");
	
	int flag = 0;
	if (isDepth) flag |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
	if (isColor) flag |= NUI_INITIALIZE_FLAG_USES_COLOR;
	if (isSkeleton) flag |= NUI_INITIALIZE_FLAG_USES_SKELETON;
	HRESULT hr = 0;

	hr = NuiCreateSensorByIndex(m_DeviceId, &m_pNuiSensor);

	if (FAILED(hr))
	{
		K_MSG("Kinect is either not connected or already opened.");
		return false;
	}

	hr = m_pNuiSensor->NuiInitialize(flag);
	if (E_NUI_SKELETAL_ENGINE_BUSY == hr)
	{
		K_MSG("Skeletal engine is used by another Kinect, thus disabled.");
		flag &= ~NUI_INITIALIZE_FLAG_USES_SKELETON;
		hr = m_pNuiSensor->NuiInitialize(flag);
	}
	if( FAILED( hr ) )
	{
		K_MSG("Kinect failed to initialize.");
		return false;
	}

	renderedSkeleton.create(cv::Size(RGB_WIDTH,RGB_HEIGHT), CV_8UC3);
	iplColor = cvCreateImageHeader(cvSize(RGB_WIDTH,RGB_HEIGHT), 8, 4);
	renderedDepth.create(cv::Size(DEPTH_WIDTH,DEPTH_HEIGHT), CV_8UC3);
	rawDepth.create(cv::Size(DEPTH_WIDTH,DEPTH_HEIGHT), CV_16UC1);
	playerIdx.create(cv::Size(DEPTH_WIDTH,DEPTH_HEIGHT), CV_8UC1);

	renderedSkeleton = cv::Scalar(0,0,0);
	renderedDepth = cv::Scalar(0,0,0);
	rawDepth = cv::Scalar(0,0,0);
	playerIdx = cv::Scalar(0,0,0);

	::ZeroMemory(isNewFrames, sizeof(isNewFrames));

	if (isColor)
	{
		evt_nextRgb = CreateEvent( NULL, TRUE, FALSE, NULL );
		hr = m_pNuiSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
#if RGB_WIDTH == 640
            NUI_IMAGE_RESOLUTION_640x480,
#else
            NUI_IMAGE_RESOLUTION_320x240,
#endif
			0,
			2,
			evt_nextRgb,
			&m_pVideoStreamHandle );
		if( FAILED( hr ) )
		{
			K_MSG("failed on NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR)");
			return false;
		}
		_threads[0] = new RgbThread(*this);
		_threads[0]->startThread();
	}

	if (isDepth)
	{
		evt_nextDepth = CreateEvent( NULL, TRUE, FALSE, NULL );
		hr = m_pNuiSensor->NuiImageStreamOpen(
			HasSkeletalEngine(m_pNuiSensor) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH,
#if DEPTH_WIDTH == 640
			NUI_IMAGE_RESOLUTION_640x480,
#else
            NUI_IMAGE_RESOLUTION_320x240,
#endif
			0,
			2,
			evt_nextDepth,
			&m_pDepthStreamHandle );
		if( FAILED( hr ) )
		{
			K_MSG("failed on NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX)");
			return false;
		}
		_threads[1] = new DepthThread(*this);
		_threads[1]->startThread();
	}

	if (isSkeleton && HasSkeletalEngine(m_pNuiSensor))
	{
		evt_nextSkeleton = CreateEvent( NULL, TRUE, FALSE, NULL );
		hr = m_pNuiSensor->NuiSkeletonTrackingEnable( evt_nextSkeleton, 0 );
		if( FAILED( hr ) )
		{
			K_MSG("failed on NuiSkeletonTrackingEnable()");
			return false;
		}
		_threads[2] = new SkeletonThread(*this);
		_threads[2]->startThread();
	}

	::ZeroMemory(isPlayerVisible, sizeof(isPlayerVisible));

	return true;
}

void KinectDevice::release()
{
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

	for (int i=0;i<3;i++)
	{
		if (!_threads[i].empty())
			_threads[i]->stopThread();
	}

	if (iplColor)
		cvReleaseImageHeader(&iplColor);

	if (m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
		m_pNuiSensor->Release();
		m_pNuiSensor = NULL;
	}
}

cv::Point2f KinectDevice::getUVFromDepthPixel(int x, int y, ushort depthValue ) 
{		
	LONG u,v;
	HRESULT hr = m_pNuiSensor->NuiImageGetColorPixelCoordinatesFromDepthPixel(
		NUI_IMAGE_RESOLUTION_640x480,NULL,x,y,depthValue<<3,&u,&v);
	return cv::Point2f(u/(float)RGB_WIDTH, v/(float)RGB_HEIGHT);
}

void KinectDevice::registerDeviceCallback()
{
	static bool first_time = true;
	if (first_time)
	{
		first_time = false;
		NuiSetDeviceStatusCallback(onDeviceStatus, NULL);
	}
}

void CALLBACK KinectDevice::onDeviceStatus(HRESULT hrStatus, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceName, void* pUserData)
{
	//TODO: implement it!!
	//K_MSG(instanceName);
	if (FAILED(hrStatus))
		K_MSG("Kinect disconnected!");
}

void KinectDevice::Nui_GotDepthAlert()
{
	FPS_CALC(fps_depth);

	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(
		m_pDepthStreamHandle,
		0,
		&imageFrame );

	if( FAILED( hr ) )
	{
		return;
	}

	//TODO: make depth image types optional
	isNewFrames[FRAME_PLAYER_IDX_U8C1] = true;
	isNewFrames[FRAME_DEPTH_U16C1] = true;
	isNewFrames[FRAME_DEPTH_U8C3] = true;

	INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		BYTE * pixels = (BYTE*)renderedDepth.ptr();
		USHORT * pBufferRun = (USHORT*) pBuffer;
		ushort * pRaw = rawDepth.ptr<ushort>();
		uchar * pIdx = playerIdx.ptr<uchar>();
		for( int i =0;i<DEPTH_WIDTH*DEPTH_HEIGHT;i++,pBufferRun++,pRaw++,pIdx++)
		{ 
			//The low-order 3 bits (bits 0-2) contain the skeleton (player) ID.
			//The high-order bits (bits 3¨C15) contain the depth value in millimeters. 
			cv::Vec3b quad = Nui_ShortToQuad_Depth( *pBufferRun );

			pixels[i*3+0] = quad.val[0];
			pixels[i*3+1] = quad.val[1];
			pixels[i*3+2] = quad.val[2];

			*pRaw = NuiDepthPixelToDepth(*pBufferRun);//(*pBufferRun & 0xfff8) >> 3;
			*pIdx = NuiDepthPixelToPlayerIndex(*pBufferRun);
		}

		char buf[10];
		sprintf(buf,"fps: %d", fps_depth);
		cv::putText(renderedDepth, buf, cv::Point(20,20), 0, 0.6, CV_RGB(255,255,255));

		if (_delegate)
			_delegate->onDepthData(rawDepth, renderedDepth, playerIdx);
	}
	else
	{
		OutputDebugString( L"Buffer length of received texture is bogus\r\n" );
	}

	m_pNuiSensor->NuiImageStreamReleaseFrame( m_pDepthStreamHandle, &imageFrame );
}

void KinectDevice::Nui_GotRgbAlert()
{
	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(
		m_pVideoStreamHandle,
		0,
		&imageFrame );
	if( FAILED( hr ) )
	{
		return;
	}

	INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		cvSetData(iplColor, LockedRect.pBits, iplColor->widthStep);
		cv::Mat ref(iplColor);
		if (_delegate)
			_delegate->onRgbData(ref);
		isNewFrames[FRAME_COLOR_U8C3] = true;
	}
	else
	{
		OutputDebugString( L"Buffer length of received texture is bogus\r\n" );
	}

	m_pNuiSensor->NuiImageStreamReleaseFrame( m_pVideoStreamHandle, &imageFrame );
}

void KinectDevice::Nui_GotSkeletonAlert()
{
	FPS_CALC(fps_skeleton);

	//trigger the event
	if (_delegate)
		_delegate->onSkeletonEventBegin();

	NUI_SKELETON_FRAME SkeletonFrame;

	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame( 0, &SkeletonFrame );

	bool bFoundSkeletonFirstPass = false;
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
		{
			bFoundSkeletonFirstPass = true;
		}
		else
		{
			if (isPlayerVisible[i])//the player is leaving us
			{
				printf("player %d -\n",i);					
				if (_delegate)
					_delegate->onPlayerLeave(i);
			}
			isPlayerVisible[i] = false;
		}
	}

	renderedSkeleton = cv::Scalar(0,0,0);

	if(!bFoundSkeletonFirstPass )
	{
		goto _Nui_GotSkeletonAlert_exit;
	}

	// smooth out the skeleton data
	m_pNuiSensor->NuiTransformSmooth(&SkeletonFrame,&smooth_param);

	// we found a skeleton, re-start the timer
	m_bScreenBlanked = false;
	m_LastSkeletonFoundTime = -1;

	// draw each skeleton color according to the slot within they are found.
	// 
	char buf[100];
	sprintf(buf,"fps: %d", fps_skeleton);
	cv::putText(renderedSkeleton, buf, cv::Point(20,20), 0, 0.6, CV_RGB(255,255,255));

	bool bFoundSkeletonSecondPass = false;
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		NUI_SKELETON_DATA* pSkel = &SkeletonFrame.SkeletonData[i];
		if(pSkel->eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_HAND_RIGHT] == NUI_SKELETON_POSITION_NOT_TRACKED &&
			pSkel->eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_HAND_LEFT] == NUI_SKELETON_POSITION_NOT_TRACKED
			//|| pSkel->eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_HEAD] != NUI_SKELETON_POSITION_NOT_TRACKED
			)
		{//if both hands not tracked
			//TODO: replacing with SkeletonFrame.SkeletonData[i].dwTrackingID
			if (isPlayerVisible[i])//the player is leaving us
			{
				printf("player %d -\n",i);
				if (_delegate)
					_delegate->onPlayerLeave(i);
			}
			isPlayerVisible[i] = false;
		}
		else
		{
			//TODO: make use of bFoundSkeletonSecondPass
			bFoundSkeletonSecondPass = true;//good news!!

			//also means joint data is new
			isNewFrames[FRAME_SKELETON_U8C3] = true;

			LONG lx=0,ly=0;
			USHORT uDepth;

			for (int k = 0; k < NUI_SKELETON_POSITION_COUNT; k++)
			{					
				NuiTransformSkeletonToDepthImage( pSkel->SkeletonPositions[k], &lx, &ly, &uDepth);

				m_Points[k].x = lx/(float)DEPTH_WIDTH;
				m_Points[k].y = ly/(float)DEPTH_HEIGHT;
				m_Points[k].z = pSkel->SkeletonPositions[k].z;

				m_Confidences[k] = (int)pSkel->eSkeletonPositionTrackingState[k]*0.5f;

				//for rendering
				cv::Point pos(m_Points[k].x*RGB_WIDTH+10, m_Points[k].y*RGB_HEIGHT);
				cv::circle(renderedSkeleton, pos, 5, vDefaultColor(k),3, -1);
				sprintf(buf, "%.2f", m_Points[k].z);
				cv::putText(renderedSkeleton, buf, pos, 0, 0.5, CV_RGB(255,255,255));
			}
			sprintf(buf, "#%d", i);
			NuiTransformSkeletonToDepthImage(pSkel->Position, &lx, &ly, &uDepth);
			cv::putText(renderedDepth, buf, cv::Point(lx/**DEPTH_WIDTH*/, ly/**DEPTH_HEIGHT*/), 0, 0.9, CV_RGB(0,0,0));

			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,4,
				NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE, 
				NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_HEAD);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,
				NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, 
				NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, 
				NUI_SKELETON_POSITION_HAND_LEFT);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,
				NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, 
				NUI_SKELETON_POSITION_ELBOW_RIGHT, 	NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,
				NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, 
				NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, 
				NUI_SKELETON_POSITION_FOOT_LEFT);
			Nui_DrawSkeletonSegment(renderedSkeleton,pSkel,5,
				NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, 
				NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, 
				NUI_SKELETON_POSITION_FOOT_RIGHT);

			bool isNewPlayer = false;
			if (!isPlayerVisible[i])
			{
				isNewPlayer = true;
				//trigger the event
				printf("player %d +\n",i);
				if (_delegate)
					_delegate->onPlayerEnter(i);
			}
			//trigger the event
			if (_delegate)
				_delegate->onPlayerData(m_Points, i, isNewPlayer, pSkel);

			for (int k=0;k<NUI_SKELETON_POSITION_COUNT;k++)
				m_PrevPoints[i][k] = m_Points[k];

			isPlayerVisible[i] = true;
		}
		//trigger the event
		//if (bFoundSkeleton)
	}

_Nui_GotSkeletonAlert_exit:

	if (_delegate)
		_delegate->onSkeletonEventEnd();
}

static const int g_IntensityShiftByPlayerR[] = { 0, 2, 0, 2, 0, 0, 2, 0 };
static const int g_IntensityShiftByPlayerG[] = { 0, 2, 2, 0, 2, 0, 0, 1 };
static const int g_IntensityShiftByPlayerB[] = { 0, 0, 2, 2, 0, 2, 0, 2 };

cv::Vec3b KinectDevice::Nui_ShortToQuad_Depth(ushort s)
{		
	USHORT RealDepth = NuiDepthPixelToDepth(s);//(s & 0xfff8) >> 3;
	USHORT Player = NuiDepthPixelToPlayerIndex(s);// & 7;

	// transform 13-bit depth information into an 8-bit intensity appropriate
	// for display (we disregard information in most significant bit)
	BYTE l = 255 - (BYTE)(256*RealDepth/0x0fff);
	//BYTE l = (BYTE)~(RealDepth >> 4);

	uchar r = l >> g_IntensityShiftByPlayerR[Player];
	uchar g = l >> g_IntensityShiftByPlayerG[Player];
	uchar b = l >> g_IntensityShiftByPlayerB[Player];

	return cv::Vec3b(r,g,b);
}

void KinectDevice::Nui_DrawSkeletonSegment(cv::Mat& frame, NUI_SKELETON_DATA * pSkel, int numJoints, ... )
{
	va_list vl;
	va_start(vl,numJoints);
	cv::Point prev;

	for (int i = 0; i < numJoints; i++)
	{
		NUI_SKELETON_POSITION_INDEX jointIndex = va_arg(vl,NUI_SKELETON_POSITION_INDEX);

		cv::Point curr(m_Points[jointIndex].x*RGB_WIDTH, m_Points[jointIndex].y*RGB_HEIGHT);
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

const cv::Mat KinectDevice::getFrame(FRAME_TYPE type)
{
	cv::Mat img;
	switch (type)
	{
	case FRAME_COLOR_U8C3:
		img = iplColor;
		break;
	case FRAME_DEPTH_U16C1:
		img = rawDepth;
		break;
	case FRAME_SKELETON_U8C3:
		img = renderedSkeleton;
		break;
	case FRAME_DEPTH_U8C3:
		img = renderedDepth;
		break;
	case FRAME_PLAYER_IDX_U8C1:
		img = playerIdx;
		break;
	default:
		assert("please assign valid IMAGE_TYPE" && 0);
		break;
	}
	isNewFrames[type] = false;
	return img;
}

NUI_TRANSFORM_SMOOTH_PARAMETERS& KinectDevice::smoothParam()
{
	return smooth_param;
}

bool KinectDevice::checkNewFrame( FRAME_TYPE type ) const
{
	return isNewFrames[type];
}
