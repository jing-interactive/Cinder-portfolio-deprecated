//a multi-threaded device wrapper for Microsoft Kinect SDK V1.0
//author: vinjn.z@gmail.com
//http://www.weibo.com/vinjnmelanie
//
//Note: although more than one Kinect is supported at one PC, only one device is capable of skeleton tracking.
//You will get warning if more than one device is initialized with SkeletonTracking flag enabled.
//

/*
The Kinect sensor returns x, y, and z values in the following ranges:

Values of x can range from approximately -2.2 to 2.2.
Values of y can range from approximately -1.6 to 1.6.
Values of z can range from 0.8 to 4.0.
PlayerIdx 0/1/2/3/4/5
*/

#ifndef KINECT_DEVICE_H
#define KINECT_DEVICE_H

#pragma warning( disable: 4244 )

#include <windows.h>
#include <objbase.h>
#include <MMSystem.h>
#include "NuiApi.h"
#include <opencv2/opencv.hpp>

#define K_MSG(str) ::MessageBox(NULL, TEXT(str), TEXT("Kinect App"), MB_OK | MB_ICONHAND)

class MiniThread;

enum{
	DEPTH_WIDTH = 320,
	DEPTH_HEIGHT = 240,
	RGB_WIDTH = 640,
	RGB_HEIGHT = 480,
	Z_NEAR = 800,//mm
	Z_FAR = 4000,//mm
	Z_RANGE = Z_FAR - Z_NEAR,//mm
};

enum FRAME_TYPE{
	FRAME_COLOR_U8C3,	 //the rgb camera image
	FRAME_DEPTH_U16C1,	 //the raw depth stream, it's u16
	FRAME_SKELETON_U8C3, //colorful skeleton image
	FRAME_DEPTH_U8C3,	 //colorful depth image, detected person are colored
	FRAME_PLAYER_IDX_U8C1,	 //player index image
	N_FRAME_TYPES,
};

struct KinectDelegate
{
	//////////////////////////////////////////////////////////////////////////
	//basic callbacks

	//called when depth stream comes, which stored at cv::Mat& depth_u16
	//A depth data value of 0 indicates that no depth data is available at that position because all the objects 
	//were either too close to the camera or too far away from it.
	virtual void onDepthData(const cv::Mat& depth_u16c1, const cv::Mat& depth_u8c3, const cv::Mat& playerIdx_u8c1){}

	//called when rgb stream comes, which stored at cv::Mat& rgb
	virtual void onRgbData(const cv::Mat& rgb){}

	//called when a skeleton is found, the joint data are stored at skel_points
	//playerIdx indicates the player index, which ranges from 0 to 5
	//isNewPlayer is true when it's the playerIdx didn't appear at previous frame
	virtual void onPlayerData(cv::Point3f skel_points[NUI_SKELETON_POSITION_COUNT], int playerIdx, bool isNewPlayer,
		NUI_SKELETON_DATA* rawData){}

	//////////////////////////////////////////////////////////////////////////
	//advanced callbacks

	//it's useful when you want to do initialization before/after each skeleton data comes 
	virtual void onSkeletonEventBegin(){}
	virtual void onSkeletonEventEnd(){}

	//called when a player enters
	virtual void onPlayerEnter(int playerIdx){}

	//called when a player leaves
	virtual void onPlayerLeave(int playerIdx){}
};

class KinectDevice
{
public:
	static int getDeviceCount();

	KinectDevice(int deviceId = 0);
	virtual ~KinectDevice();

	bool setup(bool isColor = false, bool isDepth = true, bool isSkeleton = true, KinectDelegate* delegate = NULL);	

	bool checkNewFrame(FRAME_TYPE type) const;
	const cv::Mat getFrame(FRAME_TYPE type);

	//////////////////////////////////////////////////////////////////////////
	//motor control
	//angle ~ [0, 54]
	void setDeviceAngle(int angle);
	//return [0, 54]
	int getDeviceAngle();

	NUI_TRANSFORM_SMOOTH_PARAMETERS& smoothParam();
	
protected:
	NUI_TRANSFORM_SMOOTH_PARAMETERS smooth_param;

	cv::Mat renderedSkeleton;//for rendered
	IplImage* iplColor;//for rendering
	cv::Mat renderedDepth;//for rendering
	cv::Mat rawDepth;//raw data, not quite suitable for rendering
	cv::Mat playerIdx;//player index map, not suitable for rendering

	cv::Point2f getUVFromDepthPixel(int x, int y, ushort depthValue );
	INuiSensor* m_pNuiSensor;//the object represents the physical Kinect device

	int m_DeviceId;
	cv::Point3f   m_PrevPoints[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT];
	cv::Point3f   m_Points[NUI_SKELETON_POSITION_COUNT];
	float	m_Confidences[NUI_SKELETON_POSITION_COUNT];

	bool isPlayerVisible[NUI_SKELETON_COUNT];//valid id->0/1/2/3/4/5, it's not dwTrackingID
	KinectDelegate* _delegate;

private:
	void release();

	static void registerDeviceCallback();
	static void CALLBACK onDeviceStatus(HRESULT hrStatus, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceName, void* pUserData);

	void Nui_GotDepthAlert();

	void Nui_GotRgbAlert();

	void Nui_GotSkeletonAlert();

	cv::Ptr<MiniThread> _threads[3];

	cv::Vec3b Nui_ShortToQuad_Depth(ushort s);

	void Nui_DrawSkeletonSegment(cv::Mat& frame, NUI_SKELETON_DATA * pSkel, int numJoints, ... );

	int	m_LastSkeletonFoundTime;
	bool m_bScreenBlanked;

	int fps_depth;
	int fps_rgb;
	int fps_skeleton;

	HANDLE evt_nextDepth;
	HANDLE evt_nextRgb;
	HANDLE evt_nextSkeleton;
	HANDLE m_pDepthStreamHandle;
	HANDLE m_pVideoStreamHandle;

	friend struct DepthThread;
	friend struct RgbThread;
	friend struct SkeletonThread;

	bool isNewFrames[N_FRAME_TYPES];
};

#endif