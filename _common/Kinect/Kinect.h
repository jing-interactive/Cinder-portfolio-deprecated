/*
* 
* Copyright (c) 2013, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#pragma once

#include "cinder/Cinder.h"
#include "cinder/Function.h"
#include "cinder/Exception.h"
#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include "cinder/Thread.h"
#include <ole2.h> // for Windows-specific typedefs
#include "NuiApi.h"
#include <vector>

//! Kinect SDK wrapper for Cinder
namespace nui // nui -> natural user interface
{
using ci::int32_t;
using ci::uint32_t;
using ci::uint16_t;
using ci::uint8_t;

//////////////////////////////////////////////////////////////////////////////////////////////

class Bone
{
public:
    Bone(){}
    Bone(const Vector4& position, const _NUI_SKELETON_BONE_ORIENTATION& bone);

	//! Returns rotation of the bone relative to camera coordinates.
	const ci::Quatf&		getAbsoluteRotation() const;
	//! Returns rotation matrix of the bone relative to camera coordinates.
	const ci::Matrix44f&	getAbsoluteRotationMatrix() const;
	//! Returns index of end joint.
	NUI_SKELETON_POSITION_INDEX getEndJoint() const;
	//! Returns position of the bone's start joint.
	const ci::Vec3f&		getPosition() const;
	//! Returns rotation of the bone relative to the parent bone.
	const ci::Quatf&		getRotation() const;
	//! Returns rotation matrix of the bone relative to the parent bone.
	const ci::Matrix44f&	getRotationMatrix() const;
	//! Returns index of start joint.
	NUI_SKELETON_POSITION_INDEX     getStartJoint() const;
private:
	ci::Matrix44f	mAbsRotMat;
	ci::Quatf		mAbsRotQuat;
	NUI_SKELETON_POSITION_INDEX		mJointStart;
	NUI_SKELETON_POSITION_INDEX		mJointEnd;
	ci::Vec3f		mPosition;
	ci::Matrix44f	mRotMat;
	ci::Quatf		mRotQuat;
};
typedef std::vector<Bone>	Skeleton;

//////////////////////////////////////////////////////////////////////////////////////////////

class DeviceOptions
{
public:
	//! Default settings
	DeviceOptions();

	//! Returns resolution of color image.
	NUI_IMAGE_RESOLUTION    getColorResolution() const; 
	//! Returns size of color image.
	const ci::Vec2i&		getColorSize() const; 
	//! Returns resolution of depth image.
	NUI_IMAGE_RESOLUTION    getDepthResolution() const; 
	//! Returns size of depth image.
	const ci::Vec2i&		getDepthSize() const; 
	//! Returns 0-index for this device.
	int32_t					getDeviceIndex() const;
	//! Returns true if depth tracking is enabled.
	bool					isDepthEnabled() const;
	//! Returns true if background remove is enabled.
	bool					isNearModeEnabled() const; 
	//! Returns true if seated mode is enabled.
	bool					isSeatedModeEnabled() const;
	//! Returns true if skeleton tracking is enabled.
	bool					isSkeletonTrackingEnabled() const;
	//! Returns true if color video stream is enabled.
	bool					isColorEnabled() const;
	//! Returns true if user tracking is enabled.
	bool					isUserTrackingEnabled() const;

	//! Enables color stream.
	DeviceOptions&			enableColor(bool enable = true);
	//! Enables depth tracking.
	DeviceOptions&			enableDepth(bool enable = true);
	//! Enables near mode (Kinect for Windows only).
	DeviceOptions&			enableNearMode(bool enable = true); 
	/*! Enables skeleton tracking. Set \a seatedMode to true to support seated skeletons.
		Only available on first device running at 320x240. */
	DeviceOptions&			enableSkeletonTracking(bool enable = true, bool seatedMode = false);
	/*! Enables user tracking. Only available on first device running at 320x240. */
	DeviceOptions&			enableUserTracking(bool enable = true);
	//! Sets resolution of color image.
	DeviceOptions&			setColorResolution(const NUI_IMAGE_RESOLUTION resolution = NUI_IMAGE_RESOLUTION_640x480);
	//! Sets resolution of depth image.
	DeviceOptions&			setDepthResolution(const NUI_IMAGE_RESOLUTION resolution = NUI_IMAGE_RESOLUTION_320x240); 
	//! Starts device with this 0-index.
	DeviceOptions&			setDeviceIndex(int32_t index = 0); 
protected:
	bool					mEnabledColor;
	bool					mEnabledDepth;
	bool					mEnabledSeatedMode;
	bool					mEnabledSkeletonTracking;
	bool					mEnabledUserTracking;
	
	NUI_IMAGE_RESOLUTION			mColorResolution;
	ci::Vec2i				mColorSize;
	NUI_IMAGE_RESOLUTION    mDepthResolution;
	ci::Vec2i				mDepthSize;

	int32_t					mDeviceIndex;
	bool					mEnabledNearMode;
};

//////////////////////////////////////////////////////////////////////////////////////////////

// Kinect sensor interface
class Device
{
public:
	/*! Skeleton smoothing enumeration. Smoother transform improves skeleton accuracy, 
		but increases latency. */
	enum Transform
	{
		TRANSFORM_NONE, TRANSFORM_DEFAULT, TRANSFORM_SMOOTH, TRANSFORM_VERY_SMOOTH, TRANSFORM_MAX
	};

	//! Maximum number of devices supported by the Kinect SDK.
	static const int32_t			MAXIMUM_DEVICE_COUNT	= 8;
	//! Maximum device tilt angle in positive or negative degrees.
	static const int32_t			MAXIMUM_TILT_ANGLE		= 28;

    Device();
	~Device();

	//! Returns number of Kinect devices.
	static int32_t					getDeviceCount();
	//! Returns use color for user ID \a id.
	static ci::Colorf				getUserColor(uint32_t id);

	//! Start capturing using settings specified in \a deviceOptions .
	void							start(const DeviceOptions& deviceOptions = DeviceOptions());
	//! Stop capture.
	void							stop();

    bool                            checkNewDepthSurface() const {return mNewDepthSurface;}
    bool                            checkNewSkeletons() const {return mNewSkeletons;}
    bool                            checkNewColorSurface() const {return mNewColorSurface;}

    ci::Surface16u                  getDepthSurface() const;
    std::vector<Skeleton>           getSkeletons() const;
    ci::Surface8u                   getColorSurface() const;
	
	//! Convert depth image to binary. \a invertImage to flip black and white. Default is false.
	void							enableBinaryMode(bool enable = true, bool invertImage = false);
	//! Enables user colors. Depth tracking at 320x240 or less must be enabled. Default is true.
	void							enableUserColor(bool enable = true);
	//! Enables verbose error reporting in debug console. Default is true.
	void							enableVerbose(bool enable = true);

	//! Remove background for better user tracking.
	void							removeBackground(bool remove = true);

	//! Returns depth value as 0.0 - 1.0 float for pixel at \a pos.
	float							getDepthAt(const ci::Vec2i& v) const;
	//! Returns options object for this device.
	const DeviceOptions&			getDeviceOptions() const;
	//! Returns device frame rate.
	float							getFrameRate() const;
	//! Returns accelerometer reading.
	ci::Quatf						getOrientation() const;
	//! Returns current device angle in degrees between -28 and 28.
	int32_t							getTilt();
	//! Returns number of tracked users. Depth resolution must be no more than 320x240 with user tracking enabled.
	int32_t							getUserCount();

	//! Returns true is actively capturing.
	bool							isCapturing() const;

	//! Flips input horizontally if \a flipped is true.
	void							setFlipped(bool flipped = true);
	//! Returns true if input is flipped.
	bool							isFlipped() const;

	//! Returns pixel location of skeleton position in depth image.
	ci::Vec2i						getSkeletonDepthPos(const ci::Vec3f& v);
	//! Returns pixel location of skeleton position in color image.
	ci::Vec2i						getSkeletonColorPos(const ci::Vec3f& v);

	//! Returns pixel location of color position in depth image.
	ci::Vec2i						getColorDepthPos(const ci::Vec2i& v);

	//! Sets device angle to \a degrees. Default is 0.
	void							setTilt(int32_t degrees = 0);

	//! Return skeleton transform type.
	Transform						getTransform() const;
	//! Sets skeleton transform type.
	void							setTransform(Transform transform = TRANSFORM_DEFAULT);

    void                            getColorCameraSettings(INuiColorCameraSettings **pCameraSetting);

protected:
	static const int32_t			WAIT_TIME = 100;

    typedef ci::ColorT<uint16_t>	Color16u;

	void							init(bool reset = false);

	bool							mCapture;
	
	DeviceOptions					mDeviceOptions;

	float							mFrameRate;
	double							mReadTime;

	bool							mBinary;
	bool							mFlipped;
	bool							mGreyScale;
	bool							mInverted;

	Transform   					mTransform;

	void*							mColorStreamHandle;
	void*							mDepthStreamHandle;
	
	void*							mColorEvent;
	void*							mDepthEvent;
	void*							mSkeletonEvent;

	mutable bool					mNewDepthSurface;
	mutable bool					mNewSkeletons;
	mutable bool					mNewColorSurface;
	
	ci::Surface8u					mColorSurface;
	ci::Surface16u					mDepthSurface;
	__int64							mDepthTimeStamp;
	std::vector<Skeleton>			mSkeletons;

	INuiSensor*						mSensor;
	double							mTiltRequestTime;

	bool							mIsSkeletonDevice;

	long							openColorStream();
	long							openDepthStream();
	
	bool							mRemoveBackground;

	volatile bool					mRunning;
	std::shared_ptr<std::thread>	mThread;
	void							run();

	Color16u*						mRgbDepth;
    ci::Color8u*                    mRgbColor;
	void							pixelToDepthSurface(uint16_t* buffer);
	void							pixelToColorSurface(uint8_t* buffer);
	Color16u						shortToPixel(uint16_t value);

	void							deactivateUsers();
	volatile int32_t				mUserCount;
	bool							mActiveUsers[ NUI_SKELETON_COUNT ];

	void							error(long hr);
	bool							mVerbose;

	friend void __stdcall			deviceStatus(long hr, const WCHAR* instanceName, const WCHAR* deviceId, void* data);

	//////////////////////////////////////////////////////////////////////////////////////////////

public:
	class Exception : public ci::Exception
	{
	public:
		const char* what() const throw();
	protected:
		char			mMessage[ 2048 ];
		friend class	Device;
	};

	//! Exception representing failure to create device.
	class ExcDeviceCreate : public Exception 
	{
	public:
		ExcDeviceCreate(long hr, const std::string& id) throw();
	};

	//! Exception representing failure to initialize device.
	class ExcDeviceInit : public Exception 
	{
	public:
		ExcDeviceInit(long hr, const std::string& id) throw();
	};

	//! Exception representing attempt to create device with invalid index or ID.
	class ExcDeviceInvalid : public Exception 
	{
	public:
		ExcDeviceInvalid(long hr, const std::string& id) throw();
	};

	//! Exception representing failure to open color stream.
	class ExcOpenStreamColor : public Exception
	{
	public:
		ExcOpenStreamColor(long hr) throw();
	};

	//! Exception representing failure to open depth stream.
	class ExcOpenStreamDepth : public Exception
	{
	public:
		ExcOpenStreamDepth(long hr) throw();
	};

	//! Exception representing failure to enable skeleton tracking.
	class ExcSkeletonTrackingEnable : public Exception
	{
	public:
		ExcSkeletonTrackingEnable(long hr) throw();
	};
};
}
 