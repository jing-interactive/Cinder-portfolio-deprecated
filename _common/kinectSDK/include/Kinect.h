/*
* 
* Copyright (c) 2011, Ban the Rewind
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

// Includes
#include "Wincludes.h"
#include <cinder/app/App.h>
#include <cinder/Cinder.h>
#include <cinder/Thread.h>
#include <cinder/Utilities.h>
#include <MSR_NuiApi.h>
#include <map>
#include <vector>

// Imports
using std::map;
using std::vector;
using boost::int32_t;
using boost::uint8_t;
using boost::uint16_t;
using boost::uint32_t;

// Join name alias
//typedef NUI_IMAGE_RESOLUTION NUI_IMAGE_RESOLUTION;
typedef NUI_SKELETON_POSITION_INDEX JointName;
typedef std::pair<JointName, ci::Vec3f> Joint;
typedef std::map<JointName, ci::Vec3f> Skeleton;

// Kinect NUI wrapper for Cinder
class Kinect
{

private:
    
	// Internal object
	class Obj
	{

	public:

		// Con/de-structor
		Obj(const NUI_IMAGE_RESOLUTION & videoResolution, const NUI_IMAGE_RESOLUTION & depthResolution);
		~Obj();

		// Start/stop capturing
		void start();
		void stop();

		// Kinect data getters
		const ci::Surface8u getDepth();
		const std::vector<Skeleton> getSkeletons();
		const int32_t getUserCount();
		const ci::Surface8u getVideo();

		// Remove background for better user tracking
		void removeBackground(bool remove);

		// Capturing flag
		bool mCapture;
		
		// User colors
		std::vector<ci::Colorf> mColors;

		// Flags to enable each feature
		void enableBinaryMode(bool enable = true, bool invertImage = false);
		void enableDepth(bool enable);
		void enableSkeletons(bool enable);
		void enableVideo(bool enable);
		bool mEnabledDepth;
		bool mEnabledSkeletons;
		bool mEnabledVideo;

		// Flags if data is new
		bool mNewDepthFrame;
		bool mNewSkeletons;
		bool mNewVideoFrame;

		// Frame rates
		float mFrameRateDepth;
		float mFrameRateSkeletons;
		float mFrameRateVideo;

		// Binary mode
		bool mBinary;
		bool mInverted;

	private:

		// Maximum wait time in milliseconds for new Kinect data
		static const int32_t WAIT_TIME = 250;
		
		// Initialize properties
		void init();
		
		// Kinect output data
		ci::Surface8u mDepthSurface;
		std::vector<Skeleton> mSkeletons;
		ci::Surface8u mVideoSurface;

		// Image resolution
		NUI_IMAGE_RESOLUTION mDepthResolution;
		NUI_IMAGE_RESOLUTION mVideoResolution;
		int32_t mDepthHeight;
		int32_t mDepthWidth;
		int32_t mVideoHeight;
		int32_t mVideoWidth;

		// Skeleton
		tagPOINT mPoints[NUI_SKELETON_POSITION_COUNT];
		HBITMAP__ * mSkeletonBmp;
		HDC__ * mSkeletonDc;
		void * mSkeletonOldObj;

		// Image streams
		void * mDepthStreamHandle;
		void * mVideoStreamHandle;

		// Set to true to set background to black in depth image
		bool mRemoveBackground;

		// Threading
		boost::mutex mMutexDepth;
		boost::mutex mMutexSkeletons;
		boost::mutex mMutexVideo;
		boost::thread mThreadDepth;
		boost::thread_group mThreadGroup;
		boost::thread mThreadSkeletons;
		boost::thread mThreadVideo;
		void processDepth();
		void processSkeletons();
		void processVideo();

		// Image data
		tagRGBQUAD * mRgbDepth;
		tagRGBQUAD * mRgbVideo;
		void quadToSurface(ci::Surface8u & surface, uint8_t * buffer, bool depth = false);
		const tagRGBQUAD shortToQuad(uint16_t value);
    
		// Frame rate
		double mReadTimeDepth;
		double mReadTimeSkeletons;
		double mReadTimeVideo;

		// User status
		int32_t mUserCount;
		bool mActiveUsers[NUI_SKELETON_COUNT];

		// Debug
		void trace(const std::string & message);

	};

	// Object reference
	typedef std::shared_ptr<Obj> ObjRef;
	ObjRef mObj;

public:

	// Con/de-structor
	Kinect(NUI_IMAGE_RESOLUTION videoResolution = NUI_IMAGE_RESOLUTION_640x480, 
		NUI_IMAGE_RESOLUTION depthResolution = NUI_IMAGE_RESOLUTION_320x240)
		: mObj(ObjRef(new Obj(videoResolution, depthResolution)))
	{}
	~Kinect()
	{
		mObj->stop();
	}

	// Start capturing
	void start() { mObj->start(); }
	void stop() { mObj->stop(); }

	// Remove background for better user tracking
	void removeBackground(bool remove = true) { mObj->removeBackground(remove); }

	// Getters
	const bool checkNewDepthFrame() { return mObj->mNewDepthFrame; }
	const bool checkNewSkeletons() { return mObj->mNewSkeletons; }
	const bool checkNewVideoFrame() { return mObj->mNewVideoFrame; }
	const ci::Surface8u getDepth() { return mObj->getDepth(); }
	const float getDepthFrameRate() { return mObj->mFrameRateDepth; }
	const float getSkeletonsFrameRate() { return mObj->mFrameRateSkeletons; }
	const float getVideoFrameRate() { return mObj->mFrameRateVideo; }
	const vector<Skeleton> getSkeletons() { return mObj->getSkeletons(); }
	const ci::Colorf getUserColor(uint32_t id) { return mObj->mColors[ci::math<uint32_t>::clamp(id, 0, 5)]; }
	const uint32_t getUserIdByColor(const ci::Colorf & color);
	const uint32_t getUserIdByColor(uint8_t r, uint8_t g, uint8_t b);
	const uint32_t getUserIdByColor(float r, float g, float b);
	const uint32_t getUserCount() { return mObj->getUserCount(); }
	const ci::Surface8u getVideo() { return mObj->getVideo(); }
	const bool isCapturing() { return mObj->mCapture; }
	
	// Setters
	void enableBinaryMode(bool enable = true, bool invertImage = false) { mObj->enableBinaryMode(enable, invertImage); }
	void enableDepth(bool enable = true) { mObj->enableDepth(enable); }
	void enableSkeletons(bool enable = true) { mObj->enableSkeletons(enable); }
	void enableVideo(bool enable = true) { mObj->enableVideo(enable); }

};
