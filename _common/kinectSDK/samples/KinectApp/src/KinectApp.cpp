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

// Includes
#include <algorithm>
#include <AudioInput.h>
#include <boost/algorithm/string.hpp>
#include <cinder/app/AppBasic.h>
#include <cinder/Camera.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Texture.h>
#include <cinder/ImageIo.h>
#include <cinder/params/Params.h>
#include <cinder/Vector.h>
#include <cinder/Utilities.h>
#include <Kinect.h>

// Imports
using namespace ci;
using namespace ci::app;
using namespace std;

// Kinect/Cinder test app
class KinectApp : public AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void prepareSettings(Settings * settings);
	void setup();
	void shutdown();
	void update();

	// Audio callback
	void onData(float * data, int32_t size);

private:

	// Capturing flag
	bool mCapture;
	bool mCapturePrev;
	bool mBinaryMode;
	bool mBinaryModePrev;
	bool mEnabledAudio;
	bool mEnabledAudioPrev;
	bool mEnabledDepth;
	bool mEnabledDepthPrev;
	bool mEnabledSkeletons;
	bool mEnabledSkeletonsPrev;
	bool mEnabledVideo;
	bool mEnabledVideoPrev;
	bool mInverted;
	bool mInvertedPrev;

	// Audio input
	int32_t mCallbackId;
	float * mData;
	AudioInput mInput;

	// Kinect
	void drawSegment(const Skeleton & skeleton, const vector<JointName> & joints);
	Surface8u mDepthSurface;
	Kinect mKinect;
	vector<Skeleton> mSkeletons;
	Surface8u mVideoSurface;
	int32_t mUserCount;

	// Skeleton segments
	void defineBody();
	vector<JointName> mBody;
	vector<JointName> mLeftArm;
	vector<JointName> mLeftLeg;
	vector<JointName> mRightArm;
	vector<JointName> mRightLeg;
	vector<vector<JointName> > mSegments;

	// Camera
	CameraPersp mCamera;

	// Params
	float mFrameRateApp;
	float mFrameRateDepth;
	float mFrameRateSkeletons;
	float mFrameRateVideo;
	bool mFullScreen;
	params::InterfaceGl	mParams;
	bool mRemoveBackground;
	bool mRemoveBackgroundPrev;

	// Save screen shot
	void screenShot();

};

// Define body drawing
void KinectApp::defineBody()
{

	// Bail if defined
	if (mSegments.size() > 0)
		return;
	
	// Body
	mBody.push_back(NUI_SKELETON_POSITION_HIP_CENTER);
	mBody.push_back(NUI_SKELETON_POSITION_SPINE);
	mBody.push_back(NUI_SKELETON_POSITION_SHOULDER_CENTER);
	mBody.push_back(NUI_SKELETON_POSITION_HEAD);

	// Left arm
	mLeftArm.push_back(NUI_SKELETON_POSITION_SHOULDER_CENTER);
	mLeftArm.push_back(NUI_SKELETON_POSITION_SHOULDER_LEFT);
	mLeftArm.push_back(NUI_SKELETON_POSITION_ELBOW_LEFT);
	mLeftArm.push_back(NUI_SKELETON_POSITION_WRIST_LEFT);
	mLeftArm.push_back(NUI_SKELETON_POSITION_HAND_LEFT);

	// Left leg
	mLeftLeg.push_back(NUI_SKELETON_POSITION_HIP_CENTER);
	mLeftLeg.push_back(NUI_SKELETON_POSITION_HIP_LEFT);
	mLeftLeg.push_back(NUI_SKELETON_POSITION_KNEE_LEFT);
	mLeftLeg.push_back(NUI_SKELETON_POSITION_ANKLE_LEFT);
	mLeftLeg.push_back(NUI_SKELETON_POSITION_FOOT_LEFT);

	// Right arm
	mRightArm.push_back(NUI_SKELETON_POSITION_SHOULDER_CENTER);
	mRightArm.push_back(NUI_SKELETON_POSITION_SHOULDER_RIGHT);
	mRightArm.push_back(NUI_SKELETON_POSITION_ELBOW_RIGHT);
	mRightArm.push_back(NUI_SKELETON_POSITION_WRIST_RIGHT);
	mRightArm.push_back(NUI_SKELETON_POSITION_HAND_RIGHT);

	// Right leg
	mRightLeg.push_back(NUI_SKELETON_POSITION_HIP_CENTER);
	mRightLeg.push_back(NUI_SKELETON_POSITION_HIP_RIGHT);
	mRightLeg.push_back(NUI_SKELETON_POSITION_KNEE_RIGHT);
	mRightLeg.push_back(NUI_SKELETON_POSITION_ANKLE_RIGHT);
	mRightLeg.push_back(NUI_SKELETON_POSITION_FOOT_RIGHT);

	// Build skeleton drawing list
	mSegments.push_back(mBody);
	mSegments.push_back(mLeftArm);
	mSegments.push_back(mLeftLeg);
	mSegments.push_back(mRightArm);
	mSegments.push_back(mRightLeg);

}

// Render
void KinectApp::draw()
{

	// Clear window
	gl::setViewport(getWindowBounds());
	gl::clear(Colorf(0.1f, 0.1f, 0.1f));

	// We're capturing
	if (mKinect.isCapturing())
	{

		// Set up camera for 3D
		gl::setMatrices(mCamera);

		// Move skeletons down below the rest of the interface
		glPushMatrix();
		gl::translate(0.0f, -0.62f, 0.0f);

		// Iterate through skeletons
		uint32_t i = 0;
		for (vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt, i++)
			if (skeletonIt->size() == JointName::NUI_SKELETON_POSITION_COUNT)
			{

				// Set color
				gl::color(mKinect.getUserColor(i));

				// Draw joints
				for (Skeleton::const_iterator jointIt = skeletonIt->cbegin(); jointIt != skeletonIt->cend(); ++jointIt)
					gl::drawSphere(jointIt->second * Vec3f(-1.0f, 1.0f, 1.0f), 0.025f, 16);

				// Draw body
				for (vector<vector<JointName> >::const_iterator segmentIt = mSegments.cbegin(); segmentIt != mSegments.cend(); ++segmentIt)
					drawSegment(* skeletonIt, * segmentIt);

			}

		// Draw depth and video textures
		glPopMatrix();
		gl::setMatricesWindow(getWindowSize(), true);
		gl::color(Colorf::white());
		if (mDepthSurface)
			gl::draw(gl::Texture(mDepthSurface), Area(0, 0, mDepthSurface.getWidth(), mDepthSurface.getHeight()), Rectf(265.0f, 15.0f, 505.0f, 195.0f));
		if (mVideoSurface)
			gl::draw(gl::Texture(mVideoSurface), Area(0, 0, mVideoSurface.getWidth(), mVideoSurface.getHeight()), Rectf(508.0f, 15.0f, 748.0f, 195.0f));

	}

	// Check audio data
	if (mData != 0)
	{

		// Get dimensions
		int32_t dataSize = mInput.getDataSize();
		float scale = 240.0f / (float)dataSize;
		float height = 180.0f;
		Vec2f position(751.0f, 15.0f);

		// Draw background
		gl::color(ColorAf::black());
		gl::drawSolidRect(Rectf(position.x, position.y, position.x + 240.0f, position.y + 180.0f));

		// Draw audio input
		gl::color(ColorAf::white());
		PolyLine<Vec2f> mLine;
		for (int32_t i = 0; i < dataSize; i++) 
			mLine.push_back(position + Vec2f(i * scale, math<float>::clamp(mData[i], -1.0f, 1.0f) * height * 0.5f + height * 0.5f));
		gl::draw(mLine);

	}

	// Draw the interface
	params::InterfaceGl::draw();

}

// Draw segment
void KinectApp::drawSegment(const Skeleton & skeleton, const vector<JointName> & joints)
{

	// DO IT!
	glBegin(GL_LINES);
	for (uint32_t i = 0; i < joints.size() - 1; i++)
	{
		glVertex3f(skeleton.at(joints[i]) * Vec3f(-1.0f, 1.0f, 1.0f));
		glVertex3f(skeleton.at(joints[i + 1]) * Vec3f(-1.0f, 1.0f, 1.0f));
	}
	glEnd();

}

// Called when audio buffer is full
void KinectApp::onData(float * data, int32_t size)
{

	// Get data
	mData = data;

}

// Prepare window
void KinectApp::prepareSettings(Settings * settings)
{

	// DO IT!
	settings->setWindowSize(1005, 570);
	settings->setFrameRate(60.0f);

}

// Take screen shot
void KinectApp::screenShot()
{

	// DO IT!
	writeImage(getAppPath() + "frame" + toString(getElapsedFrames()) + ".png", copyWindowSurface());

}

// Set up
void KinectApp::setup()
{

	// Set up OpenGL
	glLineWidth(2.0f);
	gl::color(ColorAf::white());

	// Start Kinect
	mKinect.removeBackground();
	mKinect.start();

	// Set up camera
	mCamera.lookAt(Vec3f(0.0f, 0.0f, -3.0f), Vec3f::zero());
	mCamera.setPerspective(45.0f, getWindowAspectRatio(), 1.0f, 1000.0f);

	// Define drawing body
	defineBody();

	// Initialize parameters
	mBinaryMode = false;
	mBinaryModePrev = false;
	mCapture = true;
	mCapturePrev = true;
	mEnabledAudio = true;
	mEnabledAudioPrev = true;
	mEnabledDepth = true;
	mEnabledDepthPrev = true;
	mEnabledSkeletons = true;
	mEnabledSkeletonsPrev = true;
	mEnabledVideo = true;
	mEnabledVideoPrev = true;
	mFrameRateApp = 0.0f;
	mFrameRateDepth = 0.0f;
	mFrameRateSkeletons = 0.0f;
	mFrameRateVideo = 0.0f;
	mFullScreen = isFullScreen();
	mInverted = false;
	mInvertedPrev = false;
	mRemoveBackground = true;
	mRemoveBackgroundPrev = true;
	mUserCount = 0;

	// Setup the parameters
	mParams = params::InterfaceGl("Parameters", Vec2i(245, 330));
	mParams.addParam("Capture", & mCapture, "key=c");
	mParams.addParam("App frame rate", & mFrameRateApp, "", true);
	mParams.addParam("Depth frame rate", & mFrameRateDepth, "", true);
	mParams.addParam("Skeleton frame rate", & mFrameRateSkeletons, "", true);
	mParams.addParam("Video frame rate", & mFrameRateVideo, "", true);
	mParams.addParam("Remove background", & mRemoveBackground, "key=b");
	mParams.addSeparator();
	mParams.addParam("Audio", & mEnabledAudio, "key=a");
	mParams.addParam("Depth", & mEnabledDepth, "key=d");
	mParams.addParam("Skeletons", & mEnabledSkeletons, "key=k");
	mParams.addParam("Video", & mEnabledVideo, "key=v");
	mParams.addParam("User count", & mUserCount, "", true);
	mParams.addSeparator();
	mParams.addParam("Binary depth mode", & mBinaryMode, "key=w");
	mParams.addParam("Invert binary image", & mInverted, "key=i");
	mParams.addSeparator();
	mParams.addParam("Full screen", & mFullScreen, "key=f");
	mParams.addButton("Screen shot", std::bind(& KinectApp::screenShot, this), "key=s");
	mParams.addButton("Quit", std::bind(& KinectApp::shutdown, this), "key=esc");

	// Find Kinect audio
	int32_t deviceId = -1;
	DeviceList devices = mInput.getDeviceList();
	for (DeviceList::const_iterator deviceIt = devices.cbegin(); deviceIt != devices.cend(); ++deviceIt)
		if (boost::contains(boost::to_lower_copy(deviceIt->second), "kinect"))
			deviceId = deviceIt->first;
	if (deviceId >= 0)
		mInput.setDevice(deviceId);

	// Start receiving audio
	mCallbackId = mInput.addCallback<KinectApp>(&KinectApp::onData, this);
	mInput.start();

}

// Quit
void KinectApp::shutdown()
{

	// Stop input
	mInput.stop();
	mInput.removeCallback(mCallbackId);

	// Delete audio buffer
	if (mData != 0)
		delete [] mData;

	// Stop Kinect
	mKinect.stop();

	// Force exit
	exit(1);

}

// Runs update logic
void KinectApp::update()
{

	// Toggle fullscreen
	if (mFullScreen != isFullScreen())
		setFullScreen(mFullScreen);

	// Toggle background remove
	if (mRemoveBackground != mRemoveBackgroundPrev)
	{
		mKinect.removeBackground(mRemoveBackground);
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	// Toggle capture
	if (mCapture != mCapturePrev)
	{
		mCapturePrev = mCapture;
		if (mCapture)
		{
			mInput.start();
			mKinect.start();
		}
		else
		{
			mInput.stop();
			mKinect.stop();
		}
	}

	// Toggle input tracking types
	if (mEnabledAudio != mEnabledAudioPrev)
	{
		mEnabledAudio ? mInput.start() : mInput.stop();
		mEnabledAudioPrev = mEnabledAudio;
	}
	if (mEnabledDepth != mEnabledDepthPrev)
	{
		mKinect.enableDepth(mEnabledDepth);
		mEnabledDepthPrev = mEnabledDepth;
	}
	if (mEnabledSkeletons != mEnabledSkeletonsPrev)
	{
		mKinect.enableSkeletons(mEnabledSkeletons);
		mEnabledSkeletonsPrev = mEnabledSkeletons;
	}
	if (mEnabledVideo != mEnabledVideoPrev)
	{
		mKinect.enableVideo(mEnabledVideo);
		mEnabledVideoPrev = mEnabledVideo;
	}

	// Toggle binary mode
	if (mBinaryMode != mBinaryModePrev || 
		mInverted != mInvertedPrev)
	{
		mKinect.enableBinaryMode(mBinaryMode, mInverted);
		mBinaryModePrev = mBinaryMode;
		mInvertedPrev = mInverted;
	}

	// Check if device is capturing
	if (mKinect.isCapturing())
	{

		// Get latest Kinect data
		if (mKinect.checkNewDepthFrame())
			mDepthSurface = mKinect.getDepth();
		if (mKinect.checkNewSkeletons())
			mSkeletons = mKinect.getSkeletons();
		if (mKinect.checkNewVideoFrame())
			mVideoSurface = mKinect.getVideo();

		// Update user count
		mUserCount = mKinect.getUserCount();
		
		// Update frame rates
		mFrameRateDepth = mKinect.getDepthFrameRate();
		mFrameRateSkeletons = mKinect.getSkeletonsFrameRate();
		mFrameRateVideo = mKinect.getVideoFrameRate();

	}

	// Update frame rate
	mFrameRateApp = getAverageFps();

}

// Run application
CINDER_APP_BASIC(KinectApp, RendererGl)
