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

// Include header
#include <Kinect.h>

// Imports
using namespace ci;
using namespace ci::app;
using namespace std;

// Constructor
Kinect::Obj::Obj(const ImageResolution & videoResolution, const ImageResolution & depthResolution)
	: mDepthResolution(depthResolution), mUserCount(0), mVideoResolution(videoResolution)
{

	// Set flags
	mBinary = false;
	mCapture = false;
	mEnabledDepth = false;
	mEnabledSkeletons = false;
	mEnabledVideo = false;
	mInverted = false;
	mRemoveBackground = false;

	// Set depth dimensions
	switch (mDepthResolution)
	{
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mDepthWidth = 320;
		mDepthHeight = 240;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mDepthWidth = 80;
		mDepthHeight = 60;
		break;
	default:
		mDepthResolution = NUI_IMAGE_RESOLUTION_320x240;
		mDepthWidth = 320;
		mDepthHeight = 240;
		trace("Invalid depth resolution specified");
		break;
	}

	// Set depth dimensions
	switch (mVideoResolution)
	{
	case ImageResolution::NUI_IMAGE_RESOLUTION_1280x1024:
		mVideoWidth = 1280;
		mVideoHeight = 1024;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mVideoWidth = 640;
		mVideoHeight = 480;
		break;
	default:
		mVideoResolution = NUI_IMAGE_RESOLUTION_640x480;
		mVideoWidth = 640;
		mVideoHeight = 480;
		trace("Invalid video resolution specified");
		break;
	}

	// Allocate quad data
	mRgbDepth = new tagRGBQUAD[mDepthWidth * mDepthHeight * 4];
	mRgbVideo = new tagRGBQUAD[mVideoWidth * mVideoHeight * 4];

	// Define colors
	mColors.push_back(Colorf(0.5f, 0.5f, 1.0f));
	mColors.push_back(Colorf(0.0f, 1.0f, 0.0));
	mColors.push_back(Colorf(1.0f, 1.0f, 0.25f));
	mColors.push_back(Colorf(0.25f, 1.0f, 1.0));
	mColors.push_back(Colorf(1.0f, 0.25f, 1.0f));
	mColors.push_back(Colorf(1.0f, 0.0f, 0.0));

	// Start threads
	enableDepth(true);
	enableSkeletons(true);
	enableVideo(true);

	// Add threads to group to ulitize core management
	mThreadGroup.add_thread(& mThreadDepth);
	mThreadGroup.add_thread(& mThreadSkeletons);
	mThreadGroup.add_thread(& mThreadVideo);

	// Initialize active users
	for (uint32_t i = 0; i < NUI_SKELETON_COUNT; i++)
		mActiveUsers[i] = false;

}

// Destructor
Kinect::Obj::~Obj()
{

	// Stop threads
	enableDepth(false);
	enableSkeletons(false);
	enableVideo(false);
	mThreadGroup.join_all();

	// Clean up
	if (mRgbDepth != 0)
		delete [] mRgbDepth;
	if (mRgbVideo != 0)
		delete [] mRgbVideo;

}

// Set binary tracking mode
void Kinect::Obj::enableBinaryMode(bool enable, bool invertImage)
{

	// Update flags
	mBinary = enable;
	mInverted = invertImage;

}

// Enable or disable depth tracking
void Kinect::Obj::enableDepth(bool enable)
{

	// Set user count to 0 if disabled
	if (!enable)
		for (uint32_t i = 0; i < NUI_SKELETON_COUNT; i++)
			mActiveUsers[i] = false;

	// Set flag
	bool toggle = mEnabledDepth != enable;
	mEnabledDepth = enable;

	// Toggle thread
	if (toggle)
		if (enable)
			mThreadDepth = boost::thread(& Kinect::Obj::processDepth, this);
		else
			mThreadDepth.join();

	// Reset frame rate and surface
	if (!mEnabledDepth)
		mFrameRateDepth = 0.0f;

	// Initialize surface
	mDepthSurface = Surface8u(mDepthWidth, mDepthHeight, false, SurfaceChannelOrder::RGBA);

}

// Enable or disable skeleton tracking
void Kinect::Obj::enableSkeletons(bool enable)
{

	// Set flag
	bool toggle = mEnabledSkeletons != enable;
	mEnabledSkeletons = enable;

	// Toggle thread
	if (toggle)
		if (enable)
			mThreadSkeletons = boost::thread(& Kinect::Obj::processSkeletons, this);
		else
			mThreadSkeletons.join();

	// Reset frame rate
	if (!mEnabledSkeletons)
		mFrameRateSkeletons = 0.0f;

	// Initialize skeletons
	mSkeletons.clear();
	for (int32_t i = 0; i < NUI_SKELETON_COUNT; i++)
		mSkeletons.push_back(Skeleton());

}

// Enable or disable video tracking
void Kinect::Obj::enableVideo(bool enable)
{

	// Set flag
	bool toggle = mEnabledVideo != enable;
	mEnabledVideo = enable;

	// Toggle thread
	if (toggle)
		if (enable)
			mThreadVideo = boost::thread(& Kinect::Obj::processVideo, this);
		else
			mThreadVideo.join();

	// Reset frame rate
	if (!mEnabledVideo)
		mFrameRateVideo = 0.0f;

	// Initialize surface
	mVideoSurface = Surface8u(mVideoWidth, mVideoHeight, false, SurfaceChannelOrder::RGBA);

}

// Get depth surface
const Surface8u Kinect::Obj::getDepth() 
{ 

	// Lock thread
	boost::lock_guard<boost::mutex> lock(mMutexDepth);

	// Return surface and turn off new flag
	mNewDepthFrame = false;
	return mDepthSurface;

}

// Get skeletons
const vector<Skeleton> Kinect::Obj::getSkeletons() 
{

	// Lock thread
	boost::lock_guard<boost::mutex> lock(mMutexSkeletons);

	// Return skeletons and turn off new flag
	mNewSkeletons = false;
	return mSkeletons;

}

// Get user count
const int32_t Kinect::Obj::getUserCount() 
{

	// Lock thread
	boost::lock_guard<boost::mutex> lock(mMutexDepth);

	// Return user count
	return mEnabledDepth ? mUserCount : 0;

}

// Get video
const Surface8u Kinect::Obj::getVideo() 
{

	// Lock thread
	boost::lock_guard<boost::mutex> lock(mMutexVideo);

	// Return surface and turn off new flag
	mNewVideoFrame = false;
	return mVideoSurface;

}

// Initialize properties
void Kinect::Obj::init()
{

	// DO IT!
	mCapture = false;
    mDepthStreamHandle = 0;
	mFrameRateDepth = 0.0f;
	mFrameRateSkeletons = 0.0f;
	mFrameRateVideo = 0.0f;
	mNewDepthFrame = false;
	mNewSkeletons = false;
	mNewVideoFrame = false;
	mReadTimeDepth = 0.0;
	mReadTimeSkeletons = 0.0;
	mReadTimeVideo = 0.0;
	mSkeletonBmp = 0;
	mSkeletonDc = 0;
    mSkeletonOldObj = 0;
	mVideoStreamHandle = 0;

}

// Depth thread
void Kinect::Obj::processDepth()
{

	// Loop
	while (true)
	{

		// Break if tracking disabled
		if (!mEnabledDepth)
			break;

		// Lock scope
		boost::mutex::scoped_lock lock(mMutexDepth);

		// Need to acquire last image first
		if (mCapture && mEnabledDepth && !mNewDepthFrame)
		{

			// Acquire depth image
			const _NUI_IMAGE_FRAME * imageFrame = 0;
			if (!FAILED(NuiImageStreamGetNextFrame(mDepthStreamHandle, WAIT_TIME, & imageFrame)))
			{

				// Reset user count
				for (uint32_t i = 0; i < NUI_SKELETON_COUNT; i++)
					mActiveUsers[i] = false;

				// Read texture to surface
				NuiImageBuffer * texture = imageFrame->pFrameTexture;
				_KINECT_LOCKED_RECT lockedRect;
				texture->LockRect(0, & lockedRect, 0, 0);
				if (lockedRect.Pitch != 0)
					quadToSurface(mDepthSurface, (uint8_t *)lockedRect.pBits, true);
				else
					trace("Invalid buffer length received");

				// Clean up
				NuiImageStreamReleaseFrame(mDepthStreamHandle, imageFrame);

				// Update frame rate
				double time = getElapsedSeconds();
				mFrameRateDepth = (float)(1.0 / (time - mReadTimeDepth));
				mReadTimeDepth = time;

				// Set flag
				mNewDepthFrame = true;

				// Update user count
				mUserCount = 0;
				for (uint32_t i = 0; i < NUI_SKELETON_COUNT; i++)
					if (mActiveUsers[i])
						mUserCount++;

			}

		}

	}

}

// Skeleton thread
void Kinect::Obj::processSkeletons()
{

	// Loop
	while (true)
	{

		// Break if tracking disabled
		if (!mEnabledSkeletons)
			break;

		// Lock scope
		boost::mutex::scoped_lock lock(mMutexSkeletons);

		// Last skeletons need to be acquired first
		if (mCapture && !mNewSkeletons)
		{

			// Acquire skeleton
			_NUI_SKELETON_FRAME skeletonFrame;
			if (!FAILED(NuiSkeletonGetNextFrame(WAIT_TIME, & skeletonFrame)))
			{
    
				// Iterate through skeletons
				bool foundSkeleton = false;
				for (int32_t i = 0 ; i < NUI_SKELETON_COUNT ; i++)
				{

					// Clear skeleton data
					mSkeletons[i].clear();

					// Mark skeleton found
					if (skeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED)
					{

						// Smooth out the skeleton data when found
						if (!foundSkeleton)
						{
							NuiTransformSmooth(& skeletonFrame, 0);
							PatBlt(mSkeletonDc, 0, 0, 8, 4, BLACKNESS);
							foundSkeleton = true;
						}

						// Get skeleton data
						_NUI_SKELETON_DATA skeletonData = skeletonFrame.SkeletonData[i];

						// Set joint data
						for (int32_t j = 0; j < (int32_t)NUI_SKELETON_POSITION_COUNT; j++)
						{
							Vector4 point = skeletonData.SkeletonPositions[j];
							mSkeletons[i].insert(std::make_pair<JointName, Vec3f>((JointName)j, Vec3f(point.x, point.y, point.z)));
						}

					}

				}

				// Update frame rate
				double time = getElapsedSeconds();
				mFrameRateSkeletons = (float)(1.0 / (time - mReadTimeSkeletons));
				mReadTimeSkeletons = time;

				// Set flag
				mNewSkeletons = true;

			}

		}

	}

}

// Video data event handler
void Kinect::Obj::processVideo()
{

	// Loop
	while (true)
	{

		// Break if tracking disabled
		if (!mEnabledVideo)
			break;

		// Lock scope
		boost::mutex::scoped_lock lock(mMutexVideo);

		// Need to acquire last frame first
		if (mCapture && !mNewVideoFrame)
		{

			// Acquire video image
			const NUI_IMAGE_FRAME * imageFrame = NULL;
			if (!FAILED(NuiImageStreamGetNextFrame(mVideoStreamHandle, WAIT_TIME, & imageFrame)))
			{

				// Read texture
				NuiImageBuffer * texture = imageFrame->pFrameTexture;
				_KINECT_LOCKED_RECT lockedRect;
				texture->LockRect(0, & lockedRect, 0, 0);
				if (lockedRect.Pitch != 0)
					quadToSurface(mVideoSurface, (uint8_t *)lockedRect.pBits);
				else
					trace("Invalid buffer length received");

				// Clean up
				NuiImageStreamReleaseFrame(mVideoStreamHandle, imageFrame);

				// Update frame rate
				double time = getElapsedSeconds();
				mFrameRateVideo = (float)(1.0 / (time - mReadTimeVideo));
				mReadTimeVideo = time;

				// Set flag
				mNewVideoFrame = true;

			}

		}

	}

}

// Convert and copy quad data to a surface
void Kinect::Obj::quadToSurface(Surface8u & surface, uint8_t * buffer, bool depth)
{

	// Last frame still needs to be acquired
	if ((depth && mNewDepthFrame) || (!depth && mNewVideoFrame))
		return;

	// Get dimensions
	int32_t height = surface.getHeight();
	int32_t width = surface.getWidth();
	int32_t size = width * height * 4;

	// This is depth data
	if (depth)
	{

		// Draw the bits to the bitmap
		tagRGBQUAD * rgbRun = mRgbDepth;
		uint16_t * bufferRun = (uint16_t *)buffer;
		for (int32_t y = 0; y < height; y++)
			for (int32_t x = 0 ; x < width; x++)
			{
				tagRGBQUAD quad = shortToQuad(* bufferRun);
				bufferRun++;
				* rgbRun = quad;
				rgbRun++;
			}

		// Copy depth data to surface
		memcpy(surface.getData(), (uint8_t *)mRgbDepth, size);
		mNewDepthFrame = true;

	}
	else
	{

		// Swap red/blue channels
		for (int32_t i = 0; i < size; i += 4)
		{
			uint8_t b = buffer[i];
			buffer[i] = buffer[i + 2];
			buffer[i + 2] = b;
		}

		// Copy color data to surface
		memcpy(surface.getData(), buffer, size);
		mNewVideoFrame = true;

	}

}

// Remove background for cleaner user tracking
void Kinect::Obj::removeBackground(bool remove)
{

	// Set flag
	mRemoveBackground = remove;

}

// Convert value to short to quad
const tagRGBQUAD Kinect::Obj::shortToQuad(uint16_t value)
{

	// Extract depth and user values
    uint16_t realDepth = (value & 0xfff8) >> 3;
    uint16_t user = value & 7;

    // Transform 13-bit depth information into an 8-bit intensity appropriate
    // for display (we disregard information in most significant bit)
    uint8_t intensity = 255 - (uint8_t)(256 * realDepth / 0x0FFF);

	// Initialize color data
    tagRGBQUAD quad;
    quad.rgbRed = quad.rgbBlue = quad.rgbGreen = 0;

	// Mark user active
	if (user > 0 && user < 7)
		mActiveUsers[user - 1] = true;

	// Binary mode
	if (mBinary)
	{

		// Set black and white values
		uint8_t backgroundColor = mInverted ? 255 : 0;
		uint8_t userColor = mInverted ? 0 : 255;

		// Set color
		if (user == 0 || user == 7)
			quad.rgbRed = quad.rgbBlue = quad.rgbGreen = mRemoveBackground ? backgroundColor : userColor;
		else
			quad.rgbRed = quad.rgbBlue = quad.rgbGreen = userColor;

	}
	else
	{

		// Colorize each user
		switch (user)
		{
		case 0:
			if (!mRemoveBackground)
			{
				quad.rgbRed = intensity / 2;
				quad.rgbBlue = intensity / 2;
				quad.rgbGreen = intensity / 2;
			}
			break;
		case 1:
			quad.rgbRed = intensity / 2;
			quad.rgbGreen = intensity / 2;
			quad.rgbBlue = intensity;
			break;
		case 2:
			quad.rgbGreen = intensity;
			break;
		case 3:
			quad.rgbRed = intensity;
			quad.rgbGreen = intensity;
			quad.rgbBlue = intensity / 4;
			break;
		case 4:
			quad.rgbRed = intensity / 4;
			quad.rgbGreen = intensity;
			quad.rgbBlue = intensity;
			break;
		case 5:
			quad.rgbRed = intensity;
			quad.rgbGreen = intensity / 4;
			quad.rgbBlue = intensity;
			break;
		case 6:
			quad.rgbRed = intensity;
			break;
		case 7:
			if (!mRemoveBackground)
			{
				quad.rgbRed = 255 - (intensity / 2);
				quad.rgbGreen = 255 - (intensity / 2);
				quad.rgbBlue = 255 - (intensity / 2);
			}
		}

	}

	// Return color
    return quad;

}

// Start capturing
void Kinect::Obj::start()
{

	// Already started
	if (mCapture)
		return;

	// Initialize properties
	init();

	// Initialize device
    if (FAILED(NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR)))
    {
        trace("Unable to initialize device");
        return;
    }

	// Initialize skeleton object
	mSkeletonOldObj = SelectObject(mSkeletonDc, mSkeletonBmp);

	// Enable skeleton tracking
    if (FAILED(NuiSkeletonTrackingEnable(0, 0)))
    {
        trace("Unable to initialize skeleton tracking");
        return;
    }

	// Open color image stream
    if (FAILED(NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, mVideoResolution, 0, 2, 0, & mVideoStreamHandle)))
    {
        trace("Unable to open color image stream");
        return;
    }

	// Open depth image stream
    if (FAILED(NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, mDepthResolution, 0, 2, 0, & mDepthStreamHandle)))
    {
        trace("Unable to open depth image stream");
        return;
    }

	// Initialized
	mCapture = true;

}

// Stop capturing
void Kinect::Obj::stop()
{

	// Reset data
	init();

	// Delete skeleton
	SelectObject(mSkeletonDc, mSkeletonOldObj);
    DeleteDC(mSkeletonDc);
    DeleteObject(mSkeletonBmp);
	
	// Shutdown Kinect
    NuiShutdown();

}

// Debug trace
void Kinect::Obj::trace(const string & message)
{

	// Write to console and debug window
	console() << message << "\n";
	OutputDebugStringA(message.c_str());
	OutputDebugStringA("\n");

}

/******************/

// Use color to get user ID
const uint32_t Kinect::getUserIdByColor(const ci::Colorf & color)
{

	// Call float overload
	return getUserIdByColor(color.r, color.g, color.b);

}

// Use color to get user ID
const uint32_t Kinect::getUserIdByColor(uint8_t r, uint8_t g, uint8_t b)
{

	// Call float overload
	return getUserIdByColor((float)r, (float)g, (float)b);

}

// Use color to get user ID
const uint32_t Kinect::getUserIdByColor(float r, float g, float b)
{

	// See Kinect::Obj::shortToQuad for this to make sense
	if (r > g && r > b)
		return 1;
	else if (g > r && g > b)
		return 2;
	else if (g > r && b > r)
		return 3;
	else if (r > b && g > b)
		return 4;
	else if (r > g && b > g)
		return 5;
	else if (b > r && b > g)
		return 6;
	return 0;

}
