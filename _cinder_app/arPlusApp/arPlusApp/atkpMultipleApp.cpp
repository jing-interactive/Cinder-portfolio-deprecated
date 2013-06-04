/** 
*  ARToolkitPlus demo app
*
*  Copyright (C) 2011 Evan Raskob / pixelpusher <info@pixelist.info>
*  http://pixelist.info
*
* Some code included in this distribution has different copyrights and
* different licenses.
*
* PLEASE NOTE THE LICENSES ON EACH FILE IN THIS PROJECT CAN BE DIFFERENT!
*
*  This file is:
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*  
*  http://www.apache.org/licenses/LICENSE-2.0
*  
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*/


// NOTE - we assume the camera size is 640 x 480 pixels.  If not, you'll need to change that in a few spots!
// 'f' key toggles fullscreen display
// 'i' key toggles drawing the camera image

#ifdef _DEBUG
#pragma comment(lib,"cinder_d.lib")
#else
#pragma comment(lib,"cinder.lib")
#endif

#include "cinder/Thread.h"
#include "cinder/app/App.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"
#include "cinder/Capture.h"
#include "cinder/Camera.h"
#include "cinder/Utilities.h"
#include "cinder/ip/Grayscale.h"
#include "ARToolKitPlus/TrackerSingleMarker.h"
#include "ARToolKitPlus/TrackerMultiMarker.h"

//#include "Resources.h"

using namespace ci;
using namespace ci::app;

int	APP_W = 800;
int	APP_H = 600;

int CAM_W = 640;
int CAM_H = 480;

class atkpMultipleApp : public AppBasic {
public:
	void	prepareSettings( Settings *settings );
	void	setup();
	void	keyUp( KeyEvent event );

	void	update();
	void	draw();
	void	shutdown(); 

	float cameraXToScreenX(float cx)
	{
		return cx*APP_W/CAM_W;
	}

	float cameraYToScreenY(float cy)
	{
		return cy*APP_H/CAM_H;
	}


private:

	Capture				mCap;					// our camera capture object
	gl::Texture			mTexture;				// our camera capture texture
	bool				mDrawCapturedImage;		// whether to draw the camera texture

	bool trackerIsValid;						// true if tracker is created successfully

	//AR
	ARToolKitPlus::TrackerSingleMarker* tracker; 
	std::vector<ARToolKitPlus::ARMarkerInfo> marker_infos;
	int n_marks;

	std::mutex mtx_AR;
};


void atkpMultipleApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( APP_W,APP_H );
	settings->setFrameRate( 60.0f );
	settings->setFullScreen( false );
}


void atkpMultipleApp::setup()
{
	console() << "ARToolKitPlus Tracker Demo App | (c) 2011 Evan Raskob | www.pixelist.info" << std::endl;


	// draw captures image by default
	mDrawCapturedImage = true;
	trackerIsValid = false;

	// list out the devices
	std::vector<Capture::DeviceRef> devices( Capture::getDevices() );
	for( std::vector<Capture::DeviceRef>::const_iterator deviceIt = devices.begin(); deviceIt != devices.end(); ++deviceIt ) {
		Capture::DeviceRef device = *deviceIt;
		console() << "Found Device " << device->getName() << " ID: " << device->getUniqueId() << std::endl;

		try {
			if( device->checkAvailable() ) {
				mCap = Capture( CAM_W, CAM_H, device );
				mCap.start();

				// placeholder text
				mTexture = gl::Texture();
				break;
			}
			console() << "device is NOT available" << std::endl;
		}
		catch( CaptureExc & ) {
			console() << "Unable to initialize device: " << device->getName() << std::endl;
		}
	}

	// AR.init	
	tracker = new ARToolKitPlus::TrackerSingleMarker(CAM_W, CAM_H, 8, 6, 6, 6, 0);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
	// load a camera file.
	std::string path(getAppPath()+"../media/no_distortion.cal");
	if (!tracker->init(path.c_str(), 1.0f, 1000.0f)) {
		console() << "ERROR: init() failed\n";
		return;
	}
	tracker->getCamera()->printSettings();
	tracker->setBorderWidth(0.125f);
	// set a threshold. we could also activate automatic thresholding
#if 1
	tracker->setThreshold(150);
#else
	tracker->activateAutoThreshold(true);
#endif
	// let's use lookup-table undistortion for high-speed
	// note: LUT only works with images up to 1024x1024
	tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
	// switch to simple ID based markers
	// use the tool in tools/IdPatGen to generate markers
	tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_BCH);

	tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);

	n_marks = 0;

	trackerIsValid = true;	
}


void atkpMultipleApp::update()
{
	if( mCap && mCap.checkNewFrame() ) 
	{
		//get the latest surface from the capture object
		Surface8u surface = mCap.getSurface();
		mTexture = gl::Texture( surface );

		//create a greyscale ci::Channel8u from the captured Surface
		Channel8u frame = Channel8u( mCap.getWidth(), mCap.getHeight() );

		ip::grayscale( surface, &frame );

		if (trackerIsValid)
		{			
			// detect all the loaded markers in this image (camera frame)
			ARToolKitPlus::ARMarkerInfo* arr_marker_infos;
			std::vector<int> markerId = tracker->calc((unsigned char*)frame.getData(), &arr_marker_infos, &n_marks);
			tracker->selectBestMarkerByCf();

			boost::lock_guard<boost::mutex> lock(mtx_AR);
			marker_infos.clear();
			for (int j = 0; j < n_marks; j++) 
			{
				if (arr_marker_infos[j].id != -1)
					marker_infos.push_back(arr_marker_infos[j]);
			}			
		}

		// end if new frame
	}		
}



void atkpMultipleApp::draw()
{	
	const ARFloat *matrix = tracker->getProjectionMatrix();

	gl::clear(Color(0,0,0));

	if( !mTexture )
		return;
	// draw image from camera first (underneath)
	if (mDrawCapturedImage)
	{
		gl::disableAlphaBlending();
		gl::pushMatrices();
		gl::enableDepthWrite( false );
		gl::setMatricesWindow( getWindowSize() );
		gl::color(Color8u(255,255,255));
		gl::draw( mTexture, getWindowBounds() );
		gl::popMatrices();
	}

	int _n_marks = 0;
	std::vector<ARToolKitPlus::ARMarkerInfo> _marker_infos;

	//lock
	{
        boost::lock_guard<boost::mutex> lock(mtx_AR);
		_n_marks = marker_infos.size();
		_marker_infos = marker_infos;
	}

	if (!trackerIsValid || _n_marks == 0)
		return;
	gl::enableDepthWrite( true );
	gl::enableDepthRead();
	gl::enableAlphaBlending();
	gl::pushMatrices();

	// center point of marker as Vec2f:
	for (int i=0;i<_n_marks;i++)
	{ 
		const ARToolKitPlus::ARMarkerInfo& marker = _marker_infos[i];			 

		// 					// draw the marker's center
		Vec2f p(cameraXToScreenX(marker.pos[0]),cameraYToScreenY(marker.pos[1]));

		gl::color(Color8u(255,255,0));
		gl::drawStrokedCircle(p, 80.0f);
		//gl::drawSolidCircle(p,20.0f);

		gl::drawStringCentered(toString(marker.id), p, ColorA::black());

		if (mDrawCapturedImage)
		{
#if 0
			// draw captured image rightside up on quad shape using vertices
			mTexture.enableAndBind();

			glBegin(GL_QUADS);
			glTexCoord2f(0,1);
			glVertex2f(cameraXToScreenX(marker.vertex[0][0]),cameraYToScreenY(marker.vertex[0][1]));
			glTexCoord2f(1,1);
			glVertex2f(cameraXToScreenX(marker.vertex[1][0]),cameraYToScreenY(marker.vertex[1][1]));
			glTexCoord2f(1,0);
			glVertex2f(cameraXToScreenX(marker.vertex[2][0]),cameraYToScreenY(marker.vertex[2][1]));
			glTexCoord2f(0,0);
			glVertex2f(cameraXToScreenX(marker.vertex[3][0]),cameraYToScreenY(marker.vertex[2][1]));
			glEnd();
			mTexture.unbind();
#endif

			// now draw 3D cube at marker location with proper scale
			gl::pushMatrices();						
			glMatrixMode( GL_PROJECTION );
			glLoadMatrixf(matrix);
			gl::pushModelView();
			glMatrixMode( GL_MODELVIEW );
			gl::color(ColorA8u(220,200,0,100));

			glLoadMatrixf( tracker->getModelViewMatrix() );
			// scale is 80mm
			gl::scale(Vec3f(2.0f, 2.0f, 2.0f));
			// make cube resting on top of marker instead of inside
			gl::drawCube(Vec3f(0.0f,0.0f,0.5f), Vec3f(1.0f,1.0f,1.0f));
			gl::popMatrices();
		}
	}	 
}

void atkpMultipleApp::keyUp( KeyEvent event )
{ 
	switch( event.getChar() ) {
case 'f':
	setFullScreen( ! isFullScreen() );
	break;
case 'i':
	mDrawCapturedImage = ! mDrawCapturedImage;
	break;
case KeyEvent::KEY_ESCAPE: 
	quit();
	break;
	}
}

void atkpMultipleApp::shutdown()
{
	// The trick here is that the update() and draw() may still run after
	// this function runs, so make sure to only delete the tracker once!

	if (trackerIsValid) 
	{
		delete tracker;
		trackerIsValid = false;
	}

	AppBasic::shutdown();
}

CINDER_APP_BASIC( atkpMultipleApp, RendererGl )
