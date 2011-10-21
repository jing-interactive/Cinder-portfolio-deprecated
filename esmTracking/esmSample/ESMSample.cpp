#ifdef CINDER_MSW
// for opengl functions
#include <windows.h>
#endif

#ifdef _DEBUG
#pragma comment(lib,"cinder_d.lib")
#else
#pragma comment(lib,"cinder.lib")
#endif

#include "cinder/Thread.h"
#include "cinder/app/App.h"
#include "cinder/ImageIo.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"
#include "cinder/Capture.h"
#include "cinder/Camera.h"
#include "cinder/Utilities.h"
#include "cinder/ip/Grayscale.h"
#include "ESMlibry.h"

using namespace ci;
using namespace ci::app;

#define LOCK_START(mtx) {boost::lock_guard<boost::mutex> lock(mtx)
#define LOCK_END() }

int	APP_W = 640;
int	APP_H = 480;

int CAM_W = 640;
int CAM_H = 480;

void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2);
void DrawResult (int sx, int sy, float H[9], imageStruct I);

const int nbimages = 10;
char filename[256];

// The tracking parameters
// miter: the number of iterations of the ESM algorithm (>= 1);
// mprec: the precision of the ESM algorithm (1..10)
// low precision = 1, high precision = 10;
// miter and mprec should be chosen depending on the available 
// computation time; 
// For low-speed PCs or high video framerate choose low miter and low mprec.
int miter = 5,  mprec = 2;    

// The window size
int sizx = 100, sizy = 120;

class esmApp : public AppBasic {
public:
	void	prepareSettings( Settings *settings );
	void	setup();
	void	keyUp( KeyEvent event );

	//mouse
	Vec2i startDragging,endDragging;

	void	mouseDown( MouseEvent event )
	{
		endDragging = startDragging = event.getPos();
	}

	void	mouseDrag( MouseEvent event )
	{
		trackerIsValid = false;
		endDragging = event.getPos();
	}

	void	mouseUp( MouseEvent event )
	{
		if (!gray_frame)
			return;

		if (startDragging.distanceSquared(endDragging) > 100)
		{
			// AR.init	
			Area clipping(startDragging,endDragging);
			LOCK_START(mtx_gray); 
			fillImage(gray_frame, esm_init);
			LOCK_END();

			LOCK_START(mtx_esm);
			// Memory allocation for the tracking
			sizx = clipping.getWidth();
			sizy = clipping.getHeight();
			if (T.images != NULL)
				FreeTrack(&T);
			if (MallTrack (&T, &esm_init, 
				clipping.getX1(), clipping.getY1(), sizx, sizy, miter, mprec))
				return;
			else
				console() << "ESM Tracking structure ready"<<std::endl;

			LOCK_END();

			// Save the reference pattern
			sprintf (filename, "../media/patr.pgm");
			if (SavePgm (filename, GetPatr (&T)))
				return;
			else
				console() << "ESM Writing " << filename<<std::endl;

			trackerIsValid = true;
		}

		endDragging = startDragging;
	}
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
	Channel8u	gray_frame;
	std::mutex	mtx_gray;

	CameraOrtho				mCam;
	bool trackerIsValid;						// true if tracker is created successfully

	//AR
	float pEnd[8];
	// The image read / acquired 
	imageStruct esm_init, esm_img; 

	// The global tracking structure
	trackStruct T; 
	std::mutex mtx_pEnd;
	std::mutex mtx_esm;

	void fillImage(const Channel8u& from, imageStruct& to);
	void fillImage(const Channel8u& from, imageStruct& to, const Area& clipping );
	void DrawResult (int sx, int sy, float H[9], imageStruct I);
};


void esmApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( APP_W,APP_H );
	settings->setFrameRate( 60.0f );
	settings->setFullScreen( false );
	mCam.setOrtho(APP_W, 0, APP_H, 0, -1000, 1000);
	mCam.setEyePoint(Vec3f(0,0,-100));
}


void esmApp::setup()
{
	console() << "ESM AR Sample| (c) 2011 vinjn | vinjn.z@gmail.com" << std::endl;

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

	//create a greyscale ci::Channel8u from the captured Surface
	gray_frame = Channel8u( CAM_W, CAM_H );

	esm_img.cols = CAM_W;
	esm_img.rows = CAM_H;
	esm_img.clrs = 256;
	esm_img.data = new float[CAM_W*CAM_H];

	esm_init.cols = CAM_W;
	esm_init.rows = CAM_H;
	esm_init.clrs = 256;
	esm_init.data = new float[CAM_W*CAM_H];

	T.images = NULL;
}


void esmApp::update()
{
	if( mCap && mCap.checkNewFrame() ) 
	{
		//get the latest surface from the capture object
		Surface8u surface = mCap.getSurface();
		mTexture = gl::Texture( surface );

		LOCK_START(mtx_gray);		 
		ip::grayscale( surface, &gray_frame );
		LOCK_END();

		if (trackerIsValid)
		{
			// Read the current image
			fillImage(gray_frame, esm_img);

			// Perform the tracking
			if (MakeTrack (&T, &esm_img))
				return;

			// Draw a black window around the pattern
			// T.homog contains the pointer to the homography matrix
			DrawResult (sizx, sizy, T.homog, esm_img); 
		}
		// end if new frame
	}		
}



void esmApp::draw()
{	
	//const ARFloat *matrix = tracker->getProjectionMatrix();

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

	if (startDragging.distanceSquared(endDragging) > 100)
	{
		gl::drawStrokedRect(Rectf(startDragging, endDragging));
	}
	//lock
	LOCK_START(mtx_pEnd);
	gl::color(Color8u::black());
	gl::drawLine(Vec2f(pEnd[0], pEnd[1]), Vec2f(pEnd[2], pEnd[3]));
	gl::drawLine(Vec2f(pEnd[2], pEnd[3]), Vec2f(pEnd[4], pEnd[5]));
	gl::drawLine(Vec2f(pEnd[4], pEnd[5]), Vec2f(pEnd[6], pEnd[7]));
	gl::drawLine(Vec2f(pEnd[6], pEnd[7]), Vec2f(pEnd[0], pEnd[1])); 
	LOCK_END();

	if (!trackerIsValid )
		return;
	gl::enableDepthWrite( true );
	gl::enableDepthRead();
	gl::enableAlphaBlending();
	gl::pushMatrices();

	// center point of marker as Vec2f:
	//for (int i=0;i<_n_marks;i++)
	{  		 

		// 					// draw the marker's center
		//	Vec2f p(cameraXToScreenX(marker.pos[0]),cameraYToScreenY(marker.pos[1]));

		gl::color(Color8u(255,255,0));
		//gl::drawStrokedCircle(p, 80.0f);
		//gl::drawSolidCircle(p,20.0f);

		//gl::drawStringCentered(toString(marker.id), p, ColorA::black());

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
			// 			gl::pushMatrices();						
			// 			glMatrixMode( GL_PROJECTION );
			// 			glLoadMatrixf(matrix);
			// 			gl::pushModelView();
			// 			glMatrixMode( GL_MODELVIEW );
			// 			gl::color(ColorA8u(220,200,0,100));
			// 
			// 			glLoadMatrixf( tracker->getModelViewMatrix() );
			// 			// scale is 80mm
			// 			gl::scale(Vec3f(2.0f, 2.0f, 2.0f));
			// 			// make cube resting on top of marker instead of inside
			// 			gl::drawCube(Vec3f(0.0f,0.0f,0.5f), Vec3f(1.0f,1.0f,1.0f));
			// 			gl::popMatrices();
		}
	}	 
}

void esmApp::keyUp( KeyEvent event )
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

void esmApp::shutdown()
{
	// The trick here is that the update() and draw() may still run after
	// this function runs, so make sure to only delete the tracker once!

	if (trackerIsValid) 
	{
		trackerIsValid = false;
		// Save the the current image 
		sprintf (filename, "../media/res_im%03d.pgm", 111);
		if (SavePgm (filename, &esm_img))
			return;
		else
			printf ("ESM Writing %s\n", filename);

		// Save the reprojection of the current pattern
		sprintf (filename, "../media/patc%03d.pgm", 111);
		if (SavePgm (filename, GetPatc (&T)))
			return;
		else
			printf ("ESM Writing %s\n", filename);
		// Free the global structure memory
		FreeTrack(&T);
// 		FreeImage(&esm_init);
// 		FreeImage(&esm_img);		
	}

	AppBasic::shutdown();
}

void esmApp::fillImage(const Channel8u& from, imageStruct& to )
{
	Channel8u::ConstIter it(from.getIter());
	const int w = to.cols;

	while (it.line())
	{
		while (it.pixel())
		{
			Vec2f pos = it.getPos();
			int idx = pos.x + pos.y * w;
			to.data[idx] = it.v();
			int a = 111;
		}
	}
}

void esmApp::fillImage(const Channel8u& from, imageStruct& to, const Area& clipping )
{
	Channel8u::ConstIter it(from.getIter(clipping));

	while (it.line())
	{
		while (it.pixel())
		{
			Vec2f pos = it.getPos();
			int idx = pos.x + pos.y * CAM_W;
			to.data[idx] = it.v();
		}
	}
}

// #####   FUNCTION DEFINITIONS  -  LOCAL TO THIS SOURCE FILE   ###############

void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2)
{
	int dr, dc, temp;
	int cols = image->cols, rows = image->rows;
	int i, point, area;

	area = cols * rows;
	if (r1 == r2 && c1 == c2) 
		return;
	if (abs (r2 - r1) < abs (c2 - c1)) {
		if (c1 > c2) {
			temp = r1; r1 = r2; r2 = temp;
			temp = c1; c1 = c2; c2 = temp;
		}
		dr = r2 - r1;
		dc = c2 - c1;
		temp = (r1 - c1 * dr / dc)*cols;
		for (i = c1; i <= c2; i++) {
			point = temp + (i * dr) / dc * cols  + i;
			if ((point >= 0) & (point < area))  
				image->data[point] = 0.0;
		}
	} 
	else {
		if (r1 > r2) {
			temp = r1; r1 = r2; r2 = temp;
			temp = c1; c1 = c2; c2 = temp;
		}
		dr = r2 - r1;
		dc = c2 - c1;
		temp =  c1 - r1 * dc / dr;
		for (i = r1; i <= r2; i++) {
			point = temp + i*cols + (i * dc) / dr;
			if ((point >= 0) & (point < area))  
				image->data[point] = 0.0;
		}
	}

	return;
}

void esmApp::DrawResult (int sx, int sy, float H[9], imageStruct I) 
{
	int i;
	float pIni[8];

	pIni[0] =        0.0;    pIni[1] =          0.0;  pIni[2] = (float) sx-1;  pIni[3] =          0.0;
	pIni[4] = (float) sx-1;  pIni[5] = (float) sy-1;  pIni[6] =          0.0;  pIni[7] = (float) sy-1;

	LOCK_START(mtx_pEnd);
	for (i = 0; i < 4; i++) {
		pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
			(H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]); 
		pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
			(H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);      
	}
	LOCK_END();

	DrawLine (&I, (int)pEnd[1], (int)pEnd[0], (int)pEnd[3], (int)pEnd[2]);
	DrawLine (&I, (int)pEnd[3], (int)pEnd[2], (int)pEnd[5], (int)pEnd[4]);
	DrawLine (&I, (int)pEnd[5], (int)pEnd[4], (int)pEnd[7], (int)pEnd[6]);
	DrawLine (&I, (int)pEnd[7], (int)pEnd[6], (int)pEnd[1], (int)pEnd[0]);

	return;
}

CINDER_APP_BASIC( esmApp, RendererGl )

