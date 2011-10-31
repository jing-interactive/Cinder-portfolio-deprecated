#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Capture.h"
#include "cinder/ip/EdgeDetect.h"
#include "CinderOpenCV.h"

#if defined _DEBUG
#pragma comment(lib,"opencv_core231d.lib")
#pragma comment(lib,"opencv_imgproc231d.lib")
#pragma comment(lib,"opencv_highgui231d.lib")
#else
#pragma comment(lib,"opencv_core231.lib")
#pragma comment(lib,"opencv_imgproc231.lib")
#pragma comment(lib,"opencv_highgui231.lib")
#endif


//#include "Resources.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct CameraBoothApp : public AppBasic 
{
	void setup();
	void mouseMove( MouseEvent event );
	void keyDown(KeyEvent event);
	void update();
	void draw();

	Surface		frame;
	Area	focusArea;
	float max_angle;

	gl::Texture		mTexture;

	Capture			mCap;
};

void CameraBoothApp::setup()
{
	focusArea = getWindowBounds();
	max_angle = 1;
	try {
		std::vector<Capture::DeviceRef> devices( Capture::getDevices() );
		mCap = Capture( 640, 480, devices[1]);
		mCap.start();
	}
	catch( ... ) {
		console() << "Failed to initialize capture" << std::endl;
	}
}

void twirl( Surface *surface, Area area, float maxAngle )
{
	// make a clone of the surface
	Surface inputSurface = surface->clone();

	// we'll need to iterate the inputSurface as well as the output surface
	Surface::ConstIter inputIter( inputSurface.getIter() );
	Surface::Iter outputIter( surface->getIter( area ) );

	float maxDistance = area.getSize().length() / 2;
	Vec2f mid = ( area.getUL() + area.getLR() ) / 2;

	while( inputIter.line() && outputIter.line() ) {
		while( inputIter.pixel() && outputIter.pixel() ) {
			Vec2f current = inputIter.getPos() - mid;
			float r = current.length();
			float twirlAngle = r / maxDistance * maxAngle;
			float angle = atan2( current.y, current.x );
			Vec2f outSample( r * cos( angle + twirlAngle ), r * sin( angle + twirlAngle ) );
			Vec2i out = outSample - current;

			outputIter.r() = inputIter.rClamped( out.x, out.y );
			outputIter.g() = inputIter.gClamped( out.x, out.y );
			outputIter.b() = inputIter.bClamped( out.x, out.y );
		}
	}
}

void CameraBoothApp::update()
{
	if( mCap && mCap.checkNewFrame() ) {
		cv::Mat input( toOcv( mCap.getSurface() ) ), output;
// 		frame =  fromOcv( input );		
// 
 		frame = mCap.getSurface().clone();
 		twirl(&frame, focusArea, max_angle);
//		cv::fi

		if (mTexture == NULL)
			mTexture = frame;
		else
			mTexture.update(frame);
	}
}

void CameraBoothApp::draw()
{
	gl::clear();

	if (mTexture != NULL)
		gl::draw(mTexture);

}

void CameraBoothApp::mouseMove( MouseEvent event )
{
	max_angle = (-0.5f+event.getX()/(float)getWindowWidth())*3;
}

void CameraBoothApp::keyDown( KeyEvent event )
{
	if (event.getCode() == KeyEvent::KEY_ESCAPE)
	{
		this->quit();
	}
}



CINDER_APP_BASIC( CameraBoothApp, RendererGl )
