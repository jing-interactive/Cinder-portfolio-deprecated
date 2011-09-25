#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Thread.h"
#include "cinder/Capture.h"
#include "CinderOpenCV.h"
#include "../../_common/vOpenCV/OpenCV.h"
#include "../../_common/vOpenCV/BlobTracker.h"
#include "../../_common/asmlibrary/asmfitting.h"

using namespace ci;
using namespace ci::app;
using namespace std;
 
class FaceOffApp : public AppBasic {
public:
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();

	void processFrame();

	std::thread processFrameThread;

	Surface		sur_frame;
	gl::Texture		tex_frame;
	Capture			mCap;
	vHaarFinder haar;
};

void FaceOffApp::setup()
{
	try {
		mCap = Capture( 640, 480 );
		mCap.start();
		processFrameThread = std::thread(&FaceOffApp::processFrame, this);
	}
	catch( ... ) {
		console() << "Failed to initialize capture" << std::endl;
	}

	if (haar.init("../../resources/haarcascade_frontalface_alt2.xml"))
		console() << "Opened haar xml" <<std::endl;
}

void FaceOffApp::mouseDown( MouseEvent event )
{
}

void FaceOffApp::update()
{
	if (sur_frame)
		tex_frame = sur_frame;
}

void FaceOffApp::draw()
{
	if (!tex_frame)
		return;
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );
	gl::draw(tex_frame);
}

void FaceOffApp::processFrame()
{
	while (true)
	{
		if( mCap && mCap.checkNewFrame() )
			sur_frame = mCap.getSurface();
		processFrameThread.yield();
	}
}


CINDER_APP_BASIC( FaceOffApp, RendererGl )
