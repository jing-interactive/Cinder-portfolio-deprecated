#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct BodyTheatreAppApp : public AppBasic {
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
	void oscEvent();
};

void BodyTheatreAppApp::setup()
{
}

void BodyTheatreAppApp::mouseDown( MouseEvent event )
{
}

void BodyTheatreAppApp::update()
{
}

void BodyTheatreAppApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP_BASIC( BodyTheatreAppApp, RendererGl )
