#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class _TBOX_PREFIX_AppApp : public AppBasic {
  public:
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
};

void _TBOX_PREFIX_AppApp::setup()
{
}

void _TBOX_PREFIX_AppApp::mouseDown( MouseEvent event )
{
}

void _TBOX_PREFIX_AppApp::update()
{
}

void _TBOX_PREFIX_AppApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP_BASIC( _TBOX_PREFIX_AppApp, RendererGl )
