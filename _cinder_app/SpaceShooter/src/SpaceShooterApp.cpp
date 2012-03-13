#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class SpaceShooterAppApp : public AppBasic {
  public:
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
};

void SpaceShooterAppApp::setup()
{
}

void SpaceShooterAppApp::mouseDown( MouseEvent event )
{
}

void SpaceShooterAppApp::update()
{
}

void SpaceShooterAppApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP_BASIC( SpaceShooterAppApp, RendererGl )
