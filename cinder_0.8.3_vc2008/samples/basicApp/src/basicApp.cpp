#include "cinder/app/AppBasic.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

// We'll create a new Cinder Application by deriving from the BasicApp class
class BasicApp : public AppBasic {
 public:
	// Cinder will always call this function whenever the user drags the mouse
	void mouseDrag( MouseEvent event )
    {
    }
    
	void keyDown( KeyEvent event )
    {    
    }
    
	// Cinder calls this function 30 times per second by default
	void draw()
    {
        gl::setMatricesWindow( getWindowSize() );
        // this pair of lines is the standard way to clear the screen in OpenGL
        gl::clear( Color( 0.8f, 0.8f, 0.8f ) );
    }
};


// This line tells Flint to actually create the application
CINDER_APP_BASIC( BasicApp, RendererGl )