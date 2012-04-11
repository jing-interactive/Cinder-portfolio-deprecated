#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/Thread.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct BodyTheatreApp : public AppBasic {
	void setup();
	void mouseDown( MouseEvent event );	
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void onOscMessage( const osc::Message* msg );

	shared_ptr<osc::Listener> _listener;
	Path2d _player;
	mutex _mtx_player;
};

void BodyTheatreApp::setup()
{
	_listener = shared_ptr<osc::Listener>(new osc::Listener);
	_listener->setup(7777);
	_listener->registerMessageReceived(this, &BodyTheatreApp::onOscMessage);
}

void BodyTheatreApp::mouseDown( MouseEvent event )
{
}

void BodyTheatreApp::update()
{
}

void BodyTheatreApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );

	{
		lock_guard<mutex> lock(_mtx_player);
		gl::drawSolid(_player);
	}	
}

void BodyTheatreApp::onOscMessage( const osc::Message* msg )
{
	console() << msg->getAddress();

	if (msg->getAddress() == "/start")
	{
	}
	else if (msg->getAddress() == "/contour")
	{
		const int IDX_NUM_PTS = 7;
		int n_pts = msg->getArgAsInt32(IDX_NUM_PTS);
		Path2d contour;
		contour.clear();
		for (int i=0;i<n_pts;i++)
		{
			float x = msg->getArgAsFloat(IDX_NUM_PTS+i*2+1);
			float y = msg->getArgAsFloat(IDX_NUM_PTS+i*2+2);
			x *= getWindowWidth();
			y *= getWindowHeight();
			if (i == 0)
				contour.moveTo(x, y);
			else
				contour.lineTo(x, y);
		}
		contour.close();

		lock_guard<mutex> lock(_mtx_player);
		_player = contour;
	}
	else if (msg->getAddress() == "/kinect")
	{

	}
}

void BodyTheatreApp::keyDown( KeyEvent event )
{
	switch (event.getCode())
	{
	case KeyEvent::KEY_ESCAPE:
		{
			quit();
		}break;
	case  KeyEvent::KEY_f:
		{
			setFullScreen(!isFullScreen());
		}break;
	}
}


CINDER_APP_BASIC( BodyTheatreApp, RendererGl )
