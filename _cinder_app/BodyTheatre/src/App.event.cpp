#include "BodyTheatreApp.h"
#include "Player.h"

void BodyTheatreApp::mouseDown( MouseEvent event )
{
}

void BodyTheatreApp::onOscMessage( const osc::Message* msg )
{
	if (msg->getAddress() == "/contour")
	{
		int id = msg->getArgAsInt32(0);

		console() << id << endl;
		lock_guard<mutex> lock(_mtx_player);
		players[id].setup(msg);
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