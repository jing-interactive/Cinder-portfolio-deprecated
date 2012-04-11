#include "BodyTheatreApp.h"
#include "cinder/Perlin.h"

void BodyTheatreApp::mouseDown( MouseEvent event )
{
}

void BodyTheatreApp::onOscMessage( const osc::Message* msg )
{
	console() << msg->getAddress() <<endl;

	if (msg->getAddress() == "/start")
	{
	}
	else if (msg->getAddress() == "/contour")
	{
		int id = msg->getArgAsInt32(0);
		if (players.find(id) == players.end())
		{
			Player ply;

			ply.setup(msg);

			lock_guard<mutex> lock(_mtx_player);
			players[id] = ply;
		}
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