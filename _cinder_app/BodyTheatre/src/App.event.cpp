#include "App.h"
#include "Player.h"
#include "KinectRoutine.h"
#include "Config.h"

void BodyTheatreApp::mouseWheel( MouseEvent event )
{
	_routine->mouseWheel(event);
}

void BodyTheatreApp::mouseUp( MouseEvent event )
{
	_routine->mouseUp(event);
}

void BodyTheatreApp::mouseDown( MouseEvent event )
{
	_routine->mouseDown(event);
}

void BodyTheatreApp::mouseDrag( MouseEvent event )
{
	_routine->mouseDrag(event);
}

void BodyTheatreApp::mouseMove( MouseEvent event )
{
	_routine->mouseMove(event);
}

void BodyTheatreApp::onOscMessage( const osc::Message* msg )
{
	string addr = msg->getAddress();
	if (msg->getAddress() == "/contour")
	{
		int id = msg->getArgAsInt32(0);
		if (!_routine->isPlayerIdValid(id))
			return;
		if (_activeIdx == INVALID_IDX)
		{//any index is welcome to me
			_activeIdx = id;
		}
	//	if (id == _activeIdx)
		{//only update active player
		//	console() << id << endl;
			lock_guard<mutex> lock(_mtx_player);
			players[id].setup(msg);
		}
	}
	else _routine->onOscMessage(msg);
}

void BodyTheatreApp::keyDown( KeyEvent event )
{
	switch (event.getCode())
	{
	case KeyEvent::KEY_ESCAPE:
		{
			quit();
		}break;
	case KeyEvent::KEY_RETURN:
		{
			loadConfig("BodyTheatre.xml");
		}
		break;
	case  KeyEvent::KEY_f:
		{
			setFullScreen(!isFullScreen());
		}break;
	}
}