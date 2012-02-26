#include "PuzzleApp.h"
#include "OscMessage.h"

namespace
{
	const float k_deltay = 10;
	const float KINECT_ROTATE = 0.3f;
}

void PuzzleApp::onOscMessage( const osc::Message* msg )
{
	if (msg->getNumArgs() != 3)
		return;
	string addr = msg->getAddress();
	string action = msg->getArgAsString(0);
	console()<<addr<<action<<endl;
	int x = getWindowWidth()*msg->getArgAsFloat(1);
	int y = getWindowHeight()*msg->getArgAsFloat(2);

	int id = (addr == "/left") ? LEFT : RIGHT;
	_hands[id].pos.set(x,y);
	_hands[id].push = (action == "push");

	if (id == RIGHT)
	{
		MouseEvent evt(0, x, y, 0, 0, 0);

		if (_sprite_selected)//if selected
		{
			if (_hands[RIGHT].push)
				mouseDrag(evt);
			else 
				mouseUp(evt);
		}
		else
		{//not selected
			if (_hands[RIGHT].push)
				mouseDown(evt);//try selecting
			else 
				mouseMove(evt);//normal move
		}
	}
}

void PuzzleApp::update()
{
	if (_hands[LEFT].push && _hands[RIGHT].push)
	{
		Vec2f diff = _hands[RIGHT].pos - _hands[LEFT].pos;
		if (diff.y > k_deltay)
			_rotate = KINECT_ROTATE;
		else if (diff.y < -k_deltay)
			_rotate = -KINECT_ROTATE;
	}
	updateStates();
}