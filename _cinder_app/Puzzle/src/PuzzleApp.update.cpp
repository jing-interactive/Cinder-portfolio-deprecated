#include "PuzzleApp.h"
#include "OscMessage.h"
#include "Hand.h"

namespace
{
	const float k_deltay = 10;
	const float KINECT_ROTATE = 0.3f;
	const float minScale = 0.5f;
	const float maxScale = 2.0f;
	int transferToClientX( int x )
	{
		return x;
	}
	int transferToClientY( int y )
	{
		return y;
	}
	const float Pi = 3.14f;
	const float Pi_half = Pi*0.5f;
	const float Pi_two = Pi*2;
}

void PuzzleApp::onOscMessage( const osc::Message* msg )
{
	if (msg->getNumArgs() != 3)
		return;
	string addr = msg->getAddress();
	string action = msg->getArgAsString(0);
//	console()<<addr<<action<<endl;
	int x = static_cast<int>(getWindowWidth()*msg->getArgAsFloat(1));
	int y = static_cast<int>(getWindowHeight()*msg->getArgAsFloat(2));

	x = transferToClientX(x);
	y = transferToClientY(y);

	int id = (addr == "/left") ? LEFT : RIGHT;
	_hands[id]->pos.set(x,y);
	if (action == "push" || action == "close")
	{
		if (_hands[id]->state == Hand::NORMAL)
			_hands[id]->state = Hand::CLICK;
		else//already clicked
			_hands[id]->state = Hand::DRAG;
	}
	else if (action == "pull" || action == "open")
		_hands[id]->state = Hand::NORMAL;
}

void PuzzleApp::update()
{
	if (_hands[LEFT]->state != Hand::NORMAL && _hands[RIGHT]->state != Hand::NORMAL)
	{
		Vec2f diff = _hands[RIGHT]->pos - _hands[LEFT]->pos;
		Vec2f polar = toPolar(diff);
		console()<<polar.x<<" "<<polar.y<<endl;
		float theta = polar.y;//[0, PI_two) clockwise
		float sign = 0.0f;
		if (theta > Pi)
			theta -= Pi;
		if (theta < Pi_half)
		{
			sign = theta*0.5f;
		}
		else if (theta > Pi_half)
		{
			theta = Pi - theta;
			sign = -theta*0.5f;
		}
		_rotate = sign;
		float loBound = 150;
		float hiBound = getWindowWidth();
		_scale = lmap(polar.x, loBound, hiBound, minScale, maxScale);
		_scale = constrain(_scale, minScale, maxScale);
	}
	StateMachine::update();
}