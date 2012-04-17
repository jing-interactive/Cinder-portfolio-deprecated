#include "KinectRoutine.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

namespace
{
	bool verbose = false;
	int toHandId(const MouseEvent& event)
	{
		if (event.isLeft())
			return LEFT;
		else if (event.isRight())
			return RIGHT;
		else return N_HANDS;
	}

	int toDragHandId(const MouseEvent& event)
	{
		if (event.isLeftDown())
			return LEFT;
		else if (event.isRightDown())
			return RIGHT;
		else return N_HANDS;
	}

	bool isIdValid(int id)
	{
		return id == LEFT || id == RIGHT;
	}
}

KinectRoutine::KinectRoutine()
{
	_scale = 1;
	_rotate = 0;

	ColorA hand_clrs[N_HANDS]={ColorA8u(0,14,60, 200),ColorA(0.5f,0,1.0f,0.8f)};
	for (int i=0;i<N_HANDS;i++)
	{
		_hands[i].clr = hand_clrs[i];
		_hands[i].pos.set(Rand::randInt(getWindowWidth()), Rand::randInt(getWindowHeight()));
	}
}

void KinectRoutine::mouseMove( MouseEvent event )
{
	if (verbose) 
		console() << "move" <<endl;
	int id = RIGHT;
	{
		_hands[id].pos.set(event.getPos());
		_hands[id].state = Hand::NORMAL;
	}
}

void KinectRoutine::mouseDrag( MouseEvent event )
{
	if (verbose) 
		console() << "drag" <<endl;
	int id = toDragHandId(event);
	if (isIdValid(id))
	{
		_hands[id].state = Hand::DRAG;
		_hands[id].pos.set(event.getPos());
	}
}

void KinectRoutine::mouseDown( MouseEvent event )
{
	if (verbose) 
		console() << "down" <<endl;
	int id = toHandId(event);
	if (isIdValid(id))
	{
		_hands[id].pos = event.getPos();
		_hands[id].state = Hand::CLICK;
	}
}

void KinectRoutine::mouseUp( MouseEvent event )
{
	if (verbose) 
		console() << "up" <<endl;
	int id = toHandId(event);
	if (isIdValid(id))
	{
		_hands[id].state = Hand::NORMAL;
	}
}

void KinectRoutine::mouseWheel( MouseEvent event )
{
	if (verbose) 
		console() << event.getWheelIncrement() <<endl;
	_rotate = event.getWheelIncrement();//{-1, 1}
}

void KinectRoutine::onOscMessage( const osc::Message* msg, int* filter_id)
{
	if (msg->getNumArgs() != 5)
		return;
	string addr = msg->getAddress();
	string action = msg->getArgAsString(0);
	if (verbose) 
		console()<<addr<<action<<endl;
	int x = static_cast<int>(getWindowWidth()*msg->getArgAsFloat(1));
	int y = static_cast<int>(getWindowHeight()*msg->getArgAsFloat(2));
	int dev = msg->getArgAsInt32(3);
	int plyIdx = msg->getArgAsInt32(4);

	if (filter_id && plyIdx != *filter_id)//skip unmatched msg
		return;

	console()<<plyIdx<<endl;

	int id = (addr == "/left") ? LEFT : RIGHT;
	_hands[id].pos.set(x,y);
	if (action == "push" || action == "close")
	{
		if (_hands[id].state == Hand::NORMAL)
			_hands[id].state = Hand::CLICK;
		else//already clicked
			_hands[id].state = Hand::DRAG;
	}
	else if (action == "pull" || action == "open")
		_hands[id].state = Hand::NORMAL;
}

void KinectRoutine::draw()
{
	gl::enableAlphaBlending();
	for (int i=0;i<N_HANDS;i++)
	{
		_hands[i].draw();
	}
}

void KinectRoutine::update()
{
	if (_hands[LEFT].state != Hand::NORMAL && _hands[RIGHT].state != Hand::NORMAL)
	{
		Vec2f diff = _hands[RIGHT].pos - _hands[LEFT].pos;
		Vec2f polar = toPolar(diff);
		console()<<polar.x<<" "<<polar.y<<endl;
		float theta = polar.y;//[0, PI_two) clockwise
		float sign = 0.0f;
		if (theta > M_PI)
			theta -= M_PI;
		if (theta < M_PI/2)
		{
			sign = theta*0.5f;
		}
		else if (theta > M_PI/2)
		{
			theta = M_PI - theta;
			sign = -theta*0.5f;
		}
		_rotate = sign;
// 		float loBound = 150;
// 		float hiBound = getWindowWidth();
// 		_scale = lmap(polar.x, loBound, hiBound, minScale, maxScale);
// 		_scale = constrain(_scale, minScale, maxScale);
	}
}

bool KinectRoutine::isPlayerIdValid( int id )
{
	return id >= 0 && id <= 5;
}
