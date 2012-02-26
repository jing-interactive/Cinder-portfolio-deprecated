#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"
#include <boost/foreach.hpp>
#include "Sprite.h"
#include "cinder/Timeline.h"

namespace
{
	bool cmpSpriteByZ(const shared_ptr<Sprite>& lhs, const shared_ptr<Sprite>& rhs)
	{
		return lhs->_z < rhs->_z;
	}
	const float k_Rotate = 5.0f;
	const float ABOUT_ZERO = 10;
}

void StateGame::enter()
{
	_app._sprite_selected.reset();
}

void StateGame::update()
{
	if (_app._sprite_selected)
	{
		if (_app._hands[RIGHT].push)//update selected sprite's pos && rotation
		{
			_app._sprite_selected->_pos = _app._hands[RIGHT].pos;
			_app._sprite_selected->_degree = _app._sprite_selected->_degree + _app._rotate * k_Rotate;

			//TODO: make more degrees
			int int_degree = _app._sprite_selected->_degree;
// 			if (abs(_app._sprite_selected->_degree - 180) < ABOUT_ZERO)
// 				_app._sprite_selected->_degree = 180;
			_app._rotate = 0;//reset
		}
		else
			_app._sprite_selected.reset();			
	}
	else if (_app._hands[RIGHT].push)//if non selected and push action fires
	{
		BOOST_REVERSE_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
		{//find the hit one
			if (spr->isPointInside(_app._hands[RIGHT].pos))
			{
				_app._sprite_selected = spr;
				break;
			}
		}
		if (_app._sprite_selected)
		{
			_app._sprite_selected->_z = _app._next_z++;
			std::sort(_app._sprites.begin(), _app._sprites.end(), cmpSpriteByZ);
		}
	}
}

void StateGame::draw()
{
	gl::color(Color8u::white());
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		spr->draw();
	}
}

void StateGame::exit()
{

}
