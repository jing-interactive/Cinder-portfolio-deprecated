#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"
#include <boost/foreach.hpp>
#include "Sprite.h"
#include "cinder/Timeline.h"
#include "Hand.h"

namespace
{
	bool cmpSpriteByZ(const shared_ptr<Sprite>& lhs, const shared_ptr<Sprite>& rhs)
	{
		return lhs->_z < rhs->_z;
	}
	shared_ptr<Sprite> the_spr;
}

void StateGame::enter()
{
	_app._sprite_selected.reset();
	the_spr.reset();
}

void StateGame::update()
{
	bool gameover = true;
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		if (!spr->isOK())
			gameover = false;
	}

	if (gameover)
	{
		_app.changeToState(_app._state_gameover);
		return;
	}

	shared_ptr<Sprite>& sprite_selected = _app._sprite_selected;
	if (sprite_selected)
	{
		if (abs(_app._rotate) > FLT_EPSILON)//update rotation
		{//ignore move action
			sprite_selected->addDegree(_app._rotate);
			_app._rotate = 0;//reset
		}
		else
		if (_app._hands[RIGHT]->state == Hand::DRAG)//update position
		{
			sprite_selected->setPosFromCursor(_app._hands[RIGHT]->pos);
		}
		else
		{
			sprite_selected->_state = Sprite::NORMAL;
			sprite_selected.reset();	
		}
	}
	else 
	{
		if (the_spr)
		{//clear previous sprite status
			the_spr->_state = Sprite::NORMAL;
			the_spr.reset();
		}
		BOOST_REVERSE_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
		{//find the hit one
			if (spr->isPointInside(_app._hands[RIGHT]->pos))
			{				
				the_spr = spr;
				break;
			}
		}
		if (the_spr)
		{
			if (_app._hands[RIGHT]->state != Hand::NORMAL)
			{//if non selected and push action fires
				console() << "ok" <<endl;
				sprite_selected = the_spr;
				sprite_selected->setPivotFromCursor(_app._hands[RIGHT]->pos);
				sprite_selected->_state = Sprite::CLICK; 
				sprite_selected->_z = _app._next_z++;
				std::sort(_app._sprites.begin(), _app._sprites.end(), cmpSpriteByZ); 
			}
			else
			{//hover on 
				the_spr->_state = Sprite::CLICK;
			}
		}
	}
}

void StateGame::draw()
{
	shared_ptr<Sprite>& sprite_selected = _app._sprite_selected;

	gl::enableAlphaBlending();
	gl::color(1,1,1,0.8f);
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		spr->draw();
	}
	gl::disableAlphaBlending();
	if (the_spr)
		the_spr->drawBox();
}

void StateGame::exit()
{
	
}
