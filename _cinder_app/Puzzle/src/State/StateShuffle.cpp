#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"
#include "cinder/Timeline.h"
#include "Sprite.h"
#include "cinder/Rand.h"
#include <boost/foreach.hpp>

namespace
{
	const int N_TILES = 3;
	const float timeAnim = 3.0f;
	EaseFn easeAnim = EaseInOutQuad();
}

void StateShuffle::enter()
{
	resetTimer();

	_app.shuffleSelectedImage(N_TILES);
	timeline().clear();
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		timeline().apply( &spr->_center, 
			Vec2f(randFloat(_app.getWindowWidth()), randFloat(_app.getWindowHeight()))
			, timeAnim, easeAnim);
		timeline().apply( &spr->_degree, randFloat(90), timeAnim*0.7f, easeAnim);
	}
}

void StateShuffle::update()
{
	if (getElapsedSeconds() > timeAnim)
		_app.changeToState(_app._state_game);
}

void StateShuffle::draw()
{
	gl::enableAlphaBlending();
	gl::color(1,1,1,0.8f);
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		spr->draw();
	}
	gl::disableAlphaBlending();
}

void StateShuffle::exit()
{

}
