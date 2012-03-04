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
	const float randomRadius = 50.0f;
	EaseFn easeAnim = EaseInOutQuad();
}

void StateShuffle::enter()
{
	resetTimer();
	timeline().clear();

	_app.shuffleSelectedImage(N_TILES);

	vector<Vec2f> centers;
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		centers.push_back(spr->_center.value() + randVec2f()*randomRadius);
	}

	std::random_shuffle(centers.begin(), centers.end());
	std::random_shuffle(centers.begin(), centers.end());
	int i = 0;
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		timeline().apply( &spr->_center, 
			centers[i++], timeAnim, easeAnim);
		if (randInt(10) > 4)
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
	gl::disableAlphaBlending();
	gl::color(1,1,1);
	BOOST_FOREACH(shared_ptr<Sprite> spr, _app._sprites)
	{
		spr->draw();
	}
}

void StateShuffle::exit()
{

}
