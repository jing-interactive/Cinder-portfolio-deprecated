#include "App.h"
#include <boost/foreach.hpp>
#include "Player.h"
#include "KinectRoutine.h"

void BodyTheatreApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.2f, 0.3f, 0.4f ) );
	{
		lock_guard<mutex> lock(_mtx_player); 
		for (int i=0;i<N_PLAYERS;i++)
		{
			players[i].draw();
		}
	}

#if 0
	glLineWidth(2);
	gl::color(ColorA(0,0,0,0.5f));
	BOOST_FOREACH(TrackedNode& n, activeNodes)
		n._ref->drawOutline();
#endif
	_routine->draw();
}
