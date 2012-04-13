#include "App.h"
#include <boost/foreach.hpp>
#include "Player.h"
#include "KinectRoutine.h"

void BodyTheatreApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.9f, 0.9f, 0.9f ) );
	{
		lock_guard<mutex> lock(_mtx_player); 
		for (int i=0;i<N_PLAYERS;i++)
		{
			players[i].draw();
		}
	}
	_routine->draw();
}
