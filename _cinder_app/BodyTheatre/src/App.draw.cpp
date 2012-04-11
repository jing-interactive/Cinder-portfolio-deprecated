#include "BodyTheatreApp.h"
#include <boost/foreach.hpp>
#include "Player.h"

void BodyTheatreApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.3f, 0.3f, 0.3f ) );
	{
		lock_guard<mutex> lock(_mtx_player); 
		for (int i=0;i<N_PLAYERS;i++)
		{
			players[i].draw();
		}
	}	
}
