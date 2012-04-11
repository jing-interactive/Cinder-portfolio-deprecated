#include "BodyTheatreApp.h"
#include <boost/foreach.hpp>

void BodyTheatreApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.3f, 0.3f, 0.3f ) );
	{
		lock_guard<mutex> lock(_mtx_player);
		BOOST_FOREACH(PlayerMap::value_type pair, players)
		{
			pair.second.draw();
		}
	}	
}
