#include "App.h"
#include <boost/foreach.hpp>
#include "Player.h"
#include "KinectRoutine.h"

#define DRAW_NODES_OUTLINE

void BodyTheatreApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear(Color::hex(0xfff1c3));
	{
		lock_guard<mutex> lock(_mtx_player);
		gl::color(0.7f, 0.7f, 0.7f, 0.1f);

#ifdef DRAW_PLAYER_OUTLINE
		for (int i=0;i<N_PLAYERS;i++)
		{
			players[i].drawOutline();
		}
#endif
		if (_activeIdx != INVALID_IDX)
		{
			players[_activeIdx].draw();

#ifdef DRAW_NODES_OUTLINE
		if (players[_activeIdx].state == Player::T_SPLITTED)
		{
			glLineWidth(2);
			gl::color(ColorA(0,0,0,0.5f));
			for (int i=0;i<2;i++)
			{
				BOOST_FOREACH(TrackedNode& n, activeNodes[i])
					n._ref->drawOutline();
			}
			_routine->draw();
		}
#endif
		}
	}
}
