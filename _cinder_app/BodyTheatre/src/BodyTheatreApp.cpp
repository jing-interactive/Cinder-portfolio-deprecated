#include "BodyTheatreApp.h"
#include "Player.h"

void BodyTheatreApp::setup()
{
	_listener = shared_ptr<osc::Listener>(new osc::Listener);
	_listener->setup(7777);
	_listener->registerMessageReceived(this, &BodyTheatreApp::onOscMessage);

	players = new Player[N_PLAYERS];

	b2Vec2 gravity(0.0f, -10.0f);
	_world = shared_ptr<b2World>(new b2World(gravity));
} 

void BodyTheatreApp::shutdown()
{
	_listener->shutdown();
	delete[] players;
}


CINDER_APP_BASIC( BodyTheatreApp, RendererGl )
