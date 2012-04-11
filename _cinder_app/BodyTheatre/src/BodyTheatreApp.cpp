#include "BodyTheatreApp.h"
#include "Player.h"

void BodyTheatreApp::setup()
{
	_listener = shared_ptr<osc::Listener>(new osc::Listener);
	_listener->setup(7777);
	_listener->registerMessageReceived(this, &BodyTheatreApp::onOscMessage);

	players = new Player[N_PLAYERS];
} 

void BodyTheatreApp::shutdown()
{
	_listener->shutdown();
	delete[] players;
}


CINDER_APP_BASIC( BodyTheatreApp, RendererGl )
