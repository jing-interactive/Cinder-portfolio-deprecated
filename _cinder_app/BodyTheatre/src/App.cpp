#include "App.h"
#include "Player.h"
#include "KinectRoutine.h"
#include "cinder/Rand.h"

void BodyTheatreApp::prepareSettings( Settings *settings )
{
	settings->setBorderless(true);
	settings->setWindowSize(800, 600);
	settings->setWindowPos(0, 0);
}

void BodyTheatreApp::setup()
{
	Rand::randomize();

	hideCursor();

	_listener = shared_ptr<osc::Listener>(new osc::Listener);
	_listener->setup(7777);
	_listener->registerMessageReceived(this, &BodyTheatreApp::onOscMessage);

	players = new Player[N_PLAYERS];
	_activeIdx = INVALID_IDX;

	_routine = shared_ptr<KinectRoutine>(new KinectRoutine);
} 

void BodyTheatreApp::shutdown()
{
	_listener->shutdown();
	delete[] players;
}

CINDER_APP_BASIC( BodyTheatreApp, RendererGl )
