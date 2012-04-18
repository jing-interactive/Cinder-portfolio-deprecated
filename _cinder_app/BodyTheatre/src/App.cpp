#include "App.h"
#include "Player.h"
#include "KinectRoutine.h"
#include "cinder/Rand.h"
#include "CiTool.h"
#include "Config.h"

void BodyTheatreApp::prepareSettings( Settings *settings )
{
	settings->setBorderless(true);
	settings->setWindowSize(APP_WIDTH, APP_HEIGHT);
	settings->setWindowPos(0, 0);
}

void BodyTheatreApp::setup()
{
	Rand::randomize();

	if (!loadConfig("BodyTheatre.xml"))
		saveConfig("BodyTheatre.xml");

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

//#pragma comment(linker, "/SUBSYSTEM:CONSOLE /entry:main")
CINDER_APP_CONSOLE( BodyTheatreApp, RendererGl )
