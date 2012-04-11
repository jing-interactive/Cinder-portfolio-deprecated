#include "BodyTheatreApp.h"

void BodyTheatreApp::setup()
{
	_listener = shared_ptr<osc::Listener>(new osc::Listener);
	_listener->setup(7777);
	_listener->registerMessageReceived(this, &BodyTheatreApp::onOscMessage);
} 

void BodyTheatreApp::shutdown()
{
	_listener->shutdown();
}


CINDER_APP_BASIC( BodyTheatreApp, RendererGl )
