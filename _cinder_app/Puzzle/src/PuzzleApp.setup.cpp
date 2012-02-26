#include "PuzzleApp.h"
#include "OscListener.h"
#include "cinder/ImageIo.h"

void PuzzleApp::prepareSettings(Settings *settings){
	settings->setFrameRate(60.0f);
	settings->setWindowSize(848, 564);
	settings->setResizable(false);
}

void PuzzleApp::setup()
{
	_next_z = 0;

	hideCursor();

	listener = shared_ptr<osc::Listener>(new osc::Listener());
	listener->setup(3333);
	listener->registerMessageReceived(this, &PuzzleApp::onOscMessage);

	_img_list.push_back(loadImage(getAppPath().generic_string()+"/media/photo_2.jpg"));
	_img_list.push_back(loadImage(getAppPath().generic_string()+"/media/photo_4.jpg"));

	setupStates();
}
 
CINDER_APP_BASIC( PuzzleApp, RendererGl )
