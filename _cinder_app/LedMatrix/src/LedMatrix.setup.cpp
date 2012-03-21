#include "LedMatrixApp.h"
#include "OscListener.h"
#include "LedManager.h"
#include "Resources.h"
#include <cinder/Arcball.h>
#include <cinder/MayaCamUI.h>
#include <cinder/Rand.h>
#include <cinder/ImageIo.h>
#include "ImageList.h"
#include "config.h"

ImageList image_lists[2];

void LedMatrixApp::reloadConfig()
{
	const char* configFile = "config.xml";
	string config = getAppPath().generic_string() + configFile;
	if (!loadConfig(config.c_str()))
		saveConfig(config.c_str());
}

void LedMatrixApp::prepareSettings( Settings *settings )
{
	reloadConfig();

	settings->setAlwaysOnTop(false);
	settings->setBorderless(true);
	settings->setFrameRate(60);
	settings->setWindowPos(Vec2i::zero());
}

void LedMatrixApp::setup()
{
	show_3d = true;
	Rand::randomize();

	//
	string folders[] = {"block01", "block02"};
	for (int i=0;i<2;i++)
	{
		image_lists[i].setup(folders[i], RIPPLE_MOVE_SPEED);
	}

	LedManager::setTexture(loadImage(loadResource(IMG_PARTICLE)));
	//kinect/osc
	debug_puts("[start]listener->registerMessageReceived()\n");
	listener = shared_ptr<osc::Listener>(new osc::Listener());
	listener->setup(OSC_PORT);
	listener->registerMessageReceived(this, &LedMatrixApp::onKinect);
	debug_puts("[end]listener->registerMessageReceived()\n");

	//cam
	maya_cam = shared_ptr<MayaCamUI>(new MayaCamUI);
	{
		CameraPersp initialCam;
		initialCam.setPerspective( 45.0f, getWindowAspectRatio(), 0.1f, 10000 );
		initialCam.setEyePoint(LedManager::getWatchPoint());
		maya_cam->setCurrentCam( initialCam );
	}

	//state
	debug_puts("[start]setupStates()\n");
	setupStates();
	debug_puts("[end]setupStates()\n");
}
