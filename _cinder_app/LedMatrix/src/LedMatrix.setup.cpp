#include "LedMatrixApp.h"
#include "OscListener.h"
#include "LedManager.h"
#include "Resources.h"
#include <cinder/Arcball.h>
#include <cinder/MayaCamUI.h>
#include <cinder/Rand.h>
#include <cinder/ImageIo.h>
#include "config.h"

namespace
{
	const char* configFile = "./config.xml";
}

void LedMatrixApp::prepareSettings( Settings *settings )
{
	if (!loadConfig(configFile))
		saveConfig(configFile);

	settings->setAlwaysOnTop(false);
	settings->setBorderless(true);
	settings->setWindowPos(Vec2i::zero());
}

void LedMatrixApp::setup()
{
	show_3d = true;
	Rand::randomize();

	LedManager::setTexture(loadImage(loadResource(IMG_PARTICLE)));
	//kinect/osc
	listener = shared_ptr<osc::Listener>(new osc::Listener());
	listener->setup(OSC_PORT);
	listener->registerMessageReceived(this, &LedMatrixApp::onKinect);

	//cam
	maya_cam = shared_ptr<MayaCamUI>(new MayaCamUI);
	{
		CameraPersp initialCam;
		initialCam.setPerspective( 45.0f, getWindowAspectRatio(), 0.1f, 10000 );
		initialCam.setEyePoint(LedManager::getWatchPoint());
		maya_cam->setCurrentCam( initialCam );
	}

	//state
	setupStates();
}
