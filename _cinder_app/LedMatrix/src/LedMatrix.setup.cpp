#include "LedMatrixApp.h"
#include "OscListener.h"
#include "LedManager.h"
#include <cinder/Arcball.h>
#include <cinder/MayaCamUI.h>

namespace
{
	const int port = 7777;
}

void LedMatrixApp::setup()
{
	//kinect/osc
	listener = shared_ptr<osc::Listener>(new osc::Listener());
	listener->setup(port);
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
