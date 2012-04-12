#include "BodyTheatreApp.h"
#include "Player.h"

#define OPENCV_VERSION CVAUX_STR(CV_MAJOR_VERSION)""CVAUX_STR(CV_MINOR_VERSION)""CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef _DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION".lib")
#else	//_DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION".lib")
#endif	//_DEBUG

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
