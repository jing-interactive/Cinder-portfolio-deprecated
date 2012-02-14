#include <ARToolKitPlus/TrackerSingleMarker.h>
#include "BookARApp.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"

#include "../../_common/SDAR/ModelDLL.h"

void BookARApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( APP_W, APP_H );
	settings->setFullScreen( false );
	settings->setResizable( true );
	settings->setFrameRate( 60 );
}

void BookARApp::setup()
{
	try {
		_capture = Capture( CAM_W, CAM_H );
		_capture.start();
	}
	catch( ... ) {
		console() << "Failed to initialize capture" << std::endl;
	}

	if (getArgs().size() == 2 && getArgs().back()=="artk")
		_ar_engine = ENGINE_ARTK;
	else
		_ar_engine = ENGINE_SNDA;

	int bRet = 0;
	SDARStart(CAM_W, CAM_H);
	bRet = SDARLoad((getAppPath()+"../../resources/movie_transformerb.mdl").c_str()); if(!bRet) return;
	bRet = SDARLoad((getAppPath()+"../../resources/movie_braveb.mdl").c_str());       if(!bRet) return;
	bRet = SDARLoad((getAppPath()+"../../resources/movie_piratesb.mdl").c_str());     if(!bRet) return;
	bRet = SDARLoad((getAppPath()+"../../resources/movie_tintinb.mdl").c_str());      if(!bRet) return;
	
	_artk_tracker = shared_ptr<ARToolKitPlus::TrackerSingleMarker>(new ARToolKitPlus::TrackerSingleMarker(CAM_W, CAM_H, 8, 6, 6, 6, 0));
#if 0
	_tex_android = loadImage(loadResource(IMG_ANDROID));
#else
	_tex_android = loadImage(getAppPath()+"../../resources/ocvcookbook.png");
#endif

	_cube_scale = 1.0f;
	mParams = shared_ptr<params::InterfaceGl>(new params::InterfaceGl( "App parameters", Vec2i( 200, 200 ) ));
	mParams->addParam( "Cube Scale", &_cube_scale, "min=1.0 max=20.0 step=0.5 keyIncr=s keyDecr=S");	
}


CINDER_APP_CONSOLE( BookARApp, RendererGl )
