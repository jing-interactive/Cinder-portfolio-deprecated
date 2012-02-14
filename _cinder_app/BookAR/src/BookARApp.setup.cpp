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

	_using_sdar = true;
	if (getArgs().size() == 2 && getArgs().back()=="artk")
		_using_sdar = false;

	SDARStart(CAM_W, CAM_H);
	{
		int bRet = 0;
		bRet = SDARLoad((getAppPath()+"../../resources/movie_transformerb.mdl").c_str()); if(!bRet) return;
		bRet = SDARLoad((getAppPath()+"../../resources/movie_braveb.mdl").c_str());       if(!bRet) return;
		bRet = SDARLoad((getAppPath()+"../../resources/movie_piratesb.mdl").c_str());     if(!bRet) return;
		bRet = SDARLoad((getAppPath()+"../../resources/movie_tintinb.mdl").c_str());      if(!bRet) return;
#if 0
		_tex_android = loadImage(loadResource(IMG_ANDROID));
#else
		_tex_android = loadImage(getAppPath()+"../../resources/ocvcookbook.png");
#endif
	}

	_artk_tracker = shared_ptr<ARToolKitPlus::TrackerSingleMarker>(new ARToolKitPlus::TrackerSingleMarker(CAM_W, CAM_H, 8, 6, 6, 6, 0));
	{
		_artk_tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
		// load a camera file.
		std::string path(getAppPath()+"../../resources/no_distortion.cal");
		if (!_artk_tracker->init(path.c_str(), 1.0f, 1000.0f)) {
			console() << "ERROR: init() failed\n";
			return;
		}
		_artk_tracker->getCamera()->printSettings();
		_artk_tracker->setBorderWidth(0.125f);
		// set a threshold. we could also activate automatic thresholding
	#if 1
		_artk_tracker->setThreshold(150);
	#else
		_artk_tracker->activateAutoThreshold(true);
	#endif
		// let's use lookup-table undistortion for high-speed
		// note: LUT only works with images up to 1024x1024
		_artk_tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
		// switch to simple ID based markers
		// use the tool in tools/IdPatGen to generate markers
		_artk_tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_BCH);

		_artk_tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
	}

	//param	
	mParams = shared_ptr<params::InterfaceGl>(new params::InterfaceGl( "App parameters", Vec2i( 200, 200 ) ));
	{
		_cube_scale = 1.0f;
		mParams->addParam( "Cube Scale", &_cube_scale, "min=1.0 max=20.0 step=0.5 keyIncr=s keyDecr=S");

		_cube_clr = ColorA( 0.25f, 0.5f, 1.0f, 1.0f );
		mParams->addParam( "Cube Color", &_cube_clr, "" );

		mParams->addSeparator();

		_light_dir = Vec3f( .5f, -.5f, -.6f );
		mParams->addParam( "Light Direction", &_light_dir, "" );

		_2dbook_visible = false;
		mParams->addParam( "book visible", &_2dbook_visible, "");

		_3dbook_visible = true;
		mParams->addParam( "model visible", &_3dbook_visible, "");

		mParams->addParam( "SNDA AR", &_using_sdar, "");

		_proj_near = 10;
		mParams->addParam( "_proj_near", &_proj_near, "min=1.0 max=100 step=1");

		_proj_far = 1000;
		mParams->addParam( "_proj_far", &_proj_far, "min=10 max=10000 step=10");
	}
}


CINDER_APP_CONSOLE( BookARApp, RendererGl )
