#ifdef USING_ARTK
#include <ARToolKitPlus/TrackerSingleMarker.h>
#endif

#include "BookARApp.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"
#include "../../../_common/SDAR/SDAR.h"

void BookARApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( APP_W, APP_H );
	settings->setFullScreen( false );
	settings->setResizable( true );
	settings->setFrameRate( 60 );
}

void BookARApp::setup()
{
#if 0
	mdl_files[0] = "movie_transformerb.mdl";
	mdl_files[1] = "movie_braveb.mdl";
	mdl_files[2] = "movie_piratesb.mdl";
	mdl_files[3] = "movie_tintinb.mdl";
#else
	mdl_files[0] = "transformer.mdl";
	mdl_files[1] = "brave.mdl";
	mdl_files[2] = "pirates.mdl";
	mdl_files[3] = "tintin.mdl";
#endif

	post_files[0] = "transformer.jpg";
	post_files[1] = "brave.jpg";
	post_files[2] = "pirates.jpg";
	post_files[3] = "tintin.jpg";

	_device_id = 0;

	vector<Capture::DeviceRef> devices( Capture::getDevices() ); 
	if (getArgs().size() == 2)
	{
		string arg = getArgs().back();
		_device_id = fromString<int>(arg);
		ci::constrain<int>(_device_id, 0, devices.size()-1);
	}

	try {
		_capture = Capture( CAM_W, CAM_H, devices[_device_id] );
		_capture.start();
	}
	catch( ... ) {
		console() << "Failed to initialize capture" << std::endl;
	}

	SDARStart(CAM_W, CAM_H);
	{
		int bRet = 0;
		for (int i=0;i<4;i++)
			bRet = SDARLoad((char*)(getAppPath().generic_string()+"../../resources/"+mdl_files[i]).c_str()); if(!bRet) return;
	}


	_tex_android = loadImage(getAppPath().generic_string()+"../../resources/android.png");

	for (int i=0;i<4;i++)
	{
		_img_posters[i] = loadImage(getAppPath().generic_string()+"../../resources/"+post_files[i]);
	}

#ifdef USING_ARTK
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
#endif

	//param	
	mParams = shared_ptr<params::InterfaceGl>(new params::InterfaceGl( "App parameters", Vec2i( 200, 200 ) ));
	{
		_cube_scale = 0.5;
		mParams->addParam( "Cube Scale", &_cube_scale, "min=0.1 max=2.0 step=0.1 keyIncr=s keyDecr=S");

		_cube_clr = ColorA( 0.25f, 0.5f, 1.0f, 1.0f );
		mParams->addParam( "Cube Color", &_cube_clr, "" );

		_mesh_translate = Vec3f( 0, 0, 50 );
		mParams->addParam( "Mesh Translate", &_mesh_translate, "");

		mParams->addSeparator();

		_light_dir = Vec3f( .5f, -.5f, -.6f );
		mParams->addParam( "Light Direction", &_light_dir, "" );

		_capture_visible = true;
		mParams->addParam( "capture stream visible", &_capture_visible, "");

		_2dbook_visible = true;
		mParams->addParam( "2d texture visible", &_2dbook_visible, "");

		_3dbook_visible = true;
		mParams->addParam( "3d mesh visible", &_3dbook_visible, ""); 

		_using_sdar = true;
		mParams->addParam( "SNDA SDK", &_using_sdar, ""); 

		_proj_near = 10;
		mParams->addParam( "_proj_near", &_proj_near, "min=1.0 max=100 step=1");

		_proj_far = 1000;
		mParams->addParam( "_proj_far", &_proj_far, "min=10 max=10000 step=10");
	}
}


void BookARApp::shutdown()
{
	SDAREnd();
}

CINDER_APP_CONSOLE( BookARApp, RendererGl )
