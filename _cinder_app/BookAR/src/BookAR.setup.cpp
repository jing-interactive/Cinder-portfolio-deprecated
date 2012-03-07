#ifdef USING_ARTK
#include <ARToolKitPlus/TrackerSingleMarker.h>
#endif

#include "BookAR.h"
#include <cinder/ImageIo.h>
#include <cinder/params/Params.h>
#include <cinder/Utilities.h>
#include <cinder/Text.h>
#include <cinder/Xml.h>
#include "ARTracker/ARTracker.h"
#include "UI/UIElement.h"
#include "UI/UIDef.h"
#include "Content/ContentManager.h"

using namespace ARContent;

namespace
{
	enum{
		//main layout
		SPAC_LEFT = 41,
		SPAC_RIGHT = 38,
		SPAC_UP = 154,
		SPAC_DOWN = 247,
		//button
		BUTTON_W = 64,
		BUTTON_H = 55,
		BUTTON_X0 = SPAC_LEFT,
		BUTTON_Y0 = 563,
		//thumb view
		THUMB_X0 = BUTTON_X0,
		THUMB_Y0 = 508,
		THUMB_H = 50,
		THUMB_SPAC = 5,
	};

	char* ui_button_images[] = {
		"UI/view2.png",
		"UI/make2.png",
		"UI/camera.png",
		"UI/share2.png",
		"UI/friend2.png"
	};
	const int N_BUTTONS = _countof(ui_button_images);
}

void BookAR::prepareSettings( Settings *settings )
{
	settings->setWindowSize( APP_W, APP_H );
	settings->setFullScreen( false );
	settings->setResizable( false );
	settings->setFrameRate( 60 );
	settings->setBorderless(false);
	settings->setTitle("SDAR App");
}

bool BookAR::readAppConfig( const string& appXml )
{	
	vector<string> mdl_files;
	vector<string> thumb_files;
	try
	{
		XmlTree doc(loadFile(appXml));
		XmlTree firstConfig = doc.getChild("SDARConfig");

		for( XmlTree::Iter item = firstConfig.begin(); item != firstConfig.end(); ++item )
		{
			//key
			string name = item->getAttribute("name");
			string thumb = item->getAttribute("thumb");
			mdl_files.push_back(name);
			thumb_files.push_back(thumb);
		}
	}
	catch( ... ) {
		console() << "[ERROR] Failed to readAppConfig from " << appXml<<std::endl;
		return false;
	}
	_ar_tracker->setup(CAM_W, CAM_H, 10, 1000, static_cast<void*>(&mdl_files));

	//thumbs
	int thumb_x = THUMB_X0;
	for (int i=0;i<thumb_files.size();i++)
	{
		Surface thumb_img = loadImage(getAppPath().generic_string()+thumb_files[i]);
		float ratio = thumb_img.getAspectRatio();
		int thumb_w = static_cast<int>(THUMB_H*ratio);
		_thumbs.push_back(shared_ptr<UIElement>(new UIElement(THUMB_BASE + i, thumb_x, THUMB_Y0, thumb_w, THUMB_H, thumb_img)));
		thumb_x += thumb_w+THUMB_SPAC;
	}

	return true;
}


void BookAR::setup()
{	
	_content_mgr = shared_ptr<ContentManager>(new ContentManager);
	bool result = _content_mgr->load(getAppPath().generic_string()+"config/interactions.plist");

	for (int i=0;i<N_BUTTONS;i++)
	{
	//	ImageSourceRef img = loadImage(getAppPath().generic_string()+ui_button_images[i]);
		_buttons.push_back(shared_ptr<UIElement>(new UIElement(BUTTON_BASE + i, BUTTON_X0+BUTTON_W*i, BUTTON_Y0, BUTTON_W, BUTTON_H/*, img*/)));
	}
	
	_tex_iphone4 = loadImage(getAppPath().generic_string()+"UI/iphone4.png");
	_area_capture.set(SPAC_LEFT,SPAC_UP,APP_W-SPAC_RIGHT,APP_H-SPAC_DOWN);

	_ar_tracker = shared_ptr<ARTracker>(ARTracker::create("SDAR"));

	_device_id = 0;

	vector<Capture::DeviceRef> devices( Capture::getDevices() ); 
	if (getArgs().size() == 2)
	{
		string arg = getArgs().back();
		_device_id = fromString<int>(arg);
		ci::constrain<int>(_device_id, 0, devices.size()-1);
	}

	if (_device_id < (int)devices.size())
	{
		try {
			_capture = Capture( CAM_W, CAM_H, devices[_device_id] );
			_capture.start();
		}
		catch( ... ) {
			console() << "Failed to initialize capture" << std::endl;
		}
	}
	else
	{
		TextLayout layout;
		layout.setFont( Font( "Arial", 24 ) );
		layout.setColor( Color( 1, 1, 1 ) );
		layout.addLine("");
		layout.addLine("");
		layout.addLine( "No camera connection!");
		layout.addLine("");
		layout.addLine("");
		_tex_bg = layout.render(true);
	}

	if (!readAppConfig(getAppPath().generic_string()+"config/sdar.xml"))
		quit();

	//param	
	mParams = shared_ptr<params::InterfaceGl>(new params::InterfaceGl( "App parameters", Vec2i( 200, 100 ) ));
	{
		_cube_scale = 0.5;
		mParams->addParam( "Cube Scale", &_cube_scale, "min=0.1 max=2.0 step=0.1 keyIncr=s keyDecr=S");

		_cube_clr = ColorA( 0.25f, 0.5f, 1.0f, 1.0f );
		mParams->addParam( "Cube Color", &_cube_clr, "" );

		_mesh_translate = Vec3f( 0, 0, 50 );
		mParams->addParam( "Mesh Translate", &_mesh_translate, "");

		//mParams->addSeparator();

		_light_dir = Vec3f( .5f, -.5f, -.6f );
		mParams->addParam( "Light Direction", &_light_dir, "" );

		_capture_visible = true;
		//mParams->addParam( "capture stream visible", &_capture_visible, "");

		_2dbook_visible = true;
		mParams->addParam( "2d texture visible", &_2dbook_visible, "");

		_3dbook_visible = true;
		mParams->addParam( "3d mesh visible", &_3dbook_visible, ""); 

		_using_sdar = true;
		//mParams->addParam( "SNDA SDK", &_using_sdar, ""); 

	//	_proj_near = 10;
	//	mParams->addParam( "_proj_near", &_proj_near, "min=1.0 max=100 step=1");

	//	_proj_far = 1000;
	//	mParams->addParam( "_proj_far", &_proj_far, "min=10 max=10000 step=10");
	}

	setupStates();

	changeToState(_state_tracking); 
}


void BookAR::shutdown()
{

}

CINDER_APP_CONSOLE( BookAR, RendererGl )
