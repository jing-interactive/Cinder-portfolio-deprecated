#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/Capture.h"
#include "cinder/Surface.h"
#include "cinder/gl/Texture.h"
#include "cinder/Matrix44.h"
#include "cinder/Thread.h"

#include "Resources.h"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace cinder{namespace params{
	class InterfaceGl;
}}

namespace ARToolKitPlus{
	class TrackerSingleMarker;
}

class BookARApp : public AppBasic
{
	enum
	{
		ENGINE_ARTK,
		ENGINE_SNDA,
	};
	enum
	{
		APP_W = 800,
		APP_H = 600,
		CAM_W = 640,
		CAM_H = 480,
	};
  public:
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
	void keyDown(KeyEvent event);

	void prepareSettings( Settings *settings );

private:
	Capture _capture;
	gl::Texture _tex_bg;
	gl::Texture _tex_android;

	std::mutex _mtx_ar;
	Matrix44d _mat_modelview;
	Matrix44d _mat_proj;
	ci::Vec2f  _pts_corner[4];
	int		_n_trackables;
	double* prj;
	double* modelview;

	shared_ptr<params::InterfaceGl>	mParams;
	float _cube_scale;

	shared_ptr<ARToolKitPlus::TrackerSingleMarker> _artk_tracker;

private:
	int _ar_engine;

private:
	float cameraXToScreenX(float cx)
	{
		return cx*APP_W/CAM_W;
	}

	float cameraYToScreenY(float cy)
	{
		return cy*APP_H/CAM_H;
	}
};
