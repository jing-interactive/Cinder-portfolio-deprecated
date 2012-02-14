#include "cinder/app/AppBasic.h"
#include "cinder/Capture.h"
#include "cinder/Surface.h"
#include "cinder/Matrix44.h"
#include "cinder/Thread.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
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
		APP_W = 800,
		APP_H = 600,
		CAM_W = 320,
		CAM_H = 240,
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

private: //rendering
	bool _2dbook_visible;
	bool _3dbook_visible;
	gl::VboMesh _book_mesh;
	uint8_t 

private:
	bool _using_sdar;
	ci::Vec3f _light_dir;
	ci::ColorA _cube_clr;
	float _proj_near;
	float _proj_far;

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
