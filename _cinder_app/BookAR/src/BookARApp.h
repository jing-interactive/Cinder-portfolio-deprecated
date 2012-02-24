#include "cinder/app/AppBasic.h"
#include "cinder/Capture.h"
#include "cinder/Surface.h"
#include "cinder/Matrix44.h"
#include "cinder/Thread.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace cinder{namespace params{
	class InterfaceGl;
}}

class BookARApp : public AppBasic
{
	enum
	{
		APP_W = 400,
		APP_H = 750,
		SPAC_LEFT = 42,
		SPAC_RIGHT = 37,
		SPAC_UP = 197,
		SPAC_DOWN = 181,
		CAM_W = 320,
		CAM_H = 240,
		N_MODELS = 3,
	};
  public:
	void setup();
	void shutdown();

	void mouseDown( MouseEvent event );	
	void update();
	void draw();
	void keyDown(KeyEvent event);

	void prepareSettings( Settings *settings );

private: //camera device
	Capture _capture;
	bool _capture_visible;
	int _device_id;
	Area _area_capture;

private:
	gl::Texture _tex_bg;
	gl::Texture _tex_default;

private: //AR
	shared_ptr<class ARTracker> _ar_tracker;
	std::mutex _mtx_ar;
	Matrix44d _mat_modelview;
	Matrix44d _mat_proj;
	ci::Vec2f  _pts_corner[4];
	int	_n_trackables;
	int _obj_id;

private: //book rendering
	float _cube_scale;
	bool _2dbook_visible;
	bool _3dbook_visible;
	gl::VboMesh _mesh_book;
	void updateData(const ci::Surface32f& image, gl::VboMesh& mesh, float max_height);
	ci::Vec3f _mesh_translate;

private: //InterfaceGl
	shared_ptr<params::InterfaceGl>	mParams;
	bool _using_sdar;
	ci::Vec3f _light_dir;
	ci::ColorA _cube_clr;

private:
	std::vector<std::string> mdl_files;
	std::string post_files[N_MODELS];
	ci::Surface32f _img_posters[N_MODELS];
	gl::Texture _tex_posters[N_MODELS];

private: //GUI
	gl::Texture _tex_iphone4;

private:
	float cameraXToScreenX(float cx);
	float cameraYToScreenY(float cy);
};