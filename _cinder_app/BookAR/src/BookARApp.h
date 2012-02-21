#include "cinder/app/AppBasic.h"
#include "cinder/Capture.h"
#include "cinder/Surface.h"
#include "cinder/Matrix44.h"
#include "cinder/Thread.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "Resources.h"

//#define USING_ARTK 

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
	void shutdown();

	void mouseDown( MouseEvent event );	
	void update();
	void draw();
	void keyDown(KeyEvent event);

	void prepareSettings( Settings *settings );

private:
	Capture _capture;
	bool _capture_visible;
	int _device_id;
private:

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
#ifdef USING_ARTK
	shared_ptr<ARToolKitPlus::TrackerSingleMarker> _artk_tracker;
#endif

private: //book rendering
	bool _2dbook_visible;
	bool _3dbook_visible;
	gl::VboMesh _mesh_book;
	void updateData(const ci::Surface32f& image, gl::VboMesh& mesh, float max_height);
	ci::Vec3f _mesh_translate;

private:
	bool _using_sdar;
	ci::Vec3f _light_dir;
	ci::ColorA _cube_clr;
	float _proj_near;
	float _proj_far;

private:
	std::string mdl_files[4];
	std::string post_files[4];
	ci::Surface32f _img_posters[4];

private:
	float cameraXToScreenX(float cx);
	float cameraYToScreenY(float cy);
};