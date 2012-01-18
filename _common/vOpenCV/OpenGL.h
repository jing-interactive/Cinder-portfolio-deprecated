#ifndef _GL_HELPER_
#define _GL_HELPER_

#include <gl/GL.h>
#include <opencv2/core/opengl_interop.hpp>
#include <string>

using std::string;

#pragma comment(lib,"opengl32.lib")

namespace cv
{
class I3DRenderer
{
public:
	I3DRenderer(const string& title, int w=800,int h = 600):_title(title)
	{ 
		namedWindow(title, WINDOW_OPENGL);
		resizeWindow(title, w, h);
		setOpenGlDrawCallback(title, I3DRenderer::static_draw_cb, this);
		setMouseCallback(title, I3DRenderer::static_mouse_cb, this);

		_near = 1.0f;
		_far = 1000.0f;
		_fov = 45.0f;
	}

	virtual void onMouseEvent(int event, int x, int y, int flags){}
	virtual void draw() = 0;

	static void static_mouse_cb(int event, int x, int y, int flags, void* userdata)
	{
		I3DRenderer* renderer = static_cast<I3DRenderer*>(userdata);
		renderer->_calculate_mouse_dxdy(event, x, y, flags);
		renderer->onMouseEvent(event, x, y, flags);
	}

	static void static_draw_cb(void* userdata)
	{
		I3DRenderer* renderer = static_cast<I3DRenderer*>(userdata);
		renderer->_update_camera_projection();
		renderer->draw();
	}
private:

	void _update_camera_projection()
	{
		double aspect = getWindowProperty(_title, WND_PROP_ASPECT_RATIO);
		_camera.setPerspectiveProjection(_fov, aspect, _near, _far);
	}

	void _calculate_mouse_dxdy(int event, int x, int y, int flags)
	{
		static int oldx = x;
		static int oldy = y;
		static bool moving = false;

		if (event == EVENT_LBUTTONDOWN)
		{
			oldx = x;
			oldy = y;
			moving = true;
		}
		else if (event == EVENT_LBUTTONUP)
		{
			moving = false;
		}

		if (moving)
		{
			_mouse_dx = oldx - x;
			_mouse_dy = oldy - y;
		}
		else
		{
			_mouse_dx = 0;
			_mouse_dy = 0;
		}
	}
protected:
	float _near;
	float _far;
	float _fov;
	GlCamera _camera;
	string _title;
	int _mouse_dx;
	int _mouse_dy;
};
}
#endif