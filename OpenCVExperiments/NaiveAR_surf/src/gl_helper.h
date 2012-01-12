#ifndef _GL_HELPER_
#define _GL_HELPER_

#include <gl/GL.h>
#include <opencv2/core/opengl_interop.hpp>
#include <string>

using namespace cv;
using std::string;

#pragma comment(lib,"opengl32.lib")

class I3DRenderer
{
public:
	I3DRenderer(const string& title, int w=800,int h = 600):_title(title)
	{ 
		namedWindow(title, WINDOW_OPENGL);
		resizeWindow(title, w, h);
		setOpenGlDrawCallback(title, I3DRenderer::static_draw_cb, this);
	}

	virtual void onMouseEvent(int event, int x, int y, int flags){}
	virtual void update(){}
	virtual void draw() = 0;

	static void static_mouse_cb(int event, int x, int y, int flags, void* userdata)
	{
		I3DRenderer* renderer = static_cast<I3DRenderer*>(userdata);
		renderer->onMouseEvent(event, x, y, flags);
	}

	static void static_draw_cb(void* userdata)
	{
		I3DRenderer* renderer = static_cast<I3DRenderer*>(userdata);
		renderer->draw();
	}
protected:
	GlCamera _camera;
	string _title;
};

#endif