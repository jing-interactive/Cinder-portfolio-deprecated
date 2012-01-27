//////////////////////////////////////////////////////////////////////////
//Kinect Point Cloud Sample
//author:vinjn
//http://weibo.com/vinjnmelanie
//displays 3d point clouds from Kinect
//

#include <opencv2/opencv.hpp>
#include "../../../_common/Kinect/KinectDevice.h"
#include "../../../_common/vOpenCV/OpenGL.h"

using namespace cv;

bool stop = false;

const string windowName = "Kinect Point Cloud";

struct KinectDevice3D : public KinectDevice, public I3DRenderer
{
	KinectDevice3D():KinectDevice(0),I3DRenderer(windowName)
	{
		this->setup(true, true, false);

		_near = 10.0f;
		_far = 10000;

		_vertices.create(Size(DEPTH_WIDTH, DEPTH_HEIGHT));
		_uvCoord.create(Size(DEPTH_WIDTH, DEPTH_HEIGHT));
	}

	void onDepthData(const cv::Mat& depth_u16)
	{
		if (stop)
			return;
		int half_w = DEPTH_WIDTH/2;
		int half_h = DEPTH_HEIGHT/2;
		for (int y=0;y<DEPTH_HEIGHT;y++)
		{
			for (int x=0;x<DEPTH_WIDTH;x++)
			{
				ushort depthValue = depth_u16.at<ushort>(y,x);

				Point3f& p = _vertices(y,x);
				p.x = x - half_w;
				p.y = y - half_h;
				p.z = -depthValue*0.2;

				Point2f& uv = _uvCoord(y,x);
				uv = getUVFromDepthPixel(x,y,depthValue);
			}
		}
	}

	void onRgbData(const cv::Mat& rgb)
	{
		if (stop)
			return;
#ifdef INVISIBLE_MAN_MODE
		static bool first = true;
		if (first)
		{
			_color = rgb.clone();
			first = false;
		}
#else
		_color = rgb;
#endif
	}

	void draw()
	{
		if (stop)
			return;

		static float sum_mouse_dx = 0;
		static float sum_mouse_dy = 0;
		sum_mouse_dx += _mouse_dx*0.1;
		sum_mouse_dy += _mouse_dy*0.1;

		glPointSize(4);
		_camera.lookAt(Point3d(0,0,1), Point3d(0,0,0), Point3d(0,-1,0));
		_camera.setupProjectionMatrix();
		_camera.setupModelViewMatrix();
		glRotatef(sum_mouse_dx, 0,1.0f,0);
		glRotatef(sum_mouse_dy, 1.0f,0,0);
		_pointCloud.setVertexArray(_vertices);
		_pointCloud.setTexCoordArray(_uvCoord);
		if (!_color.empty())
		{
			_tex.copyFrom(_color);
			_tex.bind();
		}
		render(_pointCloud,RenderMode::POINTS);
	}

	Mat_<Point3f> _vertices;
	Mat_<Point2f> _uvCoord;
	Mat _color;
	GlArrays _pointCloud;
	GlTexture _tex;
};


int main(int argc, const char* argv[])
{
	KinectDevice3D renderer;

	while (true)
	{
		updateWindow(windowName);

		int key = waitKey(1);
		if (key == VK_ESCAPE)
			break;
	}

	stop = true;
	destroyAllWindows();

	return 0;
}