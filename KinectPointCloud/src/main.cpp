//////////////////////////////////////////////////////////////////////////
//Kinect Point Cloud Sample
//author:vinjn
//http://weibo.com/vinjnmelanie
//displays 3d point clouds from Kinect
//

#include <opencv2/opencv.hpp>
#include "../../_common/Kinect/KinectDevice.h"
#include "../../_common/vOpenCV/OpenGL.h"

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
		int half_w = DEPTH_WIDTH/2;
		int half_h = DEPTH_HEIGHT/2;
		for (int y=0;y<DEPTH_HEIGHT;y++)
		{
			for (int x=0;x<DEPTH_WIDTH;x++)
			{
				Point3f& v = _vertices(y,x);
				v.x = x - half_w;
				v.y = y - half_h;
				v.z = -depth_u16.at<ushort>(y,x)*0.2;

				Point2f& uv = _uvCoord(y,x);
				uv.x = x/(float)DEPTH_WIDTH;
				uv.y = y/(float)DEPTH_HEIGHT;
			}
		}
	}

	void onRgbData(const cv::Mat& rgb)
	{
		_color = rgb;
	}

	void draw()
	{
		glPointSize(4);
		_camera.lookAt(Point3d(0,0,100), Point3d(0,0,0), Point3d(0,-1,0));
		_camera.setupProjectionMatrix();
		_camera.setupModelViewMatrix();
		glRotatef(_mouse_dx, 0,1.0f,0);
		glRotatef(-_mouse_dy, 1.0f,0,0);
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
	}

	stop = true;

	return 0;
}