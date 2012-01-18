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
		//TODO: _pointCloud  points
	}

	void onDepthData(const cv::Mat& depth_u16)
	{
		//TODO: _pointCloud.setVertexArray(points);
	}

	void onRgbData(const cv::Mat& rgb)
	{
		//TODO: pointCloud_.setColorArray(img, false);
	}

	void draw()
	{
		_camera.lookAt(Point3d(0,0,-10), Point3d(0,0,0), Point3d(0,1,0));
		_camera.setupProjectionMatrix();
		_camera.setupModelViewMatrix();
		render(_pointCloud);
	}

	Mat_<Point3d> points;
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