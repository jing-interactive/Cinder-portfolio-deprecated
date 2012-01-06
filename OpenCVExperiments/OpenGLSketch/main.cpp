#include <cstring>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/gpumat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/opengl_interop.hpp>

#if defined _DEBUG
#pragma comment(lib,"opencv_core232d.lib")
#pragma comment(lib,"opencv_imgproc232d.lib")
#pragma comment(lib,"opencv_highgui232d.lib")
#else
#pragma comment(lib,"opencv_core232.lib")
#pragma comment(lib,"opencv_imgproc232.lib")
#pragma comment(lib,"opencv_highgui232.lib")
#endif


using namespace std;
using namespace cv;
using namespace cv::gpu;

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
	int* dx = static_cast<int*>(userdata);
	int* dy = dx + 1;

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
		*dx = oldx - x;
		*dy = oldy - y;
	}
	else
	{
		*dx = 0;
		*dy = 0;
	}
}

inline int clamp(int val, int minVal, int maxVal)
{
	return max(min(val, maxVal), minVal);
}

Point3d rotate(Point3d v, double yaw, double pitch)
{
	Point3d t1;
	t1.x = v.x * cos(-yaw / 180.0 * CV_PI) - v.z * sin(-yaw / 180.0 * CV_PI);
	t1.y = v.y;
	t1.z = v.x * sin(-yaw / 180.0 * CV_PI) + v.z * cos(-yaw / 180.0 * CV_PI);

	Point3d t2;
	t2.x = t1.x;
	t2.y = t1.y * cos(pitch / 180.0 * CV_PI) - t1.z * sin(pitch / 180.0 * CV_PI);
	t2.z = t1.y * sin(pitch / 180.0 * CV_PI) + t1.z * cos(pitch / 180.0 * CV_PI);

	return t2;
}

int main(int argc, const char* argv[])
{
	namedWindow("OpenGL Sample", WINDOW_OPENGL);

	int fov = 0;
	createTrackbar("Fov", "OpenGL Sample", &fov, 100);

	int mouse[2] = {0, 0};
	setMouseCallback("OpenGL Sample", mouseCallback, mouse);

	double scale = 1.0;

	GlArrays pointCloud;
	GlCamera camera;
	camera.setScale(Point3d(scale, scale, scale));

	double yaw = 0.0;
	double pitch = 0.0;

	const Point3d dirVec(0.0, 0.0, -1.0);
	const Point3d upVec(0.0, 1.0, 0.0);
	const Point3d leftVec(-1.0, 0.0, 0.0);
	Point3d pos(0,0,10);
	Mat raw,gray;

	VideoCapture input;
	input.open(0);

	input>>raw;
	Mat_<Point3d> points(raw.size());

	while (true)
	{
		input>>raw;
		cvtColor(raw, gray, COLOR_BGR2GRAY);		
		for (int y=0;y<gray.rows;y++)
		{
			for (int x=0;x<gray.cols;x++)
			{
				Point3d& p = points(y,x);
				p.x = x;
				p.y = y;
				p.z = gray.at<uchar>(y,x);
			}
		}
 
		pointCloud.setVertexArray(points); 
		pointCloud.setColorArray(raw, false);

		double aspect = getWindowProperty("OpenGL Sample", WND_PROP_ASPECT_RATIO);

		const double posStep = 0.1;
		const double mouseStep = 0.001;
		const int mouseClamp = 300;

		camera.setPerspectiveProjection(30.0 + fov / 100.0 * 40.0, aspect, 0.1, 1000.0); 

		int mouse_dx = clamp(mouse[0], -mouseClamp, mouseClamp);
		int mouse_dy = clamp(mouse[1], -mouseClamp, mouseClamp);

		yaw += mouse_dx * mouseStep;
		pitch += mouse_dy * mouseStep;

		int key = waitKey(1);

		if (key == 27)
			break;
		key = tolower(key);
		if (key == 'w')
			pos += posStep * rotate(dirVec, yaw, pitch);
		else if (key == 's')
			pos -= posStep * rotate(dirVec, yaw, pitch);
		else if (key == 'a')
			pos += posStep * rotate(leftVec, yaw, pitch);
		else if (key == 'd')
			pos -= posStep * rotate(leftVec, yaw, pitch);
		else if (key == 'q')
			pos += posStep * rotate(upVec, yaw, pitch);
		else if (key == 'e')
			pos -= posStep * rotate(upVec, yaw, pitch);

		cout<<pos<<endl;

		camera.setCameraPos(pos, yaw, pitch, 0.0);

		pointCloudShow("OpenGL Sample", camera, pointCloud);
		//imshow("v",pointCloud.);
	}

	return 0;
}
