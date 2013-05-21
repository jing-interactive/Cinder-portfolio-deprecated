//////////////////////////////////////////////////////////////////////////
//Point Cloud Sample
//author:vinjn
//http://weibo.com/vinjnmelanie
//displays 3d point clouds from camera, the code is based on /samples/cpp/point_cloud.cpp
//

#include <opencv2/core/core.hpp>
#include <opencv2/core/opengl_interop.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <windows.h>
#include <gl/GL.h>
#pragma comment(lib,"opengl32.lib")

#if defined _DEBUG
#pragma comment(lib,"opencv_core243d.lib")
#pragma comment(lib,"opencv_imgproc243d.lib")
#pragma comment(lib,"opencv_highgui243d.lib")
#else
#pragma comment(lib,"opencv_core244.lib")
#pragma comment(lib,"opencv_imgproc244.lib")
#pragma comment(lib,"opencv_highgui244.lib")
#endif

using namespace cv;

bool stop = false;

class PointCloudRenderer
{
public:
	PointCloudRenderer(double scale);

	void onMouseEvent(int event, int x, int y, int flags);
	void draw();
	void update(Mat_<Point3d> points, Mat img, double aspect);

	int fov_;

	static void mouseCallback(int event, int x, int y, int flags, void* userdata)
	{
		if (stop)
			return;

		PointCloudRenderer* renderer = static_cast<PointCloudRenderer*>(userdata);
		renderer->onMouseEvent(event, x, y, flags);
	}

	static void openGlDrawCallback(void* userdata)
	{
		if (stop)
			return;

		PointCloudRenderer* renderer = static_cast<PointCloudRenderer*>(userdata);
		renderer->draw();
	}
private:
	int mouse_dx_;
	int mouse_dy_;

	Point3d pos_;

	GlCamera camera_;
	GlArrays pointCloud_;
};

int main(int argc, const char* argv[])
{
	double scale = 0.2;

	const string windowName = "OpenGL Point Cloud";

	VideoCapture input;
	input.open(0);

	Mat raw,gray,clr;
	input>>raw; 

	Mat_<Point3d> points(raw.size());

	namedWindow(windowName, WINDOW_OPENGL);
	resizeWindow(windowName, 800, 600);

	PointCloudRenderer renderer(scale);

	createTrackbar("Fov", windowName, &renderer.fov_, 100);
	setMouseCallback(windowName, PointCloudRenderer::mouseCallback, &renderer);
	setOpenGlDrawCallback(windowName, PointCloudRenderer::openGlDrawCallback, &renderer);

	while (true)
	{
		input>>raw;

		cvtColor(raw, clr, COLOR_BGR2RGB);
		flip(clr, clr,0);
		cvtColor(clr, gray, COLOR_RGB2GRAY);
		int half_w = gray.cols/2;
		int half_h = gray.rows/2;
		for (int y=0;y<gray.rows;y++)
		{
			for (int x=0;x<gray.cols;x++)
			{
				Point3d& p = points(y,x);
				p.x = x - half_w;
				p.y = y - half_h;
				p.z = 256-gray.at<uchar>(y,x);
			}
		}	

		double aspect = getWindowProperty(windowName, WND_PROP_ASPECT_RATIO);
		renderer.update(points, clr, aspect);

		updateWindow(windowName);

		int key = waitKey(1);
	}

	stop = true;

	return 0;
}

PointCloudRenderer::PointCloudRenderer(double scale)
{
	mouse_dx_ = 0;
	mouse_dy_ = 0;

	fov_ = 0;
	pos_ = Point3d(0,0,-100);
 
	camera_.setScale(Point3d(scale, scale, scale));
}

void PointCloudRenderer::onMouseEvent(int event, int x, int y, int flags)
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
		mouse_dx_ = oldx - x;
		mouse_dy_ = oldy - y;
	}
	else
	{
		mouse_dx_ = 0;
		mouse_dy_ = 0;
	}
}

void PointCloudRenderer::update(Mat_<Point3d> points, Mat img, double aspect)
{
	const Point3d dirVec(0.0, 0.0, -1.0);
	const Point3d upVec(0.0, 1.0, 0.0);
	const Point3d leftVec(-1.0, 0.0, 0.0);

	const double posStep = 1;

	const double mouseStep = 0.001;

	camera_.setPerspectiveProjection(30.0 + fov_ / 100.0 * 40.0, aspect, 0.1, 1000.0);
 	camera_.lookAt(pos_, Point3d(0,0,0), Point3d(0,1,0));

	pointCloud_.setVertexArray(points);
	pointCloud_.setColorArray(img, false);
}

void PointCloudRenderer::draw()
{
	camera_.setupProjectionMatrix();
	camera_.setupModelViewMatrix();
	
	glPushMatrix();
	glRotatef(mouse_dx_, 0,1.0f,0);
	glRotatef(-mouse_dy_, 1.0f,0,0);
	render(pointCloud_);
	glPopMatrix();

	render("OpenCV+OpenGL", GlFont::get("Courier New", 16), Scalar::all(255), Point2d(3.0, 0.0));
}