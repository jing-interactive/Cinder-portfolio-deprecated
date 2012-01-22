#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include "../../_common/vOpenCV/OpenGL.h"

#ifdef _DEBUG
#pragma comment(lib, "pcl_io-gd.lib")
#pragma comment(lib, "pcl_common-gd.lib")
#pragma comment(lib,"opencv_core232d.lib")
#pragma comment(lib,"opencv_imgproc232d.lib")
#pragma comment(lib,"opencv_highgui232d.lib")
#else
#pragma comment(lib, "pcl_io.lib")
#pragma comment(lib, "pcl_common.lib")
#pragma comment(lib,"opencv_core232.lib")
#pragma comment(lib,"opencv_imgproc232.lib")
#pragma comment(lib,"opencv_highgui232.lib")
#endif

using namespace cv;
using namespace pcl;

template <typename type>
void mat2cloud(const cv::Mat& depth, PointCloud<PointXYZ>& cloud )
{
	cloud.width = depth.cols;
	cloud.height = depth.rows;
	cloud.resize(cloud.width*cloud.height);
	cloud.is_dense = true;

	cv::Mat_<type> M(depth);
	for(int j = 0; j < M.cols; j++)
		for(int i = 0; i < M.rows; i++)
	{
		cloud(j,i).x = 4400/320.0f*j;
		cloud(j,i).y = 3200/240.0f*i;
		cloud(j,i).z = M(i,j);
	}
}
 
void cloud2mat(const PointCloud<PointXYZ>& cloud, vector<Point3f>& points)
{
	points.reserve(cloud.width * cloud.height);
	int n_points = cloud.size();

	for(int i=0;i<n_points;i++)
	{
		const PointXYZ& p = cloud[i];
		points.push_back(Point3f(p.x,p.y,p.z));
	}
}

const string windowName = "pcd viewer";

struct PcdViewer : public I3DRenderer
{
	PcdViewer(PointCloud<PointXYZ>& cloud):I3DRenderer(windowName),_pclCloud(cloud){}
	void draw()
	{
		static float sum_mouse_dx = 0;
		static float sum_mouse_dy = 0;
		sum_mouse_dx += _mouse_dx*0.1;
		sum_mouse_dy += _mouse_dy*0.1;

		_camera.lookAt(Point3d(0,0,1), Point3d(0,0,0), Point3d(0,-1,0));
		_camera.setScale(Point3d(5,5,5));
		_camera.setupProjectionMatrix();
		_camera.setupModelViewMatrix();
		glPointSize(4);
		glRotatef(sum_mouse_dx, 0,1.0f,0);
		glRotatef(sum_mouse_dy, 1.0f,0,0);
		cloud2mat(_pclCloud, _vertices);
		_glCloud.setVertexArray(_vertices);
		render(_glCloud, RenderMode::POINTS);
	}
	vector<Point3f> _vertices; 
	GlArrays _glCloud;
	PointCloud<PointXYZ>& _pclCloud;
};
 
int main(int argc, char** argv)
{
	PCL_INFO("Syntax is %s [-i pcd file]\n", argv[0]);

	PointCloud<PointXYZ> cloud;
	std::string pcd_file;
	if (console::parse_argument (argc, argv, "-i", pcd_file) > 0)
	{
		PCL_INFO ("Reading %s\n",pcd_file.c_str());		
		int r = io::loadPCDFile(pcd_file, cloud);
		if (r <0)
		{
			PCL_ERROR("Please specify a valid pcd file\n");
			return -1;
		}
	}
	else
	{
		PCL_ERROR("Please specify a pcd file\n");
		return -1;
	}

	PcdViewer renderer(cloud);

	while (true)
	{
		updateWindow(windowName);

		int key = waitKey(1);
		if (key == VK_ESCAPE)
			break;
	}

	destroyAllWindows();

 	return 0;
}