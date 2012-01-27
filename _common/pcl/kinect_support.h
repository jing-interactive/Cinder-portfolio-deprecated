//MS Kinect SDK -> OpenCV -> PCL ->OpenGL
#ifndef _KINECT_SUPPORT_H_
#define _KINECT_SUPPORT_H_

#include <opencv2/opencv.hpp>
#include "../vOpenCV/OpenGL.h"
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
#include "pcl.h"
#include <vector>

const float range_dx = 4400;//mm
const float range_dy = 3200;//mm

template <typename cvPointType, typename pclPointType>
cv::Point3_<cvPointType> to_cv(const pclPointType& point)
{
	return cv::Point3_<cvPointType>(point.x, point.y, point.z);
}

template <typename PointType>
void mat_to_cloud(const cv::Mat& depth_u16, pcl::PointCloud<PointType>& cloud )
{
	cloud.width = depth_u16.cols;
	cloud.height = depth_u16.rows;
	cloud.resize(cloud.width*cloud.height);
	cloud.is_dense = true;

	cv::Mat_<ushort> M(depth_u16);
	for(int j = 0; j < M.cols; j++)
		for(int i = 0; i < M.rows; i++)
	{
		cloud(j,i).x = range_dx/320.0f*i-range_dx*0.5f;
		cloud(j,i).y = range_dy/240.0f*j-range_dy*0.5f;
		cloud(j,i).z = M(i,j);
	}
}

template <typename PointType>
void cloud_to_points(const pcl::PointCloud<PointType>& cloud, std::vector<cv::Point3f>& points)
{
	points.clear();
	points.reserve(cloud.width * cloud.height);
	int n_points = cloud.size();

	for(int i=0;i<n_points;i++)
	{
		const PointType& p = cloud[i];
		points.push_back(cv::Point3f(p.x,p.y,p.z));
	}
}

// Ptr<PointCloudViewer> renderer = NEW_CLOUD_VIEWER(raw);
// 
// while (true)
// {
// 	renderer->update();
// 
// 	int key = waitKey(1);
// 	if (key == VK_ESCAPE)
// 		break;
// }
#define NEW_XYZ_CLOUD_VIEWER(cloud_ptr) new PointCloudViewer<pcl::PointXYZ>(cloud_ptr, #cloud_ptr)
#define NEW_XYZRGB_CLOUD_VIEWER(cloud_ptr) new PointCloudViewer<pcl::PointXYZRGB>(cloud_ptr, #cloud_ptr)
#define NEW_XYZ_CLOUD_VIEWER_SMALL(cloud_ptr) new PointCloudViewer<pcl::PointXYZ>(cloud_ptr, #cloud_ptr, 400, 300)
#define NEW_XYZRGB_CLOUD_VIEWER_SMALL(cloud_ptr) new PointCloudViewer<pcl::PointXYZRGB>(cloud_ptr, #cloud_ptr, 400, 300)

template <typename PointType>
struct PointCloudViewer : public cv::I3DRenderer
{
 	typedef typename pcl::PointCloud<PointType>::Ptr TheCloudPtr;

	PointCloudViewer(const TheCloudPtr& cloud, const string& name ="CloudViewer", int w = 640, int h = 480, float distance = 20.f)
		:cv::I3DRenderer(name,w,h),_distance(distance)
	{
		PointType min,max;
		pcl::getMinMax3D(*cloud, min, max);
		_min = to_cv<double>(min);
		_max = to_cv<double>(max);
		_center = (_min + _max)*0.5f;
		_box = _max - _min;

		setInputCloud(cloud);

		_sum_mouse_dx = _sum_mouse_dy = 0;
	}

	void onMouseEvent(int event, int x, int y, int flags)
	{
		if (event == cv::EVENT_RBUTTONUP)
		{//reset rotation to zero
			_sum_mouse_dx = _sum_mouse_dy = 0;
		}
	}

	void setInputCloud(const TheCloudPtr& cloud)
	{
		cloud_to_points(*cloud, _vertices);
	}

	void draw()
	{
		if (_vertices.empty())
			return;

		_sum_mouse_dx += _mouse_dx*0.1f;
		_sum_mouse_dy += _mouse_dy*0.1f;

		_camera.lookAt(cv::Point3d(0,0,abs(_box.y)*_distance), cv::Point3d(0,0,0), cv::Point3d(0,1,0));
		_camera.setScale(cv::Point3d(5,5,5));
		_camera.setupProjectionMatrix();
		_camera.setupModelViewMatrix();
		//glPointSize(4);
		glTranslated(-_center.x,-_center.y,-_center.z);
		glRotatef(-_sum_mouse_dx, 0,1.0f,0);
		glRotatef(-_sum_mouse_dy, 1.0f,0,0);
		_glCloud.setVertexArray(_vertices);
		cv::render(_glCloud, cv::RenderMode::POINTS);
	}

	std::vector<cv::Point3f> _vertices; 
	cv::GlArrays _glCloud;
	cv::Point3d _min,_max,_center,_box;
	float _distance;
	float _sum_mouse_dx;
	float _sum_mouse_dy;
};

#endif //_KINECT_SUPPORT_H_