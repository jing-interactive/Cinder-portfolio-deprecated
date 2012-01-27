#include <boost/thread/thread.hpp>
#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>

#include "../../_common/Kinect/KinectDevice.h"
#include "../../_common/pcl/pcl.h"
#include "../../_common/pcl/registration.h"
#include "../../_common/pcl/feature_estimation.h"
#include "../../_common/pcl/filters.h"

#include "../../_common/pcl/kinect_support.h"

struct KinectFushionApp : public KinectDevice
{ 
	bool exit;

	KinectFushionApp()
		:KinectDevice(0)
	{
		exit = false;

		_cloud = PointCloudRgbPtr(new PointCloudRgb);
		_cloud_raw = PointCloudRgbPtr(new PointCloudRgb);
		_cloud_registered = PointCloudRgbPtr(new PointCloudRgb);
		_model = PointCloudRgbPtr(new PointCloudRgb);

		tform = Eigen::Matrix4f::Identity();
	}

	bool setup()
	{
		HRESULT hr = KinectDevice::setup(false, true, false);
		if (FAILED(hr))
			return false;

		//wait
		_evt.wait();
		boost::mutex::scoped_lock lock (_mtx);
		{
			_raw_viewer = NEW_XYZRGB_CLOUD_VIEWER_SMALL(_cloud_raw);
			_raw_viewer->_distance = 10;
			_raw_viewer->_far = 100000;

			_icp_viewer = NEW_XYZRGB_CLOUD_VIEWER_SMALL(_cloud_registered);
			_icp_viewer->_distance = 10;
			_icp_viewer->_far = 100000;
		}

		return true;
	}

	void compute_features(PointCloudRgbPtr cloud, PointCloudRgbPtr keypoints, LocalDescriptorsPtr local_descriptors)
	{
		// Estimate surface normals
		float surface_radius = 0.03;
		SurfaceNormalsPtr normals = estimateSurfaceNormals (cloud, surface_radius);

		// Detect keypoints
		float min_scale = 0.005f;
		int nr_octaves = 5;
		int nr_scales = 8;
		float min_contrast = 0.1f;
		keypoints = detectKeypoints (cloud, normals, min_scale, nr_octaves, nr_scales, min_contrast);
		PCL_INFO("Detected %zu keypoints\n", keypoints->size ());

		// Compute local descriptors
		double feature_radius = 0.06;
		assert (normals && keypoints);
		local_descriptors = computeLocalDescriptors (cloud, normals, keypoints, feature_radius);
		PCL_INFO("Computed local descriptors\n");

		// Compute global descriptor
// 		GlobalDescriptorsPtr global_descriptor;
// 		global_descriptor = computeGlobalDescriptor (cloud, normals);
// 		PCL_INFO ("Computed global descriptor\n");
	}

	Eigen::Matrix4f initial_alignment(PointCloudRgbPtr src_points, PointCloudRgbPtr dst_points)
	{ // Compute the intial alignment
		PointCloudRgbPtr keypoints[2];
		LocalDescriptorsPtr local_descriptors[2];
		compute_features(src_points, keypoints[0], local_descriptors[0]);
		compute_features(dst_points, keypoints[1], local_descriptors[1]);

		//param
		double min_sample_dist = 0.05, max_correspondence_dist=0.02, nr_iters=500;

		// Find the transform that roughly aligns the points
		Eigen::Matrix4f tform = computeInitialAlignment (keypoints[0], local_descriptors[0], keypoints[1], local_descriptors[1],
			min_sample_dist, max_correspondence_dist, nr_iters);		
		PCL_INFO("Computed initial alignment\n");

		return tform;
	}

	Eigen::Matrix4f icp_alignment(PointCloudRgbPtr src_points, PointCloudRgbPtr dst_points, Eigen::Matrix4f& tform)
	{// Refine the initial alignment
		float max_correspondence_distance = 0.05;
		float outlier_rejection_threshold = 0.05;
		float transformation_epsilon = 0;
		int max_iterations = 100;

		tform = refineAlignment (src_points, dst_points, tform, max_correspondence_distance,  
			outlier_rejection_threshold, transformation_epsilon, max_iterations);

		PCL_INFO("Refined alignment\n");

		if (0)
		{
			// Transform the source point to align them with the target points
			pcl::transformPointCloud (*src_points, *src_points, tform);

			 // Merge the two clouds
			(*src_points) += (*dst_points);
		}

		return tform;
	}

	void onDepthData(const cv::Mat& depth_u16)
	{
		if (exit)
			return;

		static int fps = 0;
		FPS_CALC (fps);
		 
		boost::mutex::scoped_lock lock (_mtx);
		{
			mat_to_cloud(depth_u16, *_cloud_raw);
			_cloud_registered = thresholdDepth(_cloud_raw, 1000,2000);
		}

		if (_raw_viewer.empty())
		{
			_evt.set();
		}
		else
		{
			_raw_viewer->update();
			_icp_viewer->update();

			_icp_viewer->_sum_mouse_dx = _raw_viewer->_sum_mouse_dx;
			_icp_viewer->_sum_mouse_dy = _raw_viewer->_sum_mouse_dy;
		}

		_cloud = _model;

	//	pcl::io::savePLYFile("kinect_raw.ply", *_cloud_raw);
	//	pcl::io::savePLYFile("kinect_reg.ply", *_cloud_registered);
	}
	PointCloudRgbPtr _cloud;
	PointCloudRgbPtr _cloud_raw;
	PointCloudRgbPtr _cloud_registered;
	PointCloudRgbPtr _model;

	Eigen::Matrix4f tform;

	boost::mutex _mtx;
	CSimpleEvent _evt;

	//OpenGL
	cv::Ptr<PointCloudViewer<pcl::PointXYZRGB> > _raw_viewer;
	cv::Ptr<PointCloudViewer<pcl::PointXYZRGB> > _icp_viewer;
};


int main(int argc, char** argv)
{
	KinectFushionApp fushion;
	if (!fushion.setup())
		return -1;

	while (true)
	{
		int key = cv::waitKey(1);
		if (key == VK_ESCAPE)
			break;
	}
	fushion.exit = true;

 	return 0;
}