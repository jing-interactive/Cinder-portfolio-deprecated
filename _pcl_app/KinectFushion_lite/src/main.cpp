#include <boost/thread/thread.hpp>
#include <pcl/common/time.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/registration.h> 

#include "../../_common/Kinect/KinectDevice.h"
#include "../../_common/pcl/pcl.h"
#include "../../_common/pcl/registration.h"
#include "../../_common/pcl/feature_estimation.h"

#include "../../_common/pcl/kinect_support.h"

#include <windows.h>

struct KinectFushionApp : public KinectDevice
{ 
	KinectFushionApp()
		:KinectDevice(0)
	{
		setup(false, true, false);

		cloud_ = PointCloudRgbPtr(new PointCloudRgb);
		cloud_raw_ = PointCloudRgbPtr(new PointCloudRgb);
		cloud_registered_ = PointCloudRgbPtr(new PointCloudRgb);
		model_ = PointCloudRgbPtr(new PointCloudRgb);

		tform = Eigen::Matrix4f::Identity();
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
		static int fps = 0;
		FPS_CALC (fps);
		boost::mutex::scoped_lock lock (mtx_);

		mat_to_cloud(depth_u16, *cloud_raw_);

		if (_raw_viewer.empty())
		{
			_raw_viewer = NEW_XYZRGB_CLOUD_VIEWER(cloud_raw_);
		}
		else
		{
		//	_raw_viewer->update();
		}

		cloud_ = model_;

	//	pcl::io::savePLYFile("kinect.ply", *cloud_);
	}
	PointCloudRgbPtr cloud_;
	PointCloudRgbPtr cloud_raw_;
	PointCloudRgbPtr cloud_registered_;
	PointCloudRgbPtr model_;

	Eigen::Matrix4f tform;

	boost::mutex mtx_;

	//OpenGL
	cv::Ptr<PointCloudViewer<pcl::PointXYZRGB> > _raw_viewer;
	cv::Ptr<PointCloudViewer<pcl::PointXYZRGB> > _icp_viewer;
};

 
int main(int argc, char** argv)
{
	KinectFushionApp fushion;

	while (true)
	{
		if (!fushion._raw_viewer.empty())
			fushion._raw_viewer->update();

		int key = cv::waitKey(1);
		if (key == VK_ESCAPE)
			break;
	}

 	return 0;
}