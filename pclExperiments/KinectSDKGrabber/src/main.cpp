#include <boost/thread/thread.hpp>
#include <pcl/common/time.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/registration.h> 

#include "../../_common/Kinect/KinectDevice.h"
#include "../../_common/pcl/pcl.h"
#include "../../_common/pcl/registration.h"
#include "../../_common/pcl/feature_estimation.h"

#include <windows.h>

template <typename type>
void mat2cloud(const cv::Mat& depth, PointCloud& cloud )
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

struct MyKinectDevice : public KinectDevice
{ 
	MyKinectDevice()
		:KinectDevice(0)
	{
		setup(false, true, false);

		cloud_ = PointCloudPtr(new PointCloud);
		cloud_raw_ = PointCloudPtr(new PointCloud);
		cloud_registered_ = PointCloudPtr(new PointCloud);
		model_ = PointCloudPtr(new PointCloud);

		tform = Eigen::Matrix4f::Identity();
	}

	void compute_features(PointCloudPtr cloud, PointCloudPtr keypoints, LocalDescriptorsPtr local_descriptors)
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

	Eigen::Matrix4f initial_alignment(PointCloudPtr src_points, PointCloudPtr dst_points)
	{ // Compute the intial alignment
		PointCloudPtr keypoints[2];
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

	Eigen::Matrix4f icp_alignment(PointCloudPtr src_points, PointCloudPtr dst_points, Eigen::Matrix4f& tform)
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

	void onDepthEvent(const cv::Mat& depth_u16)
	{
		static int fps = 0;
		FPS_CALC (fps);
		boost::mutex::scoped_lock lock (mtx_);

		mat2cloud<ushort>(depth_u16, *cloud_raw_);

		cloud_ = model_;

	//	pcl::io::savePLYFile("kinect.ply", *cloud_);
	}
	PointCloudPtr cloud_;
	PointCloudPtr cloud_raw_;
	PointCloudPtr cloud_registered_;
	PointCloudPtr model_;

	Eigen::Matrix4f tform;

	boost::mutex mtx_;
};

 
int main(int argc, char** argv)
{
	MyKinectDevice device;

	while (true)
	{
		//do some rendering
		::Sleep(30);
	}

 	return 0;
}