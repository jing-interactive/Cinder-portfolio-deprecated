#include "Fushion.h"
#include "../../_common/pcl/registration.h"
#include "../../_common/pcl/feature_estimation.h"

KinectFushionApp::KinectFushionApp()
	:KinectDevice(0)
{
	exit = false;

	_cloud = PointCloudRgbPtr(new PointCloudRgb);
	_cloud_raw = PointCloudRgbPtr(new PointCloudRgb);
	_cloud_registered = PointCloudRgbPtr(new PointCloudRgb);
	_model = PointCloudRgbPtr(new PointCloudRgb);

	tform = Eigen::Matrix4f::Identity();

	_thresh_near = 10;
	_thresh_far = 90;
}

void KinectFushionApp::compute_features(PointCloudRgbPtr cloud, PointCloudRgbPtr keypoints, LocalDescriptorsPtr local_descriptors)
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

Eigen::Matrix4f KinectFushionApp::initial_alignment(PointCloudRgbPtr src_points, PointCloudRgbPtr dst_points)
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

Eigen::Matrix4f KinectFushionApp::icp_alignment(PointCloudRgbPtr src_points, PointCloudRgbPtr dst_points, Eigen::Matrix4f& tform)
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
