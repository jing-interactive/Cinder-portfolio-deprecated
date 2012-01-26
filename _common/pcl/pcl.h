#ifndef _PCL_H_
#define _PCL_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

/* Define some custom types to make the rest of our code easier to read */

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZRGB PointRgb;
typedef pcl::PointCloud<PointT> PointCloudRrb;
typedef pcl::PointCloud<PointT>::Ptr PointCloudRgbPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudRgbConstPtr;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// Define "LocalDescriptors" to be a pcl::PointCloud of pcl::FPFHSignature33 points
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
typedef pcl::VFHSignature308 GlobalDescriptorT;
typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;

#endif