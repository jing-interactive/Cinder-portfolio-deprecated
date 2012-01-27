#ifndef FILTERS_H
#define FILTERS_H

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "pcl.h"

/* Use a PassThrough filter to remove points with depth values that are too large or too small */
PointCloudRgbPtr
thresholdDepth (const PointCloudRgbPtr & input, float min_depth, float max_depth)
{
  pcl::PassThrough<PointRgb> pass_through;
  pass_through.setInputCloud (input);
  pass_through.setFilterFieldName ("z");
  pass_through.setFilterLimits (min_depth, max_depth);
  PointCloudRgbPtr thresholded (new PointCloudRgb);
  pass_through.filter (*thresholded);

  return (thresholded);
}

/* Use a VoxelGrid filter to reduce the number of points */
PointCloudRgbPtr
downsample (const PointCloudRgbPtr & input, float leaf_size)
{
  pcl::VoxelGrid<PointRgb> voxel_grid;
  voxel_grid.setInputCloud (input);
  voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  PointCloudRgbPtr downsampled (new PointCloudRgb);
  voxel_grid.filter (*downsampled);

  return (downsampled);
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
PointCloudRgbPtr
removeOutliers (const PointCloudRgbPtr & input, float radius, int min_neighbors)
{
  pcl::RadiusOutlierRemoval<PointRgb> radius_outlier_removal;
  radius_outlier_removal.setInputCloud (input);
  radius_outlier_removal.setRadiusSearch (radius);
  radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
  PointCloudRgbPtr inliers (new PointCloudRgb);
  radius_outlier_removal.filter (*inliers);

  return (inliers);
}

/* Apply a series of filters (threshold depth, downsample, and remove outliers) */
PointCloudRgbPtr
applyFilters (const PointCloudRgbPtr & input, float min_depth, float max_depth, float leaf_size, float radius, 
              float min_neighbors)
{
  PointCloudRgbPtr filtered;
  filtered = thresholdDepth (input, min_depth, max_depth);
  filtered = downsample (filtered, leaf_size);
  filtered = removeOutliers (filtered, radius, min_neighbors);

  return (filtered);
}

#endif
