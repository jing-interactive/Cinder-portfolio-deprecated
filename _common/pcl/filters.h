#ifndef FILTERS_H
#define FILTERS_H

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "pcl.h"

/* Use a PassThrough filter to remove points with depth values that are too large or too small */
void
thresholdDepth (const PointCloudRgbPtr & input, PointCloudRgbPtr& output, float min_depth, float max_depth)
{
  pcl::PassThrough<PointRgb> pass_through;
  pass_through.setInputCloud (input);
  pass_through.setFilterFieldName ("z");
  pass_through.setFilterLimits (min_depth, max_depth); 
  pass_through.filter (*output);
}

/* Use a VoxelGrid filter to reduce the number of points */
void
downsample (const PointCloudRgbPtr & input, PointCloudRgbPtr& output, float leaf_size)
{
  pcl::VoxelGrid<PointRgb> voxel_grid;
  voxel_grid.setInputCloud (input);
  voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  voxel_grid.filter (*output);
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
void
removeOutliers (const PointCloudRgbPtr & input, PointCloudRgbPtr& output, float radius, int min_neighbors)
{
  pcl::RadiusOutlierRemoval<PointRgb> radius_outlier_removal;
  radius_outlier_removal.setInputCloud (input);
  radius_outlier_removal.setRadiusSearch (radius);
  radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
  radius_outlier_removal.filter (*output);
}

/* Apply a series of filters (threshold depth, downsample, and remove outliers) */
void
applyFilters (const PointCloudRgbPtr & input, PointCloudRgbPtr& output, float min_depth, float max_depth, float leaf_size, float radius, 
              float min_neighbors)
{
  thresholdDepth (input, output, min_depth, max_depth);
  downsample (output, output, leaf_size);
  removeOutliers (output, output, radius, min_neighbors);
}

#endif
