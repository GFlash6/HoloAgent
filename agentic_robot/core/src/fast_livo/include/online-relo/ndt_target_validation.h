#pragma once

#include <cmath>
#include <cstddef>

#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/point_cloud.h>

template <typename PointT>
std::size_t countNdtTargetVoxels(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    float resolution) {
  if (!cloud || cloud->empty() || !std::isfinite(resolution) ||
      resolution <= 0.0f) {
    return 0;
  }

  pcl::VoxelGridCovariance<PointT> covariance_grid;
  covariance_grid.setLeafSize(resolution, resolution, resolution);
  covariance_grid.setInputCloud(cloud);
  pcl::PointCloud<PointT> centroids;
  covariance_grid.filter(centroids, false);
  return centroids.size();
}
