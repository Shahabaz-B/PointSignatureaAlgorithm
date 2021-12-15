/*
This class can be used to modify the given input cloud.
it can do:
Upsampling and smoothing, Downsampling and smoothing.
*/

#pragma once
#include <iostream>
#include <pointCloudFilters/KdtreeFlann.hpp>
#include <pointCloudFilters/PointCloudFilters.hpp>
#include <pointCloudFilters/point.hpp>
#include <string>

typedef pointCloud pcl::PointCloud;
class cleanCloud {
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, voxelCloud, mlsCloud, cleanSample,
      newCloud, Sample;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float calcMean(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

public:
  /* Default constructor to initialize the variables and
  set K = 7 for optimal noise removal*/
  cleanCloud();

  // Removes the noise, does Up/Down sampling for  the given cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
};
#endif