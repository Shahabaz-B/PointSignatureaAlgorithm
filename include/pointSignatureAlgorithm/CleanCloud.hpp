/*
This class can be used to modify the given input cloud.
it can do:
Upsampling and smoothing, Downsampling and smoothing.
*/

#pragma once
#include <iostream>
#include <pointCloudFilters/KdtreeFlann.hpp>
#include <pointCloudFilters/PointCloudFilters.hpp>
#include <pointCloudFilters/PointDefinition.hpp>
#include <string>

class cleanCloud {
private:
  PCF::pointCloud cloud, voxelCloud, mlsCloud, cleanSample, newCloud, Sample;
  pcl::search::KdTree<pcl::PointXYZ> tree;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float calcMean(PCF::pointCloud input);

public:
  /* Default constructor to initialize the variables and
  set K = 7 for optimal noise removal*/
  cleanCloud();

  // Removes the noise, does Up/Down sampling for  the given cloud
  PCF::pointCloud removeNoise(PCF::pointCloud inputCloud);
};
#endif