/*
This class can be used to modify the given input cloud.
it can do:
Upsampling and smoothing, Downsampling and smoothing.
*/

#ifndef CLENACLOUD_H_
#define CLEANCLOUD_H_
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <iostream>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class cleanCloud
{
private: 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, voxelCloud, mlsCloud, cleanSample, newCloud, Sample;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
		
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	
	float calcMean(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
public:
	/* Default constructor to initialize the variables and 
	set K = 7 for optimal noise removal*/
	cleanCloud();

	// Removes the noise, does Up/Down sampling for  the given cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

};
#endif