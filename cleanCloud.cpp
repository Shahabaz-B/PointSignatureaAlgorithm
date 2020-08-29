#include "cleanCloud.hpp"
inline double round(double value) { return value < 0 ? -std::floor(0.5 - value) : std::floor(0.5 + value); }
cleanCloud::cleanCloud()
{
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	voxelCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	mlsCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cleanSample = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	newCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	tree =	pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>);
	Sample = pcl::PointCloud<pcl::PointXYZ>::Ptr  (new pcl::PointCloud<pcl::PointXYZ>);
}		

float cleanCloud::calcMean(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	float meanDist = 0.0, totalDistance = 0.0, variance = 0.0, standardDeviation = 0.0;
	std::vector<float> EuclidianDistance;
	int totalcount = input->points.size();
	pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
	kd_tree.setInputCloud(input);

	int _K = 4;
	/* K = 1, stands for the point itself.
	k = 2, indicates the first neighbour of the point */

	// index and distance of the neighbouring point
	std::vector<int> pointIdxNKNSearch(_K);
	std::vector<float> pointNKNSquaredDistance(_K);

	for (int i = 0; i < totalcount; i++)
	{
		if (kd_tree.nearestKSearch(input->points[i], _K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int j = 0; j < pointIdxNKNSearch.size(); j++)
			{
				// SQRT since it is squared distance
				EuclidianDistance.push_back(sqrt(pointNKNSquaredDistance[j]));
			}
		}
	}
	EuclidianDistance.shrink_to_fit();
	for (int i = 0; i < EuclidianDistance.size(); ++i)
	{
		// Accumulating all the distances
		totalDistance = totalDistance + EuclidianDistance[i];
	}

	// Dividing it by size of point cloud
	meanDist = totalDistance / EuclidianDistance.size();

	variance = 0.0;

	for (int i = 0; i < EuclidianDistance.size(); i++)
	{
		// Standard variance formula
		variance += (EuclidianDistance[i] - meanDist)*(EuclidianDistance[i] - meanDist);
	}

	variance /= EuclidianDistance.size();
	standardDeviation = 0.0;

	// Standard Deviation formula
	standardDeviation = sqrt(variance);

	return (meanDist + standardDeviation);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cleanCloud::removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
	std::cout << "Before filtering Cloud size: " << inputCloud->points.size() << std::endl;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (14);
	float leafSize = 0.8 * calcMean(inputCloud);
    
    Eigen::Vector4f min, max;
	pcl::getMinMax3D(*inputCloud, min, max);
	clock_t tStart = clock();
	
	for (int i = 0; i < inputCloud->points.size(); i++)
	{
		float pos_x, pos_y, pos_z;
		int abs_x, abs_y, abs_z;

		// calculate the relative world co-ordinate position from minimum point
		pos_x = min[0] - inputCloud->points[i].x;
		pos_y = min[1] - inputCloud->points[i].y;
		pos_z = min[2] - inputCloud->points[i].z;

        //calculate the number of grid required to reach the point from
		//minimum point in respective axis direction
		abs_x = std::abs(round(pos_x / leafSize));
		abs_y = std::abs(round(pos_y / leafSize));
		abs_z = std::abs(round(pos_z / leafSize)); 

		inputCloud->points[i].x = abs_x * leafSize + min [0];
		inputCloud->points[i].y = abs_y * leafSize + min [1];
		inputCloud->points[i].z = abs_z * leafSize + min [2];
	}
    
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (inputCloud);
	vg.setLeafSize (leafSize, leafSize, leafSize);
	vg.filter (*voxelCloud);
	
	printf("Voxel Cloud Size = %d\n", voxelCloud->points.size());

	std::vector<float> avgX(voxelCloud->points.size()), avgY(voxelCloud->points.size()), avgZ(voxelCloud->points.size()), RGBData(voxelCloud->points.size());
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	octree.setInputCloud (voxelCloud);
	octree.addPointsFromInputCloud ();
	int counter = 0;
	float radius = 3 * calcMean(voxelCloud);
	
	for (int i = 0; i < voxelCloud->points.size(); i++)
	{
		if ( octree.radiusSearch (voxelCloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			if(pointIdxRadiusSearch.size() > 5) // Optimal values 4 when k = 6 or 7
			{
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
				{
					// Accumulating the co-ordinates of the points who comes in the volume sphere
					avgX[counter] = avgX[counter] + voxelCloud->points[ pointIdxRadiusSearch[j] ].x;
					avgY[counter] = avgY[counter] + voxelCloud->points[ pointIdxRadiusSearch[j] ].y;
					avgZ[counter] = avgZ[counter] + voxelCloud->points[ pointIdxRadiusSearch[j] ].z;
					//RGBData[counter] = RGBData[counter] + cleanSample->points[pointIdxRadiusSearch[j]].rgb;
				}
				// Taking the average of the points to make new point cloud
				avgX[counter] = avgX[counter]/pointIdxRadiusSearch.size();
				avgY[counter] = avgY[counter]/pointIdxRadiusSearch.size();
				avgZ[counter] = avgZ[counter]/pointIdxRadiusSearch.size();
				//RGBData[counter] = RGBData[counter] / pointIdxRadiusSearch.size();
				counter++;
			}
		}
	}
	// Resize again since that the removed points will change the size.
	avgX.resize(counter);
	avgY.resize(counter);
	avgZ.resize(counter);
	//RGBData.resize(counter);

	// Savaing the reduced pointCloud in to new PCD file.
	newCloud->points.resize(avgX.size());
	newCloud->height = 1;

	for(int i = 0; i < avgX.size(); i++)
	{
		newCloud->points[i].x = avgX[i];
		newCloud->points[i].y = avgY[i];
		newCloud->points[i].z = avgZ[i];
		//newCloud->points[i].rgb = RGBData[i];
	}
	newCloud->width = newCloud->points.size();
	newCloud->points.shrink_to_fit();
	
	//=========================================
	
	//float _Delta = calcMean(newCloud);

	//Eigen::Vector4f min, max;

	//pcl::getMinMax3D(*newCloud, min, max);
	//clock_t tStart = clock();
	//printf("New Cloud Size = %d\n", newCloud->points.size());
	//for (int i = 0; i < newCloud->points.size(); i++)
	//{
	//	float pos_x, pos_y, pos_z;
	//	int abs_x, abs_y, abs_z;

	//	// calculate the relative world co-ordinate postion from minimum point
	//	pos_x = min[0] - newCloud->points[i].x;
	//	pos_y = min[1] - newCloud->points[i].y;
	//	pos_z = min[2] - newCloud->points[i].z;

	//	/* calculate the number of grid required to reach the point from
	//	minimum point in respective axis direction*/
	//	abs_x = std::abs(round(pos_x / _Delta));
	//	abs_y = std::abs(round(pos_y / _Delta));
	//	abs_z = std::abs(round(pos_z / _Delta)); 

	//	newCloud->points[i].x = abs_x * _Delta + min [0];
	//	newCloud->points[i].y = abs_y * _Delta + min [1];
	//	newCloud->points[i].z = abs_z * _Delta + min [2];
	//}

	leafSize = 0.5 * calcMean(newCloud);
	vg.setInputCloud (newCloud);
	vg.setLeafSize (leafSize, leafSize, leafSize);
	vg.filter (*Sample);
	printf("Sample Cloud Size = %d\n", Sample->points.size());
	return(Sample);
	//==========================================

	//printf("Before Mls\n");

	//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	//mls.setComputeNormals (true);
	//
	//// Set parameters
	//mls.setInputCloud (voxelCloud);
	//mls.setPolynomialFit (false);
	//mls.setSearchMethod (tree);
	//mls.setSearchRadius (2 * calcMean(voxelCloud));
	//mls.process (*mlsCloud);
	//printf("Mlscloud size = %d\n", mlsCloud->points.size());
	//if (mlsCloud->size() > 0)
	//{
	//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	//	sor.setInputCloud (mlsCloud);
	//	sor.setMeanK (50);
	//	sor.setStddevMulThresh (1.0);
	//	sor.filter (*cleanSample);

	//	vg.setInputCloud (cleanSample);
	//	vg.setLeafSize (leafSize, leafSize, leafSize);

	//	vg.filter (*Sample);

	//	return (Sample); 
	//}
	//else
	//	return (voxelCloud);
}