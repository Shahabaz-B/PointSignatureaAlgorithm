#include "meanDistance.hpp"

meanDistance::~meanDistance()
{
}

meanDistance::meanDistance()
{
}

void meanDistance::calcMean(pcl::PointCloud<PointT>::Ptr m_cloud, int _K)
{
	totalcount = m_cloud->points.size();
	kdtree.setInputCloud(m_cloud);
	meanDist = 0.0;
	totalDistance = 0.0;

	/* K = 1, stands for the point itself.
	k = 2, indicates the first neighbour of the point */

	// index and distance of the neighbouring point
	std::vector<int> pointIdxNKNSearch(_K);
	std::vector<float> pointNKNSquaredDistance(_K);

	for (int i = 0; i < totalcount; i++)
	{
		if (kdtree.nearestKSearch(m_cloud->points[i], _K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int j = 1; j < pointIdxNKNSearch.size(); j++)
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
	
	float variance = 0.0;
	for (int i = 0; i < EuclidianDistance.size(); i++)
	{
		// Standard variance formula
		variance += (EuclidianDistance[i] - meanDist)*(EuclidianDistance[i] - meanDist);
	}

	variance /= EuclidianDistance.size();
	standardDeviation = 0.0;

	// Standard Deviation formula
	standardDeviation = sqrt(variance);
	printf("mean Distance = %0.4f, standard Deviation = %0.4f\n" , meanDist,standardDeviation );
}


float meanDistance::getMean()
{
	return meanDist + standardDeviation;
}