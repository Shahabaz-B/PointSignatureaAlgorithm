#ifndef MEANDISTANCE_H_
#define MEANDISTANCE_H_
#include <vector>

typedef pcl::PointXYZ PointT;

class meanDistance {
private:
  float meanDist, totalDistance, standardDeviation;
  std::vector<float> EuclidianDistance;
  int totalcount;

  // Instance of KdTree class
  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::KdTreeFLANN<PointT> kdtree;

public:
  // Default constructor
  meanDistance();

  // Default Destructor
  ~meanDistance();

  /*meanDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud);

meanDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud, int K);*/

  void calcMean(pcl::PointCloud<PointT>::Ptr m_cloud, int K);

  // Returns the calculated mean value
  float getMean();
};
#endif