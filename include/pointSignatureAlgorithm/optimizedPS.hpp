#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <pointCloudFilters/pointCloudFilters.hpp>
#include <string>
#include <time.h>
#include <vector>

#define PI 3.14159265

struct temp {
  float theta;
  float scaling;
};

/*
Used to sort the struct only on the basis of angle
which will sort the signed distance with it
*/

class pointSignature {

public:
  // Default Constructor
  pointSignature();

  void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

  // To ouput the cloud form the class
  pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();

  // to set the center of the searching ring, mostly used by "Clustering class"
  // void setCenter(int _center);

  void setCenter(int _center);

  int getCenter();

  // set the radius / margin of seraching ring/sphere manually
  void setRadius(float _radius);

  float getRadius();

  void setMargin(float _margin);

  float getMargin(void);

  // calculates the seraching ring for the given center point.
  void calculateSearchRing();

  pcl::PointCloud<pcl::PointXYZ>::Ptr getSearchRing();

  void calculateNormals();

  // Computes the siganture of the surface and saves it in 15 Degrees of
  // interval
  void computeSignature();

  // Sets flag for detected object viz. Plane, Sphere, Cylinder
  void SigCompare();

  void setShape(unsigned int shapeSelect);

  // checks the shape
  void checkShape();

  // Returns the size of detected object which is use in PS_RANSAC later
  int getShape();

private:
  // Used to store total number of points present in PC (PointCloud)
  float totalcount;

  // point of interest in PS
  int center_point, counter;

  pcl::PointXYZ center_pnt;

  // Radius and margin fro the seraching sphere
  float margin, radius;

  // Input clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudforAvg;

  int size;

  int totalpoints;
  float volSphereRad;

  pcl::PointCloud<pcl::PointXYZ>::Ptr iterationCloud, loopingCloud, filterCloud,
      outputCloud, planeCloud, filteredPlaneCloud;

  pcl::PointCloud<pcl::Normal>::Ptr Normalcloud;

  /*
  pcl::SACMODEL_PLANE    = 0
  pcl::SACMODEL_SPHERE   = 4
  pcl::SACMODEL_CYLINDER = 5
  pcl::SACMODEL_CONE     = 6
  */
  int shape;

  float mean, variance, standardDeviation, sum, upperLimit, lowerLimit,
      swap_var;

  // Flag for the counter True if inside, false if outside. Default = false
  bool isPlane, isSphere, isCylinder;

  inline void PointInPolygonTest(float *TXData, float *TYData,
                                 std::vector<temp> arrayinput, int nbK,
                                 int vertices, int &pointsInside);

  // Varable of type "Strct temp"
  std::vector<temp> normArray, structArray, descreteArray;

  // Refer Thesis documetation to understand meaning of these varaibles
  std::vector<float> vcix, vciy, vciz, vciLength, vciDotProd, vciLengthProd,
      cosTheta, sFactor, pNx, pNy, pNz, vnix, vniy, vniz, vniLength, cosThetaNi,
      acosThetaNi, finalPlaneX, finalPlaneY, value;

  // for saving new point cloud of searched ring and average searching ring
  pcl::PointCloud<pcl::PointXYZ>::Ptr SearchRingCloud;

  /* required in kdtree calculation to store index of points which
  satisfies the given condition and to store the squared distance of the same
  points*/
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  // saves the number of points present in the avgSearchiRingCloud
  size_t avgSearchRingCloudSize;
};
#endif