#pragma once
#include "pointCloudFilters/PointDefinition.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <pointCloudFilters/PointCloudFilters.hpp>
#include <string>
#include <time.h>
#include <vector>

#define PI 3.14159265
// 0.2617993878 = (15*2*PI)/360), standard forumula for arc length
#define ARC_LENGTH_CONST 0.2617993878

namespace {
  struct angleAndScaling
  {
    float theta;
    float scaling;
  };

} // namespace

namespace PSA {
  enum class Shape
  {

    PLANE    = 0,
    SPHERE   = 1,
    CYLINDER = 2,
    CONE     = 3
  };

  class pointSignature
  {
  public:
    pointSignature& operator=( pointSignature const& ) = delete;
    pointSignature& operator=( pointSignature&& ) = delete;
    pointSignature( pointSignature const& )       = delete;
    pointSignature( pointSignature&& )            = delete;
    ~pointSignature()                             = default;

    pointSignature( PCF::pointCloud const& inputCloud );

    // set the radius / margin of seraching ring/sphere manually
    void setRadius( float const& radius );

    float getRadius();

    void setMargin( float const& margin );

    float getMargin( void );

    void setShape( Shape const& shapeSelect );

    // checks the shape
    void checkShape();

    // Returns the size of detected object which is use in PS_RANSAC later
    int getShape();

  private:
    // calculates the seraching ring for the given center point.
    void calculateSearchRing( uint const& centerId );

    // Computes the siganture of the surface and saves it in 15 Degrees of
    // interval
    void computeSignature( uint const& centerId );

    void computeNormals();
    // Sets flag for detected object viz. Plane, Sphere, Cylinder
    void SigCompare();

    // Used to store total number of points present in PC (PointCloud)
    float totalcount_;

    int counter_;

    PCF::Point centerPnt_;

    // Radius and margin fro the seraching sphere
    float margin_, radius_;

    // Input clouds
    PCF::pointCloud inputCloud_;
    PCF::pointCloud inputCloudforAvg_;

    int totalpoints_;
    float volSphereRad_;

    PCF::pointCloud iterationCloud_, loopingCloud_, filterCloud_, outputCloud_,
      planeCloud_, filteredPlaneCloud_;

    std::vector< PCF::normalsAndCurvature > normalCloud_;

    /*
    PLANE    = 0
    SPHERE   = 1
    CYLINDER = 2
    CONE     = 3
    */
    Shape shape_;

    double mean_, variance_, standardDeviation_, sum_, upperLimit_, lowerLimit_,
      swap_var_;

    bool isPlane_    = false;
    bool isSphere_   = false;
    bool isCylinder_ = false;
    inline void PointInPolygonTest( float* TXData, float* TYData,
                                    std::vector< angleAndScaling > arrayinput,
                                    int nbK, int vertices, int& pointsInside );

    // Varable of type "Strct angleAndScaling"
    std::vector< angleAndScaling > normArray_, structArray_, descreteArray_;

    // for saving new point cloud of searched ring and average searching ring
    PCF::pointCloud SearchRingCloud_;

    /* required in kdtree calculation to store index of points which
    satisfies the given condition and to store the squared distance of the same
    points*/
    std::vector< size_t > pointIdxRadiusSearch_;
    std::vector< double > pointRadiusSquaredDistance_;

    // saves the number of points present in the avgSearchiRingCloud
    size_t avgSearchRingCloudSize_;
  };
} // namespace PSA
