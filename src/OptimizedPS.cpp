#include "pointCloudFilters/KdtreeFlann.hpp"
#include "pointCloudFilters/PointCloudFilters.hpp"
#include "pointCloudFilters/PointDefinition.hpp"
#include <cstddef>
#include <pointSignatureAlgorithm/OptimizedPS.hpp>

namespace PSA {

  pointSignature::pointSignature( PCF::pointCloud const& inputCloud )
    : inputCloud_( inputCloud )
  {
    avgSearchRingCloudSize_ = 0;
    normArray_.resize( 25 );
    descreteArray_.resize( 25 );
    counter_ = 0;
  }

  void pointSignature::setRadius( float const& radius ) { radius_ = radius; }

  void pointSignature::setMargin( float const& margin ) { margin_ = margin; }

  void pointSignature::calculateSearchRing( uint const& centerId )
  {
    int count = 0;
    std::vector< int > ptIndex;
    ptIndex.resize( 1000 );
    std::vector< int > pointIdxRadSearch;
    std::vector< float > pointRadSqDistance;
    PCF::KDtreeFlann kdtree;
    centerPnt_.x = inputCloud_[ centerId ].x; // center_pnt.x;
    centerPnt_.y = inputCloud_[ centerId ].y; // center_pnt.y;
    centerPnt_.z = inputCloud_[ centerId ].z; // center_pnt.z;

    for( size_t i = 0; i < inputCloud_.size(); i++ ) {
      double temp = 0.0, squareRoot = 0.0;
      // standard eqauation of sphere
      temp = ( ( ( inputCloud_[ i ].x - centerPnt_.x ) *
                 ( inputCloud_[ i ].x - centerPnt_.x ) ) +
               ( ( inputCloud_[ i ].y - centerPnt_.y ) *
                 ( inputCloud_[ i ].y - centerPnt_.y ) ) +
               ( ( inputCloud_[ i ].z - centerPnt_.z ) *
                 ( inputCloud_[ i ].z - centerPnt_.z ) ) );

      // ADDED sqrt function
      squareRoot = sqrt( temp );

      if( squareRoot - radius_ <= margin_ && squareRoot - radius_ >= 0.0 ) {
        ptIndex[ count ] = i;
        count++;
        /*to count the number of point which satisfy the condition,
        later used as size of new point cloud*/
      }
    }

    // writing all the data into a PCD file
    ptIndex.resize( count );
    ptIndex.shrink_to_fit();
    SearchRingCloud_.resize( count );

    for( size_t i = 0; i < SearchRingCloud_.size(); i++ ) {
      // adding values for new points from the old point cloud which satisfy the
      // condition
      SearchRingCloud_[ i ].x = inputCloud_[ ptIndex[ i ] ].x;
      SearchRingCloud_[ i ].y = inputCloud_[ ptIndex[ i ] ].y;
      SearchRingCloud_[ i ].z = inputCloud_[ ptIndex[ i ] ].z;
    }

    totalcount_ = SearchRingCloud_.size();

    if( totalcount_ > 0 ) {
      kdtree.setInputCloud( SearchRingCloud_ );

      std::vector< float > avgX( totalcount_ ), avgY( totalcount_ ),
        avgZ( totalcount_ );

      // calculation
      float volume_sphere_radius = ( ARC_LENGTH_CONST * ( radius_ ) );
      for( size_t i = 0; i < totalcount_; i++ ) {
        if( kdtree.radiusSearch( SearchRingCloud_[ i ], volume_sphere_radius,
                                 pointIdxRadiusSearch_,
                                 pointRadiusSquaredDistance_ ) > 0 ) {
          for( size_t j = 0; j < pointIdxRadiusSearch_.size(); j++ ) {
            // accumulating the co-ordinates of the points which comes in the
            // volume sphere
            avgX[ i ] =
              avgX[ i ] + SearchRingCloud_[ pointIdxRadiusSearch_[ j ] ].x;
            avgY[ i ] =
              avgY[ i ] + SearchRingCloud_[ pointIdxRadiusSearch_[ j ] ].y;
            avgZ[ i ] =
              avgZ[ i ] + SearchRingCloud_[ pointIdxRadiusSearch_[ j ] ].z;
          }
          // taking the average of the points to make new point cloud
          avgX[ i ] = avgX[ i ] / pointIdxRadiusSearch_.size();
          avgY[ i ] = avgY[ i ] / pointIdxRadiusSearch_.size();
          avgZ[ i ] = avgZ[ i ] / pointIdxRadiusSearch_.size();
        }
      }

      // saving the avgRing in to new PCD file
      for( size_t i = 0; i < totalcount_; i++ ) {
        SearchRingCloud_[ i ].x = avgX[ i ];
        SearchRingCloud_[ i ].y = avgY[ i ];
        SearchRingCloud_[ i ].z = avgZ[ i ];
      }
      avgSearchRingCloudSize_ = SearchRingCloud_.size();
    }
  }

  bool sort_by_angle( const angleAndScaling& a, const angleAndScaling& b )
  {
    return a.theta < b.theta;
  }

  void pointSignature::computeNormals()
  {

    PCF::Filter3D filter3d;
    filter3d.findNormals( inputCloud_, 30, normalCloud_ );
  }

  void pointSignature::computeSignature( uint const& centerId )
  {

    // Refer Thesis documetation to understand meaning of these varaibles
    PCF::pointCloud vci, pN, vni;
    std::vector< double > vciLength, vciDotProd, vciLengthProd, cosTheta,
      sFactor, vniLength, cosThetaNi, acosThetaNi, value, finalPlaneX,
      finalPlaneY;

    if( avgSearchRingCloudSize_ > 0 ) {
      // co-ordinate of vector center and Intersecting point
      vci.resize( avgSearchRingCloudSize_ );

      // length of the intersecting vector
      vciLength.resize( avgSearchRingCloudSize_ );

      // Dot product between Normal vector of center point and intersecting
      // vector
      vciDotProd.resize( avgSearchRingCloudSize_ );

      // length product between Normal vector of center point and intersecting
      // vector
      vciLengthProd.resize( avgSearchRingCloudSize_ );

      // cosTheta between Normal vector of center point and intersecting vector
      cosTheta.resize( avgSearchRingCloudSize_ );

      // scaling factor of the intersecting vector on the given Normal vector
      sFactor.resize( avgSearchRingCloudSize_ );

      // co-ordinate of the point which is scaled on noraml vector
      pN.resize( avgSearchRingCloudSize_ );

      // co-ordinate of the scaled normal vector
      vni.resize( avgSearchRingCloudSize_ );

      // Length of the vector between scaled point and the intersecting point
      vniLength.resize( avgSearchRingCloudSize_ );

      // cosTheta value for final plotting
      cosThetaNi.resize( avgSearchRingCloudSize_ );

      // Angle for final plotting of profile, angle
      // between each Vni vector w.r.t to reference
      // Vni vector
      acosThetaNi.resize( avgSearchRingCloudSize_ );

      // for extinding the angle value form 180 to
      // 360 we need determinant of a value...
      finalPlaneX.resize( avgSearchRingCloudSize_ );

      // which requires X and Y co-ordinate of the
      // plane vector, which will be stored in
      // these..
      finalPlaneY.resize( avgSearchRingCloudSize_ );

      // variables.
      value.resize( avgSearchRingCloudSize_ );

      structArray_.resize( avgSearchRingCloudSize_ );

      auto& center = inputCloud_[ centerId ];
      // loop for Vci Calculation
      for( size_t i = 0; i < avgSearchRingCloudSize_; i++ ) {
        vci[ i ].x = SearchRingCloud_[ i ].x - center.x;
        vci[ i ].y = SearchRingCloud_[ i ].y - center.y;
        vci[ i ].z = SearchRingCloud_[ i ].z - center.z;
        vciLength[ i ] =
          sqrt( ( vci[ i ].x * vci[ i ].x ) + ( vci[ i ].y * vci[ i ].y ) +
                ( vci[ i ].z * vci[ i ].z ) );
      }

      auto& normalOfCenter = normalCloud_[ centerId ];

      // Loop for vector calculation
      for( size_t i = 0; i < avgSearchRingCloudSize_; i++ ) {
        vciDotProd[ i ] = ( normalOfCenter.nx * vci[ i ].x ) +
                          ( normalOfCenter.ny * vci[ i ].y ) +
                          ( normalOfCenter.nz * vci[ i ].z );

        vciLengthProd[ i ] =
          ( sqrt( pow( normalOfCenter.nx, 2 ) + pow( normalOfCenter.ny, 2 ) +
                  pow( normalOfCenter.nz, 2 ) ) ) *
          vciLength[ i ];
        cosTheta[ i ] = vciDotProd[ i ] / vciLengthProd[ i ];

        sFactor[ i ] = cosTheta[ i ] * vciLength[ i ];

        pN[ i ].x = center.x + ( sFactor[ i ] * normalOfCenter.nx );
        pN[ i ].y = center.y + ( sFactor[ i ] * normalOfCenter.ny );
        pN[ i ].z = center.z + ( sFactor[ i ] * normalOfCenter.nz );

        vni[ i ].x     = SearchRingCloud_[ i ].x - pN[ i ].x;
        vni[ i ].y     = SearchRingCloud_[ i ].y - pN[ i ].y;
        vni[ i ].z     = SearchRingCloud_[ i ].z - pN[ i ].z;
        vniLength[ i ] = sqrt( pow( vni[ i ].x, 2 ) + pow( vni[ i ].y, 2 ) +
                               pow( vni[ i ].z, 2 ) );
      }

      float max    = sFactor[ 0 ];
      int maxindex = 0;

      //	find maxindex
      for( unsigned int j = 0; j < avgSearchRingCloudSize_; j++ ) {
        if( max < sFactor[ j ] ) {
          max      = sFactor[ j ];
          maxindex = j;
        }
      }

      //	for finding the cos angle between each Vni vector with respect
      // to
      // reference vector
      for( size_t i = 0; i < avgSearchRingCloudSize_; i++ ) {
        cosThetaNi[ i ] = ( ( vni[ i ].x * vni[ maxindex ].x ) +
                            ( vni[ i ].y * vni[ maxindex ].y ) +
                            ( vni[ i ].z * vni[ maxindex ].z ) ) /
                          ( ( vniLength[ i ] ) * ( vniLength[ maxindex ] ) );

        //	due to rounding error the value of cosThetaNi might go higer
        // than 1 or lower than -1 so we need this inequality for getting
        // correct value of angle
        if( ( ( ( cosThetaNi[ i ] - 1 <= 0.00001 ) &&
                ( cosThetaNi[ i ] - 1 >= 0.0 ) ) ||
              ( ( cosThetaNi[ i ] + 1 <= 0.00001 ) &&
                ( cosThetaNi[ i ] + 1 >= 0.0 ) ) ) ) {
          acosThetaNi[ i ] = 0;
        } else {
          acosThetaNi[ i ] = ( ( acos( cosThetaNi[ i ] ) ) * 180 ) / PI;
        }

        // calculating the X and Y co-ordinate of the scaled circle of
        // intersectin points
        finalPlaneX[ i ] = center.x + vni[ i ].x;
        finalPlaneY[ i ] = center.y + vni[ i ].y;
      }

      //	condition for checking if the point is on left or right
      for( size_t i = 0; i < avgSearchRingCloudSize_; i++ ) {
        // this calculates the determinant
        value[ i ] = ( ( finalPlaneX[ maxindex ] - center.x ) *
                       ( finalPlaneY[ i ] - center.y ) ) -
                     ( ( finalPlaneY[ maxindex ] - center.y ) *
                       ( finalPlaneX[ i ] - center.x ) );
      }

      //		assigning correct value for final Theta (extending the
      // anlge
      // form 0-180 to 0-360)
      for( size_t i = 0; i < avgSearchRingCloudSize_; i++ ) {
        if( value[ i ] < 0 ) {
          acosThetaNi[ i ] = 360 - acosThetaNi[ i ];
        } else {
          acosThetaNi[ i ] = acosThetaNi[ i ];
        }
      }

      for( size_t i = 0; i < avgSearchRingCloudSize_; i++ ) {
        structArray_[ i ].theta   = acosThetaNi[ i ];
        structArray_[ i ].scaling = sFactor[ i ];
      }

      std::sort( structArray_.begin(), structArray_.end(), sort_by_angle );

      for( int i = 0; i < 25; i++ ) {
        float min = 10000000;
        float dif;
        int index = -1;
        for( size_t j = 0; j < avgSearchRingCloudSize_; j++ ) {
          dif = abs( structArray_[ j ].theta - i * 15 );
          if( dif < min ) {
            min   = dif;
            index = j;
            if( dif < 0.5 )
              break;
          }
        }
        if( index != -1 ) {
          descreteArray_[ i ].theta   = structArray_[ index ].theta;
          descreteArray_[ i ].scaling = structArray_[ index ].scaling;
        }
      }
      for( int i = 0; i < 25; i++ ) {
        normArray_[ i ].scaling =
          ( ( descreteArray_[ i ].scaling ) / sFactor[ maxindex ] );
        normArray_[ i ].theta = descreteArray_[ i ].theta;
      }
    }
  }

  void pointSignature::PointInPolygonTest(
    float* TXData, float* TYData, std::vector< angleAndScaling > arrayinput,
    int nbK, int vertices, int& pointsInside )
  {
    pointsInside  = 0;
    bool isInside = false;
    for( int k = 0; k < nbK; k++ ) {
      for( int i = 0, j = vertices - 1; i < vertices; j = i++ ) {
        if( ( ( TYData[ i ] >= arrayinput[ k ].scaling ) !=
              ( TYData[ j ] >= arrayinput[ k ].scaling ) ) &&
            ( arrayinput[ k ].theta <=
              ( ( TXData[ j ] - TXData[ i ] ) /
                ( TYData[ j ] - TYData[ i ] ) ) *
                  ( arrayinput[ k ].scaling - TYData[ i ] ) +
                TXData[ i ] ) ) {
          isInside = !isInside;
        }
      }

      if( isInside == true ) {
        ++pointsInside;
        isInside = false;
      }
    }
  }

  void pointSignature::SigCompare()
  {
    isPlane_    = false;
    isSphere_   = false;
    isCylinder_ = false;
    int vertices;
    double sum = 0.0, mean = 0.0;
    for( int i = 0; i < 25; i++ ) {
      sum += normArray_[ i ].scaling;
    }
    mean = sum / 25;

    //=======================start of checking if clicked point is plane or
    // not=======================================

    if( ( mean > -0.2 ) && ( mean < 0.4 ) ) {
      float planeTXData[ 4 ] = { 0, 360, 360, 0 };
      float planeTYData[ 4 ] = { 0.35, 0.35, -0.35, -0.35 };
      vertices               = 4;
      int planarPoints       = 0;

      float numberOfPointsInside = 0.6 * 25;

      PointInPolygonTest( planeTXData, planeTYData, normArray_, 25, vertices,
                          planarPoints );

      if( planarPoints >= numberOfPointsInside ) {
        isPlane_ = true;
      }
    }

    else if( ( mean > 0.8 ) && ( mean < 1.0 ) ) {
      float sphereTXData[ 4 ] = { 0, 360, 360, 0 };
      float sphereTYData[ 4 ] = { 1.2, 1.2, 0.8, 0.8 };
      int spherePoints        = 0;
      vertices                = 4;

      float numberOfPointsInside = 0.7 * 25;

      PointInPolygonTest( sphereTXData, sphereTYData, normArray_, 25, vertices,
                          spherePoints );

      if( spherePoints >= numberOfPointsInside ) {
        isSphere_ = true;
      }

    }

    else if( ( mean > 0.4 ) && ( mean < 0.7 ) ) {
      float cylinderTXData[ 16 ] = { 0,   62,  96,  160, 190, 252, 268, 360,
                                     360, 268, 252, 190, 160, 96,  62,  0 };

      float cylinderTYData[ 16 ] = {
        1.25, 0.36646, 0.197709, 1.18, 1.2,  0.23,  0.186678,  1.25,
        0.75, -0.21,   -0.2,     0.7,  0.65, -0.25, -0.033531, 0.8 };

      vertices = 16;

      int cylinderPoints = 0;

      float numberOfPointsInside = 0.78 * 25;

      PointInPolygonTest( cylinderTXData, cylinderTYData, normArray_, 25,
                          vertices, cylinderPoints );

      if( cylinderPoints >= numberOfPointsInside ) {
        isCylinder_ = true;
      }
    }
  }

  void pointSignature::setShape( Shape const& shapeSelect )
  {
    shape_ = shapeSelect;
  }

  void pointSignature::checkShape()
  {
    size_t size  = 0;
    totalpoints_ = inputCloud_.size();
    std::vector< size_t > pointIdxRadSearch;
    std::vector< double > pointRadSqDistance;
    PCF::KDtreeFlann kdtree;
    kdtree.setInputCloud( inputCloud_ );
    this->computeNormals();
    for( int i = 0; i < totalpoints_; i++ ) {
      this->calculateSearchRing( i );
      if( avgSearchRingCloudSize_ > 10 ) {
        this->computeSignature( i );
        this->SigCompare();

        if( ( shape_ == Shape::CYLINDER && isCylinder_ == true ||
              shape_ == Shape::CONE && isCylinder_ == true ) &&
            ( kdtree.radiusSearch( inputCloud_[ i ], radius_, pointIdxRadSearch,
                                   pointRadSqDistance ) > 10 ) ) {
          for( size_t j = 0; j < pointIdxRadSearch.size(); j++ )
            loopingCloud_.push_back( inputCloud_[ pointIdxRadSearch[ j ] ] );

        }

        else if( shape_ == Shape::SPHERE && isSphere_ == true &&
                 ( kdtree.radiusSearch( inputCloud_[ i ], radius_,
                                        pointIdxRadSearch,
                                        pointRadSqDistance ) > 10 ) ) {
          for( size_t j = 0; j < pointIdxRadSearch.size(); j++ )
            loopingCloud_.push_back( inputCloud_[ pointIdxRadSearch[ j ] ] );

        }

        else if( shape_ == Shape::PLANE && isPlane_ == true &&
                 ( kdtree.radiusSearch( inputCloud_[ i ], radius_,
                                        pointIdxRadSearch,
                                        pointRadSqDistance ) > 10 ) ) {
          for( size_t j = 0; j < pointIdxRadSearch.size(); j++ )
            planeCloud_.push_back( iterationCloud_[ pointIdxRadSearch[ j ] ] );
        }
      }
    }

    PCF::Filter3D sor;
    if( planeCloud_.size() != 0 ) {
      planeCloud_.shrink_to_fit();

      sor.voxelFilter( planeCloud_, 0.08f * margin_, filteredPlaneCloud_ );
    }

    if( loopingCloud_.size() != 0 ) {

      sor.voxelFilter( loopingCloud_, 0.08f * margin_, outputCloud_ );
    }

    if( shape_ == Shape::CYLINDER || shape_ == Shape::SPHERE ) {
      size = outputCloud_.size();
      std::cout << "Cylinder Or Sphere, size = " << size << std::endl;
    }

    else if( ( outputCloud_.size() >= 0.60 * totalpoints_ ) &&
             ( shape_ == Shape::CONE ) ) {
      size = ( outputCloud_.size() + 0.35 * filteredPlaneCloud_.size() );
      std::cout << "Cone, size = " << size << std::endl;
    }

    else {
      size = 0;
    }
  }
} // namespace PSA
