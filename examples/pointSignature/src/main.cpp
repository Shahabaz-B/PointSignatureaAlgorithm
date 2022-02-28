#include <pointCloudFilters/FileHandler.hpp>
#include <pointCloudFilters/KdtreeFlann.hpp>
#include <pointCloudFilters/PointCloudFilters.hpp>
#include <pointSignatureAlgorithm/OptimizedPS.hpp>

inline double round( double value )
{
  return value < 0 ? -std::floor( 0.5 - value ) : std::floor( 0.5 + value );
}
// int main( int argc, const char** argv ) { return 0; }
int main( int argc, char** argv )
{
  PCF::FileHandler fh;
  PCF::pointCloud cloud = fh.loadXYZfile( argv[ 1 ] );
  PCF::pointCloud extractedCloud;
  PCF::pointCloud smoothCloud;

  // std::string file;
  // printf("Give the input cloud\n");
  // std::cin >> file;

  PCF::Filter3D cc;
  cc.MakeSmoothPointCloud( 30, 0.8 * cc.getMean( 4, cloud ), cloud,
                           smoothCloud );

  pcl::SampleConsensusModelPlane< pcl::PointXYZ > model_p(
    new pcl::SampleConsensusModelPlane< pcl::PointXYZ >( cloud ) );
  std::vector< int > inliers;
  pcl::RandomSampleConsensus< pcl::PointXYZ > ransac( model_p );
  ransac.setDistanceThreshold( 0.15 );
  ransac.computeModel();
  ransac.getInliers( inliers );

  pcl::PointCloud< PointT > cloud_plane( new pcl::PointCloud< PointT >() );

  // Write the planar inliers to disk
  pcl::copyPointCloud< PointT >( *cloud, inliers, *extractedCloud );

  ///\\\=====================================================================================

  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid( *extractedCloud, pcaCentroid );
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized( *extractedCloud, pcaCentroid, covariance );
  Eigen::SelfAdjointEigenSolver< Eigen::Matrix3f > eigen_solver(
    covariance, Eigen::ComputeEigenvectors );
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenVectorsPCA.col( 2 ) =
    eigenVectorsPCA.col( 0 ).cross( eigenVectorsPCA.col( 1 ) );

  // Transform the original cloud to the origin where the principal
  components
    // correspond to the axes.
    Eigen::Matrix4f projectionTransform( Eigen::Matrix4f::Identity() );
  projectionTransform.block< 3, 3 >( 0, 0 ) = eigenVectorsPCA.transpose();
  projectionTransform.block< 3, 1 >( 0, 3 ) =
    -1.f *
    ( projectionTransform.block< 3, 3 >( 0, 0 ) * pcaCentroid.head< 3 >() );

  PCF::pointCloud cloudPointsProjected( new PCF::pointCloud );
  pcl::transformPointCloud( *extractedCloud, *cloudPointsProjected,
                            projectionTransform );

  pcl::io::savePCDFileBinaryCompressed( "file.pcd", *cloudPointsProjected );

  ///\\\====================================================================================
  visualizer->addCoordinateSystem( 10 );
  visualizer->addPointCloud( cloudPointsProjected, "c1" );
  visualizer->spin();
  visualizer->removeAllPointClouds();
  //	cleanCloud cc;

  // cloud = cc.removeNoise(cloud);

  // file = file + "_clean.pcd";
  // pcl::io::savePCDFileBinaryCompressed(file,*cloud);

  float radius = 15 /*10  to 20*/, margin = 0.17, squareRoot;
  /*pcl::getMinMax3D(*cloud, min, max);

  radius = std::sqrt( std::pow( (cntr.x - min.x), 2) +
  std::pow( (cntr.y - min.y), 2) +
  std::pow( (cntr.z - min.z), 2));*/

  int count = 0;
  std::vector< int > ptIndex;
  ptIndex.resize( 3 * cloudPointsProjected->points.size() );
  std::vector< int > pointIdxRadSearch;
  std::vector< float > pointRadSqDistance;
  pcl::KdTreeFLANN< pcl::PointXYZ > kdtree;

  pcl::PointXYZ center;
  Eigen::Vector4f centeroid;

  pcl::compute3DCentroid( *cloudPointsProjected, centeroid );
  center.x = centeroid[ 0 ];
  center.y = centeroid[ 1 ];
  center.z = centeroid[ 2 ];

  // for (radius = 11; radius < 20; radius = radius + 0.1)
  //{
  for( size_t i = 0; i < cloudPointsProjected->points.size(); i++ ) {
    double temp = 0.0, squareRoot = 0.0;
    // standard eqauation of sphere
    temp = ( ( ( cloudPointsProjected->points[ i ].x - center.x ) *
               ( cloudPointsProjected->points[ i ].x - center.x ) ) +
             ( ( cloudPointsProjected->points[ i ].y - center.y ) *
               ( cloudPointsProjected->points[ i ].y - center.y ) ) +
             ( ( cloudPointsProjected->points[ i ].z - center.z ) *
               ( cloudPointsProjected->points[ i ].z - center.z ) ) );

    // ADDED sqrt function
    squareRoot = sqrt( temp );

    if( squareRoot - radius <= margin && squareRoot - radius >= 0.0 ) {
      ptIndex[ count ] = i;
      count++;
      /*to count the number of point which satisfy the condition,
      later used as size of new point cloud*/
    }
  }
  //}

  // writing all the data into a PCD file
  ptIndex.resize( count );
  ptIndex.shrink_to_fit();
  std::sort( ptIndex.begin(), ptIndex.end() );
  ptIndex.erase( std::unique( ptIndex.begin(), ptIndex.end() ), ptIndex.end() );

  PCF::pointCloud SearchRingCloud( new PCF::pointCloud );
  SearchRingCloud->width  = count;
  SearchRingCloud->height = 1;
  SearchRingCloud->points.resize( SearchRingCloud->width );

  for( size_t i = 0; i < SearchRingCloud->points.size(); i++ ) {
    // adding values for new points from the old point cloud which satisfy
    the
      // condition
      SearchRingCloud->points[ i ]
        .x = cloudPointsProjected->points[ ptIndex[ i ] ].x;
    SearchRingCloud->points[ i ].y =
      cloudPointsProjected->points[ ptIndex[ i ] ].y;
    SearchRingCloud->points[ i ].z =
      cloudPointsProjected->points[ ptIndex[ i ] ].z;
  }

  visualizer->addPointCloud( SearchRingCloud, "c2" );
  visualizer->spin();
  pcl::compute3DCentroid( *SearchRingCloud, centeroid );
  center.x = centeroid[ 0 ];
  center.y = centeroid[ 1 ];
  center.z = centeroid[ 2 ];

  std::vector< float > vcix, vciy, vciz, vciLength, vciDotProd, vciLengthProd,
    cosTheta, sFactor, pNx, pNy, pNz, vnix, vniy, vniz, vniLength, cosThetaNi,
    acosThetaNi, finalPlaneX, finalPlaneY, value;

  int avgSearchRingCloudSize = SearchRingCloud->points.size();

  vcix.resize( avgSearchRingCloudSize );
  vciy.resize( avgSearchRingCloudSize );
  vciz.resize( avgSearchRingCloudSize );
  vciLength.resize( avgSearchRingCloudSize );
  vciDotProd.resize( avgSearchRingCloudSize );
  vciLengthProd.resize( avgSearchRingCloudSize );
  cosTheta.resize( avgSearchRingCloudSize );
  sFactor.resize( avgSearchRingCloudSize );
  pNx.resize( avgSearchRingCloudSize );
  pNy.resize( avgSearchRingCloudSize );
  pNz.resize( avgSearchRingCloudSize );
  vnix.resize( avgSearchRingCloudSize );
  vniy.resize( avgSearchRingCloudSize );
  vniz.resize( avgSearchRingCloudSize );
  vniLength.resize( avgSearchRingCloudSize );
  cosThetaNi.resize( avgSearchRingCloudSize );
  acosThetaNi.resize( avgSearchRingCloudSize );
  finalPlaneX.resize( avgSearchRingCloudSize );
  finalPlaneY.resize( avgSearchRingCloudSize );
  value.resize( avgSearchRingCloudSize );

  //	pcl::PointXYZ  center = cloud->points[center_point];
  for( size_t i = 0; i < avgSearchRingCloudSize;
       i++ ) // loop for Vci Calculation
  {
    vnix[ i ] = SearchRingCloud->points[ i ].x - center.x;
    vniy[ i ] = SearchRingCloud->points[ i ].y - center.y;
    vniz[ i ] = SearchRingCloud->points[ i ].z - center.z;
    vniLength[ i ] =
      sqrt( ( vnix[ i ] * vnix[ i ] ) + ( vniy[ i ] * vniy[ i ] ) +
            ( vniz[ i ] * vniz[ i ] ) );
  }

  float max    = 0;
  int maxindex = 0;

  //	find maxindex
  for( unsigned int j = 0; j < avgSearchRingCloudSize; j++ ) {
    if( max < SearchRingCloud->points[ j ].z ) {
      max      = SearchRingCloud->points[ j ].z;
      maxindex = j;
    }
  }

  //	for finding the cos angle between each Vni vector with respect to
  // reference vector
  for( size_t i = 0; i < avgSearchRingCloudSize; i++ ) {
    cosThetaNi[ i ] =
      ( ( vnix[ i ] * vnix[ maxindex ] ) + ( vniy[ i ] * vniy[ maxindex ] ) +
        ( vniz[ i ] * vniz[ maxindex ] ) ) /
      ( ( vniLength[ i ] ) * ( vniLength[ maxindex ] ) );

    //	due to rounding error the value of cosThetaNi might go higer
    than 1 or
      // lower than -1 so we need this inequality for getting correct value of
      // angle
      if( ( ( ( cosThetaNi[ i ] - 1 <= 0.00001 ) &&
              ( cosThetaNi[ i ] - 1 >= 0.0 ) ) ||
            ( ( cosThetaNi[ i ] + 1 <= 0.00001 ) &&
              ( cosThetaNi[ i ] + 1 >= 0.0 ) ) ) )
    {
      acosThetaNi[ i ] = 0;
    }
    else { acosThetaNi[ i ] = ( ( acos( cosThetaNi[ i ] ) ) * 180 ) / PI; }

    // calculating the X and Y co-ordinate of the scaled circle of
    intersectin
      // points
      finalPlaneX[ i ] = center.x + vnix[ i ];
    finalPlaneY[ i ]   = center.y + vniy[ i ];
  }

  //	condition for checking if the point is on left or right
  for( size_t i = 0; i < avgSearchRingCloudSize; i++ ) {
    // this calculates the determinant
    value[ i ] = ( ( finalPlaneX[ maxindex ] - center.x ) *
                   ( finalPlaneY[ i ] - center.y ) ) -
                 ( ( finalPlaneY[ maxindex ] - center.y ) *
                   ( finalPlaneX[ i ] - center.x ) );
  }

  //		assigning correct value for final Theta (extending the anlge
  // form 0-180 to 0-360)
  for( size_t i = 0; i < avgSearchRingCloudSize; i++ ) {
    if( value[ i ] < 0 ) {
      acosThetaNi[ i ] = 360 - acosThetaNi[ i ];
    } else {
      acosThetaNi[ i ] = acosThetaNi[ i ];
    }
  }

  float mid = 0;
  std::vector< float > var;
  var.resize( SearchRingCloud->points.size() );

  for( int i = 0; i < SearchRingCloud->points.size(); i++ )
    mid = mid + SearchRingCloud->points[ i ].x;

  mid = mid / SearchRingCloud->points.size();

  for( int i = 0; i < SearchRingCloud->points.size(); i++ )
    var[ i ] = ( SearchRingCloud->points[ i ].x - mid ) *
               ( SearchRingCloud->points[ i ].x - mid );

  float stdDev = 0;

  for( int i = 0; i < var.size(); i++ ) {
    stdDev += var[ i ];
  }

  stdDev = std::sqrt( stdDev );
  printf( "%0.4f\n", stdDev );
  std::ofstream fileData;
  fileData.open( "data.xyz" );
  for( int i = 0; i < SearchRingCloud->points.size(); i++ ) {
    // if(var[i] > 0.0012)
    fileData << SearchRingCloud->points[ i ].x << "\t"
             << SearchRingCloud->points[ i ].y << "\t"
             << SearchRingCloud->points[ i ].z << "\t"
             << SearchRingCloud->points[ i ].x << "\t" << acosThetaNi[ i ]
             << "\t" << var[ i ] << "\n";
  }
  fileData.close();
  ///=================================================================

  // clock_t tStart = clock();
  // pointSignature ps;
  // ps.setCloud(cloud);
  // ps.setCenter(0);
  // ps.setMargin(md1.getMean());
  // ps.calculateNormals();
  //
  ///*for(int i = 0; i < (int)radius/(10*margin); i++ )
  //{*/
  //	ps.setRadius(5 * md1.getMean());
  //	ps.calculateSearchRing();
  //	ps.computeSignature();
  ////}

  /// ps.SigCompare();
  ////ps.setShape(pcl::SACMODEL_CYLINDER);
  ////ps.checkShape();
  ////ps.getShape();
  // printf("Time taken for voxel: %.4fs\n", (double)(clock() - tStart) /
  // CLOCKS_PER_SEC);
  return 0;
}