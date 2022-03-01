#include <pointCloudFilters/FileHandler.hpp>
#include <pointCloudFilters/KdtreeFlann.hpp>
#include <pointCloudFilters/PointCloudFilters.hpp>
#include <pointSignatureAlgorithm/OptimizedPS.hpp>

int main( int argc, char** argv )
{
  PCF::FileHandler fh;
  PCF::pointCloud cloud = fh.loadXYZfile( argv[ 1 ] );
  PCF::pointCloud extractedCloud;
  PCF::pointCloud smoothCloud;

  PCF::Filter3D cc;
  cc.MakeSmoothPointCloud( 30, 0.8 * cc.getMean( 4, cloud ), cloud,
                           smoothCloud );

  ///\\\=====================================================================================

  // Eigen::Vector4f pcaCentroid;
  // pcl::compute3DCentroid( *extractedCloud, pcaCentroid );
  // Eigen::Matrix3f covariance;
  // computeCovarianceMatrixNormalized( *extractedCloud, pcaCentroid, covariance
  // ); Eigen::SelfAdjointEigenSolver< Eigen::Matrix3f > eigen_solver(
  //   covariance, Eigen::ComputeEigenvectors );
  // Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  // eigenVectorsPCA.col( 2 ) =
  //   eigenVectorsPCA.col( 0 ).cross( eigenVectorsPCA.col( 1 ) );

  // // Transform the original cloud to the origin where the principal
  // components
  //   // correspond to the axes.
  //   Eigen::Matrix4f projectionTransform( Eigen::Matrix4f::Identity() );
  // projectionTransform.block< 3, 3 >( 0, 0 ) = eigenVectorsPCA.transpose();
  // projectionTransform.block< 3, 1 >( 0, 3 ) =
  //   -1.f *
  //   ( projectionTransform.block< 3, 3 >( 0, 0 ) * pcaCentroid.head< 3 >() );

  // PCF::pointCloud cloudPointsProjected( new PCF::pointCloud );
  // pcl::transformPointCloud( *extractedCloud, *cloudPointsProjected,
  //                           projectionTransform );

  // pcl::io::savePCDFileBinaryCompressed( "file.pcd", *cloudPointsProjected );

  ///\\\====================================================================================
  return 0;
}