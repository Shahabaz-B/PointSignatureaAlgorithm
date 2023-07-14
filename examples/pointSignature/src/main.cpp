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
  auto meanVal = cc.getMean( 100, cloud );
  std::cout << meanVal << std::endl;
  cc.MakeSmoothPointCloud( 200, 0.8 * meanVal, cloud, smoothCloud );

  PSA::pointSignature psa( smoothCloud );
  psa.setRadius( 3 * meanVal );
  psa.setMargin( 0.8 * meanVal );
  psa.setShape( PSA::Shape::SPHERE );
  psa.checkShape();

  return 0;
}
