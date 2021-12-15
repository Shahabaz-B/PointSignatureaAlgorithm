#include <pointSignatureAlgorithm/optimizedPS.hpp>

pointSignature::pointSignature() {
  cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  avgSearchRingCloudSize = 0;
  loopingCloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  planeCloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  filteredPlaneCloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  filterCloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  outputCloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  SearchRingCloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  Normalcloud =
      pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  normArray.resize(25);
  descreteArray.resize(25);
  counter = 0;
}

void pointSignature::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input) {
  cloud = input;
}

void pointSignature::setRadius(float _radius) { radius = _radius; }

void pointSignature::setCenter(int _center) { center_point = _center; }

// void pointSignature::setCenter(pcl::PointXYZ _center)
//{
//	center_pnt = _center;
// }

void pointSignature::setMargin(float _margin) { margin = _margin; }

pcl::PointCloud<pcl::PointXYZ>::Ptr pointSignature::getCloud() { return cloud; }

float pointSignature::getRadius() { return radius; }

int pointSignature::getCenter() { return center_point; }

float pointSignature::getMargin() { return margin; }

void pointSignature::calculateSearchRing() {
  int count = 0;
  std::vector<int> ptIndex;
  ptIndex.resize(1000);
  std::vector<int> pointIdxRadSearch;
  std::vector<float> pointRadSqDistance;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ center;
  center.x = cloud->points[center_point].x; // center_pnt.x;
  center.y = cloud->points[center_point].y; // center_pnt.y;
  center.z = cloud->points[center_point].z; // center_pnt.z;

  for (size_t i = 0; i < cloud->points.size(); i++) {
    double temp = 0.0, squareRoot = 0.0;
    // standard eqauation of sphere
    temp =
        (((cloud->points[i].x - center.x) * (cloud->points[i].x - center.x)) +
         ((cloud->points[i].y - center.y) * (cloud->points[i].y - center.y)) +
         ((cloud->points[i].z - center.z) * (cloud->points[i].z - center.z)));

    // ADDED sqrt function
    squareRoot = sqrt(temp);

    if (squareRoot - radius <= margin && squareRoot - radius >= 0.0) {
      ptIndex[count] = i;
      count++;
      /*to count the number of point which satisfy the condition,
      later used as size of new point cloud*/
    }
  }

  // writing all the data into a PCD file
  ptIndex.resize(count);
  ptIndex.shrink_to_fit();
  SearchRingCloud->width = count;
  SearchRingCloud->height = 1;
  SearchRingCloud->points.resize(SearchRingCloud->width);

  for (size_t i = 0; i < SearchRingCloud->points.size(); i++) {
    // adding values for new points from the old point cloud which satisfy the
    // condition
    SearchRingCloud->points[i].x = cloud->points[ptIndex[i]].x;
    SearchRingCloud->points[i].y = cloud->points[ptIndex[i]].y;
    SearchRingCloud->points[i].z = cloud->points[ptIndex[i]].z;
  }

  totalcount = SearchRingCloud->points.size();

  if (totalcount > 0) {
    kdtree.setInputCloud(SearchRingCloud);

    std::vector<float> avgX(totalcount), avgY(totalcount), avgZ(totalcount);

    // 0.2617993878 = (15*2*PI)/360), standard forumula for arc length
    // calculation
    float volume_sphere_radius = (0.2617993878 * (radius));
    for (size_t i = 0; i < totalcount; i++) {
      if (kdtree.radiusSearch(SearchRingCloud->points[i], volume_sphere_radius,
                              pointIdxRadiusSearch,
                              pointRadiusSquaredDistance) > 0) {
        for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
          // accumulating the co-ordinates of the points which comes in the
          // volume sphere
          avgX[i] =
              avgX[i] + SearchRingCloud->points[pointIdxRadiusSearch[j]].x;
          avgY[i] =
              avgY[i] + SearchRingCloud->points[pointIdxRadiusSearch[j]].y;
          avgZ[i] =
              avgZ[i] + SearchRingCloud->points[pointIdxRadiusSearch[j]].z;
        }
        // taking the average of the points to make new point cloud
        avgX[i] = avgX[i] / pointIdxRadiusSearch.size();
        avgY[i] = avgY[i] / pointIdxRadiusSearch.size();
        avgZ[i] = avgZ[i] / pointIdxRadiusSearch.size();
      }
    }

    // saving the avgRing in to new PCD file
    for (size_t i = 0; i < totalcount; i++) {
      SearchRingCloud->points[i].x = avgX[i];
      SearchRingCloud->points[i].y = avgY[i];
      SearchRingCloud->points[i].z = avgZ[i];
    }
    avgSearchRingCloudSize = SearchRingCloud->points.size();
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointSignature::getSearchRing() {
  return SearchRingCloud;
}

void pointSignature::calculateNormals() {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  // To save the Normals of point cloud

  ne.setSearchMethod(tree);
  ne.setKSearch(30);

  // computes cloud normal
  ne.compute(*Normalcloud);
}

bool sort_by_angle(const temp &a, const temp &b) { return a.theta < b.theta; }

void pointSignature::computeSignature() {
  if (avgSearchRingCloudSize > 0) {

    vcix.resize(avgSearchRingCloudSize); // X co-ordiante of vector center and
                                         // Intersecting point
    vciy.resize(avgSearchRingCloudSize); // Y co-ordiante of vector center and
                                         // Intersecting point
    vciz.resize(avgSearchRingCloudSize); // Z co-ordiante of vector center and
                                         // Intersecting point
    vciLength.resize(
        avgSearchRingCloudSize); // length of the intersecting vector
    vciDotProd.resize(
        avgSearchRingCloudSize); // Dot product between Normal vector of center
                                 // point and intersecting vector
    vciLengthProd.resize(
        avgSearchRingCloudSize); // length product between Normal vector of
                                 // center point and intersecting vector
    cosTheta.resize(
        avgSearchRingCloudSize); // cosTheta between Normal vector of center
                                 // point and intersecting vector
    sFactor.resize(
        avgSearchRingCloudSize); // scaling factor of the intersecting vector on
                                 // the given Normal vector
    pNx.resize(avgSearchRingCloudSize); // X co-ordinate of the point which is
                                        // scaled on noraml vector
    pNy.resize(avgSearchRingCloudSize); // Y co-ordinate of the point which is
                                        // scaled on noraml vector
    pNz.resize(avgSearchRingCloudSize); // Z co-ordinate of the point which is
                                        // scaled on noraml vector
    vnix.resize(
        avgSearchRingCloudSize); // X co-ordinate of the scaled noraml vector
    vniy.resize(
        avgSearchRingCloudSize); // Y co-ordinate of the scaled noraml vector
    vniz.resize(
        avgSearchRingCloudSize); // Z co-ordinate of the scaled noraml vector
    vniLength.resize(
        avgSearchRingCloudSize); // Length of the vector between scaled point
                                 // and the intersecting point
    cosThetaNi.resize(
        avgSearchRingCloudSize); // cosTheta value for final plotting
    acosThetaNi.resize(
        avgSearchRingCloudSize); // Angle for final plotting of profile, angle
                                 // between each Vni vector w.r.t to reference
                                 // Vni vector
    finalPlaneX.resize(
        avgSearchRingCloudSize); // for extinding the angle value form 180 to
                                 // 360 we need determinant of a value...
    finalPlaneY.resize(
        avgSearchRingCloudSize); // which requires X and Y co-ordinate of the
                                 // plane vector, which will be stored in
                                 // these..
    value.resize(avgSearchRingCloudSize); // variables.
    structArray.resize(avgSearchRingCloudSize);

    pcl::PointXYZ center = cloud->points[center_point];
    for (size_t i = 0; i < avgSearchRingCloudSize;
         i++) // loop for Vci Calculation
    {
      vcix[i] = SearchRingCloud->points[i].x - center.x;
      vciy[i] = SearchRingCloud->points[i].y - center.y;
      vciz[i] = SearchRingCloud->points[i].z - center.z;
      vciLength[i] =
          sqrt((vcix[i] * vcix[i]) + (vciy[i] * vciy[i]) + (vciz[i] * vciz[i]));
    }

    pcl::Normal normalOfCenter = Normalcloud->points[center_point];

    // Loop for vector calculation
    for (size_t i = 0; i < avgSearchRingCloudSize; i++) {
      vciDotProd[i] = (normalOfCenter.normal_x * vcix[i]) +
                      (normalOfCenter.normal_y * vciy[i]) +
                      (normalOfCenter.normal_z * vciz[i]);

      vciLengthProd[i] = (sqrt(pow(normalOfCenter.normal_x, 2) +
                               pow(normalOfCenter.normal_y, 2) +
                               pow(normalOfCenter.normal_z, 2))) *
                         vciLength[i];
      cosTheta[i] = vciDotProd[i] / vciLengthProd[i];

      sFactor[i] = cosTheta[i] * vciLength[i];

      pNx[i] = center.x + (sFactor[i] * normalOfCenter.normal_x);

      pNy[i] = center.y + (sFactor[i] * normalOfCenter.normal_y);

      pNz[i] = center.z + (sFactor[i] * normalOfCenter.normal_z);

      vnix[i] = SearchRingCloud->points[i].x - pNx[i];
      vniy[i] = SearchRingCloud->points[i].y - pNy[i];
      vniz[i] = SearchRingCloud->points[i].z - pNz[i];
      vniLength[i] = sqrt(pow(vnix[i], 2) + pow(vniy[i], 2) + pow(vniz[i], 2));
    }

    float max = sFactor[0];
    int maxindex = 0;

    //	find maxindex
    for (unsigned int j = 0; j < avgSearchRingCloudSize; j++) {
      if (max < sFactor[j]) {
        max = sFactor[j];
        maxindex = j;
      }
    }

    //	for finding the cos angle between each Vni vector with respect to
    //reference vector
    for (size_t i = 0; i < avgSearchRingCloudSize; i++) {
      cosThetaNi[i] = ((vnix[i] * vnix[maxindex]) + (vniy[i] * vniy[maxindex]) +
                       (vniz[i] * vniz[maxindex])) /
                      ((vniLength[i]) * (vniLength[maxindex]));

      //	due to rounding error the value of cosThetaNi might go higer
      //than 1 or lower than -1 so we need this inequality for getting correct
      //value of angle
      if ((((cosThetaNi[i] - 1 <= 0.00001) && (cosThetaNi[i] - 1 >= 0.0)) ||
           ((cosThetaNi[i] + 1 <= 0.00001) && (cosThetaNi[i] + 1 >= 0.0)))) {
        acosThetaNi[i] = 0;
      } else {
        acosThetaNi[i] = ((acos(cosThetaNi[i])) * 180) / PI;
      }

      // calculating the X and Y co-ordinate of the scaled circle of intersectin
      // points
      finalPlaneX[i] = center.x + vnix[i];
      finalPlaneY[i] = center.y + vniy[i];
    }

    //	condition for checking if the point is on left or right
    for (size_t i = 0; i < avgSearchRingCloudSize; i++) {
      // this calculates the determinant
      value[i] =
          ((finalPlaneX[maxindex] - center.x) * (finalPlaneY[i] - center.y)) -
          ((finalPlaneY[maxindex] - center.y) * (finalPlaneX[i] - center.x));
    }

    //		assigning correct value for final Theta (extending the anlge form
    //0-180 to 0-360)
    for (size_t i = 0; i < avgSearchRingCloudSize; i++) {
      if (value[i] < 0) {
        acosThetaNi[i] = 360 - acosThetaNi[i];
      } else {
        acosThetaNi[i] = acosThetaNi[i];
      }
    }

    for (size_t i = 0; i < avgSearchRingCloudSize; i++) {
      structArray[i].theta = acosThetaNi[i];
      structArray[i].scaling = sFactor[i];
    }

    std::sort(structArray.begin(), structArray.end(), sort_by_angle);

    for (int i = 0; i < 25; i++) {
      float min = 10000000;
      float dif;
      int index = -1;
      for (size_t j = 0; j < avgSearchRingCloudSize; j++) {
        dif = abs(structArray[j].theta - i * 15);
        if (dif < min) {
          min = dif;
          index = j;
          if (dif < 0.5)
            break;
        }
      }
      if (index != -1) {
        descreteArray[i].theta = structArray[index].theta;
        descreteArray[i].scaling = structArray[index].scaling;
      }
    }
    for (int i = 0; i < 25; i++) {
      normArray[i].scaling = ((descreteArray[i].scaling) / sFactor[maxindex]);
      normArray[i].theta = descreteArray[i].theta;
    }
  }
}
void pointSignature::PointInPolygonTest(float *TXData, float *TYData,
                                        std::vector<temp> arrayinput, int nbK,
                                        int vertices, int &pointsInside) {
  pointsInside = 0;
  bool isInside = false;
  for (int k = 0; k < nbK; k++) {
    for (int i = 0, j = vertices - 1; i < vertices; j = i++) {
      if (((TYData[i] >= arrayinput[k].scaling) !=
           (TYData[j] >= arrayinput[k].scaling)) &&
          (arrayinput[k].theta <=
           ((TXData[j] - TXData[i]) / (TYData[j] - TYData[i])) *
                   (arrayinput[k].scaling - TYData[i]) +
               TXData[i])) {
        isInside = !isInside;
      }
    }

    if (isInside == true) {
      ++pointsInside;
      isInside = false;
    }
  }
}

void pointSignature::SigCompare() {
  isPlane = false;
  isSphere = false;
  isCylinder = false;
  int vertices;
  sum = 0.0;
  mean = 0.0;
  for (int i = 0; i < 25; i++) {
    sum += normArray[i].scaling;
  }
  mean = sum / 25;

  //=======================start of checking if clicked point is plane or
  //not=======================================

  if ((mean > -0.2) && (mean < 0.4)) {
    float planeTXData[4] = {0, 360, 360, 0};
    float planeTYData[4] = {0.35, 0.35, -0.35, -0.35};
    vertices = 4;
    int planarPoints = 0;

    float numberOfPointsInside = 0.6 * 25;

    PointInPolygonTest(planeTXData, planeTYData, normArray, 25, vertices,
                       planarPoints);

    if (planarPoints >= numberOfPointsInside) {
      isPlane = true;
    }
  }

  else if ((mean > 0.8) && (mean < 1.0)) {
    float sphereTXData[4] = {0, 360, 360, 0};
    float sphereTYData[4] = {1.2, 1.2, 0.8, 0.8};
    int spherePoints = 0;
    vertices = 4;

    float numberOfPointsInside = 0.7 * 25;

    PointInPolygonTest(sphereTXData, sphereTYData, normArray, 25, vertices,
                       spherePoints);

    if (spherePoints >= numberOfPointsInside) {
      isSphere = true;
    }

  }

  else if ((mean > 0.4) && (mean < 0.7)) {
    float cylinderTXData[16] = {0,   62,  96,  160, 190, 252, 268, 360,
                                360, 268, 252, 190, 160, 96,  62,  0};

    float cylinderTYData[16] = {1.25,     0.36646, 0.197709,  1.18,  1.2,  0.23,
                                0.186678, 1.25,    0.75,      -0.21, -0.2, 0.7,
                                0.65,     -0.25,   -0.033531, 0.8};

    vertices = 16;

    int cylinderPoints = 0;

    float numberOfPointsInside = 0.78 * 25;

    PointInPolygonTest(cylinderTXData, cylinderTYData, normArray, 25, vertices,
                       cylinderPoints);

    if (cylinderPoints >= numberOfPointsInside) {
      isCylinder = true;
    }
  }
}

void pointSignature::setShape(unsigned int shapeSelect) { shape = shapeSelect; }

void pointSignature::checkShape() {
  size = 0;
  totalpoints = cloud->points.size();
  std::vector<int> pointIdxRadSearch;
  std::vector<float> pointRadSqDistance;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  calculateNormals();
  for (int i = 0; i < totalpoints; i++) {
    center_point = i;
    calculateSearchRing();
    if (avgSearchRingCloudSize > 10) {
      computeSignature();
      SigCompare();

      if ((shape == pcl::SACMODEL_CYLINDER && isCylinder == true ||
           shape == pcl::SACMODEL_CONE && isCylinder == true) &&
          (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadSearch,
                               pointRadSqDistance) > 10)) {
        for (size_t j = 0; j < pointIdxRadSearch.size(); j++)
          loopingCloud->push_back(cloud->points[pointIdxRadSearch[j]]);

      }

      else if (shape == pcl::SACMODEL_SPHERE && isSphere == true &&
               (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadSearch,
                                    pointRadSqDistance) > 10)) {
        for (size_t j = 0; j < pointIdxRadSearch.size(); j++)
          loopingCloud->push_back(cloud->points[pointIdxRadSearch[j]]);

      }

      else if (shape == pcl::SACMODEL_PLANE && isPlane == true &&
               (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadSearch,
                                    pointRadSqDistance) > 10)) {
        for (size_t j = 0; j < pointIdxRadSearch.size(); j++)
          planeCloud->push_back(iterationCloud->points[pointIdxRadSearch[j]]);
      }
    }
  }

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  if (planeCloud->points.size() != 0) {
    planeCloud->height = 1;
    planeCloud->points.resize(planeCloud->width);
    planeCloud->points.shrink_to_fit();

    sor.setInputCloud(planeCloud);
    sor.setLeafSize(0.08 * margin, 0.08 * margin, 0.08 * margin);
    sor.filter(*filteredPlaneCloud);
  }

  if (loopingCloud->points.size() != 0) {
    filterCloud->height = 1;
    filterCloud->is_dense = true;
    filterCloud->points.resize(loopingCloud->width * loopingCloud->height);
    pcl::copyPointCloud(*loopingCloud, *filterCloud);

    sor.setInputCloud(filterCloud);
    sor.setLeafSize(0.08 * margin, 0.08 * margin, 0.08 * margin);
    sor.filter(*outputCloud);
  }

  if (shape == pcl::SACMODEL_CYLINDER || shape == pcl::SACMODEL_SPHERE) {
    size = outputCloud->points.size();
    printf("Cylinder Or Sphere, size = %d\n", size);
  }

  else if ((outputCloud->points.size() >= 0.60 * totalpoints) &&
           (shape == pcl::SACMODEL_CONE)) {
    size =
        (outputCloud->points.size() + 0.35 * filteredPlaneCloud->points.size());
    printf("Cone, size = %d\n", size);
  }

  else {
    pcl::PointCloud<pcl::PointXYZ> falseCloud;
    size = falseCloud.points.size();
  }
}

int pointSignature::getShape() { return (size); }