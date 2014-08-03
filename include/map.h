#ifndef RVISION_MAP_H_
#define RVISION_MAP_H_

#include <Eigen/Dense>
#include "camera.h"
#include "feature.h"
#include <vector>

class Map {
  public:
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;

    Camera *cam;
    std::vector<Feature> features;


    Map( const CameraParams params );

    void addFeature( double u, double v );

    void predict( double dt ); // propagate the state estimation with dt seconds
};


#endif  // RVISION_MAP_H_