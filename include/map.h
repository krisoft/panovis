#ifndef RVISION_MAP_H_
#define RVISION_MAP_H_

#include <Eigen/Dense>
#include "camera.h"

class Map {
  public:
    Eigen::VectorXd x;
    Eigen::MatrixXd P;

    Camera *cam;

    Map( const CameraParams params );
};


#endif  // RVISION_MAP_H_