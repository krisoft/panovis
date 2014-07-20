#include "feature.h"
#include "map.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
using ceres::internal::AutoDiff;

#include <assert.h>

const int Feature::DIM = 2;

Feature::Feature( const int index_offset, Map *map, double u, double v ){
  this->index_offset = index_offset;
  this->map = map;

  double elevation;
  double azimuth;

  map->cam->convert_uv_to_ea( u, v, elevation, azimuth);

  // init azimuth, elevation
  map->state( index_offset + 0 ) = elevation;
  map->state( index_offset + 1) = azimuth;
  // init covariance
  for(int i=0; i<2; i++){
    map->covariance( index_offset + i, index_offset + i ) = 0.4; // FIGURE OUT SOMETHING BETTER
  }
}


void predict( double dt, Eigen::VectorXd &new_x, Eigen::MatrixXd &jacobi ){
}


void predict_noise( Eigen::MatrixXd &noise ){

}