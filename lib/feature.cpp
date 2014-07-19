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

  this->map->cam->convert_uv_to_ea( u, v, elevation, azimuth);

  // init quaternion
  this->map->x( this->index_offset + 0 ) = 1.0;
  this->map->x( this->index_offset + 1) = 0.0;
  // init covariance
  for(int i=0; i<2; i++){
    this->map->P( this->index_offset + i, this->index_offset + i ) = 0.4; // FIGURE OUT SOMETHING BETTER
  }
}
