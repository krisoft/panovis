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


void Feature::predict( double dt, Eigen::VectorXd &new_x, Eigen::MatrixXd &jacobi, Eigen::MatrixXd &noise ){
  for(int i=0; i<2; i++){
      new_x( index_offset + i ) = map->state( index_offset + i );
      jacobi( index_offset + i, index_offset + i ) = 1.0;
  }
}




bool search_area( int &top, int &left, int &width, int &height ){
  /*double x_ = problem.x / problem.z;
  double y_ = problem.y / problem.z;
  double r2 = x_*x_ + y_*y_;
  double d = 1 + problem.k1*r2 + problem.k2*r2*r2 + problem.k3*r2*r2*r2;
  double x__ = x_*d + 2*problem.p1*x_*y_ + problem.p2*(r2 + 2*x_*x_);
  double y__ = y_*d + problem.p1*(r2+2*y_*y_) + 2*problem.p2*x_*y_;
  solution.u = problem.fx * x__ + problem.cx;
  solution.v = problem.fy * y__ + problem.cy;

  Eigen::Matrix< double, 3, 3 > f2f1;
  f2f1 << 1./problem.z, 0.0, -1.*problem.x/(problem.z*problem.z),
          0,  1./problem.z,  -1*problem.y/(problem.z*problem.z),
          2.*x_/problem.z, 2*y_/problem.z, (-2*x_*problem.x - 2*y_*problem.y)/(problem.z*problem.z);


  Eigen::Matrix< double, 2, 3 > f3;
  f3 << (d + 2*problem.p1*y_ + 4*problem.p2*x_), 2*problem.p1*x_, (x_*problem.k1 + 2*x_*problem.k2*r2 + 3*x_*problem.k3*r2*r2 + problem.p2),
        (2*problem.p2*y_), d + 4*problem.p1*y_ + 2*problem.p2*x_, (y_*problem.k1 + 2*y_*problem.k2*r2 + 3*y_*problem.k3*r2*r2 + problem.p1);

  Eigen::Matrix< double, 2, 2 > f4;
  f4 << problem.fx, 0,
        0, problem.fy;*/
}