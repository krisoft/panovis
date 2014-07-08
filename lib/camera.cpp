#include "camera.h"
#include "map.h"

#include "ceres/rotation.h"
#include "ceres/ceres.h"

const int Camera::DIM = 7;

Camera::Camera( const CameraParams params, const int index_offset, Map *map ){
  this->params = params;
  this->index_offset = index_offset;
  this->map = map;

  // init quaternion
  this->map->x( this->index_offset + 0 ) = 1.0;
  this->map->x( this->index_offset + 1) = 0.0;
  this->map->x( this->index_offset + 2) = 0.0;
  this->map->x( this->index_offset + 3) = 0.0;
  // init angular velocity vector
  this->map->x( this->index_offset + 4 ) = 0.0;
  this->map->x( this->index_offset + 5) = 0.0;
  this->map->x( this->index_offset + 6) = 0.0;

  // init quaternion covariance
  for(int i=0; i<4; i++){
    this->map->P( this->index_offset + i, this->index_offset + i ) = 0.4;
  }
  // init angular velocity covariance
  for(int i=0; i<3; i++){
    this->map->P( this->index_offset + i + 4, this->index_offset + i + 4 ) = 1000.0;
  }
}

void Camera::update_index_offset( const int index_offset ){
  this->index_offset = index_offset;
}

struct CameraPredictFunc {
  CameraPredictFunc(double dt) : dt(dt) {}

  template <typename A>
  bool operator()(A const in[Camera::DIM], A result[Camera::DIM]) const {
    A Adt = A(dt);
    A qattitude[4];
    qattitude[0] = in[0];
    qattitude[1] = in[1];
    qattitude[2] = in[2];
    qattitude[3] = in[3];

    A q_norm = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
    q[0] /= q_norm;
    q[1] /= q_norm;
    q[2] /= q_norm;
    q[3] /= q_norm;

    A omegadt[3];
    omegadt[0] = in[4]*Adt;
    omegadt[1] = in[5]*Adt;
    omegadt[2] = in[6]*Adt;

    A qomegadt[4];
    A newq[4];
    AngleAxisToQuaternion( omegadt, qomegadt );
    QuaternionProduct( qattitude, qomegadt, newq );
    result[0] = newq[0];
    result[1] = newq[1];
    result[2] = newq[2];
    result[3] = newq[3];
    for(int i=4; i<13; i++){
      result[i] = in[i];
    }
    return true;
  }

  private:
    double dt;
};


void Camera::predict( double dt, Eigen::VectorXd &new_x, Eigen::MatrixXd &jacobi );
  assert( new_x.cols()>=this->index_offset+Camera::DIM );
  assert( jacobi.cols()==jacobi.rows() );
  assert( jacobi.cols()>=this->index_offset+Camera::DIM );

}


/*void Camera::project_point( const Eigen::Vector3d point, bool &visible, Eigen::Vector2d &uv){

  double quaternion[4];
  for(int i=0; i<4; i++){
    quaternion[i] = this->map->x( this->index_offset + i );
  }

  Eigen::Vector3d rotated_point;
  ceres::QuaternionRotatePoint(quaternion, point.data(), rotated_point.data());




}*/