#include "camera.h"
#include "map.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
using ceres::internal::AutoDiff;

#include <assert.h>
#include <limits>

#include <iostream>
using namespace std;

const int Camera::DIM = 7;

Camera::Camera( const CameraParams params, const int index_offset, Map *map ){
  this->params = params;
  this->index_offset = index_offset;
  this->map = map;

  // init quaternion
  this->map->state( this->index_offset + 0 ) = 1.0;
  this->map->state( this->index_offset + 1) = 0.0;
  this->map->state( this->index_offset + 2) = 0.0;
  this->map->state( this->index_offset + 3) = 0.0;
  // init angular velocity vector
  this->map->state( this->index_offset + 4 ) = 0.0;
  this->map->state( this->index_offset + 5) = 0.0;
  this->map->state( this->index_offset + 6) = 0.0;

  // init quaternion covariance
  for(int i=0; i<4; i++){
    this->map->covariance( this->index_offset + i, this->index_offset + i ) = 0.4;
  }
  // init angular velocity covariance
  for(int i=4; i<7; i++){
    this->map->covariance( this->index_offset + i , this->index_offset + i ) = 1000.0;
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
    A q[4];
    q[0] = in[0];
    q[1] = in[1];
    q[2] = in[2];
    q[3] = in[3];

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
    QuaternionProduct( q, qomegadt, newq );
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


void Camera::predict( double dt, Eigen::VectorXd &new_x, Eigen::MatrixXd &jacobi, Eigen::MatrixXd &noise ){
  assert( new_x.rows()>=this->index_offset+Camera::DIM );
  assert( jacobi.cols()==jacobi.rows() );
  assert( jacobi.cols()>=this->index_offset+Camera::DIM );

  CameraPredictFunc fv(dt);
  double calc_jacobian[Camera::DIM*Camera::DIM];
  double predicted_state[Camera::DIM];
  double old_x[Camera::DIM];

  for(int i = 0; i<Camera::DIM; i++){
    old_x[i] = this->map->state( this->index_offset+i );
  }

  double *parameters[] = { old_x };
  double *jacobians[] = { calc_jacobian };

  AutoDiff<CameraPredictFunc, double, Camera::DIM>::Differentiate(
    fv, parameters, Camera::DIM, predicted_state, jacobians);

  for(int i = 0; i<Camera::DIM; i++){
    new_x( this->index_offset+i ) = predicted_state[i];
    for(int j = 0; j<Camera::DIM; j++){
      jacobi( this->index_offset+i, this->index_offset+j ) = calc_jacobian[ j+i*Camera::DIM ];
    }
  }

  for(int i=0; i<4; i++){
    noise( this->index_offset+i, this->index_offset+i ) = 0.001;
  }
  for(int i=4; i<7; i++){
    noise( this->index_offset+i, this->index_offset+i ) = 0.1;
  }
}

void Camera::convert_uv_to_ea( double u, double v, double &elevation, double &azimuth){
  cv::Mat src = cv::Mat::zeros(1, 1, CV_64FC2);
  src.at<cv::Vec2d>(0)[0] = u;
  src.at<cv::Vec2d>(0)[1] = v;
  cv::Mat dst = cvCreateMat(1, 1, CV_64FC2);
  undistortPoints(src, dst, this->params.intrinsic, this->params.dist);

  assert( dst.type()==CV_64FC2 );
  assert( dst.size().width==1 );
  assert( dst.size().height==1 );

  double point[3];
  point[0] = dst.at<cv::Vec2d>(0)[0];
  point[1] = dst.at<cv::Vec2d>(0)[1];
  point[2] = 1.0;
  double backrotated_point[3];
  double inverse_quaternion[4];
  for(int i=0; i<4; i++){
    inverse_quaternion[ i ] = this->map->state( this->index_offset+i );
  }
  inverse_quaternion[ 0 ] *= -1.0;

  ceres::QuaternionRotatePoint(inverse_quaternion, point, backrotated_point);

  azimuth = atan2( backrotated_point[0], backrotated_point[2] );
  if( sqrt(backrotated_point[2]*backrotated_point[2] + backrotated_point[0]*backrotated_point[0])==0 ){
    backrotated_point[2] += numeric_limits<double>::epsilon( );
    // I don't know if we really need this, could not trigger a division by zero even without it.
    // on the other hand without it I could not sleep.
  }
  elevation = atan( backrotated_point[1] / sqrt(backrotated_point[2]*backrotated_point[2] + backrotated_point[0]*backrotated_point[0]) );
}


void Camera::project_point( const Eigen::Vector3d point, bool &visible, Eigen::Vector2d &uv,  Eigen::Matrix< double, 2, 3 > &jacobi){

  double quaternion[4];
  for(int i=0; i<4; i++){
    quaternion[i] = map->state( this->index_offset + i );
  }

  Eigen::Matrix< double, 3, 3, Eigen::RowMajor > f0;
  ceres::QuaternionToScaledRotation( quaternion, f0.data() );

  Eigen::Vector3d rotated_point = f0*point;

  // Projection with opencv 5parameter distorsion model

  double x = rotated_point(0);
  double y = rotated_point(1);
  double z = rotated_point(2);

  if( z<0 ){
    visible = false;
    return;
  }


  double fx = params.intrinsic.at<double>( 0, 0);
  double fy = params.intrinsic.at<double>( 1, 1);
  double cx = params.intrinsic.at<double>( 0, 2);
  double cy = params.intrinsic.at<double>( 1, 2);

  double k1 = params.dist.at<double>( 0, 0);
  double k2 = params.dist.at<double>( 1, 0);
  double k3 = params.dist.at<double>( 2, 0);
  double p1 = params.dist.at<double>( 3, 0);
  double p2 = params.dist.at<double>( 4, 0);

  double x_ = x / z;
  double y_ = y / z;
  double r2 = x_*x_ + y_*y_;
  double d = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
  double x__ = x_*d + 2*p1*x_*y_ + p2*(r2 + 2*x_*x_);
  double y__ = y_*d + p1*(r2+2*y_*y_) + 2*p2*x_*y_;
  uv(0) = fx * x__ + cx;
  uv(1) = fy * y__ + cy;

  if( uv(0)<0 || uv(0)>params.pic_width || uv(1)<0 || uv(1)>params.pic_height ){
    visible = false;
    return;
  }
  visible = true;

  Eigen::Matrix< double, 3, 3 > f2f1;
  f2f1 << 1./z, 0.0, -1.*x/(z*z),
          0,  1./z,  -1*y/(z*z),
          2.*x_/z, 2*y_/z, (-2*x_*x - 2*y_*y)/(z*z);


  Eigen::Matrix< double, 2, 3 > f3;
  f3 << (d + 2*p1*y_ + 4*p2*x_), 2*p1*x_, (x_*k1 + 2*x_*k2*r2 + 3*x_*k3*r2*r2 + p2),
        (2*p2*y_), d + 4*p1*y_ + 2*p2*x_, (y_*k1 + 2*y_*k2*r2 + 3*y_*k3*r2*r2 + p1);

  Eigen::Matrix< double, 2, 2 > f4;
  f4 << fx, 0,
        0, fy;


  jacobi = f4*f3*f2f1*f0;
}