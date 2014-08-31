#include "gtest/gtest.h"
#include "map.h"
#include "camera.h"
#include "random_utils.h"
#include "ceres/rotation.h"

#include <fstream>
#include <math.h>
#include <iostream>
using namespace std;

TEST(camera_test, predict)
{

  CameraParams params = initAndroidParams();
  Map map(params);

  std::ifstream inp("camera_predict.txt");
  ASSERT_TRUE( inp );

  int test_case_count;
  inp >> test_case_count;

  for(int test_id=0; test_id<test_case_count; test_id++){
    for(int j=0; j<7; j++){
      inp >> map.state( j );
    }
    double dt;
    inp >> dt;
    Eigen::VectorXd new_x(7);
    Eigen::MatrixXd jacobi = Eigen::MatrixXd::Zero( 7, 7);
    Eigen::MatrixXd noise = Eigen::MatrixXd::Zero( 7, 7);
    map.cam->predict( dt, new_x, jacobi, noise );
    for(int j=0; j<7; j++){
      double expected;
      inp >> expected;
      EXPECT_NEAR( expected, new_x(j) , 0.00001);
    }
    for(int i=0; i<7; i++){
      for(int j=0; j<7; j++){
        double expected;
        inp >> expected;
        EXPECT_NEAR( expected, jacobi(i,j), 0.00001);
      }
    }
    for(int i=0; i<7; i++){
      for(int j=0; j<7; j++){
        if( i==j && i<4 ){
          EXPECT_NEAR( 0.001, noise(i,j), 0.00001);
        }else if( i==j && i>=4 ){
          EXPECT_NEAR( 0.1, noise(i,j), 0.00001);
        }else{
          EXPECT_NEAR( 0.0, noise(i,j), 0.00001);
        }
      }
    }
  }
}

TEST(camera_test, convert_uv_to_ea_simple)
{
  CameraParams params = initSimpleParams();
  Map map(params);
  for(int i=0; i<10000; i++){
    double azimuth = fRand(-0.5*M_PI, 0.5*M_PI);
    double elevation = fRand(-0.5*M_PI, 0.5*M_PI);

    double point[3];
    point[0] = 100.0*sin(azimuth)*cos( elevation );
    point[1] = 100.0*sin( elevation );
    point[2] = 100.0*cos(azimuth)*cos( elevation );

    double q[4];
    double rotated_point[3];
    cv::Mat imagePoints;

    while(true){

      random_unit_quaternion( q );

      assert( abs(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]-1)<0.001 );
      
      ceres::QuaternionRotatePoint(q, point, rotated_point);

      if( rotated_point[2]<=0.0 ){
        continue;
      }

      cv::Mat objectPoints = cv::Mat::zeros(1, 3, CV_64F);
      objectPoints.at<double>(0,0) = rotated_point[0];
      objectPoints.at<double>(0,1) = rotated_point[1];
      objectPoints.at<double>(0,2) = rotated_point[2];
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
      projectPoints(objectPoints, rvec, tvec, params.intrinsic, params.dist, imagePoints);
      if( imagePoints.at<double>(0,0)>0 && imagePoints.at<double>(0,0)<params.pic_width && imagePoints.at<double>(0,1)>0 && imagePoints.at<double>(0,1)<params.pic_height ){
        break;
      }
    }

    assert( imagePoints.at<double>(0,0)>0 && imagePoints.at<double>(0,0)<params.pic_width && imagePoints.at<double>(0,1)>0 && imagePoints.at<double>(0,1)<params.pic_height );

    double u = imagePoints.at<double>(0,0);
    double v = imagePoints.at<double>(0,1);
    double a,e;
    for( int i=0; i<4; i++ ){
     map.state( i ) = q[i];
    }
    map.cam->convert_uv_to_ea( u, v, e, a );
    EXPECT_NEAR( azimuth, a, 0.001);
    EXPECT_NEAR( elevation, e, 0.001);
  }
}


TEST(camera_test, convert_uv_to_ea_android)
{
  CameraParams params = initAndroidParams();
  Map map(params);
  for(int i=0; i<10000; i++){
    double azimuth = fRand(-0.5*M_PI, 0.5*M_PI);
    double elevation = fRand(-0.5*M_PI, 0.5*M_PI);

    double point[3];
    point[0] = 100.0*sin(azimuth)*cos( elevation );
    point[1] = 100.0*sin( elevation );
    point[2] = 100.0*cos(azimuth)*cos( elevation );

    double q[4];
    double rotated_point[3];
    cv::Mat imagePoints;

    while(true){

      random_unit_quaternion( q );

      assert( abs(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]-1)<0.001 );
      

      ceres::QuaternionRotatePoint(q, point, rotated_point);

      if( rotated_point[2]<=0.0 ){
        continue;
      }

      cv::Mat objectPoints = cv::Mat::zeros(1, 3, CV_64F);
      objectPoints.at<double>(0,0) = rotated_point[0];
      objectPoints.at<double>(0,1) = rotated_point[1];
      objectPoints.at<double>(0,2) = rotated_point[2];
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
      projectPoints(objectPoints, rvec, tvec, params.intrinsic, params.dist, imagePoints);
      if( imagePoints.at<double>(0,0)>0 && imagePoints.at<double>(0,0)<params.pic_width && imagePoints.at<double>(0,1)>0 && imagePoints.at<double>(0,1)<params.pic_height ){
        break;
      }
    }

    assert( imagePoints.at<double>(0,0)>0 && imagePoints.at<double>(0,0)<params.pic_width && imagePoints.at<double>(0,1)>0 && imagePoints.at<double>(0,1)<params.pic_height );

    double u = imagePoints.at<double>(0,0);
    double v = imagePoints.at<double>(0,1);
    double a,e;
    for( int i=0; i<4; i++ ){
     map.state( i ) = q[i];
    }
    map.cam->convert_uv_to_ea( u, v, e, a );
    EXPECT_NEAR( azimuth, a, 0.001);
    EXPECT_NEAR( elevation, e, 0.001);
  }
}

TEST(camera_test, project_point_nodistorsion)
{
  CameraParams params = initSimpleParams();
  Map map(params);

  Eigen::Vector3d point;
  Eigen::Vector2d uv;
  Eigen::Matrix< double, 2, 3 > jacobi;
  bool visible;


  point << 0, 0, 100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_TRUE( visible );
  EXPECT_NEAR( 255, uv(0), 0.001);
  EXPECT_NEAR( 255, uv(1), 0.001);

  EXPECT_NEAR( 1, jacobi(0,0), 0.001);
  EXPECT_NEAR( 0, jacobi(0,1), 0.001);
  EXPECT_NEAR( 0, jacobi(0,2), 0.001);

  EXPECT_NEAR( 0, jacobi(1,0), 0.001);
  EXPECT_NEAR( 1, jacobi(1,1), 0.001);
  EXPECT_NEAR( 0, jacobi(1,2), 0.001);


  point << 0, 0, -100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_FALSE( visible );


  point << 10, 0, 100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_TRUE( visible );
  EXPECT_NEAR( 265, uv(0), 0.001);
  EXPECT_NEAR( 255, uv(1), 0.001);

  EXPECT_NEAR( 1, jacobi(0,0), 0.001);
  EXPECT_NEAR( 0, jacobi(0,1), 0.001);
  EXPECT_NEAR( -0.1, jacobi(0,2), 0.001);
  
  EXPECT_NEAR( 0, jacobi(1,0), 0.001);
  EXPECT_NEAR( 1, jacobi(1,1), 0.001);
  EXPECT_NEAR( 0, jacobi(1,2), 0.001);

  point << -10, 30, 100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_TRUE( visible );
  EXPECT_NEAR( 245, uv(0), 0.001);
  EXPECT_NEAR( 285, uv(1), 0.001);

  EXPECT_NEAR( 1, jacobi(0,0), 0.001);
  EXPECT_NEAR( 0, jacobi(0,1), 0.001);
  EXPECT_NEAR( 0.1, jacobi(0,2), 0.001);
  
  EXPECT_NEAR( 0, jacobi(1,0), 0.001);
  EXPECT_NEAR( 1, jacobi(1,1), 0.001);
  EXPECT_NEAR( -0.3, jacobi(1,2), 0.001);

  point << -260, 0, 100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_FALSE( visible );

  point << 260, 0, 100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_FALSE( visible );

  point << 0, -260, 100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_FALSE( visible );

  point << 0, 260, 100;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_FALSE( visible );

  point << 10, 28, 200;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_TRUE( visible );
  EXPECT_NEAR( 260, uv(0), 0.001);
  EXPECT_NEAR( 269, uv(1), 0.001);

  EXPECT_NEAR( 0.5, jacobi(0,0), 0.001);
  EXPECT_NEAR( 0, jacobi(0,1), 0.001);
  EXPECT_NEAR( -0.025, jacobi(0,2), 0.001);
  
  EXPECT_NEAR( 0, jacobi(1,0), 0.001);
  EXPECT_NEAR( 0.5, jacobi(1,1), 0.001);
  EXPECT_NEAR( -0.07, jacobi(1,2), 0.001);

  double angle = -1.0*M_PI/4.0; // 45

  map.state( 0 ) = cos( angle*0.5 );
  map.state( 1 ) = 0.0;
  map.state( 2 ) = 1.0 * sin( angle*0.5 );
  map.state( 3 ) = 0.0;

  point << 70.71067811865474, 0, 70.71067811865474;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_TRUE( visible );
  EXPECT_NEAR( 255, uv(0), 0.001);
  EXPECT_NEAR( 255, uv(1), 0.001);

  EXPECT_NEAR( 0.70710678, jacobi(0,0), 0.001);
  EXPECT_NEAR( 0, jacobi(0,1), 0.001);
  EXPECT_NEAR( -0.70710678, jacobi(0,2), 0.001);
  
  EXPECT_NEAR( 0, jacobi(1,0), 0.001);
  EXPECT_NEAR( 1, jacobi(1,1), 0.001);
  EXPECT_NEAR( 0.0, jacobi(1,2), 0.001);


  if( visible ){
    cout << "visible" << endl;  
  }else{
    cout << "not visible" << endl;  
  }
  cout << "uv:" << uv << endl;
  cout << "jacobi" << endl;
  cout << jacobi << endl;
  cout << "/jacobi" << endl;

}

TEST(camera_test, project_point_distorsion)
{
  CameraParams params = initAndroidParams();

  Map map(params);

  Eigen::Vector3d point;
  Eigen::Vector2d uv;
  Eigen::Matrix< double, 2, 3 > jacobi;
  bool visible;

  point << -42.0, 32.0, 87.0;
  map.cam->project_point( point, visible, uv,  jacobi);
  EXPECT_TRUE( visible );

  EXPECT_NEAR( 167.366, uv(0), 0.001);
  EXPECT_NEAR( 424.805, uv(1), 0.001);

  EXPECT_NEAR( 3.53121, jacobi(0,0), 0.001);
  EXPECT_NEAR( 1.13932, jacobi(0,1), 0.001);
  EXPECT_NEAR( 1.28566, jacobi(0,2), 0.001);

  EXPECT_NEAR( 1.13995, jacobi(1,0), 0.001);
  EXPECT_NEAR( 6.35896, jacobi(1,1), 0.001);
  EXPECT_NEAR( -1.78861, jacobi(1,2), 0.001);

}

