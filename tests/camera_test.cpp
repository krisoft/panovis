#include "gtest/gtest.h"
#include "map.h"
#include "camera.h"

#include <iostream>
#include <fstream>

using namespace std;

TEST(camera_test, predict)
{
  CameraParams params;
  params.pic_width = 432;
  params.pic_height = 768;
  params.intrinsic << 668.48649151, 0, 391.70678497,
  0, 668.85548011, 218.61361818,
  0, 0, 1;
  params.dist << 0.09967969, -0.1972944,  -0.00055862,  0.0030309,   0.18338698;

  Map map(params);

  std::ifstream inp("camera_predict.txt");
  ASSERT_TRUE( inp );

  int test_case_count;
  inp >> test_case_count;

  for(int test_id=0; test_id<test_case_count; test_id++){
    for(int j=0; j<7; j++){
      inp >> map.x( j );
    }
    double dt;
    inp >> dt;
    Eigen::VectorXd new_x(7);
    Eigen::MatrixXd jacobi(7,7);
    map.cam->predict( dt, new_x, jacobi );
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
  }
}

TEST(camera_test, predict_noise)
{
  CameraParams params;
  params.pic_width = 432;
  params.pic_height = 768;
  params.intrinsic << 668.48649151, 0, 391.70678497,
  0, 668.85548011, 218.61361818,
  0, 0, 1;
  params.dist << 0.09967969, -0.1972944,  -0.00055862,  0.0030309,   0.18338698;

  Map map(params);

  Eigen::MatrixXd noise(7,7);
  map.cam->predict_noise( noise );
  cout << noise << endl;
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