#include "camera.h"

CameraParams initAndroidParams(){
  CameraParams params;
  params.pic_width = 432;
  params.pic_height = 768;

  params.intrinsic = cv::Mat::eye(3, 3, CV_64F);
  params.intrinsic.at<double>( 0, 0) = 668.48649151;
  params.intrinsic.at<double>( 0, 1) = 0.0;
  params.intrinsic.at<double>( 0, 2) = 391.70678497;
  
  params.intrinsic.at<double>( 1, 0) = 0.0;
  params.intrinsic.at<double>( 1, 1) = 668.85548011;
  params.intrinsic.at<double>( 1, 2) = 218.61361818;

  params.intrinsic.at<double>( 2, 0) = 0.0;
  params.intrinsic.at<double>( 2, 1) = 0.0;
  params.intrinsic.at<double>( 2, 2) = 1.0;


  params.dist = cv::Mat::zeros(5, 1, CV_64F);
  params.dist.at<double>(0,0) = 0.09967969;
  params.dist.at<double>(1,0) = -0.1972944;
  params.dist.at<double>(2,0) = -0.00055862;
  params.dist.at<double>(3,0) = 0.0030309;
  params.dist.at<double>(4,0) = 0.18338698;

  return params;
};


CameraParams initSimpleParams(){
  CameraParams params;
  params.pic_width = 500;
  params.pic_height = 500;

  params.intrinsic = cv::Mat::eye(3, 3, CV_64F);
  params.intrinsic.at<double>( 0, 0) = 100.0;
  params.intrinsic.at<double>( 0, 1) = 0.0;
  params.intrinsic.at<double>( 0, 2) = 255;
  
  params.intrinsic.at<double>( 1, 0) = 0.0;
  params.intrinsic.at<double>( 1, 1) = 100.0;
  params.intrinsic.at<double>( 1, 2) = 255;

  params.intrinsic.at<double>( 2, 0) = 0.0;
  params.intrinsic.at<double>( 2, 1) = 0.0;
  params.intrinsic.at<double>( 2, 2) = 1.0;


  params.dist = cv::Mat::zeros(5, 1, CV_64F);

  return params;
}