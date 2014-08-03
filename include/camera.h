#ifndef RVISION_CAMERA_H_
#define RVISION_CAMERA_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

struct CameraParams {
  cv::Mat intrinsic;
  cv::Mat  dist;
  int pic_width;
  int pic_height;
};

class Map;

class Camera {
    CameraParams params;
    Map *map;
    int index_offset;

  public:
    static const int DIM;
    Camera( const CameraParams params, const int index_offset, Map *map );

    void update_index_offset( const int index_offset );

    void predict( double dt, Eigen::VectorXd &new_x, Eigen::MatrixXd &jacobi, Eigen::MatrixXd &noise  );

    void convert_uv_to_ea( double u, double v, double &elevation, double &azimuth);

/*    void project_point( const Eigen::Vector3d point, bool &visible, Eigen::Vector2d &uv);*/
    
};

CameraParams initAndroidParams();
CameraParams initSimpleParams();

#endif  // RVISION_CAMERA_H_