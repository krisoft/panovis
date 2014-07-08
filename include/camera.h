#ifndef RVISION_CAMERA_H_
#define RVISION_CAMERA_H_

#include <Eigen/Dense>

struct CameraParams {
  Eigen::Matrix3d intrinsic;
  Eigen::Matrix< double, 5, 1 > dist;
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

    void predict( double dt, Eigen::VectorXd &new_x, Eigen::MatrixXd &jacobi );
    void predict_noise( Eigen::MatrixXd &noise );

/*    void project_point( const Eigen::Vector3d point, bool &visible, Eigen::Vector2d &uv);*/
    
};


#endif  // RVISION_CAMERA_H_