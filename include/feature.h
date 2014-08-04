#ifndef RVISION_FEATURE_H_
#define RVISION_FEATURE_H_

#include <Eigen/Dense>

class Map;

class Feature {
    
    Map *map;
    int index_offset;

  public:

    static const int DIM;
    Feature( const int index_offset, Map *map, double u, double v );

    void predict( double dt, Eigen::VectorXd &new_x, Eigen::MatrixXd &jacobi, Eigen::MatrixXd &noise );
    
};


#endif  // RVISION_FEATURE_H_