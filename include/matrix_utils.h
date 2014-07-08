#ifndef RVISION_MATRIX_UTILS_H_
#define RVISION_MATRIX_UTILS_H_


#include <Eigen/Dense>

void deleteMatrixBlock( Eigen::MatrixXd &matrix, const int block_start_index, const int block_length );

void addMatrixBlock( Eigen::MatrixXd &matrix, const int block_length );


void deleteVectorBlock( Eigen::VectorXd &matrix, const int block_start_index, const int block_length );

void addVectorBlock( Eigen::VectorXd &matrix, const int block_length );



#endif  // RVISION_MATRIX_UTILS_H_