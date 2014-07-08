#include "matrix_utils.h"
#include <assert.h>

#include <iostream>

void deleteMatrixBlock( Eigen::MatrixXd &matrix, const int block_start_index, const int block_length ){
  assert( matrix.rows()==matrix.cols() );
  assert( matrix.rows()>=block_start_index+block_length );
  assert( block_start_index>=0 );

  int size = matrix.rows();

  /*
    R: remain
    D: delete
    Mx: move

    R  D  M1
    D  D  D
    M2 D  M3
  */

  int size_to_move = size-block_start_index-block_length;
  int move_from = block_start_index+block_length;

  // M1
  matrix.block( 0, block_start_index, block_start_index,  size_to_move) = matrix.block( 0, move_from, block_start_index, size_to_move );
  // M2
  matrix.block( block_start_index, 0, size_to_move, block_start_index ) = matrix.block( move_from, 0, size_to_move, block_start_index );
  // M3
  matrix.block( block_start_index, block_start_index, size_to_move, size_to_move) = matrix.block( move_from, move_from, size_to_move, size_to_move );

  matrix.conservativeResize( size-block_length, size-block_length );

}


void addMatrixBlock( Eigen::MatrixXd &matrix, const int block_length ){
  assert( matrix.rows()==matrix.cols() );

  int old_size = matrix.rows();
  matrix.conservativeResize( old_size+block_length, old_size+block_length );
  /*
    O: old
    Nx: newly initalized
    O   N1
    N2  N1
  */
  // N1
  matrix.block( 0, old_size, old_size+block_length, block_length ) = Eigen::MatrixXd::Zero( old_size+block_length, block_length);
  // N2
  matrix.block( old_size, 0, block_length, old_size ) = Eigen::MatrixXd::Zero( block_length, old_size);

}


void deleteVectorBlock( Eigen::VectorXd &matrix, const int block_start_index, const int block_length ){

  assert( 1==matrix.cols() );
  assert( matrix.rows()>=block_start_index+block_length );

  int size = matrix.rows();
  /*
    R
    D
    M
  */

  matrix.block( block_start_index, 0, size-block_start_index-block_length, 1 ) = matrix.block( block_start_index+block_length, 0,  size-block_start_index-block_length, 1 );
  matrix.conservativeResize( size-block_length, 1 );

}

void addVectorBlock( Eigen::VectorXd &matrix, const int block_length ){
  assert( 1==matrix.cols() );

  int old_size = matrix.rows();
  matrix.conservativeResize( old_size+block_length, 1 );
  matrix.block( old_size, 0, block_length, 1 ) = Eigen::MatrixXd::Zero( block_length, 1);

}


