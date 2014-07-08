#include "gtest/gtest.h"
#include "matrix_utils.h"


TEST(matrix_utils_test, delete_matrix_block_1)
{
	Eigen::MatrixXd m(5,5);
	for( int i=0; i<5; i++ ){
		for( int j=0; j<5; j++ ){
			m( i,j )= j+i*5;
		}
	}
	/*
	 0  1  2  3  4
	 5  6  7  8  9
	10 11 12 13 14
	15 16 17 18 19
	20 21 22 23 24
	*/


	deleteMatrixBlock( m, 0, 1 );


	/*
	 6  7  8  9
	11 12 13 14
	16 17 18 19
	21 22 23 24
	*/

	EXPECT_EQ(4, m.cols() );
	EXPECT_EQ(4, m.rows() );




	EXPECT_EQ(6, m(0,0) );
	EXPECT_EQ(7, m(0,1) );
	EXPECT_EQ(8, m(0,2) );
	EXPECT_EQ(9, m(0,3) );

	EXPECT_EQ(11, m(1,0) );
	EXPECT_EQ(12, m(1,1) );
	EXPECT_EQ(13, m(1,2) );
	EXPECT_EQ(14, m(1,3) );

	EXPECT_EQ(16, m(2,0) );
	EXPECT_EQ(17, m(2,1) );
	EXPECT_EQ(18, m(2,2) );
	EXPECT_EQ(19, m(2,3) );

	EXPECT_EQ(21, m(3,0) );
	EXPECT_EQ(22, m(3,1) );
	EXPECT_EQ(23, m(3,2) );
	EXPECT_EQ(24, m(3,3) );

}



TEST(matrix_utils_test, delete_matrix_block_2)
{
	Eigen::MatrixXd m(5,5);
	for( int i=0; i<5; i++ ){
		for( int j=0; j<5; j++ ){
			m( i,j )= j+i*5;
		}
	}
	/*
	 0  1  2  3  4
	 5  6  7  8  9
	10 11 12 13 14
	15 16 17 18 19
	20 21 22 23 24
	*/


	deleteMatrixBlock( m, 0, 2 );

	/*
	12 13 14
	17 18 19
	22 23 24
	*/

	EXPECT_EQ(3, m.cols() );
	EXPECT_EQ(3, m.rows() );




	EXPECT_EQ(12, m(0,0) );
	EXPECT_EQ(13, m(0,1) );
	EXPECT_EQ(14, m(0,2) );

	EXPECT_EQ(17, m(1,0) );
	EXPECT_EQ(18, m(1,1) );
	EXPECT_EQ(19, m(1,2) );

	EXPECT_EQ(22, m(2,0) );
	EXPECT_EQ(23, m(2,1) );
	EXPECT_EQ(24, m(2,2) );

}

TEST(matrix_utils_test, delete_matrix_block_3)
{
	Eigen::MatrixXd m(5,5);
	for( int i=0; i<5; i++ ){
		for( int j=0; j<5; j++ ){
			m( i,j )= j+i*5;
		}
	}
	/*
	 0  1  2  3  4
	 5  6  7  8  9
	10 11 12 13 14
	15 16 17 18 19
	20 21 22 23 24
	*/


	deleteMatrixBlock( m, 2, 2 );

	/*
	 0  1  4
	 5  6  9
	20 21 24
	*/

	EXPECT_EQ(3, m.cols() );
	EXPECT_EQ(3, m.rows() );

	EXPECT_EQ(0, m(0,0) );
	EXPECT_EQ(1, m(0,1) );
	EXPECT_EQ(4, m(0,2) );

	EXPECT_EQ(5, m(1,0) );
	EXPECT_EQ(6, m(1,1) );
	EXPECT_EQ(9, m(1,2) );

	EXPECT_EQ(20, m(2,0) );
	EXPECT_EQ(21, m(2,1) );
	EXPECT_EQ(24, m(2,2) );

}

TEST(matrix_utils_test, delete_matrix_block_4)
{
	Eigen::MatrixXd m(5,5);
	for( int i=0; i<5; i++ ){
		for( int j=0; j<5; j++ ){
			m( i,j )= j+i*5;
		}
	}
	/*
	 0  1  2  3  4
	 5  6  7  8  9
	10 11 12 13 14
	15 16 17 18 19
	20 21 22 23 24
	*/


	deleteMatrixBlock( m, 4, 1 );

	/*
	 0  1  2  3
	 5  6  7  8
	10 11 12 13
	15 16 17 18
	*/

	EXPECT_EQ(4, m.cols() );
	EXPECT_EQ(4, m.rows() );




	EXPECT_EQ(0, m(0,0) );
	EXPECT_EQ(1, m(0,1) );
	EXPECT_EQ(2, m(0,2) );
	EXPECT_EQ(3, m(0,3) );

	EXPECT_EQ(5, m(1,0) );
	EXPECT_EQ(6, m(1,1) );
	EXPECT_EQ(7, m(1,2) );
	EXPECT_EQ(8, m(1,3) );

	EXPECT_EQ(10, m(2,0) );
	EXPECT_EQ(11, m(2,1) );
	EXPECT_EQ(12, m(2,2) );
	EXPECT_EQ(13, m(2,3) );

	EXPECT_EQ(15, m(3,0) );
	EXPECT_EQ(16, m(3,1) );
	EXPECT_EQ(17, m(3,2) );
	EXPECT_EQ(18, m(3,3) );

}

TEST(matrix_utils_test, delete_matrix_block_5)
{
	Eigen::MatrixXd m(5,5);
	for( int i=0; i<5; i++ ){
		for( int j=0; j<5; j++ ){
			m( i,j )= j+i*5;
		}
	}
	/*
	 0  1  2  3  4
	 5  6  7  8  9
	10 11 12 13 14
	15 16 17 18 19
	20 21 22 23 24
	*/


	deleteMatrixBlock( m, 0, 5 );

	EXPECT_EQ(0, m.cols() );
	EXPECT_EQ(0, m.rows() );

}



TEST(matrix_utils_test, add_matrix_block)
{
	Eigen::MatrixXd m(5,5);
	for( int i=0; i<5; i++ ){
		for( int j=0; j<5; j++ ){
			m( i,j )= j+i*5+42;
		}
	}

	addMatrixBlock( m, 3 );

	EXPECT_EQ(8, m.cols() );
	EXPECT_EQ(8, m.rows() );

	for( int i=0; i<8; i++ ){
		for( int j=0; j<8; j++ ){
			if( j>4 || i>4 ){
				EXPECT_EQ(0, m(i,j) );
			} else {
				EXPECT_EQ(j+i*5+42, m(i,j) );
			}
		}
	}
}


TEST(matrix_utils_test, delete_vector_block_1)
{
	Eigen::VectorXd m(5);
	for( int i=0; i<5; i++ ){
		m( i )= i;
	}

	deleteVectorBlock( m, 0, 1 );


	EXPECT_EQ(1, m.cols() );
	EXPECT_EQ(4, m.rows() );

	EXPECT_EQ(1, m(0) );
	EXPECT_EQ(2, m(1) );
	EXPECT_EQ(3, m(2) );
	EXPECT_EQ(4, m(3) );

}

TEST(matrix_utils_test, delete_vector_block_2)
{
	Eigen::VectorXd m(5);
	for( int i=0; i<5; i++ ){
		m( i )= i;
	}

	deleteVectorBlock( m, 1, 2 );


	EXPECT_EQ(1, m.cols() );
	EXPECT_EQ(3, m.rows() );

	EXPECT_EQ(0, m(0) );
	EXPECT_EQ(3, m(1) );
	EXPECT_EQ(4, m(2) );

}

TEST(matrix_utils_test, add_vector_block)
{
	Eigen::VectorXd m(5);
	for( int i=0; i<5; i++ ){
		m( i )= i+32;
	}

	addVectorBlock( m, 3 );


	EXPECT_EQ(1, m.cols() );
	EXPECT_EQ(8, m.rows() );

	for( int i=0; i<5; i++){
		EXPECT_EQ(i+32, m(i) );
	}
	for( int i=5; i<8; i++){
		EXPECT_EQ(0, m(i) );
	}
}
