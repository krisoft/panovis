#include "gtest/gtest.h"
#include "map.h"

TEST(map_test, init_block_1)
{
	CameraParams params = initAndroidParams();
	Map map(params);

	EXPECT_EQ(1, map.x.cols() );
	EXPECT_EQ(Camera::DIM, map.x.rows() );

	// Quaternion
	EXPECT_EQ(1, map.x(0) );
	EXPECT_EQ(0, map.x(1) );
	EXPECT_EQ(0, map.x(2) );
	EXPECT_EQ(0, map.x(3) );
	EXPECT_EQ(0, map.x(4) );
	EXPECT_EQ(0, map.x(5) );
	EXPECT_EQ(0, map.x(6) );

	EXPECT_EQ(Camera::DIM, map.P.cols() );
	EXPECT_EQ(Camera::DIM, map.P.rows() );
	for(int i=0; i<7; i++){
		for(int j=0; j<7; j++){
			if(i==j){
				EXPECT_NE(0, map.P(i,j) );
			}else{
				EXPECT_EQ(0, map.P(i,j) );
			}
		}
	}

}

