#include "gtest/gtest.h"
#include "map.h"

TEST(map_test, init_block_1)
{
	CameraParams params;
	params.pic_width = 432;
	params.pic_height = 768;
	params.intrinsic << 668.48649151, 0, 391.70678497,
	0, 668.85548011, 218.61361818,
	0, 0, 1;
	params.dist << 0.09967969, -0.1972944,  -0.00055862,  0.0030309,   0.18338698;

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

