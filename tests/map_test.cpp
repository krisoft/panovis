#include "gtest/gtest.h"
#include "map.h"

TEST(map_test, init_block_1)
{
	CameraParams params = initAndroidParams();
	Map map(params);

	EXPECT_EQ(1, map.state.cols() );
	EXPECT_EQ(Camera::DIM, map.state.rows() );

	// Quaternion
	EXPECT_EQ(1, map.state(0) );
	EXPECT_EQ(0, map.state(1) );
	EXPECT_EQ(0, map.state(2) );
	EXPECT_EQ(0, map.state(3) );
	EXPECT_EQ(0, map.state(4) );
	EXPECT_EQ(0, map.state(5) );
	EXPECT_EQ(0, map.state(6) );

	EXPECT_EQ(Camera::DIM, map.covariance.cols() );
	EXPECT_EQ(Camera::DIM, map.covariance.rows() );
	for(int i=0; i<7; i++){
		for(int j=0; j<7; j++){
			if(i==j){
				EXPECT_NE(0, map.covariance(i,j) );
			}else{
				EXPECT_EQ(0, map.covariance(i,j) );
			}
		}
	}
}

TEST(map_test, add_feature)
{
	CameraParams params = initSimpleParams();
	Map map(params);
	map.addFeature( 255, 255 );

	EXPECT_EQ(1, map.state.cols() );
	EXPECT_EQ(Camera::DIM+Feature::DIM, map.state.rows() );

	// Quaternion
	EXPECT_EQ(1, map.state(0) );
	EXPECT_EQ(0, map.state(1) );
	EXPECT_EQ(0, map.state(2) );
	EXPECT_EQ(0, map.state(3) );
	EXPECT_EQ(0, map.state(4) );
	EXPECT_EQ(0, map.state(5) );
	EXPECT_EQ(0, map.state(6) );

	// feature azimuth elevation
	EXPECT_EQ(0, map.state(7) );
	EXPECT_EQ(0, map.state(8) );

	EXPECT_EQ(Camera::DIM+Feature::DIM, map.covariance.cols() );
	EXPECT_EQ(Camera::DIM+Feature::DIM, map.covariance.rows() );
	for(int i=0; i<9; i++){
		for(int j=0; j<9; j++){
			if(i==j){
				EXPECT_NE(0, map.covariance(i,j) );
			}else{
				EXPECT_EQ(0, map.covariance(i,j) );
			}
		}
	}
}

