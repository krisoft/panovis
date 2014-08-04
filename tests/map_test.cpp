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

TEST(map_test, predict)
{
	CameraParams params = initSimpleParams();
	Map map(params);
	map.addFeature( 100, 125 );

	double feature_e = map.state(7);
	double feature_a = map.state(8);

	double w1 = 0.42;
	double w2 = 0.32;
	double w3 = 0.22;
	double dt = 0.1;

	map.state(4) = w1;
	map.state(5) = w2;
	map.state(6) = w3;

	map.predict( dt );

	EXPECT_NEAR( w1, map.state(4), 0.00001);
	EXPECT_NEAR( w2, map.state(5), 0.00001);
	EXPECT_NEAR( w3, map.state(6), 0.00001);
	EXPECT_NEAR( feature_e, map.state(7), 0.00001);	
	EXPECT_NEAR( feature_a, map.state(8), 0.00001);	

	w1 *= dt;
	w2 *= dt;
	w3 *= dt;

	double length_omega = sqrt(w1*w1 + w2*w2 + w3*w3) * dt;
	double qw = cos( length_omega*0.5 );
	double qx = sin( length_omega*0.5 ) * (w1 / length_omega);
	double qy = sin( length_omega*0.5 ) * (w2 / length_omega);
	double qz = sin( length_omega*0.5 ) * (w3 / length_omega);

	EXPECT_NEAR( qw, map.state(0), 0.001);
	EXPECT_NEAR( qx, map.state(1), 0.001);
	EXPECT_NEAR( qy, map.state(2), 0.001);
	EXPECT_NEAR( qz, map.state(3), 0.001);

	
}