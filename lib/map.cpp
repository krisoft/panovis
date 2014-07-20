#include "map.h"
#include "camera.h"
#include "feature.h"
#include "matrix_utils.h"

Map::Map( const CameraParams params ){
	addVectorBlock( state, Camera::DIM );
	addMatrixBlock( covariance, Camera::DIM );

	cam = new Camera(params, 0, this);
}

void Map::addFeature( double u, double v ){

  int index_offset = state.rows();
  addVectorBlock( state, Feature::DIM );
  addMatrixBlock( covariance, Feature::DIM );

  features.push_back( Feature(index_offset, this, u, v) );
}