#include "map.h"
#include "camera.h"
#include "matrix_utils.h"

Map::Map( const CameraParams params ){
	addVectorBlock( this->x, Camera::DIM );
	addMatrixBlock( this->P, Camera::DIM );

	this->cam = new Camera(params, 0, this);
}