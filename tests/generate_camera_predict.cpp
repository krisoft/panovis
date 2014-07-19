#include <iostream>
#include <fstream>
#include <math.h>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
using ceres::internal::AutoDiff;

#include "random_utils.h"
using namespace std;

struct CameraPredictFunc {
  CameraPredictFunc(double dt) : dt(dt) {}

  template <typename A>
  bool operator()(A const in[7], A result[7]) const {
    A Adt = A(dt);
    A q[4];
    q[0] = in[0];
    q[1] = in[1];
    q[2] = in[2];
    q[3] = in[3];

    A q_norm = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
    q[0] /= q_norm;
    q[1] /= q_norm;
    q[2] /= q_norm;
    q[3] /= q_norm;

    A omegadt[3];
    omegadt[0] = in[4]*Adt;
    omegadt[1] = in[5]*Adt;
    omegadt[2] = in[6]*Adt;

    A qomegadt[4];
    A newq[4];
    AngleAxisToQuaternion( omegadt, qomegadt );
    QuaternionProduct( q, qomegadt, newq );
    result[0] = newq[0];
    result[1] = newq[1];
    result[2] = newq[2];
    result[3] = newq[3];
    for(int i=4; i<7; i++){
      result[i] = in[i];
    }
    return true;
  }

  private:
    double dt;
};

int main()
{
  std::ofstream outp("camera_predict.txt");
  int test_case_count = 1000;
  outp << test_case_count << endl;

  for(int test_case_id=0; test_case_id<test_case_count; test_case_id++){
    cout << test_case_id << endl;
    double q[4];
    if( test_case_id%2==0 ){
      random_unit_quaternion( q );
    }else{
      random_quaternion( q );
    }
    double x[3];
    x[0] = fRand( -1.0, 1.0 );
    x[1] = fRand( -1.0, 1.0 );
    x[2] = fRand( -1.0, 1.0 );

    double dt = fRand( 0.01, 2.0 );

    CameraPredictFunc fv(dt);
    double jacobian[7*7];
    double new_x[7];
    double old_x[7];

    for(int i=0; i<4; i++){
      old_x[i] = q[i];
    }
    for(int i=0; i<3; i++){
      old_x[i+4] = x[i];
    }

    double *parameters[] = { old_x };
    double *jacobians[] = { jacobian };

    AutoDiff<CameraPredictFunc, double, 7>::Differentiate(
      fv, parameters, 7, new_x, jacobians);

    for( int i=0; i<7; i++){
      outp << old_x[ i ]  << " ";
    }
    outp << endl;
    outp << dt << endl;
    for( int i=0; i<7; i++){
      outp << new_x[ i ] << " ";
    }
    outp << endl;
    for( int i=0; i<7; i++){
      for( int j=0; j<7; j++){
        outp << jacobian[ j+i*7 ] << " ";
      }
      outp << endl;
    }
  }
}