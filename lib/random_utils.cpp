#include "random_utils.h"

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void random_quaternion(double q[4]){
  while(true){
    double s = 0;
    for(int i=0; i<4; i++){
      q[i] =  fRand( -1.0, 1.0 );  
      s += q[i]*q[i];
    }
    if(s<0.1){
      continue;
    }
    return;
  }
}


void random_unit_quaternion(double q[4]){
  while(true){
    double s = 0;
    for(int i=0; i<4; i++){
      q[i] =  fRand( -1.0, 1.0 );  
      s += q[i]*q[i];
    }
    if(s<0.1){
      continue;
    }
    s = sqrt(s);
    for(int i=0; i<4; i++){
      q[i] /= s;
    }
    return;
  }
}