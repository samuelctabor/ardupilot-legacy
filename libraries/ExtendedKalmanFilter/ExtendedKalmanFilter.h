/* 
Extended Kalman Filter class by Sam Tabor, 2013.
http://diydrones.com/forum/topics/autonomous-soaring
Set up for identifying thermals of Gaussian form, but could be adapted to other 
purposes by adapting the equations for the jacobians.
*/
#ifndef ExtendedKalmanFilter_h
#define ExtendedKalmanFilter_h

#include "Arduino.h"
#include "MatrixMath.h"
#include "math.h"

#define N (4)

class ExtendedKalmanFilter
{
  MatrixMath mmath;
  float x1[4];	
  float H[1][4];
  float Htrans[4][1];
  float P_predict[4][4];
  float P12[4][4];
  float temp1[4][4];
  float temp2[4][4];
  float K[4][4];
  float measurementpredandjacobian (float* A);
  public:
  float X[4];
  float P[4][4];
  float Q[4][4];
  float R[4][4];
  //ExtendedKalmanFilter (float x[N], float p[N][N], float q[4][4], float r[1][1]);
  void reset(float x[N], float p[N][N], float q[4][4], float r[1][1]);
  void update(float z,float Vx, float Vy);
};


#endif