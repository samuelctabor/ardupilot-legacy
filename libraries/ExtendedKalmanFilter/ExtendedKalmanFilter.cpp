#include "Arduino.h"
#include "ExtendedKalmanFilter.h"
#include "MatrixMath.h"
#include "math.h"

float ExtendedKalmanFilter::measurementpredandjacobian (float* A)
{
	//This function computes the jacobian using equations from
	//analytical derivation of Gaussian updraft distribution
	//This expression gets used lots
	float expon = exp(-(pow(x1[2],2)+pow(x1[3],2))/pow(x1[1],2));
	//Expected measurement
	float w = x1[0]*expon;
	
	//Elements of the Jacobian
	A[0]= expon;
	A[1]= 2*x1[0]*((pow(x1[2],2)+pow(x1[3],2))/pow(x1[1],3))*expon;
	A[2]=-2*(x1[0]*x1[2]/pow(x1[1],2))*expon;
	A[3]=-2*(x1[0]*x1[3]/pow(x1[1],2))*expon; 
	return w;
}


void ExtendedKalmanFilter::reset(float x[N], float p[N][N],float q[4][4], float r[1][1])
{
	mmath.MatrixCopy((float*)p,N,N,(float*)P);
	mmath.MatrixCopy((float*)x,N,1,(float*)X);
    mmath.MatrixCopy((float*)q,N,N,(float*)Q); //
	mmath.MatrixCopy((float*)r,1,1,(float*)R); //
}
void ExtendedKalmanFilter::update(float z,float Vx, float Vy)
{      
	//LINE 28  
	//Estimate new state from old. 
	mmath.MatrixCopy((float*)X,N,1,(float*)x1);
	x1[2]-=Vx;
	x1[3]-=Vy;
	
	
	if (0) {
		mmath.MatrixCopy((float*)x1,N,1,(float*)X);
		return;
		}
		
	//LINE 33
	//Update the covariance matrix 
	//P=A*ekf.P*A'+ekf.Q;              
	//We know A is identity so
	//P=ekf.P+ekf.Q;
	mmath.MatrixAdd((float*)P,(float*)Q,N,N,(float*)P_predict);
	
	//What measurement do we expect to receive in the estimated
	//state
	// LINE 37
	//[z1,H]=ekf.jacobian_h(x1);
	float z1 = measurementpredandjacobian((float*)H);    
	
	// LINE 40
	//P12=P*H';
	mmath.MatrixTranspose((float*)H,1,N,(float*)Htrans);
	mmath.MatrixMult((float*)P_predict,(float*)Htrans,N,N,1,(float*)P12);      //cross covariance

	// LINE 41
    //Calculate the KALMAN GAIN
	// K=P12*inv(H*P12+ekf.R);                     %Kalman filter gain
	mmath.MatrixMult((float*)H,(float*)P12,1,N,1,(float*)temp1);
	mmath.MatrixAdd((float*)temp1,(float*)R,1,1,(float*)temp2);
	//mmath.MatrixInvert((float*)temp2,N);
	temp2[0][0]=float(1)/temp2[0][0];
	mmath.MatrixMult((float*)P12,(float*)temp2,N,1,1,(float*)K);
	
	//Correct the state estimate using the measurement residual.
	//LINE 44
	//X=x1+K*(z-z1);
	float residual = z-z1;
	mmath.MatrixMult((float*)K,&residual,N,1,1,(float*)temp1);
	mmath.MatrixAdd((float*)temp1,(float*)x1,N,1,(float*)X);
	
	//Correct the covariance too.
	// LINE 46
	// NB should be altered to reflect Stengel
	//P=P_predict-K*P12'; 
	mmath.MatrixTranspose((float*)P12,N,1,(float*)temp1);
	mmath.MatrixMult((float*)K,(float*)temp1,N,1,N,(float*)temp2);
	mmath.MatrixSubtract((float*)P_predict,(float*)temp2,N,N,(float*)P);
	
	mmath.MatrixForceSymmetry((float*)P,N);
}