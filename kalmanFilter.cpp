/*
 * File name: kalmanFilter.cpp
 *
 * Contains: Functions for kalman filter  
 * 
 * Author: Findlay Brown
 * Language: C++
 */


#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "kalmanFilter.h"
#include "Eigen\Dense"
using namespace std;
using namespace Eigen;
/*kalmanFilter::kalmanFilter(double initialX){
    // Default constructor
    X[0] = initialX;
}*/

void kalmanFilter::predict(double u){
/*
    Function: predict
    ----------
    Predict next state (prior) using the Kalman filter state propagation
    equations.
    Parameters
    ----------
    u : double, Control value: acceleration
    ----------
    B : double [2][2], or None
        Optional control transition matrix; a value of None
        will cause the filter to use `self.B`.
    F : np.array(dim_x, dim_x), or None
        Optional state transition matrix; a value of None
        will cause the filter to use `self.F`.
    Q : np.array(dim_x, dim_x), scalar, or None
        Optional process noise matrix; a value of None will cause the
        filter to use `self.Q`.
*/
    // X = FX + Bu
    // X = dot(F, X) + dot(B, u)
    X = F*X + B * u;

    // P = FPF' + Q
    // F' = F transposed
    // P = dot(dot(F, P), F') + Q
    P = F * P * F.transpose() + Q;
    
    // self.x_prior = self.x.copy()
    // self.P_prior = self.P.copy()
    X_prior = X;
    P_prior = P;
    writePrev();
}

void kalmanFilter::update(double z){
/*
    Function: update
    ----------
    Add a new measurement (z) to the Kalman filter.
    If z is None, nothing is computed. However, x_post and P_post are
    updated with the prior (x_prior, P_prior), and self.z is set to None.
    Parameters
    ----------
    z : (dim_z, 1): array_like
        measurement for this update. z can be a scalar if dim_z is 1,
        otherwise it must be convertible to a column vector.
        If you pass in a value of H, z must be a column vector the
        of the correct size.
    kalFil.R : np.array, scalar, or None
        Optionally provide R to override the measurement noise for this
        one call, otherwise  self.R will be used.
    kalFil.H : np.array, or None
        Optionally provide H to override the measurement function for this
        one call, otherwise self.H will be used.
*/
    // y = z - Hx
    // error (residual) between measurement and prediction
    double y = z - H.dot(X);
    
    // S = HPH' + R
    // project system uncertainty into measurement space
    MatrixXd S(1,1); 
    S(0) = H.transpose() * P * H + R;

    // K = PH'inv(S)
    // map system uncertainty into kalman gain
    Vector2d K;
    K = (P * H) * S.inverse();

    // x = x + Ky
    // predict new x with residual scaled by the kalman gain
    X = X + K*y;

    // P = (I-KH)P(I-KH)' + KRK'
    // This is more numerically stable
    // and works for non-optimal K vs the equation
    // P = (I-KH)P usually seen in the literature.
    Matrix2d I; I << 1, 0, 0, 1;
    I = I - K*H.transpose();
    
    P = I * P * I.transpose() + K * R * K.transpose();

}

void kalmanFilter::writePrev(){
/*
    Writes previous altitude and velocity predictions 
    to the textfile 'outputData.txt'
*/
    fstream outFile;
    outFile.open("outputData.txt", ios_base::app);
    outFile << endl << X_prior(0) << ",\t\t" << X_prior(1);
    outFile.close();
}